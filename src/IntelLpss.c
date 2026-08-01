/** @file
  Intel Serial IO (LPSS) I2C controller discovery and power bring-up.
  See IntelLpss.h for the model.

  Copyright (c) 2026, jlobue10 and contributors. All rights reserved.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <Uefi.h>
#include <Library/BaseLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <IndustryStandard/Pci.h>
#include <Protocol/PciIo.h>

#include "IntelLpss.h"

/**
  Walk the PCI capability list for the Power Management capability and force
  the function to D0. A function left in D3hot by firmware decodes config
  space but not its BARs, so this must precede any MMIO access.
**/
STATIC
VOID
LpssForceD0 (
  IN EFI_PCI_IO_PROTOCOL  *PciIo
  )
{
  UINT16  StatusReg;
  UINT8   CapPtr;
  UINT8   CapId;
  UINT16  Pmcsr;
  UINTN   Guard;

  if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint16,
                                  PCI_PRIMARY_STATUS_OFFSET, 1, &StatusReg)) ||
      ((StatusReg & EFI_PCI_STATUS_CAPABILITY) == 0)) {
    return;
  }
  if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint8,
                                  PCI_CAPBILITY_POINTER_OFFSET, 1, &CapPtr))) {
    return;
  }

  for (Guard = 0; (CapPtr >= 0x40) && (Guard < 48); Guard++) {
    if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint8,
                                    CapPtr, 1, &CapId))) {
      return;
    }
    if (CapId == EFI_PCI_CAPABILITY_ID_PMI) {
      UINT32  PmcsrOff = (UINT32)CapPtr + 4;

      if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint16,
                                      PmcsrOff, 1, &Pmcsr))) {
        return;
      }
      if ((Pmcsr & 0x3) != 0) {
        Pmcsr &= ~0x3u;                       // request D0
        (VOID)PciIo->Pci.Write (PciIo, EfiPciIoWidthUint16,
                                PmcsrOff, 1, &Pmcsr);
        gBS->Stall (10000);                   // D3hot -> D0 recovery time
        DEBUG ((DEBUG_INFO, "TouchI2c: LPSS function moved to D0\n"));
      }
      return;
    }
    if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint8,
                                    (UINT32)CapPtr + 1, 1, &CapPtr))) {
      return;
    }
  }
}

EFI_STATUS
IntelLpssPrepareI2c (
  IN  UINT16   PciDid,
  IN  UINT8    PciDevice,
  IN  UINT8    PciFunction,
  OUT UINTN    *Base,
  OUT BOOLEAN  *FreshPowerOn
  )
{
  EFI_STATUS           Status;
  EFI_HANDLE           *Handles;
  UINTN                HandleCount;
  UINTN                i;
  EFI_PCI_IO_PROTOCOL  *PciIo;
  UINTN                Seg, Bus, Dev, Func;
  UINT32               Ids;
  UINT32               BarLo;
  UINT32               BarHi;
  UINT64               Bar;
  UINT16               Command;
  UINT32               Resets;

  *Base         = 0;
  *FreshPowerOn = FALSE;

  Status = gBS->LocateHandleBuffer (ByProtocol, &gEfiPciIoProtocolGuid,
                                    NULL, &HandleCount, &Handles);
  if (EFI_ERROR (Status)) {
    return EFI_NOT_FOUND;
  }

  for (i = 0; i < HandleCount; i++) {
    if (EFI_ERROR (gBS->HandleProtocol (Handles[i], &gEfiPciIoProtocolGuid,
                                        (VOID **)&PciIo))) {
      continue;
    }
    if (EFI_ERROR (PciIo->GetLocation (PciIo, &Seg, &Bus, &Dev, &Func)) ||
        (Seg != 0) || (Bus != 0) ||
        (Dev != PciDevice) || (Func != PciFunction)) {
      continue;
    }

    //
    // Location matches; both IDs must too, or this is not the controller the
    // profile was written for and nothing gets poked (fail closed).
    //
    if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint32,
                                    PCI_VENDOR_ID_OFFSET, 1, &Ids)) ||
        ((Ids & 0xFFFF) != 0x8086) || ((Ids >> 16) != PciDid)) {
      FreePool (Handles);
      return EFI_NOT_FOUND;
    }

    LpssForceD0 (PciIo);

    //
    // BAR0; the Serial IO controllers advertise a 64-bit memory BAR that
    // firmware frequently places above 4 GiB.
    //
    if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint32,
                                    PCI_BASE_ADDRESSREG_OFFSET, 1, &BarLo))) {
      FreePool (Handles);
      return EFI_DEVICE_ERROR;
    }
    Bar = BarLo & 0xFFFFFFF0u;
    if ((BarLo & 0x6) == 0x4) {
      if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint32,
                                      PCI_BASE_ADDRESSREG_OFFSET + 4,
                                      1, &BarHi))) {
        FreePool (Handles);
        return EFI_DEVICE_ERROR;
      }
      Bar |= LShiftU64 (BarHi, 32);
    }
    if (Bar == 0) {
      FreePool (Handles);
      return EFI_NO_MAPPING;
    }

    //
    // Make sure memory decode is on. Attributes() is the sanctioned route;
    // if the host bridge rejects it, set the command-register bit directly.
    //
    Status = PciIo->Attributes (PciIo, EfiPciIoAttributeOperationEnable,
                                EFI_PCI_IO_ATTRIBUTE_MEMORY, NULL);
    if (EFI_ERROR (Status)) {
      if (!EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint16,
                                       PCI_COMMAND_OFFSET, 1, &Command)) &&
          ((Command & EFI_PCI_COMMAND_MEMORY_SPACE) == 0)) {
        Command |= EFI_PCI_COMMAND_MEMORY_SPACE;
        (VOID)PciIo->Pci.Write (PciIo, EfiPciIoWidthUint16,
                                PCI_COMMAND_OFFSET, 1, &Command);
      }
    }

    //
    // Release the LPSS resets. If the function-reset bits were still
    // asserted, the firmware never ran this controller: the DesignWare core
    // now holds only IP reset defaults and the caller must program bus
    // timing itself (FreshPowerOn).
    //
    Resets = MmioRead32 ((UINTN)Bar + LPSS_PRIV_RESETS);
    if ((Resets & LPSS_PRIV_RESETS_FUNC) != LPSS_PRIV_RESETS_FUNC) {
      MmioWrite32 ((UINTN)Bar + LPSS_PRIV_RESETS,
                   LPSS_PRIV_RESETS_FUNC | LPSS_PRIV_RESETS_IDMA);
      MmioWrite32 ((UINTN)Bar + LPSS_PRIV_REMAP_LO, (UINT32)Bar);
      MmioWrite32 ((UINTN)Bar + LPSS_PRIV_REMAP_HI, (UINT32)RShiftU64 (Bar, 32));
      *FreshPowerOn = TRUE;
      DEBUG ((DEBUG_INFO, "TouchI2c: LPSS resets released at %lx\n", Bar));
    }

    *Base = (UINTN)Bar;
    FreePool (Handles);
    return EFI_SUCCESS;
  }

  FreePool (Handles);
  return EFI_NOT_FOUND;
}
