/** @file
  TouchProbe -- a standalone UEFI Shell application that answers the single
  go/no-go question for the TouchI2cDxe project on a new device:

    "At the rEFInd / UEFI-Shell stage, is the AMD DesignWare I2C controller live,
     and does the touchscreen ACK on its I2C bus WITHOUT us doing any GPIO /
     power / clock bring-up?"

  It sweeps the candidate FCH I2C controller bases, verifies each with
  IC_COMP_TYPE, then (for each live controller) tries to read the HID-over-I2C
  descriptor from every candidate slave address (Novatek 0x01 for the Ally X,
  FocalTech 0x38 for the Steam Deck OLED, Goodix 0x14/0x5D). Results:

    * Controller present  -> IC_COMP_TYPE == 0x44570140 at base+0xF8
    * Panel ACKs + desc   -> scenario (a): firmware left it live; the driver is
                             "just" an I2C-HID reader.
    * Address NAK         -> scenario (b): panel is gated; the driver must add a
                             reset-GPIO / _PS0 nudge before talking to it.

  This is a diagnostic only -- it configures the master conservatively (100 kHz)
  and issues a single short transaction per address. It does not install any
  protocol. Build it with the same EDK2 toolchain used for UsbXbox360Dxe.

  Copyright (c) 2026, jlobue10 and contributors. All rights reserved.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <Uefi.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/IoLib.h>
#include <Library/BaseLib.h>
#include <IndustryStandard/Pci.h>
#include <Protocol/PciIo.h>

#include "../../src/DwI2c.h"
#include "../../src/FchAoac.h"
#include "../../src/I2cHid.h"
#include "../../src/IntelLpss.h"

STATIC CONST UINT32  mCandidateBases[] = {
  DW_I2C_FCH_BASE_0, DW_I2C_FCH_BASE_1, DW_I2C_FCH_BASE_2,
  DW_I2C_FCH_BASE_3, DW_I2C_FCH_BASE_4
};

STATIC CONST UINT8   mCandidateAddrs[] = {
  NVTK_I2C_ADDR, FTS_I2C_ADDR, GOODIX_I2C_ADDR_A, GOODIX_I2C_ADDR_B,
  ELAN_I2C_ADDR
};

STATIC CONST UINT16  mCandidateDescRegs[] = { 0x0000, 0x0001, 0x0020 };

#define POLL_LIMIT  100000   // ~ arbitrary bounded spins for polled status

STATIC
UINT32
RegRd (
  IN UINTN   Base,
  IN UINT32  Off
  )
{
  return MmioRead32 (Base + Off);
}

STATIC
VOID
RegWr (
  IN UINTN   Base,
  IN UINT32  Off,
  IN UINT32  Val
  )
{
  MmioWrite32 (Base + Off, Val);
}

/**
  Is a DesignWare I2C controller decoded at this base?
**/
STATIC
BOOLEAN
ControllerPresent (
  IN UINTN  Base
  )
{
  return (BOOLEAN)(RegRd (Base, DW_IC_COMP_TYPE) == DW_IC_COMP_TYPE_VALUE);
}

/**
  Bring the master to a known, disabled state and program conservative 100 kHz
  standard-speed timing (150 MHz IC clock assumed for Phoenix; a ping tolerates
  approximate HCNT/LCNT).
**/
STATIC
VOID
MasterInit (
  IN UINTN   Base,
  IN UINT8   SlaveAddr,
  IN UINT16  SsHcnt,
  IN UINT16  SsLcnt
  )
{
  UINT32  Spin;

  // Disable and wait for the enable status to clear.
  RegWr (Base, DW_IC_ENABLE, 0);
  for (Spin = 0; Spin < POLL_LIMIT; Spin++) {
    if ((RegRd (Base, DW_IC_ENABLE_STATUS) & DW_IC_ENABLE_STATUS_EN) == 0) {
      break;
    }
  }

  RegWr (Base, DW_IC_CON,
         DW_IC_CON_MASTER | DW_IC_CON_SPEED_STD |
         DW_IC_CON_RESTART_EN | DW_IC_CON_SLAVE_DISABLE);

  // ~100 kHz counts for the platform's reference clock (150 MHz on the AMD
  // FCH, 133 MHz on Intel Serial IO); a ping tolerates approximate values.
  RegWr (Base, DW_IC_SS_SCL_HCNT, SsHcnt);
  RegWr (Base, DW_IC_SS_SCL_LCNT, SsLcnt);

  RegWr (Base, DW_IC_TX_TL, 0);
  RegWr (Base, DW_IC_RX_TL, 0);

  RegWr (Base, DW_IC_TAR, SlaveAddr & 0x3FF);

  RegWr (Base, DW_IC_ENABLE, DW_IC_ENABLE_ENABLE);
  // Wait for the enable to take effect: on a tile that was just AOAC-powered,
  // commands written before EN reads back are discarded, which shows up as a
  // spurious timeout and a wrong "nothing at this address" verdict.
  for (Spin = 0; Spin < POLL_LIMIT; Spin++) {
    if (RegRd (Base, DW_IC_ENABLE_STATUS) & DW_IC_ENABLE_STATUS_EN) {
      break;
    }
  }
}

/**
  Write WLen bytes then repeated-START read RLen bytes from the current IC_TAR.
  Returns EFI_SUCCESS on ACK+data, EFI_NO_RESPONSE on address NAK (TX_ABRT with
  7-bit-addr-noack), EFI_TIMEOUT otherwise.
**/
STATIC
EFI_STATUS
XferReadReg (
  IN  UINTN   Base,
  IN  CONST UINT8  *WBuf,
  IN  UINTN   WLen,
  OUT UINT8   *RBuf,
  IN  UINTN   RLen
  )
{
  UINTN   i;
  UINT32  Spin;
  UINT32  Cmd;
  UINT32  Abrt;

  // Clear any latched abort.
  (VOID)RegRd (Base, DW_IC_CLR_TX_ABRT);

  // Write phase (the register address to read from).
  for (i = 0; i < WLen; i++) {
    for (Spin = 0; Spin < POLL_LIMIT; Spin++) {
      if (RegRd (Base, DW_IC_STATUS) & DW_IC_STATUS_TFNF) {
        break;
      }
    }
    // Without this the command is written into a full TX FIFO and dropped,
    // and the drain loop below then reports "no ACK" for a device that is
    // actually present -- a false negative in the tool that decides whether
    // a new device profile gets added.
    if (Spin >= POLL_LIMIT) {
      return EFI_TIMEOUT;
    }
    Cmd = WBuf[i];
    if ((i == 0) && (WLen > 0)) {
      // nothing special on first write beyond master START (implicit)
    }
    RegWr (Base, DW_IC_DATA_CMD, Cmd);
  }

  // Read phase: issue RLen read commands; RESTART on the first, STOP on the last.
  for (i = 0; i < RLen; i++) {
    for (Spin = 0; Spin < POLL_LIMIT; Spin++) {
      if (RegRd (Base, DW_IC_STATUS) & DW_IC_STATUS_TFNF) {
        break;
      }
    }
    if (Spin >= POLL_LIMIT) {
      return EFI_TIMEOUT;
    }
    Cmd = DW_IC_DATA_CMD_READ;
    if (i == 0) {
      Cmd |= DW_IC_DATA_CMD_RESTART;
    }
    if (i == (RLen - 1)) {
      Cmd |= DW_IC_DATA_CMD_STOP;
    }
    RegWr (Base, DW_IC_DATA_CMD, Cmd);
  }

  // Drain RLen bytes, watching for an address NAK.
  for (i = 0; i < RLen; i++) {
    for (Spin = 0; Spin < POLL_LIMIT; Spin++) {
      Abrt = RegRd (Base, DW_IC_RAW_INTR_STAT);
      if (Abrt & DW_IC_INTR_TX_ABRT) {
        UINT32 Src = RegRd (Base, DW_IC_TX_ABRT_SOURCE);
        (VOID)RegRd (Base, DW_IC_CLR_TX_ABRT);
        return (Src & DW_IC_ABRT_7B_ADDR_NOACK) ? EFI_NO_RESPONSE : EFI_DEVICE_ERROR;
      }
      if (RegRd (Base, DW_IC_STATUS) & DW_IC_STATUS_RFNE) {
        break;
      }
    }
    if (Spin >= POLL_LIMIT) {
      return EFI_TIMEOUT;
    }
    RBuf[i] = (UINT8)(RegRd (Base, DW_IC_DATA_CMD) & 0xFF);
  }

  return EFI_SUCCESS;
}

/**
  Try every candidate slave address x HID descriptor register on one live
  DesignWare controller, printing each verdict. Returns TRUE if any panel
  ACKed with a plausible HID descriptor.
**/
STATIC
BOOLEAN
ProbePanelsOnController (
  IN UINTN   Base,
  IN UINT16  SsHcnt,
  IN UINT16  SsLcnt
  )
{
  UINTN    a, r, i;
  BOOLEAN  AnyPanel = FALSE;

  for (a = 0; a < ARRAY_SIZE (mCandidateAddrs); a++) {
    for (r = 0; r < ARRAY_SIZE (mCandidateDescRegs); r++) {
      UINT8       Addr    = mCandidateAddrs[a];
      UINT16      DescReg = mCandidateDescRegs[r];
      UINT8       Reg2[2];
      UINT8       Desc[30];
      EFI_STATUS  Status;

      Reg2[0] = (UINT8)(DescReg & 0xFF);
      Reg2[1] = (UINT8)((DescReg >> 8) & 0xFF);

      MasterInit (Base, Addr, SsHcnt, SsLcnt);
      Status = XferReadReg (Base, Reg2, sizeof (Reg2), Desc, sizeof (Desc));
      RegWr (Base, DW_IC_ENABLE, 0);

      Print (L"    addr 0x%02x descreg 0x%04x: ", Addr, DescReg);
      if (Status == EFI_SUCCESS) {
        UINT16 VendorId = (UINT16)(Desc[20] | (Desc[21] << 8));
        Print (L"ACK. HID desc:");
        for (i = 0; i < sizeof (Desc); i++) {
          Print (L" %02x", Desc[i]);
        }
        Print (L"\n              wHIDDescLength=%u wVendorID=0x%04x %s\n",
               (UINT16)(Desc[0] | (Desc[1] << 8)), VendorId,
               (VendorId == 0x27C6) ? L"(Goodix!)" :
               (VendorId == 0x04F3) ? L"(ELAN!)" : L"");
        AnyPanel = TRUE;
      } else if (Status == EFI_NO_RESPONSE) {
        Print (L"no ACK (nothing at this address)\n");
      } else {
        Print (L"error %r\n", Status);
      }
    }
  }
  return AnyPanel;
}

/**
  Intel path: find every Serial IO I2C controller (PCI class 0x0C/0x80,
  vendor 0x8086), power it to D0, enable memory decode, release the LPSS
  resets, and probe the candidate panels behind it. The Intel counterpart of
  the fixed-base FCH sweep below.
**/
STATIC
VOID
IntelScanSerialIo (
  IN OUT BOOLEAN  *AnyController,
  IN OUT BOOLEAN  *AnyPanel
  )
{
  EFI_STATUS           Status;
  EFI_HANDLE           *Handles;
  UINTN                HandleCount;
  UINTN                h;
  EFI_PCI_IO_PROTOCOL  *PciIo;
  UINTN                Seg, Bus, Dev, Func;

  Handles     = NULL;
  HandleCount = 0;
  Status = gBS->LocateHandleBuffer (ByProtocol, &gEfiPciIoProtocolGuid,
                                    NULL, &HandleCount, &Handles);
  if (EFI_ERROR (Status)) {
    Print (L"No EFI_PCI_IO handles (%r) -- cannot scan Serial IO.\n", Status);
    return;
  }

  for (h = 0; h < HandleCount; h++) {
    UINT32   Ids;
    UINT32   Class;
    UINT16   Command;
    UINT32   BarLo, BarHi;
    UINT64   Bar;
    UINT32   Resets;
    UINT16   StatusReg;
    UINT8    CapPtr, CapId;
    UINTN    Guard;

    if (EFI_ERROR (gBS->HandleProtocol (Handles[h], &gEfiPciIoProtocolGuid,
                                        (VOID **)&PciIo)) ||
        EFI_ERROR (PciIo->GetLocation (PciIo, &Seg, &Bus, &Dev, &Func)) ||
        EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint32,
                                    PCI_VENDOR_ID_OFFSET, 1, &Ids)) ||
        ((Ids & 0xFFFF) != 0x8086)) {
      continue;
    }
    //
    // Serial IO I2C functions are base class 0x0C (serial bus), subclass
    // 0x80 (other) on every PCH generation that has them.
    //
    if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint32,
                                    PCI_REVISION_ID_OFFSET, 1, &Class)) ||
        ((Class >> 24) != 0x0C) || (((Class >> 16) & 0xFF) != 0x80)) {
      continue;
    }

    Print (L"[PCI %02x:%02x.%x DID 0x%04x] Serial IO candidate\n",
           (UINT32)Bus, (UINT32)Dev, (UINT32)Func, Ids >> 16);

    //
    // Force D0 through the PM capability, as a D3hot function does not
    // decode its BARs.
    //
    if (!EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint16,
                                     PCI_PRIMARY_STATUS_OFFSET, 1,
                                     &StatusReg)) &&
        ((StatusReg & EFI_PCI_STATUS_CAPABILITY) != 0) &&
        !EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint8,
                                     PCI_CAPBILITY_POINTER_OFFSET, 1,
                                     &CapPtr))) {
      for (Guard = 0; (CapPtr >= 0x40) && (Guard < 48); Guard++) {
        if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint8,
                                        CapPtr, 1, &CapId))) {
          break;
        }
        if (CapId == EFI_PCI_CAPABILITY_ID_PMI) {
          UINT16  Pmcsr;
          UINT32  PmcsrOff = (UINT32)CapPtr + 4;

          if (!EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint16,
                                           PmcsrOff, 1, &Pmcsr)) &&
              ((Pmcsr & 0x3) != 0)) {
            Print (L"    PMCSR 0x%04x -> forcing D0\n", Pmcsr);
            Pmcsr &= ~0x3u;
            (VOID)PciIo->Pci.Write (PciIo, EfiPciIoWidthUint16,
                                    PmcsrOff, 1, &Pmcsr);
            gBS->Stall (10000);
          }
          break;
        }
        if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint8,
                                        (UINT32)CapPtr + 1, 1, &CapPtr))) {
          break;
        }
      }
    }

    if (!EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint16,
                                     PCI_COMMAND_OFFSET, 1, &Command)) &&
        ((Command & EFI_PCI_COMMAND_MEMORY_SPACE) == 0)) {
      Command |= EFI_PCI_COMMAND_MEMORY_SPACE;
      (VOID)PciIo->Pci.Write (PciIo, EfiPciIoWidthUint16,
                              PCI_COMMAND_OFFSET, 1, &Command);
    }

    if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint32,
                                    PCI_BASE_ADDRESSREG_OFFSET, 1, &BarLo))) {
      continue;
    }
    Bar = BarLo & 0xFFFFFFF0u;
    if ((BarLo & 0x6) == 0x4) {
      if (EFI_ERROR (PciIo->Pci.Read (PciIo, EfiPciIoWidthUint32,
                                      PCI_BASE_ADDRESSREG_OFFSET + 4,
                                      1, &BarHi))) {
        continue;
      }
      Bar |= LShiftU64 (BarHi, 32);
    }
    if (Bar == 0) {
      Print (L"    BAR0 unassigned -- skipping\n");
      continue;
    }

    Resets = MmioRead32 ((UINTN)Bar + LPSS_PRIV_RESETS);
    Print (L"    BAR0 0x%lx, LPSS RESETS 0x%08x%s\n", Bar, Resets,
           ((Resets & LPSS_PRIV_RESETS_FUNC) == LPSS_PRIV_RESETS_FUNC)
             ? L" (already released)" : L" (held in reset -- releasing)");
    if ((Resets & LPSS_PRIV_RESETS_FUNC) != LPSS_PRIV_RESETS_FUNC) {
      MmioWrite32 ((UINTN)Bar + LPSS_PRIV_RESETS,
                   LPSS_PRIV_RESETS_FUNC | LPSS_PRIV_RESETS_IDMA);
      MmioWrite32 ((UINTN)Bar + LPSS_PRIV_REMAP_LO, (UINT32)Bar);
      MmioWrite32 ((UINTN)Bar + LPSS_PRIV_REMAP_HI,
                   (UINT32)RShiftU64 (Bar, 32));
    }

    Print (L"    IC_COMP_TYPE = 0x%08x",
           RegRd ((UINTN)Bar, DW_IC_COMP_TYPE));
    if (!ControllerPresent ((UINTN)Bar)) {
      Print (L"  (no DesignWare block)\n");
      continue;
    }
    Print (L"  <- DesignWare I2C present\n");
    *AnyController = TRUE;

    // ~100 kHz counts for the 133 MHz Serial IO reference clock.
    if (ProbePanelsOnController ((UINTN)Bar, 569, 664)) {
      *AnyPanel = TRUE;
    }
  }
  gBS->FreePool (Handles);
}

EFI_STATUS
EFIAPI
UefiMain (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  UINTN       b;
  UINT32      CpuSigEbx;
  BOOLEAN     IsAmd;
  BOOLEAN     AnyController = FALSE;
  BOOLEAN     AnyPanel = FALSE;

  Print (L"TouchProbe -- DesignWare I2C + HID-over-I2C panel liveness check\n");
  Print (L"================================================================\n\n");

  //
  // The AOAC registers and fixed FCH MMIO bases below belong to the AMD FCH
  // and mean something else entirely on other platforms; the Intel Serial IO
  // controllers are PCI functions instead. Route by CPU vendor.
  //
  AsmCpuid (0, NULL, &CpuSigEbx, NULL, NULL);
  IsAmd = (BOOLEAN)(CpuSigEbx == SIGNATURE_32 ('A', 'u', 't', 'h'));
  Print (L"CPU vendor: %s\n\n", IsAmd ? L"AMD (FCH sweep)"
                                      : L"non-AMD (Intel Serial IO scan)");

  if (!IsAmd) {
    IntelScanSerialIo (&AnyController, &AnyPanel);
    goto Verdict;
  }

  //
  // The firmware leaves the touch bus's controller tile power-gated on a
  // normal boot; un-gate it first (what the DSDT's _PS0 does) or the MMIO
  // sweep below reads garbage. AOAC devices 5..8 are I2C0..I2C3 (Ally X
  // touch is I2C0/dev 5; Galileo's \_SB.I2CB is I2C1/dev 6, per its DSDT
  // I2CB.RSET -> SRAD(0x06)). Un-gate all four so the controller sweep
  // below sees every bus.
  //
  {
    UINTN  Idx;

    for (Idx = FCH_AOAC_DEV_I2C0; Idx <= FCH_AOAC_DEV_I2C3; Idx++) {
      UINT8  AoacState = MmioRead8 (FCH_AOAC_DEV_STATUS (Idx));

      Print (L"AOAC I2C%u (dev %u) state: 0x%02x %s\n",
             (UINT32)(Idx - FCH_AOAC_DEV_I2C0), (UINT32)Idx, AoacState,
             ((AoacState & FCH_AOAC_STATE_MASK) == FCH_AOAC_STATE_D0)
               ? L"(already D0)" : L"(gated -- powering on)");
      if ((AoacState & FCH_AOAC_STATE_MASK) != FCH_AOAC_STATE_D0) {
        UINT8  Ctl;
        UINTN  Us;

        Ctl  = MmioRead8 (FCH_AOAC_DEV_CTL (Idx));
        Ctl &= ~FCH_AOAC_TARGET_STATE_MASK;
        Ctl |= FCH_AOAC_PWR_ON_DEV;
        MmioWrite8 (FCH_AOAC_DEV_CTL (Idx), Ctl);
        for (Us = 0; Us < 100000; Us += 100) {
          if ((MmioRead8 (FCH_AOAC_DEV_STATUS (Idx)) & FCH_AOAC_STATE_MASK)
              == FCH_AOAC_STATE_D0) {
            break;
          }
          gBS->Stall (100);
        }
        Print (L"AOAC I2C%u (dev %u) state now: 0x%02x\n",
               (UINT32)(Idx - FCH_AOAC_DEV_I2C0), (UINT32)Idx,
               MmioRead8 (FCH_AOAC_DEV_STATUS (Idx)));
      }
    }
    Print (L"\n");
  }

  for (b = 0; b < ARRAY_SIZE (mCandidateBases); b++) {
    UINTN  Base = mCandidateBases[b];
    UINT32 Type = RegRd (Base, DW_IC_COMP_TYPE);

    Print (L"[base 0x%08x] IC_COMP_TYPE = 0x%08x", (UINT32)Base, Type);
    if (!ControllerPresent (Base)) {
      Print (L"  (no controller)\n");
      continue;
    }
    Print (L"  <- DesignWare I2C present\n");
    AnyController = TRUE;

    // ~100 kHz counts for the 150 MHz FCH reference clock.
    if (ProbePanelsOnController (Base, 600, 900)) {
      AnyPanel = TRUE;
    }
  }

Verdict:
  Print (L"\n---- verdict ----\n");
  if (!AnyController) {
    Print (L"No DesignWare I2C controller found (fixed FCH bases on AMD,\n");
    Print (L"Serial IO PCI scan on Intel).\n");
    Print (L"=> Fill in the real controller from the collected DSDT\n");
    Print (L"   (AMDI0010 _CRS on AMD; I2Cx PCI _ADR on Intel).\n");
  } else if (AnyPanel) {
    Print (L"SCENARIO (a): panel is LIVE at UEFI stage with no bring-up.\n");
    Print (L"=> Green light: the driver is an I2C-HID reader. Note the base+addr\n");
    Print (L"   above and the HID descriptor's wInputRegister for the reader.\n");
  } else {
    Print (L"SCENARIO (b): controller live but panel did not ACK.\n");
    Print (L"=> The driver must add a reset-GPIO / _PS0 power-on before talking\n");
    Print (L"   to the panel. Collect the Goodix _CRS/_PS0 GPIO details.\n");
  }

  return EFI_SUCCESS;
}
