/** @file
  Intel Serial IO (LPSS) I2C controller discovery and power bring-up.

  The Intel counterpart of the FCH AOAC layer: on Intel PCHs the DesignWare
  I2C instances are PCI functions (e.g. Raptor Lake-P 00:15.x, DID 0x51E8+)
  rather than ACPI devices at fixed FCH MMIO addresses. Their register block
  is BAR0 -- assigned by firmware, commonly above 4 GiB -- and carries an
  extra LPSS private register set at BAR0 + 0x200 that holds the function in
  reset until released (what the intel-lpss driver does on Linux).

  Discovery goes through EFI_PCI_IO_PROTOCOL and is DMI-gated by the caller
  exactly like the AMD path: a profile names the expected device/function
  *and* device ID, and anything that does not match both fails closed.

  References:
    - Linux drivers/mfd/intel-lpss.c / intel-lpss-pci.c
    - Intel 600/700 Series PCH EDS (Serial IO I2C)

  Copyright (c) 2026, jlobue10 and contributors. All rights reserved.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef _INTEL_LPSS_H_
#define _INTEL_LPSS_H_

#include <Uefi.h>

//
// LPSS private registers, relative to BAR0. RESETS holds the host controller
// (bits [1:0]) and its integrated DMA (bit 2) in reset while zero; releasing
// all three is the Linux intel-lpss bring-up. REMAP tells the IP its own BAR
// so it can decode private-space accesses.
//
#define LPSS_PRIV_RESETS        0x204
#define LPSS_PRIV_RESETS_FUNC   0x3
#define LPSS_PRIV_RESETS_IDMA   BIT2
#define LPSS_PRIV_REMAP_LO      0x240
#define LPSS_PRIV_REMAP_HI      0x244

//
// Private clock parameters: bit0 = clock enable, bits [15:1] = M and
// [30:16] = N of the fractional divider, bit31 = update/gate. 1:1 passes
// the 133 MHz reference through unchanged, which is what the DesignWare
// SCL counts (both the firmware's and gDwTimingIntelLpss133M) assume.
// The I2C instances observed so far ignore writes here (no divider
// hardware, matching Linux intel-lpss skipping clock registration for
// I2C), so the guarded write below is a no-op on them -- it exists for
// any future Serial IO instance where the divider is real and parked.
//
#define LPSS_PRIV_CLOCK_PARAMS  0x200
#define LPSS_PRIV_CLOCK_1TO1    0x80010003

//
// Serial IO I2C reference clock on Tiger/Alder/Raptor Lake PCHs
// (Linux intel-lpss-pci.c bxt_i2c_info.clk_rate), from which the DesignWare
// SCL counts in DwI2c.h's Intel timing set are derived.
//
#define LPSS_I2C_CLOCK_HZ       133000000

/**
  Find the Serial IO I2C controller at PCI 00:Device.Function, verify its
  device ID, power it to D0, enable memory decode, and release the LPSS
  resets.

  @param[in]  PciDid        Expected PCI device ID (vendor must be 0x8086).
  @param[in]  PciDevice     PCI device number on bus 0 (e.g. 0x15).
  @param[in]  PciFunction   PCI function number.
  @param[out] Base          BAR0 -- the DesignWare register block.
  @param[out] FreshPowerOn  TRUE if this call released the function reset,
                            i.e. the firmware never ran this controller and
                            its bus timing must be programmed from scratch.

  @retval EFI_SUCCESS    Controller is powered, decoded and out of reset.
  @retval EFI_NOT_FOUND  No PciIo handle matches location + IDs (fail closed).
  @retval EFI_NO_MAPPING BAR0 is unassigned.
**/
EFI_STATUS
IntelLpssPrepareI2c (
  IN  UINT16   PciDid,
  IN  UINT8    PciDevice,
  IN  UINT8    PciFunction,
  OUT UINTN    *Base,
  OUT BOOLEAN  *FreshPowerOn
  );

#endif // _INTEL_LPSS_H_
