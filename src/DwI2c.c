/** @file
  Polled master-mode driver for the Synopsys DesignWare APB I2C controller as
  instantiated on the AMD FCH ("AMDI0010"). Layer 1 of AllyTouchI2cDxe.

  Transfers interleave command-queueing with RX draining so reads larger than
  the RX FIFO cannot deadlock (the master pauses SCL when the RX FIFO fills;
  if we blocked on the TX FIFO at the same time nothing would ever drain).

  Copyright (c) 2026, jlobue10 and contributors. All rights reserved.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <Uefi.h>
#include <Library/IoLib.h>

#include "DwI2c.h"

//
// Bounded spins for polled status. At 100 kHz a byte time is ~90 us and one
// MMIO read is well under 1 us, so this allows on the order of hundreds of
// milliseconds of stall before giving up.
//
#define DW_POLL_LIMIT       200000

//
// Whole-transfer stall budget. DW_POLL_LIMIT is reset by every byte of
// progress, so on its own it is a per-byte deadline: a panel that clock
// stretches just under the limit on each of 1024 bytes could hold the poll
// timer (TPL_CALLBACK) for minutes and freeze rEFInd's UI. This caps the
// total time any single transfer can spend stalled.
//
#define DW_XFER_SPIN_BUDGET (4 * DW_POLL_LIMIT)

//
// Never allow more than this many read commands in flight beyond what we have
// drained, so the RX FIFO (16+ entries on the FCH instances) cannot overflow.
//
#define DW_RX_INFLIGHT_MAX  8

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

BOOLEAN
DwI2cControllerPresent (
  IN UINTN  Base
  )
{
  return (BOOLEAN)(RegRd (Base, DW_IC_COMP_TYPE) == DW_IC_COMP_TYPE_VALUE);
}

EFI_STATUS
DwI2cDisable (
  IN UINTN  Base
  )
{
  UINT32  Spin;

  RegWr (Base, DW_IC_ENABLE, 0);
  for (Spin = 0; Spin < DW_POLL_LIMIT; Spin++) {
    if ((RegRd (Base, DW_IC_ENABLE_STATUS) & DW_IC_ENABLE_STATUS_EN) == 0) {
      return EFI_SUCCESS;
    }
  }
  //
  // Callers must not treat a failed disable as cosmetic: IC_CON, IC_TAR and
  // the SCL/SDA timing registers are ignored by the hardware while the
  // controller is enabled, so continuing would silently keep the previous
  // slave address and timing.
  //
  return EFI_TIMEOUT;
}

//
// SCL counts for 100/400 kHz (the DSDTs advertise 400 kHz for every known
// touch bus) and ~300 ns SDA hold, per reference clock. Programmed when the
// tile had to be powered on here -- the firmware never ran it, so there is
// no programming to inherit.
//
// AMD FCH ("Phoenix"/"Sonoma Valley" APUs): 150 MHz.
//
CONST DW_I2C_TIMING  gDwTimingAmdFch150M = {
  600,   // SsHcnt  ~4.0 us high
  705,   // SsLcnt  ~4.7 us low
  136,   // FsHcnt  ~0.9 us high
  225,   // FsLcnt  ~1.5 us low
  45     // SdaHold ~300 ns
};

//
// Intel Serial IO (Tiger/Alder/Raptor Lake PCH): 133 MHz
// (LPSS_I2C_CLOCK_HZ), counts per the Linux i2c-designware formula.
//
CONST DW_I2C_TIMING  gDwTimingIntelLpss133M = {
  569,   // SsHcnt  ~4.3 us high
  664,   // SsLcnt  ~5.0 us low
  117,   // FsHcnt  ~0.9 us high
  212,   // FsLcnt  ~1.6 us low
  40     // SdaHold ~300 ns
};

EFI_STATUS
DwI2cInit (
  IN UINTN                Base,
  IN UINT8                SlaveAddr,
  IN CONST DW_I2C_TIMING  *ForceTiming  OPTIONAL
  )
{
  UINT32                Spin;
  UINT32                Speed;
  CONST DW_I2C_TIMING   *Timing;

  //
  // The fallback for the (unexpected) case of inherited timing with zeroed
  // counts; a non-NULL ForceTiming always knows its own platform.
  //
  Timing = (ForceTiming != NULL) ? ForceTiming : &gDwTimingAmdFch150M;

  //
  // Every register written below is ignored while the controller is enabled,
  // and the enable-confirmation loop at the end would pass immediately on the
  // still-set EN bit. Failing here instead means a stuck controller can never
  // be mistaken for one that is correctly retargeted at a new slave address --
  // which in the fallback sweep would attribute a panel to the wrong address.
  //
  if (EFI_ERROR (DwI2cDisable (Base))) {
    return EFI_DEVICE_ERROR;
  }

  //
  // Keep the firmware-selected speed if it is one we can drive (standard or
  // fast); otherwise -- including the fresh-power case -- run the bus at the
  // 400 kHz the DSDT advertises for it.
  //
  Speed = RegRd (Base, DW_IC_CON) & (3u << 1);
  if ((ForceTiming != NULL) || (Speed != DW_IC_CON_SPEED_STD)) {
    Speed = DW_IC_CON_SPEED_FAST;
  }
  RegWr (Base, DW_IC_CON,
         DW_IC_CON_MASTER | Speed |
         DW_IC_CON_RESTART_EN | DW_IC_CON_SLAVE_DISABLE);

  if (Speed == DW_IC_CON_SPEED_STD) {
    if ((RegRd (Base, DW_IC_SS_SCL_HCNT) == 0) ||
        (RegRd (Base, DW_IC_SS_SCL_LCNT) == 0)) {
      RegWr (Base, DW_IC_SS_SCL_HCNT, Timing->SsHcnt);
      RegWr (Base, DW_IC_SS_SCL_LCNT, Timing->SsLcnt);
    }
  } else {
    if ((ForceTiming != NULL) ||
        (RegRd (Base, DW_IC_FS_SCL_HCNT) == 0) ||
        (RegRd (Base, DW_IC_FS_SCL_LCNT) == 0)) {
      RegWr (Base, DW_IC_FS_SCL_HCNT, Timing->FsHcnt);
      RegWr (Base, DW_IC_FS_SCL_LCNT, Timing->FsLcnt);
    }
  }
  if (ForceTiming != NULL) {
    RegWr (Base, DW_IC_SDA_HOLD, Timing->SdaHold);
  }

  RegWr (Base, DW_IC_INTR_MASK, 0);   // fully polled; no interrupts
  RegWr (Base, DW_IC_TX_TL, 0);
  RegWr (Base, DW_IC_RX_TL, 0);

  RegWr (Base, DW_IC_TAR, SlaveAddr & 0x3FF);

  RegWr (Base, DW_IC_ENABLE, DW_IC_ENABLE_ENABLE);
  for (Spin = 0; Spin < DW_POLL_LIMIT; Spin++) {
    if (RegRd (Base, DW_IC_ENABLE_STATUS) & DW_IC_ENABLE_STATUS_EN) {
      return EFI_SUCCESS;
    }
  }
  return EFI_TIMEOUT;
}

/**
  Return the controller to a known-idle state after a failed transfer.

  Disabling flushes both FIFOs (DesignWare databook), which is the point: an
  error exit can leave undrained RX bytes AND still-queued READ commands that
  keep producing more. Those would be consumed as the leading bytes of the
  next transfer, shifting an I2C-HID length prefix and silently corrupting
  every report after it. CON/TAR/timing survive the disable, so the caller's
  target and bus speed are preserved.
**/
STATIC
VOID
DwI2cRecover (
  IN UINTN  Base
  )
{
  UINT32  Spin;

  if (EFI_ERROR (DwI2cDisable (Base))) {
    return;                            // stuck; the next DwI2cInit will fail
  }
  RegWr (Base, DW_IC_ENABLE, DW_IC_ENABLE_ENABLE);
  for (Spin = 0; Spin < DW_POLL_LIMIT; Spin++) {
    if (RegRd (Base, DW_IC_ENABLE_STATUS) & DW_IC_ENABLE_STATUS_EN) {
      break;
    }
  }
  (VOID)RegRd (Base, DW_IC_CLR_TX_ABRT);
  (VOID)RegRd (Base, DW_IC_CLR_STOP_DET);
}

/**
  Classify and clear a latched TX abort.

  @retval EFI_NO_RESPONSE   address NAK -- nothing at the slave address
  @retval EFI_DEVICE_ERROR  any other abort source
**/
STATIC
EFI_STATUS
ClassifyAbort (
  IN UINTN  Base
  )
{
  UINT32  Src;

  Src = RegRd (Base, DW_IC_TX_ABRT_SOURCE);
  (VOID)RegRd (Base, DW_IC_CLR_TX_ABRT);
  return (Src & DW_IC_ABRT_7B_ADDR_NOACK) ? EFI_NO_RESPONSE : EFI_DEVICE_ERROR;
}

EFI_STATUS
DwI2cXfer (
  IN  UINTN        Base,
  IN  CONST UINT8  *WBuf,   OPTIONAL
  IN  UINTN        WLen,
  OUT UINT8        *RBuf,   OPTIONAL
  IN  UINTN        RLen
  )
{
  UINTN       WSent     = 0;
  UINTN       RQueued   = 0;
  UINTN       RGot      = 0;
  UINT32      Spin      = 0;
  UINT32      TotalSpin = 0;
  EFI_STATUS  Status;
  UINT32      Cmd;
  UINT32      IcStatus;

  if (((WLen == 0) && (RLen == 0)) ||
      ((WLen > 0) && (WBuf == NULL)) ||
      ((RLen > 0) && (RBuf == NULL))) {
    return EFI_INVALID_PARAMETER;
  }

  // Clear stale latched conditions from any previous transfer.
  (VOID)RegRd (Base, DW_IC_CLR_TX_ABRT);
  (VOID)RegRd (Base, DW_IC_CLR_STOP_DET);

  while ((WSent < WLen) || (RQueued < RLen) || (RGot < RLen)) {
    BOOLEAN  Progress = FALSE;

    if (RegRd (Base, DW_IC_RAW_INTR_STAT) & DW_IC_INTR_TX_ABRT) {
      Status = ClassifyAbort (Base);
      if (Status == EFI_DEVICE_ERROR) {
        // An address NAK aborts before any data phase and the hardware
        // flushes TX itself, so the cheap common case (polling a device with
        // nothing pending) stays cheap. Anything else may strand data.
        DwI2cRecover (Base);
      }
      return Status;
    }

    IcStatus = RegRd (Base, DW_IC_STATUS);

    if ((WSent < WLen) && (IcStatus & DW_IC_STATUS_TFNF)) {
      Cmd = WBuf[WSent];
      if ((WSent == WLen - 1) && (RLen == 0)) {
        Cmd |= DW_IC_DATA_CMD_STOP;
      }
      RegWr (Base, DW_IC_DATA_CMD, Cmd);
      WSent++;
      Progress = TRUE;
    } else if ((WSent == WLen) && (RQueued < RLen) &&
               (IcStatus & DW_IC_STATUS_TFNF) &&
               ((RQueued - RGot) < DW_RX_INFLIGHT_MAX)) {
      Cmd = DW_IC_DATA_CMD_READ;
      if ((RQueued == 0) && (WLen > 0)) {
        Cmd |= DW_IC_DATA_CMD_RESTART;
      }
      if (RQueued == RLen - 1) {
        Cmd |= DW_IC_DATA_CMD_STOP;
      }
      RegWr (Base, DW_IC_DATA_CMD, Cmd);
      RQueued++;
      Progress = TRUE;
    }

    if ((RGot < RQueued) &&
        (RegRd (Base, DW_IC_STATUS) & DW_IC_STATUS_RFNE)) {
      RBuf[RGot++] = (UINT8)(RegRd (Base, DW_IC_DATA_CMD) & 0xFF);
      Progress = TRUE;
    }

    if (Progress) {
      Spin = 0;
    } else {
      Spin++;
      TotalSpin++;
      if ((Spin > DW_POLL_LIMIT) || (TotalSpin > DW_XFER_SPIN_BUDGET)) {
        DwI2cRecover (Base);
        return EFI_TIMEOUT;
      }
    }
  }

  // Wait for the STOP to complete so the next transfer starts from idle.
  for (Spin = 0; Spin < DW_POLL_LIMIT; Spin++) {
    if (RegRd (Base, DW_IC_RAW_INTR_STAT) & DW_IC_INTR_TX_ABRT) {
      Status = ClassifyAbort (Base);
      if (Status == EFI_DEVICE_ERROR) {
        DwI2cRecover (Base);
      }
      return Status;
    }
    if ((RegRd (Base, DW_IC_STATUS) & DW_IC_STATUS_MST_ACTIVITY) == 0) {
      (VOID)RegRd (Base, DW_IC_CLR_STOP_DET);
      return EFI_SUCCESS;
    }
  }
  DwI2cRecover (Base);
  return EFI_TIMEOUT;
}
