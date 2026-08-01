/** @file
  Synopsys DesignWare I2C master controller register definitions.

  These offsets and bit definitions are part of the DesignWare APB I2C IP and
  are identical across the AMD FCH instances (ACPI _HID "AMDI0010") used on the
  Ryzen "Phoenix" APU in the ASUS ROG Xbox Ally X. They are hardware-independent
  (spec/IP constants), so they are correct regardless of which controller
  instance the touchscreen is wired to.

  References:
    - Linux drivers/i2c/busses/i2c-designware-core.h
    - u-boot drivers/i2c/designware_i2c.h
    - coreboot src/drivers/i2c/designware/dw_i2c.c

  Copyright (c) 2026, jlobue10 and contributors. All rights reserved.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef _DW_I2C_H_
#define _DW_I2C_H_

//
// Candidate MMIO bases for the DesignWare I2C controllers on the AMD FCH.
// One 4 KiB window each. The touchscreen's controller instance is confirmed
// per-device from ACPI (AMDI0010 _CRS Memory resource); these are the fallback
// fixed addresses and the set the probe app sweeps.
//
#define DW_I2C_FCH_BASE_0   0xFEDC2000
#define DW_I2C_FCH_BASE_1   0xFEDC3000
#define DW_I2C_FCH_BASE_2   0xFEDC4000
#define DW_I2C_FCH_BASE_3   0xFEDC5000
#define DW_I2C_FCH_BASE_4   0xFEDC6000

//
// Register offsets (add to the controller MMIO base).
//
#define DW_IC_CON              0x00
#define DW_IC_TAR              0x04
#define DW_IC_SAR              0x08
#define DW_IC_DATA_CMD         0x10
#define DW_IC_SS_SCL_HCNT      0x14
#define DW_IC_SS_SCL_LCNT      0x18
#define DW_IC_FS_SCL_HCNT      0x1C
#define DW_IC_FS_SCL_LCNT      0x20
#define DW_IC_INTR_STAT        0x2C
#define DW_IC_INTR_MASK        0x30
#define DW_IC_RAW_INTR_STAT    0x34
#define DW_IC_RX_TL            0x38
#define DW_IC_TX_TL            0x3C
#define DW_IC_CLR_INTR         0x40
#define DW_IC_CLR_RX_UNDER     0x44
#define DW_IC_CLR_TX_ABRT      0x54
#define DW_IC_CLR_STOP_DET     0x60
#define DW_IC_ENABLE           0x6C
#define DW_IC_STATUS           0x70
#define DW_IC_TXFLR            0x74
#define DW_IC_RXFLR            0x78
#define DW_IC_SDA_HOLD         0x7C
#define DW_IC_TX_ABRT_SOURCE   0x80
#define DW_IC_ENABLE_STATUS    0x9C
#define DW_IC_COMP_PARAM_1     0xF4
#define DW_IC_COMP_VERSION     0xF8
#define DW_IC_COMP_TYPE        0xFC

//
// IC_COMP_TYPE magic identifying a DesignWare I2C block.
//
#define DW_IC_COMP_TYPE_VALUE  0x44570140

//
// IC_CON bits.
//
#define DW_IC_CON_MASTER          BIT0
#define DW_IC_CON_SPEED_STD       (1u << 1)   // 100 kHz
#define DW_IC_CON_SPEED_FAST      (2u << 1)   // 400 kHz
#define DW_IC_CON_RESTART_EN      BIT5
#define DW_IC_CON_SLAVE_DISABLE   BIT6

//
// IC_DATA_CMD bits.
//
#define DW_IC_DATA_CMD_READ       BIT8
#define DW_IC_DATA_CMD_STOP       BIT9
#define DW_IC_DATA_CMD_RESTART    BIT10

//
// IC_STATUS bits.
//
#define DW_IC_STATUS_ACTIVITY     BIT0
#define DW_IC_STATUS_TFNF         BIT1   // TX FIFO not full
#define DW_IC_STATUS_TFE          BIT2   // TX FIFO empty
#define DW_IC_STATUS_RFNE         BIT3   // RX FIFO not empty
#define DW_IC_STATUS_RFF          BIT4   // RX FIFO full
#define DW_IC_STATUS_MST_ACTIVITY BIT5

//
// IC_RAW_INTR_STAT bits (subset used for polled transfers).
//
#define DW_IC_INTR_RX_FULL        BIT2
#define DW_IC_INTR_TX_EMPTY       BIT4
#define DW_IC_INTR_TX_ABRT        BIT6
#define DW_IC_INTR_STOP_DET       BIT9

//
// IC_ENABLE / IC_ENABLE_STATUS bits.
//
#define DW_IC_ENABLE_ENABLE       BIT0
#define DW_IC_ENABLE_STATUS_EN    BIT0

//
// IC_TX_ABRT_SOURCE: bit0 set => 7-bit address phase got no ACK (no device at
// that slave address). This is the signal the probe uses to distinguish
// "panel present/ACKed" from "nothing there".
//
#define DW_IC_ABRT_7B_ADDR_NOACK  BIT0

//
// Novatek NVTK0603 touchscreen 7-bit I2C slave address, confirmed from the
// Ally X (RC73XA) DSDT: \_SB.I2CA.TPL0 _CRS I2cSerialBusV2 with SADR[0]=0x01
// patched into the SlaveAddress field. (0x01 is in the I2C reserved range but
// is what the hardware really uses -- Linux i2c-hid drives it there.)
//
#define NVTK_I2C_ADDR       0x01

//
// FocalTech FTS3528 touchscreen 7-bit slave address, confirmed from the
// Steam Deck OLED (Galileo) DSDT: \_SB.I2CB.TPNL _CRS I2cSerialBusV2 slave
// 0x38 -- the classic FocalTech address.
//
#define FTS_I2C_ADDR        0x38

//
// Goodix candidate 7-bit slave addresses, kept for the fallback detection
// sweep (other Ally-family panels are Goodix parts at one of these).
//
#define GOODIX_I2C_ADDR_A   0x14
#define GOODIX_I2C_ADDR_B   0x5D

//
// ELAN touchscreen 7-bit slave address, confirmed from the ASUS Zenbook
// UX8402VV DSDT: \_SB.PC00.I2C0.TPL0 (ELAN9008) _CRS I2cSerialBusV2 with
// SADR[0]=0x10 patched into the SlaveAddress field -- the classic ELAN
// address.
//
#define ELAN_I2C_ADDR       0x10

//
// SCL high/low counts and SDA hold for one reference-clock rate. Programmed
// only when this driver powered the controller on itself (the firmware never
// ran the bus, so there is no programming to inherit); which set applies is
// a property of the platform's I2C tile, so the matched profile picks it.
//
typedef struct {
  UINT16  SsHcnt;      // 100 kHz high count
  UINT16  SsLcnt;      // 100 kHz low count
  UINT16  FsHcnt;      // 400 kHz high count
  UINT16  FsLcnt;      // 400 kHz low count
  UINT16  SdaHold;     // ~300 ns data hold
} DW_I2C_TIMING;

extern CONST DW_I2C_TIMING  gDwTimingAmdFch150M;    // AMD FCH, 150 MHz ref
extern CONST DW_I2C_TIMING  gDwTimingIntelLpss133M; // Intel Serial IO, 133 MHz

//
// Layer-1 API (DwI2c.c): polled master-mode init and combined transfer.
// Bases are UINTN: the AMD FCH instances sit at fixed 32-bit addresses, but
// Intel Serial IO BAR0 is commonly above 4 GiB.
//

/** TRUE if IC_COMP_TYPE at this base reads back the DesignWare magic. **/
BOOLEAN
DwI2cControllerPresent (
  IN UINTN  Base
  );

/**
  Disable the controller and wait (bounded) for it to report disabled.
  Disabling also flushes both FIFOs, which is what makes it usable as the
  recovery step after a failed transfer.

  @retval EFI_SUCCESS  the controller reports disabled.
  @retval EFI_TIMEOUT  still enabled; its CON/TAR/timing registers cannot be
                       reprogrammed and any queued commands are still live.
**/
EFI_STATUS
DwI2cDisable (
  IN UINTN  Base
  );

/**
  Bring the controller into polled master mode targeting SlaveAddr and enable.

  When the firmware ran this bus, the SCL high/low counts and SDA hold it
  programmed are known good for this board; they are kept and only master
  mode, target address and FIFO thresholds are reprogrammed. When ForceTiming
  is non-NULL -- the tile was just powered on (AOAC on AMD, LPSS reset
  release on Intel) and holds only IP reset defaults -- the given 400 kHz
  counts for that platform's reference clock are programmed instead.
**/
EFI_STATUS
DwI2cInit (
  IN UINTN                Base,
  IN UINT8                SlaveAddr,
  IN CONST DW_I2C_TIMING  *ForceTiming  OPTIONAL
  );

/**
  Write WLen bytes, then (if RLen > 0) repeated-START read RLen bytes, ending
  with STOP. WLen == 0 issues a pure read. Returns EFI_NO_RESPONSE on address
  NAK, EFI_DEVICE_ERROR on other aborts, EFI_TIMEOUT on stuck bus.
**/
EFI_STATUS
DwI2cXfer (
  IN  UINTN        Base,
  IN  CONST UINT8  *WBuf,   OPTIONAL
  IN  UINTN        WLen,
  OUT UINT8        *RBuf,   OPTIONAL
  IN  UINTN        RLen
  );

#endif // _DW_I2C_H_
