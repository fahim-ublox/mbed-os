; 
; Copyright (C) u-blox
; All rights reserved.
; This source file is the sole property of u-blox. Reproduction or utilization
; of this source in whole or part is forbidden without the written consent of
; u-blox.
; 
; FILE: psram.hs

PSRAM_HAL_VERSION EQU 1
PSRAM_AXI_ID_PRIO_SEL_MASK0 EQU 0x000
PSRAM_AXI_ID_PRIO_SEL_MASK1 EQU 0x004
PSRAM_AXI_ID_PRIO_SEL_MASK2 EQU 0x008
PSRAM_AXI_ID_PRIO_SEL_MASK3 EQU 0x00C
PSRAM_AXI_ID_PRIO_SEL_LEVEL0 EQU 0x010
PSRAM_AXI_ID_PRIO_SEL_LEVEL1 EQU 0x014
PSRAM_AXI_ID_PRIO_SEL_LEVEL2 EQU 0x018
PSRAM_AXI_ID_PRIO_SEL_LEVEL3 EQU 0x01C
PSRAM_AXI_CTRL EQU 0x020
PSRAM_SYNC_RESET_REQ EQU 0x024
PSRAM_AXI_BUSY EQU 0x028
PSRAM_PSRAM_ADDR_WIDTH EQU 0x02C
PSRAM_PSRAM_ROW_LENGTH_SPLIT_CFG EQU 0x030
PSRAM_PSRAM_AR_AW_ARB_PRIO_CNT_SEL EQU 0x034
PSRAM_PSRAM_IRQ_AND_FAULT_STATUS_RAW EQU 0x100
PSRAM_PSRAM_IRQ_AND_FAULT_STATUS EQU 0x104
PSRAM_PSRAM_IRQ_AND_FAULT_STATUS_MASK_SET EQU 0x108
PSRAM_PSRAM_IRQ_AND_FAULT_STATUS_MASK_CLR EQU 0x10C
PSRAM_PSRAM_IRQ_AND_FAULT_STATUS_MASK_STATUS EQU 0x110
PSRAM_PSRAM_IRQ_AND_FAULT_STATUS_CLR EQU 0x114
PSRAM_PSRAM_FE_DEBUG_FSM_STATES EQU 0x200
PSRAM_PSRAM_FE_DEBUG_COMMAND_ADDR EQU 0x204
PSRAM_PSRAM_FE_DEBUG_COMMAND EQU 0x208
PSRAM_PSRAM_FE_DEBUG_FIFO_STATUS EQU 0x20C
PSRAM_PSRAM_FE_DEBUG_ARBITER_CNT EQU 0x210
PSRAM_PSRAM_CONTROLLER_PHY_DEBUG EQU 0x214
PSRAM_PSRAM_CONTROLLER_PHY_DEBUG_CNT EQU 0x218
PSRAM_PSRAM_HWOBS_CONTROL EQU 0x21C
PSRAM_BCR EQU 0x300
PSRAM_RCR EQU 0x304
PSRAM_DIDR EQU 0x308
PSRAM_BCR_WRITE_REQ EQU 0x30C
PSRAM_BCR_READ_REQ EQU 0x310
PSRAM_RCR_WRITE_REQ EQU 0x314
PSRAM_RCR_READ_REQ EQU 0x318
PSRAM_DIDR_READ_REQ EQU 0x31C
PSRAM_PHY_TX_DLL_CONFIGURATION EQU 0x400
PSRAM_PHY_RX_DLL_CONFIGURATION EQU 0x404
PSRAM_PHY_DLL_MASTER_CONTROL EQU 0x408
PSRAM_PHY_TX_DLL_OBS_REG_SLAVE EQU 0x410
PSRAM_PHY_DLL_OBS_REG_MASTER EQU 0x414
PSRAM_PHY_RX_DLL_OBS_REG_SLAVE EQU 0x418
PSRAM_PHY_READ_DATA_CAPTURE EQU 0x420
PSRAM_ASYNC_MODE_LATENCY EQU 0x424
PSRAM_DPD_EXIT_COUNTER EQU 0x428
PSRAM_DPD_EXIT EQU 0x42C
PSRAM_PSRAM_CONTROLLER_PHY_STATUS EQU 0x430
PSRAM_PSRAM_CONTROLLER_PHY_IGNORE EQU 0x434
PSRAM_PSRAM_CONTROLLER_PHY_CFG EQU 0x438
PSRAM_SYNC_MODE_LATENCY EQU 0x43C
PSRAM_DFT_LOOPBACK_CONTROL_LB EQU 0x500
PSRAM_DFT_LOOPBACK_CONTROL_UB EQU 0x504
PSRAM_DFT_LOOPBACK_OBS_REG EQU 0x508
PSRAM_DFT_MEM_CTRL EQU 0x50C
PSRAM_DFT_MEM_COMMAND EQU 0x510
PSRAM_DFT_MEM_WRITE_DATA_POS EQU 0x514
PSRAM_DFT_MEM_WRITE_DATA_NEG EQU 0x518
PSRAM_DFT_MEM_READ_DATA_POS EQU 0x51C
PSRAM_DFT_MEM_READ_DATA_NEG EQU 0x520
PSRAM_DFT_MEM_READ_DATA_CFG_REGS EQU 0x524
; EOF: psram.hs
   END
