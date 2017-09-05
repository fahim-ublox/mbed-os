; 
; Copyright (C) u-blox
; All rights reserved.
; This source file is the sole property of u-blox. Reproduction or utilization
; of this source in whole or part is forbidden without the written consent of
; u-blox.
; 
; FILE: ospi.hs

OSPI_HAL_VERSION EQU 1
OSPI_CONFIG_REG EQU 0x000
OSPI_DEV_INSTR_RD_CONFIG_REG EQU 0x004
OSPI_DEV_INSTR_WR_CONFIG_REG EQU 0x008
OSPI_DEV_DELAY_REG EQU 0x00C
OSPI_RD_DATA_CAPTURE_REG EQU 0x010
OSPI_DEV_SIZE_CONFIG_REG EQU 0x014
OSPI_SRAM_PARTITION_CFG_REG EQU 0x018
OSPI_IND_AHB_ADDR_TRIGGER_REG EQU 0x01C
OSPI_DMA_PERIPH_CONFIG_REG EQU 0x020
OSPI_REMAP_ADDR_REG EQU 0x024
OSPI_MODE_BIT_CONFIG_REG EQU 0x028
OSPI_SRAM_FILL_REG EQU 0x02C
OSPI_TX_THRESH_REG EQU 0x030
OSPI_RX_THRESH_REG EQU 0x034
OSPI_WRITE_COMPLETION_CTRL_REG EQU 0x038
OSPI_NO_OF_POLLS_BEF_EXP_REG EQU 0x03C
OSPI_IRQ_STATUS_REG EQU 0x040
OSPI_IRQ_MASK_REG EQU 0x044
OSPI_LOWER_WR_PROT_REG EQU 0x050
OSPI_UPPER_WR_PROT_REG EQU 0x054
OSPI_WR_PROT_CTRL_REG EQU 0x058
OSPI_INDIRECT_READ_XFER_CTRL_REG EQU 0x060
OSPI_INDIRECT_READ_XFER_WATERMARK_REG EQU 0x064
OSPI_INDIRECT_READ_XFER_START_REG EQU 0x068
OSPI_INDIRECT_READ_XFER_NUM_BYTES_REG EQU 0x06C
OSPI_INDIRECT_WRITE_XFER_CTRL_REG EQU 0x070
OSPI_INDIRECT_WRITE_XFER_WATERMARK_REG EQU 0x074
OSPI_INDIRECT_WRITE_XFER_START_REG EQU 0x078
OSPI_INDIRECT_WRITE_XFER_NUM_BYTES_REG EQU 0x07C
OSPI_INDIRECT_TRIGGER_ADDR_RANGE_REG EQU 0x080
OSPI_FLASH_COMMAND_CTRL_MEM_REG EQU 0x08C
OSPI_FLASH_CMD_CTRL_REG EQU 0x090
OSPI_FLASH_CMD_ADDR_REG EQU 0x094
OSPI_FLASH_RD_DATA_LOWER_REG EQU 0x0A0
OSPI_FLASH_RD_DATA_UPPER_REG EQU 0x0A4
OSPI_FLASH_WR_DATA_LOWER_REG EQU 0x0A8
OSPI_FLASH_WR_DATA_UPPER_REG EQU 0x0AC
OSPI_POLLING_FLASH_STATUS_REG EQU 0x0B0
OSPI_PHY_CONFIGURATION_REG EQU 0x0B4
OSPI_PHY_MASTER_CONTROL_REG EQU 0x0B8
OSPI_DLL_OBSERVABLE_LOWER_REG EQU 0x0BC
OSPI_DLL_OBSERVABLE_UPPER_REG EQU 0x0C0
OSPI_OPCODE_EXT_LOWER_REG EQU 0x0E0
OSPI_OPCODE_EXT_UPPER_REG EQU 0x0E4
OSPI_MODULE_ID_REG EQU 0x0FC
; EOF: ospi.hs
   END