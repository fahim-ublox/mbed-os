; 
; Copyright (C) u-blox
; All rights reserved.
; This source file is the sole property of u-blox. Reproduction or utilization
; of this source in whole or part is forbidden without the written consent of
; u-blox.
; 
; FILE: rfdma.hs

RFIF_NR_RX_CHANNELS EQU 1
RFIF_NR_TX_CHANNELS EQU 1
RFDMA_TXP_ADDR EQU 0x000
RFDMA_TXP_SIZE EQU 0x004
RFDMA_TXP_CONF EQU 0x008
RFDMA_TXP_FILL EQU 0x00C
RFDMA_TXP_FILL_FIELDS EQU 1
RFDMA_RXP_ADDR EQU 0x000
RFDMA_RXP_SIZE EQU 0x004
RFDMA_RXP_CONF EQU 0x008
RFDMA_RXP_FILL EQU 0x00C
RFDMA_RXP_FILL_FIELDS EQU 1
RFDMA_TXR_ADDR EQU 0x000
RFDMA_TXR_SIZE EQU 0x004
RFDMA_TXR_CONF EQU 0x008
RFDMA_TXR_FILL EQU 0x00C
RFDMA_TXR_FILL_FIELDS EQU 1
RFDMA_RXR_ADDR EQU 0x000
RFDMA_RXR_SIZE EQU 0x004
RFDMA_RXR_CONF EQU 0x008
RFDMA_RXR_FILL EQU 0x00C
RFDMA_RXR_FILL_FIELDS EQU 1
RFDMA_TXC_COMD EQU 0x000
RFDMA_TXC_STAT EQU 0x004
RFDMA_TXC_FILL0 EQU 0x008
RFDMA_TXC_FILL0_FIELDS EQU 1
RFDMA_TXC_FILL1 EQU 0x00C
RFDMA_TXC_FILL1_FIELDS EQU 1
RFDMA_RXC_COMD EQU 0x000
RFDMA_RXC_STAT EQU 0x004
RFDMA_RXC_CONF EQU 0x008
RFDMA_RXC_FILL0 EQU 0x00C
RFDMA_RXC_FILL0_FIELDS EQU 1
RFDMA_FLTCLEAR EQU 0x300
RFDMA_FLTSTATUS EQU 0x304
RFDMA_FLTRAWSTATUS EQU 0x308
RFDMA_FLTMASKCLR EQU 0x30C
RFDMA_FLTMASKSET EQU 0x310
RFDMA_FLTMASKSTATUS EQU 0x314
RFDMA_TXDIVERSITYCFG EQU 0x400
RFDMA_PERIPHERALID4 EQU 0xFD0
RFDMA_PERIPHERALID5 EQU 0xFD4
RFDMA_PERIPHERALID6 EQU 0xFD8
RFDMA_PERIPHERALID7 EQU 0xFDC
RFDMA_PERIPHERALID0 EQU 0xFE0
RFDMA_PERIPHERALID1 EQU 0xFE4
RFDMA_PERIPHERALID2 EQU 0xFE8
RFDMA_PERIPHERALID3 EQU 0xFEC
RFDMA_COMPONENTID0 EQU 0xFF0
RFDMA_COMPONENTID1 EQU 0xFF4
RFDMA_COMPONENTID2 EQU 0xFF8
RFDMA_COMPONENTID3 EQU 0xFFC
; EOF: rfdma.hs
   END