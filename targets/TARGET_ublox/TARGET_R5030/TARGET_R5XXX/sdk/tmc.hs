; 
; Copyright (C) u-blox
; All rights reserved.
; This source file is the sole property of u-blox. Reproduction or utilization
; of this source in whole or part is forbidden without the written consent of
; u-blox.
; 
; FILE: tmc.hs

TMC_RSZ EQU 0x004
TMC_STS EQU 0x00C
TMC_RRD EQU 0x010
TMC_RRP EQU 0x014
TMC_RWP EQU 0x018
TMC_TRG EQU 0x01C
TMC_CTL EQU 0x020
TMC_RWD EQU 0x024
TMC_MODE EQU 0x028
TMC_LBUFLEVEL EQU 0x02C
TMC_CBUFLEVEL EQU 0x030
TMC_BUFWM EQU 0x034
TMC_RRPHI EQU 0x038
TMC_RWPHI EQU 0x03C
TMC_AXICTL EQU 0x110
TMC_DBALO EQU 0x118
TMC_DBAHI EQU 0x11C
TMC_FFSR EQU 0x300
TMC_FFCR EQU 0x304
TMC_PSCR EQU 0x308
TMC_ITATBMDATA0 EQU 0xED0
TMC_ITATBMCTR2 EQU 0xED4
TMC_ITATBMCTR1 EQU 0xED8
TMC_ITATBMCTR0 EQU 0xEDC
TMC_ITMISCOP0 EQU 0xEE0
TMC_ITTRFLIN EQU 0xEE8
TMC_ITATBDATA0 EQU 0xEEC
TMC_ITATBCTR2 EQU 0xEF0
TMC_ITATBCTR1 EQU 0xEF4
TMC_ITATBCTR0 EQU 0xEF8
TMC_ITCTRL EQU 0xF00
TMC_CLAIMSET EQU 0xFA0
TMC_CLAIMCLR EQU 0xFA4
TMC_LAR EQU 0xFB0
TMC_LSR EQU 0xFB4
TMC_AUTHSTATUS EQU 0xFB8
TMC_DEVID EQU 0xFC8
TMC_DEVTYPE EQU 0xFCC
TMC_PERIPHERALID4 EQU 0xFD0
TMC_PERIPHERALID5 EQU 0xFD4
TMC_PERIPHERALID6 EQU 0xFD8
TMC_PERIPHERALID7 EQU 0xFDC
TMC_PERIPHERALID0 EQU 0xFE0
TMC_PERIPHERALID1 EQU 0xFE4
TMC_PERIPHERALID2 EQU 0xFE8
TMC_PERIPHERALID3 EQU 0xFEC
TMC_COMPONENTID0 EQU 0xFF0
TMC_COMPONENTID1 EQU 0xFF4
TMC_COMPONENTID2 EQU 0xFF8
TMC_COMPONENTID3 EQU 0xFFC
; EOF: tmc.hs
   END