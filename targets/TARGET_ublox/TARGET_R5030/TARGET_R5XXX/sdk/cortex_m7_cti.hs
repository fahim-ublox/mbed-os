; 
; Copyright (C) u-blox
; All rights reserved.
; This source file is the sole property of u-blox. Reproduction or utilization
; of this source in whole or part is forbidden without the written consent of
; u-blox.
; 
; FILE: cortex_m7_cti.hs

CORTEX_M7_CTI_CTICONTROL EQU 0x000
CORTEX_M7_CTI_CTIINTACK EQU 0x010
CORTEX_M7_CTI_CTIAPPSET EQU 0x014
CORTEX_M7_CTI_CTIAPPCLEAR EQU 0x018
CORTEX_M7_CTI_CTIAPPPULSE EQU 0x01C
CORTEX_M7_CTI_CTIINEN EQU 0x020
CORTEX_M7_CTI_CTIINEN_FIELDS EQU 8
CORTEX_M7_CTI_CTIOUTEN EQU 0x0A0
CORTEX_M7_CTI_CTIOUTEN_FIELDS EQU 8
CORTEX_M7_CTI_CTITRIGINSTATUS EQU 0x130
CORTEX_M7_CTI_CTITRIGOUTSTATUS EQU 0x134
CORTEX_M7_CTI_CTICHINSTATUS EQU 0x138
CORTEX_M7_CTI_CTICHOUTSTATUS EQU 0x13C
CORTEX_M7_CTI_CTIGATE EQU 0x140
CORTEX_M7_CTI_ASICCTL EQU 0x144
CORTEX_M7_CTI_ITCTRL EQU 0xF00
CORTEX_M7_CTI_CLAIMSET EQU 0xFA0
CORTEX_M7_CTI_CLAIMCLR EQU 0xFA4
CORTEX_M7_CTI_LOCKACCESS EQU 0xFB0
CORTEX_M7_CTI_LOCKSTATUS EQU 0xFB4
CORTEX_M7_CTI_AUTHSTATUS EQU 0xFB8
CORTEX_M7_CTI_DEVID EQU 0xFC8
CORTEX_M7_CTI_DEVTYPE EQU 0xFCC
CORTEX_M7_CTI_PERIPHERALID4 EQU 0xFD0
CORTEX_M7_CTI_PERIPHERALID5 EQU 0xFD4
CORTEX_M7_CTI_PERIPHERALID6 EQU 0xFD8
CORTEX_M7_CTI_PERIPHERALID7 EQU 0xFDC
CORTEX_M7_CTI_PERIPHERALID0 EQU 0xFE0
CORTEX_M7_CTI_PERIPHERALID1 EQU 0xFE4
CORTEX_M7_CTI_PERIPHERALID2 EQU 0xFE8
CORTEX_M7_CTI_PERIPHERALID3 EQU 0xFEC
CORTEX_M7_CTI_COMPONENTID0 EQU 0xFF0
CORTEX_M7_CTI_COMPONENTID1 EQU 0xFF4
CORTEX_M7_CTI_COMPONENTID2 EQU 0xFF8
CORTEX_M7_CTI_COMPONENTID3 EQU 0xFFC
; EOF: cortex_m7_cti.hs
   END
