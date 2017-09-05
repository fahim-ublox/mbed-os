; 
; Copyright (C) u-blox
; All rights reserved.
; This source file is the sole property of u-blox. Reproduction or utilization
; of this source in whole or part is forbidden without the written consent of
; u-blox.
; 
; FILE: security.hs

SECURITY_SEC_ID EQU 0x000
SECURITY_SEC_STATUS EQU 0x004
SECURITY_SEC_CFG EQU 0x008
SECURITY_SEC_CHALLENGE EQU 0x200
SECURITY_SEC_CHALLENGE_FIELDS EQU 128
SECURITY_SEC_RESPONSE EQU 0x400
SECURITY_SEC_RESPONSE_FIELDS EQU 128
SECURITY_PERIPHERALID4 EQU 0xFD0
SECURITY_PERIPHERALID5 EQU 0xFD4
SECURITY_PERIPHERALID6 EQU 0xFD8
SECURITY_PERIPHERALID7 EQU 0xFDC
SECURITY_PERIPHERALID0 EQU 0xFE0
SECURITY_PERIPHERALID1 EQU 0xFE4
SECURITY_PERIPHERALID2 EQU 0xFE8
SECURITY_PERIPHERALID3 EQU 0xFEC
SECURITY_COMPONENTID0 EQU 0xFF0
SECURITY_COMPONENTID1 EQU 0xFF4
SECURITY_COMPONENTID2 EQU 0xFF8
SECURITY_COMPONENTID3 EQU 0xFFC
; EOF: security.hs
   END