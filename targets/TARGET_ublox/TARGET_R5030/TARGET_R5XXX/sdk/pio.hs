; 
; Copyright (C) u-blox
; All rights reserved.
; This source file is the sole property of u-blox. Reproduction or utilization
; of this source in whole or part is forbidden without the written consent of
; u-blox.
; 
; FILE: pio.hs

PIO_HAL_VERSION EQU 4
PIO_PIO_PER_0 EQU 0x000
PIO_PIO_PDR_0 EQU 0x004
PIO_PIO_PSR_0 EQU 0x008
PIO_PIO_OER_0 EQU 0x010
PIO_PIO_ODR_0 EQU 0x014
PIO_PIO_OSR_0 EQU 0x018
PIO_PIO_POSR_0 EQU 0x01C
PIO_PIO_IFER_0 EQU 0x020
PIO_PIO_IFDR_0 EQU 0x024
PIO_PIO_IFSR_0 EQU 0x028
PIO_PIO_SODR_0 EQU 0x030
PIO_PIO_CODR_0 EQU 0x034
PIO_PIO_ODSR_0 EQU 0x038
PIO_PIO_PDSR_0 EQU 0x03C
PIO_PIO_IER1_0 EQU 0x040
PIO_PIO_IDR1_0 EQU 0x044
PIO_PIO_IMR1_0 EQU 0x048
PIO_PIO_IER2_0 EQU 0x050
PIO_PIO_IDR2_0 EQU 0x054
PIO_PIO_IMR2_0 EQU 0x058
PIO_PIO_ISR_0 EQU 0x05C
PIO_PIO_MDER_0 EQU 0x060
PIO_PIO_MDDR_0 EQU 0x064
PIO_PIO_MDSR_0 EQU 0x068
PIO_PIO_CKIN_0 EQU 0x070
PIO_PIO_GPSR_0 EQU 0x074
PIO_PIO_ASR_0 EQU 0x078
PIO_PIO_BSR_0 EQU 0x07C
PIO_PIO_CSR_0 EQU 0x080
PIO_PIO_DSR_0 EQU 0x084
PIO_PIO_MSELSR_0 EQU 0x088
PIO_PIO_LSELSR_0 EQU 0x08C
PIO_PIO_MDSER_0 EQU 0x090
PIO_PIO_MDSDR_0 EQU 0x094
PIO_PIO_MDSSR_0 EQU 0x098
PIO_PIO_LDSER_0 EQU 0x0A0
PIO_PIO_LDSDR_0 EQU 0x0A4
PIO_PIO_LDSSR_0 EQU 0x0A8
PIO_PIO_SRER_0 EQU 0x0B0
PIO_PIO_SRDR_0 EQU 0x0B4
PIO_PIO_SRSR_0 EQU 0x0B8
PIO_PIO_PHER_0 EQU 0x0C0
PIO_PIO_PHDR_0 EQU 0x0C4
PIO_PIO_PHSR_0 EQU 0x0C8
PIO_PIO_PLER_0 EQU 0x0D0
PIO_PIO_PLDR_0 EQU 0x0D4
PIO_PIO_PLSR_0 EQU 0x0D8
PIO_PIO_STER_0 EQU 0x0E0
PIO_PIO_STDR_0 EQU 0x0E4
PIO_PIO_STSR_0 EQU 0x0E8
PIO_PIO_INER_0 EQU 0x0F0
PIO_PIO_INDR_0 EQU 0x0F4
PIO_PIO_INSR_0 EQU 0x0F8
PIO_PIO_PERCPER_0 EQU 0x100
PIO_PIO_PERCPDR_0 EQU 0x104
PIO_PIO_PERCPSR_0 EQU 0x108
PIO_PIO_HWOBSSR_0 EQU 0x110
PIO_PIO_IF2SR_0 EQU 0x114
PIO_PIO_IF3SR_0 EQU 0x118
PIO_PIO_EVER_0 EQU 0x120
PIO_PIO_EVDR_0 EQU 0x124
PIO_PIO_EVSR_0 EQU 0x128
PIO_PIO_EVTER_0 EQU 0x130
PIO_PIO_EVTDR_0 EQU 0x134
PIO_PIO_EVTSR_0 EQU 0x138
PIO_PIO_DEEVSR_0 EQU 0x13C
PIO_PIO_WSER_0 EQU 0x140
PIO_PIO_WSDR_0 EQU 0x144
PIO_PIO_WSMR_0 EQU 0x148
PIO_PIO_WSSR_0 EQU 0x14C
PIO_PIO_PWSER_0 EQU 0x150
PIO_PIO_PWSDR_0 EQU 0x154
PIO_PIO_PWSMR_0 EQU 0x158
PIO_PIO_PWSSR_0 EQU 0x15C
PIO_PIO_WEER_0 EQU 0x160
PIO_PIO_WEDR_0 EQU 0x164
PIO_PIO_WESR_0 EQU 0x168
PIO_PIO_WDSR_0 EQU 0x16C
PIO_PIO_WETER_0 EQU 0x170
PIO_PIO_WETDR_0 EQU 0x174
PIO_PIO_WETSR_0 EQU 0x178
PIO_PIO_WEDESR_0 EQU 0x17C
PIO_PIO_PWEER_0 EQU 0x180
PIO_PIO_PWEDR_0 EQU 0x184
PIO_PIO_PWESR_0 EQU 0x188
PIO_PIO_PWETER_0 EQU 0x190
PIO_PIO_PWETDR_0 EQU 0x194
PIO_PIO_PWETSR_0 EQU 0x198
PIO_PIO_PWEDESR_0 EQU 0x19C
PIO_PIO_WUR0SR_0 EQU 0x1A0
PIO_PIO_WUR1SR_0 EQU 0x1A4
PIO_PIO_WUR2SR_0 EQU 0x1A8
PIO_PIO_WUR3SR_0 EQU 0x1AC
PIO_PIO_WUR4SR_0 EQU 0x1B0
PIO_PIO_WUR5SR_0 EQU 0x1B4
PIO_PIO_WUR6SR_0 EQU 0x1B8
PIO_PIO_WUR7SR_0 EQU 0x1BC
PIO_PIO_PER_1 EQU 0x220
PIO_PIO_PDR_1 EQU 0x224
PIO_PIO_PSR_1 EQU 0x228
PIO_PIO_OER_1 EQU 0x230
PIO_PIO_ODR_1 EQU 0x234
PIO_PIO_OSR_1 EQU 0x238
PIO_PIO_POSR_1 EQU 0x23C
PIO_PIO_IFER_1 EQU 0x240
PIO_PIO_IFDR_1 EQU 0x244
PIO_PIO_IFSR_1 EQU 0x248
PIO_PIO_SODR_1 EQU 0x250
PIO_PIO_CODR_1 EQU 0x254
PIO_PIO_ODSR_1 EQU 0x258
PIO_PIO_PDSR_1 EQU 0x25C
PIO_PIO_IER1_1 EQU 0x260
PIO_PIO_IDR1_1 EQU 0x264
PIO_PIO_IMR1_1 EQU 0x268
PIO_PIO_IER2_1 EQU 0x270
PIO_PIO_IDR2_1 EQU 0x274
PIO_PIO_IMR2_1 EQU 0x278
PIO_PIO_ISR_1 EQU 0x27C
PIO_PIO_MDER_1 EQU 0x280
PIO_PIO_MDDR_1 EQU 0x284
PIO_PIO_MDSR_1 EQU 0x288
PIO_PIO_CKIN_1 EQU 0x290
PIO_PIO_GPSR_1 EQU 0x294
PIO_PIO_ASR_1 EQU 0x298
PIO_PIO_BSR_1 EQU 0x29C
PIO_PIO_CSR_1 EQU 0x2A0
PIO_PIO_DSR_1 EQU 0x2A4
PIO_PIO_MSELSR_1 EQU 0x2A8
PIO_PIO_LSELSR_1 EQU 0x2AC
PIO_PIO_MDSER_1 EQU 0x2B0
PIO_PIO_MDSDR_1 EQU 0x2B4
PIO_PIO_MDSSR_1 EQU 0x2B8
PIO_PIO_LDSER_1 EQU 0x2C0
PIO_PIO_LDSDR_1 EQU 0x2C4
PIO_PIO_LDSSR_1 EQU 0x2C8
PIO_PIO_SRER_1 EQU 0x2D0
PIO_PIO_SRDR_1 EQU 0x2D4
PIO_PIO_SRSR_1 EQU 0x2D8
PIO_PIO_PHER_1 EQU 0x2E0
PIO_PIO_PHDR_1 EQU 0x2E4
PIO_PIO_PHSR_1 EQU 0x2E8
PIO_PIO_PLER_1 EQU 0x2F0
PIO_PIO_PLDR_1 EQU 0x2F4
PIO_PIO_PLSR_1 EQU 0x2F8
PIO_PIO_STER_1 EQU 0x300
PIO_PIO_STDR_1 EQU 0x304
PIO_PIO_STSR_1 EQU 0x308
PIO_PIO_INER_1 EQU 0x310
PIO_PIO_INDR_1 EQU 0x314
PIO_PIO_INSR_1 EQU 0x318
PIO_PIO_PERCPER_1 EQU 0x320
PIO_PIO_PERCPDR_1 EQU 0x324
PIO_PIO_PERCPSR_1 EQU 0x328
PIO_PIO_HWOBSSR_1 EQU 0x330
PIO_PIO_IF2SR_1 EQU 0x334
PIO_PIO_IF3SR_1 EQU 0x338
PIO_PIO_EVER_1 EQU 0x340
PIO_PIO_EVDR_1 EQU 0x344
PIO_PIO_EVSR_1 EQU 0x348
PIO_PIO_EVTER_1 EQU 0x350
PIO_PIO_EVTDR_1 EQU 0x354
PIO_PIO_EVTSR_1 EQU 0x358
PIO_PIO_DEEVSR_1 EQU 0x35C
PIO_PIO_WSER_1 EQU 0x360
PIO_PIO_WSDR_1 EQU 0x364
PIO_PIO_WSMR_1 EQU 0x368
PIO_PIO_WSSR_1 EQU 0x36C
PIO_PIO_PWSER_1 EQU 0x370
PIO_PIO_PWSDR_1 EQU 0x374
PIO_PIO_PWSMR_1 EQU 0x378
PIO_PIO_PWSSR_1 EQU 0x37C
PIO_PIO_WEER_1 EQU 0x380
PIO_PIO_WEDR_1 EQU 0x384
PIO_PIO_WESR_1 EQU 0x388
PIO_PIO_WDSR_1 EQU 0x38C
PIO_PIO_WETER_1 EQU 0x390
PIO_PIO_WETDR_1 EQU 0x394
PIO_PIO_WETSR_1 EQU 0x398
PIO_PIO_WEDESR_1 EQU 0x39C
PIO_PIO_PWEER_1 EQU 0x3A0
PIO_PIO_PWEDR_1 EQU 0x3A4
PIO_PIO_PWESR_1 EQU 0x3A8
PIO_PIO_PWETER_1 EQU 0x3B0
PIO_PIO_PWETDR_1 EQU 0x3B4
PIO_PIO_PWETSR_1 EQU 0x3B8
PIO_PIO_PWEDESR_1 EQU 0x3BC
PIO_PIO_WUR0SR_1 EQU 0x3C0
PIO_PIO_WUR1SR_1 EQU 0x3C4
PIO_PIO_WUR2SR_1 EQU 0x3C8
PIO_PIO_WUR3SR_1 EQU 0x3CC
PIO_PIO_WUR4SR_1 EQU 0x3D0
PIO_PIO_WUR5SR_1 EQU 0x3D4
PIO_PIO_WUR6SR_1 EQU 0x3D8
PIO_PIO_WUR7SR_1 EQU 0x3DC
PIO_PIO_PER_2 EQU 0x440
PIO_PIO_PDR_2 EQU 0x444
PIO_PIO_PSR_2 EQU 0x448
PIO_PIO_OER_2 EQU 0x450
PIO_PIO_ODR_2 EQU 0x454
PIO_PIO_OSR_2 EQU 0x458
PIO_PIO_POSR_2 EQU 0x45C
PIO_PIO_IFER_2 EQU 0x460
PIO_PIO_IFDR_2 EQU 0x464
PIO_PIO_IFSR_2 EQU 0x468
PIO_PIO_SODR_2 EQU 0x470
PIO_PIO_CODR_2 EQU 0x474
PIO_PIO_ODSR_2 EQU 0x478
PIO_PIO_PDSR_2 EQU 0x47C
PIO_PIO_IER1_2 EQU 0x480
PIO_PIO_IDR1_2 EQU 0x484
PIO_PIO_IMR1_2 EQU 0x488
PIO_PIO_IER2_2 EQU 0x490
PIO_PIO_IDR2_2 EQU 0x494
PIO_PIO_IMR2_2 EQU 0x498
PIO_PIO_ISR_2 EQU 0x49C
PIO_PIO_MDER_2 EQU 0x4A0
PIO_PIO_MDDR_2 EQU 0x4A4
PIO_PIO_MDSR_2 EQU 0x4A8
PIO_PIO_CKIN_2 EQU 0x4B0
PIO_PIO_GPSR_2 EQU 0x4B4
PIO_PIO_ASR_2 EQU 0x4B8
PIO_PIO_BSR_2 EQU 0x4BC
PIO_PIO_CSR_2 EQU 0x4C0
PIO_PIO_DSR_2 EQU 0x4C4
PIO_PIO_MSELSR_2 EQU 0x4C8
PIO_PIO_LSELSR_2 EQU 0x4CC
PIO_PIO_MDSER_2 EQU 0x4D0
PIO_PIO_MDSDR_2 EQU 0x4D4
PIO_PIO_MDSSR_2 EQU 0x4D8
PIO_PIO_LDSER_2 EQU 0x4E0
PIO_PIO_LDSDR_2 EQU 0x4E4
PIO_PIO_LDSSR_2 EQU 0x4E8
PIO_PIO_SRER_2 EQU 0x4F0
PIO_PIO_SRDR_2 EQU 0x4F4
PIO_PIO_SRSR_2 EQU 0x4F8
PIO_PIO_PHER_2 EQU 0x500
PIO_PIO_PHDR_2 EQU 0x504
PIO_PIO_PHSR_2 EQU 0x508
PIO_PIO_PLER_2 EQU 0x510
PIO_PIO_PLDR_2 EQU 0x514
PIO_PIO_PLSR_2 EQU 0x518
PIO_PIO_STER_2 EQU 0x520
PIO_PIO_STDR_2 EQU 0x524
PIO_PIO_STSR_2 EQU 0x528
PIO_PIO_INER_2 EQU 0x530
PIO_PIO_INDR_2 EQU 0x534
PIO_PIO_INSR_2 EQU 0x538
PIO_PIO_PERCPER_2 EQU 0x540
PIO_PIO_PERCPDR_2 EQU 0x544
PIO_PIO_PERCPSR_2 EQU 0x548
PIO_PIO_HWOBSSR_2 EQU 0x550
PIO_PIO_IF2SR_2 EQU 0x554
PIO_PIO_IF3SR_2 EQU 0x558
PIO_PIO_EVER_2 EQU 0x560
PIO_PIO_EVDR_2 EQU 0x564
PIO_PIO_EVSR_2 EQU 0x568
PIO_PIO_EVTER_2 EQU 0x570
PIO_PIO_EVTDR_2 EQU 0x574
PIO_PIO_EVTSR_2 EQU 0x578
PIO_PIO_DEEVSR_2 EQU 0x57C
PIO_PIO_WSER_2 EQU 0x580
PIO_PIO_WSDR_2 EQU 0x584
PIO_PIO_WSMR_2 EQU 0x588
PIO_PIO_WSSR_2 EQU 0x58C
PIO_PIO_PWSER_2 EQU 0x590
PIO_PIO_PWSDR_2 EQU 0x594
PIO_PIO_PWSMR_2 EQU 0x598
PIO_PIO_PWSSR_2 EQU 0x59C
PIO_PIO_WEER_2 EQU 0x5A0
PIO_PIO_WEDR_2 EQU 0x5A4
PIO_PIO_WESR_2 EQU 0x5A8
PIO_PIO_WDSR_2 EQU 0x5AC
PIO_PIO_WETER_2 EQU 0x5B0
PIO_PIO_WETDR_2 EQU 0x5B4
PIO_PIO_WETSR_2 EQU 0x5B8
PIO_PIO_WEDESR_2 EQU 0x5BC
PIO_PIO_PWEER_2 EQU 0x5C0
PIO_PIO_PWEDR_2 EQU 0x5C4
PIO_PIO_PWESR_2 EQU 0x5C8
PIO_PIO_PWETER_2 EQU 0x5D0
PIO_PIO_PWETDR_2 EQU 0x5D4
PIO_PIO_PWETSR_2 EQU 0x5D8
PIO_PIO_PWEDESR_2 EQU 0x5DC
PIO_PIO_WUR0SR_2 EQU 0x5E0
PIO_PIO_WUR1SR_2 EQU 0x5E4
PIO_PIO_WUR2SR_2 EQU 0x5E8
PIO_PIO_WUR3SR_2 EQU 0x5EC
PIO_PIO_WUR4SR_2 EQU 0x5F0
PIO_PIO_WUR5SR_2 EQU 0x5F4
PIO_PIO_WUR6SR_2 EQU 0x5F8
PIO_PIO_WUR7SR_2 EQU 0x5FC
PIO_PIO_PER_3 EQU 0x660
PIO_PIO_PDR_3 EQU 0x664
PIO_PIO_PSR_3 EQU 0x668
PIO_PIO_OER_3 EQU 0x670
PIO_PIO_ODR_3 EQU 0x674
PIO_PIO_OSR_3 EQU 0x678
PIO_PIO_POSR_3 EQU 0x67C
PIO_PIO_IFER_3 EQU 0x680
PIO_PIO_IFDR_3 EQU 0x684
PIO_PIO_IFSR_3 EQU 0x688
PIO_PIO_SODR_3 EQU 0x690
PIO_PIO_CODR_3 EQU 0x694
PIO_PIO_ODSR_3 EQU 0x698
PIO_PIO_PDSR_3 EQU 0x69C
PIO_PIO_IER1_3 EQU 0x6A0
PIO_PIO_IDR1_3 EQU 0x6A4
PIO_PIO_IMR1_3 EQU 0x6A8
PIO_PIO_IER2_3 EQU 0x6B0
PIO_PIO_IDR2_3 EQU 0x6B4
PIO_PIO_IMR2_3 EQU 0x6B8
PIO_PIO_ISR_3 EQU 0x6BC
PIO_PIO_MDER_3 EQU 0x6C0
PIO_PIO_MDDR_3 EQU 0x6C4
PIO_PIO_MDSR_3 EQU 0x6C8
PIO_PIO_CKIN_3 EQU 0x6D0
PIO_PIO_GPSR_3 EQU 0x6D4
PIO_PIO_ASR_3 EQU 0x6D8
PIO_PIO_BSR_3 EQU 0x6DC
PIO_PIO_CSR_3 EQU 0x6E0
PIO_PIO_DSR_3 EQU 0x6E4
PIO_PIO_MSELSR_3 EQU 0x6E8
PIO_PIO_LSELSR_3 EQU 0x6EC
PIO_PIO_MDSER_3 EQU 0x6F0
PIO_PIO_MDSDR_3 EQU 0x6F4
PIO_PIO_MDSSR_3 EQU 0x6F8
PIO_PIO_LDSER_3 EQU 0x700
PIO_PIO_LDSDR_3 EQU 0x704
PIO_PIO_LDSSR_3 EQU 0x708
PIO_PIO_SRER_3 EQU 0x710
PIO_PIO_SRDR_3 EQU 0x714
PIO_PIO_SRSR_3 EQU 0x718
PIO_PIO_PHER_3 EQU 0x720
PIO_PIO_PHDR_3 EQU 0x724
PIO_PIO_PHSR_3 EQU 0x728
PIO_PIO_PLER_3 EQU 0x730
PIO_PIO_PLDR_3 EQU 0x734
PIO_PIO_PLSR_3 EQU 0x738
PIO_PIO_STER_3 EQU 0x740
PIO_PIO_STDR_3 EQU 0x744
PIO_PIO_STSR_3 EQU 0x748
PIO_PIO_INER_3 EQU 0x750
PIO_PIO_INDR_3 EQU 0x754
PIO_PIO_INSR_3 EQU 0x758
PIO_PIO_PERCPER_3 EQU 0x760
PIO_PIO_PERCPDR_3 EQU 0x764
PIO_PIO_PERCPSR_3 EQU 0x768
PIO_PIO_HWOBSSR_3 EQU 0x770
PIO_PIO_IF2SR_3 EQU 0x774
PIO_PIO_IF3SR_3 EQU 0x778
PIO_PIO_EVER_3 EQU 0x780
PIO_PIO_EVDR_3 EQU 0x784
PIO_PIO_EVSR_3 EQU 0x788
PIO_PIO_EVTER_3 EQU 0x790
PIO_PIO_EVTDR_3 EQU 0x794
PIO_PIO_EVTSR_3 EQU 0x798
PIO_PIO_DEEVSR_3 EQU 0x79C
PIO_PIO_WSER_3 EQU 0x7A0
PIO_PIO_WSDR_3 EQU 0x7A4
PIO_PIO_WSMR_3 EQU 0x7A8
PIO_PIO_WSSR_3 EQU 0x7AC
PIO_PIO_PWSER_3 EQU 0x7B0
PIO_PIO_PWSDR_3 EQU 0x7B4
PIO_PIO_PWSMR_3 EQU 0x7B8
PIO_PIO_PWSSR_3 EQU 0x7BC
PIO_PIO_WEER_3 EQU 0x7C0
PIO_PIO_WEDR_3 EQU 0x7C4
PIO_PIO_WESR_3 EQU 0x7C8
PIO_PIO_WDSR_3 EQU 0x7CC
PIO_PIO_WETER_3 EQU 0x7D0
PIO_PIO_WETDR_3 EQU 0x7D4
PIO_PIO_WETSR_3 EQU 0x7D8
PIO_PIO_WEDESR_3 EQU 0x7DC
PIO_PIO_PWEER_3 EQU 0x7E0
PIO_PIO_PWEDR_3 EQU 0x7E4
PIO_PIO_PWESR_3 EQU 0x7E8
PIO_PIO_PWETER_3 EQU 0x7F0
PIO_PIO_PWETDR_3 EQU 0x7F4
PIO_PIO_PWETSR_3 EQU 0x7F8
PIO_PIO_PWEDESR_3 EQU 0x7FC
PIO_PIO_WUR0SR_3 EQU 0x800
PIO_PIO_WUR1SR_3 EQU 0x804
PIO_PIO_WUR2SR_3 EQU 0x808
PIO_PIO_WUR3SR_3 EQU 0x80C
PIO_PIO_WUR4SR_3 EQU 0x810
PIO_PIO_WUR5SR_3 EQU 0x814
PIO_PIO_WUR6SR_3 EQU 0x818
PIO_PIO_WUR7SR_3 EQU 0x81C
; EOF: pio.hs
   END
