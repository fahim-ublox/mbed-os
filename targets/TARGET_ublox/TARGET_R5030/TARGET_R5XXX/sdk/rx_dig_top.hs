; 
; Copyright (C) u-blox
; All rights reserved.
; This source file is the sole property of u-blox. Reproduction or utilization
; of this source in whole or part is forbidden without the written consent of
; u-blox.
; 
; FILE: rx_dig_top.hs

RX_DIG_TOP_HAL_VERSION EQU 8
RX_DIG_TOP_PADC_GAIN_CTL EQU 0x000
RX_DIG_TOP_MIX_CTL_1 EQU 0x004
RX_DIG_TOP_MIX_CTL_2 EQU 0x008
RX_DIG_TOP_WB_DCOC_CTL EQU 0x00C
RX_DIG_TOP_WB_DCOC_CORR_I EQU 0x010
RX_DIG_TOP_WB_DCOC_CORR_Q EQU 0x014
RX_DIG_TOP_NB_DCOC_CTL EQU 0x018
RX_DIG_TOP_NB_DCOC_CORR EQU 0x01C
RX_DIG_TOP_ADC_IMB_CTRL EQU 0x020
RX_DIG_TOP_WB_IQ_CORR_CTRL EQU 0x024
RX_DIG_TOP_NB_IQ_CORR_CTRL EQU 0x028
RX_DIG_TOP_INIT_SAMPLE_CTRL EQU 0x02C
RX_DIG_TOP_HIST_CTRL EQU 0x030
RX_DIG_TOP_FILTS_CTRL_1 EQU 0x034
RX_DIG_TOP_FILTS_CTRL_2 EQU 0x038
RX_DIG_TOP_FILTS_CTRL_3 EQU 0x03C
RX_DIG_TOP_SEL_COEFFS_1 EQU 0x040
RX_DIG_TOP_SEL_COEFFS_2 EQU 0x044
RX_DIG_TOP_SEL_COEFFS_3 EQU 0x048
RX_DIG_TOP_SEL_COEFFS_4 EQU 0x04C
RX_DIG_TOP_SEL_COEFFS_5 EQU 0x050
RX_DIG_TOP_SEL_COEFFS_6 EQU 0x054
RX_DIG_TOP_SEL_COEFFS_7 EQU 0x058
RX_DIG_TOP_WB_DCOC_EST_I EQU 0x05C
RX_DIG_TOP_WB_DCOC_EST_Q EQU 0x060
RX_DIG_TOP_NB_DCOC_ESTS EQU 0x064
RX_DIG_TOP_RX_STATUS EQU 0x068
RX_DIG_TOP_WB_PWR EQU 0x06C
RX_DIG_TOP_NB_PWR EQU 0x070
RX_DIG_TOP_CORR_IWB EQU 0x074
RX_DIG_TOP_CORR_QWB EQU 0x078
RX_DIG_TOP_CORR_IQWB EQU 0x07C
RX_DIG_TOP_CORR_INB EQU 0x080
RX_DIG_TOP_CORR_QNB EQU 0x084
RX_DIG_TOP_CORR_IQNB EQU 0x088
RX_DIG_TOP_CORR_IWB_1 EQU 0x08C
RX_DIG_TOP_CORR_IWB_2 EQU 0x090
RX_DIG_TOP_CORR_QWB_1 EQU 0x094
RX_DIG_TOP_CORR_QWB_2 EQU 0x098
RX_DIG_TOP_HIST_I_0 EQU 0x09C
RX_DIG_TOP_HIST_I_1 EQU 0x0A0
RX_DIG_TOP_HIST_I_2 EQU 0x0A4
RX_DIG_TOP_HIST_I_3 EQU 0x0A8
RX_DIG_TOP_HIST_I_4 EQU 0x0AC
RX_DIG_TOP_HIST_I_5 EQU 0x0B0
RX_DIG_TOP_HIST_I_6 EQU 0x0B4
RX_DIG_TOP_HIST_I_7 EQU 0x0B8
RX_DIG_TOP_HIST_I_8 EQU 0x0BC
RX_DIG_TOP_HIST_I_9 EQU 0x0C0
RX_DIG_TOP_HIST_I_10 EQU 0x0C4
RX_DIG_TOP_HIST_I_11 EQU 0x0C8
RX_DIG_TOP_HIST_I_12 EQU 0x0CC
RX_DIG_TOP_HIST_I_13 EQU 0x0D0
RX_DIG_TOP_HIST_I_14 EQU 0x0D4
RX_DIG_TOP_HIST_I_15 EQU 0x0D8
RX_DIG_TOP_HIST_Q_0 EQU 0x0DC
RX_DIG_TOP_HIST_Q_1 EQU 0x0E0
RX_DIG_TOP_HIST_Q_2 EQU 0x0E4
RX_DIG_TOP_HIST_Q_3 EQU 0x0E8
RX_DIG_TOP_HIST_Q_4 EQU 0x0EC
RX_DIG_TOP_HIST_Q_5 EQU 0x0F0
RX_DIG_TOP_HIST_Q_6 EQU 0x0F4
RX_DIG_TOP_HIST_Q_7 EQU 0x0F8
RX_DIG_TOP_HIST_Q_8 EQU 0x0FC
RX_DIG_TOP_HIST_Q_9 EQU 0x100
RX_DIG_TOP_HIST_Q_10 EQU 0x104
RX_DIG_TOP_HIST_Q_11 EQU 0x108
RX_DIG_TOP_HIST_Q_12 EQU 0x10C
RX_DIG_TOP_HIST_Q_13 EQU 0x110
RX_DIG_TOP_HIST_Q_14 EQU 0x114
RX_DIG_TOP_HIST_Q_15 EQU 0x118
RX_DIG_TOP_ADCI1 EQU 0x11C
RX_DIG_TOP_ADCI2 EQU 0x120
RX_DIG_TOP_ADCQ1 EQU 0x124
RX_DIG_TOP_ADCQ2 EQU 0x128
RX_DIG_TOP_ADC_CONV EQU 0x12C
RX_DIG_TOP_ADC_CAL_FLAG_EN EQU 0x130
RX_DIG_TOP_ADC_TMODE EQU 0x134
RX_DIG_TOP_RX_LOOP_CTL EQU 0x138
RX_DIG_TOP_FILTS_CTRL_4 EQU 0x13C
RX_DIG_TOP_FILTS_CTRL_5 EQU 0x140
RX_DIG_TOP_RX_DFE_RESET EQU 0x144
RX_DIG_TOP_RX_DFE_STRT_CTRL EQU 0x148
RX_DIG_TOP_RX_DFE_END_CTRL EQU 0x14C
RX_DIG_TOP_RX_IRQ EQU 0x150
; EOF: rx_dig_top.hs
   END