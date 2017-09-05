#ifndef INCLUDED_PMU
#define INCLUDED_PMU
/*
 * Copyright (C) u-blox 
 * All rights reserved. 
 * This source file is the sole property of u-blox. Reproduction or utilization 
 * of this source in whole or part is forbidden without the written consent of 
 * u-blox.
 *
 */

#include <stdint.h>

/** Maximum Number of Event Count Registers */
#define NR_PMUEVCNT 4
#define PMU_PERIPHERALID4 0x07
#define PMU_PERIPHERALID5 0x00
#define PMU_PERIPHERALID6 0x00
#define PMU_PERIPHERALID7 0x00
#define PMU_PERIPHERALID0 0x85
#define PMU_PERIPHERALID1 0xDA
#define PMU_PERIPHERALID2 0x08
#define PMU_PERIPHERALID3 0x00
#define PMU_COMPONENTID0 0x0D
#define PMU_COMPONENTID1 0x90
#define PMU_COMPONENTID2 0x05
#define PMU_COMPONENTID3 0xB1

/** Performance Monitor Unit.
The performance monitor is used to profile and optimise the performance of software being developed to run on the Ardbeg cluster, by allowing the occurrence of system events to be monitored and analysed.
The PMU is programmed using the debug APB (via the Debug APB Interface).
The trace output generated by the PMU is routed to the CoreSight subsystem which combines the trace data streams from all trace data sources in the system and routes the combined trace data to the external trace port.

*/
struct pmu_s {
   /** Event Counter Registers at address offset 0x000, read-write */
   uint32_t pmevcntr[NR_PMUEVCNT];
   /** Reserved space */
   uint8_t fill0[1008];
   /** Event Counter Control at address offset 0x400, read-write */
   uint32_t pmevtyper[NR_PMUEVCNT];
   /** Reserved space */
   uint8_t fill1[2032];
   /** Counter Enable Set Register at address offset 0xC00, read-write */
   uint32_t pmcntenset;
   /** Reserved space */
   uint8_t fill2[28];
   /** Counter Enable Clear Register at address offset 0xC20, read-write */
   uint32_t pmcntenclr;
   /** Reserved space */
   uint8_t fill3[28];
   /** Interrupt Enable Set Register at address offset 0xC40, read-write */
   uint32_t pmintenset;
   /** Reserved space */
   uint8_t fill4[28];
   /** Interrupt Enable Clear Register at address offset 0xC60, read-write */
   uint32_t pmintenclr;
   /** Reserved space */
   uint8_t fill5[28];
   /** Overflow Flag Status Register at address offset 0xC80, read-write */
   uint32_t pmovsr;
   /** Reserved space */
   uint8_t fill6[28];
   /** Software Increment Register at address offset 0xCA0, write-only */
   uint32_t pmswinc;
   /** Reserved space */
   uint8_t fill7[220];
   /** Counter 0 AXI Destination Address.  The Counter 0 AXI Destination Register contains the address that counter 0 will generate on the AXI interconnect. The other counters will use the same address except for bits [11:8] which are replaced by the counter ID of the associated counter. at address offset 0xD80, read-write */
   uint32_t pmaxidest;
   /** Fault Status Register.  The Fault Flag Status Register holds the state of the fault flags of each counter. 0 = no counter fault, 1 = Counter fault. These flags can be cleared by writing a 1 to the associated field (Writing 0 has no effect). Odd bits contain AXI error response flags. Even bits contain flush overflow faults flags. Bits [1:0] correspond to counter 0. at address offset 0xD84, read-write */
   uint32_t pmfltsr;
   /** Reserved space */
   uint8_t fill8[8];
   /** fault Enable Set Register.  The Fault Enable Set Register allows setting enable bits in the Fault Enable Register (FLTENR). Each bit enables propagation of fault flags of the associated counter to the fault_irq_o output. Bits [1:0] relate to counter 0. Reading returns the current value of the FLTENR register. Odd bits contain AXI error response flags. Even bits contain flush overflow faults flags. at address offset 0xD90, read-write */
   uint32_t pmfltenset;
   /** Reserved space */
   uint8_t fill9[12];
   /** fault Enable Clear Register.  The Fault Enable Clear Register allows clearing enable bits in the Fault Enable Register (FLTENR). Each bit enables propagation of fault flags of the associated counter to the fault_irq_o output. Bits [1:0] relate to counter 0. Reading returns the current value of the FLTENR register. Odd bits contain AXI error response flags. Even bits contain flush overflow faults flags. at address offset 0xDA0, read-write */
   uint32_t pmfltenclr;
   /** Reserved space */
   uint8_t fill10[92];
   /** Configuration Register at address offset 0xE00, read-only */
   uint32_t pmcfgr;
   /** Performance Monitor Control Register at address offset 0xE04, read-write */
   uint32_t pmcr;
   /** Reserved space */
   uint8_t fill11[424];
   /** Lock Access Register at address offset 0xFB0, write-only */
   uint32_t pmlar;
   /** Lock Status Register at address offset 0xFB4, read-only */
   uint32_t pmlsr;
   /** Authentication Status Register at address offset 0xFB8, read-only */
   uint32_t pmauthstatus;
   /** Reserved space */
   uint8_t fill12[12];
   /** Device Identifier at address offset 0xFC8, read-only */
   uint32_t pmdevid;
   /** Device Type at address offset 0xFCC, read-only */
   uint32_t pmdevtype;
   /** Performance Monitor Peripheral ID4 Register at address offset 0xFD0, read-only constant 0x00000007 */
   uint32_t peripheralid4;
   /** Performance Monitor Peripheral ID5 Register at address offset 0xFD4, read-only constant 0x00000000 */
   uint32_t peripheralid5;
   /** Performance Monitor Peripheral ID6 Register at address offset 0xFD8, read-only constant 0x00000000 */
   uint32_t peripheralid6;
   /** Performance Monitor Peripheral ID7 Register at address offset 0xFDC, read-only constant 0x00000000 */
   uint32_t peripheralid7;
   /** Performance Monitor Peripheral ID0 Register at address offset 0xFE0, read-only constant 0x00000085 */
   uint32_t peripheralid0;
   /** Performance Monitor Peripheral ID1 Register at address offset 0xFE4, read-only constant 0x000000DA */
   uint32_t peripheralid1;
   /** Performance Monitor Peripheral ID2 Register at address offset 0xFE8, read-only constant 0x00000008 */
   uint32_t peripheralid2;
   /** Performance Monitor Peripheral ID3 Register at address offset 0xFEC, read-only constant 0x00000000 */
   uint32_t peripheralid3;
   /** Performance Monitor Component ID0 Register at address offset 0xFF0, read-only constant 0x0000000D */
   uint32_t componentid0;
   /** Performance Monitor Component ID1 Register at address offset 0xFF4, read-only constant 0x00000090 */
   uint32_t componentid1;
   /** Performance Monitor Component ID2 Register at address offset 0xFF8, read-only constant 0x00000005 */
   uint32_t componentid2;
   /** Performance Monitor Component ID3 Register at address offset 0xFFC, read-only constant 0x000000B1 */
   uint32_t componentid3;
};

/** bit field defines for pmu_s#pmevcntr */
#define PMU_PMEVCNTR_EVCNTR_OFFSET 0
#define PMU_PMEVCNTR_EVCNTR_SIZE 32

/** bit field defines for pmu_s#pmevtyper */
#define PMU_PMEVTYPER_SEL_INCREMENT_OFFSET 0
#define PMU_PMEVTYPER_SEL_INCREMENT_SIZE 8
#define PMU_PMEVTYPER_SEL_START_OFFSET 8
#define PMU_PMEVTYPER_SEL_START_SIZE 8
#define PMU_PMEVTYPER_SEL_STOP_OFFSET 16
#define PMU_PMEVTYPER_SEL_STOP_SIZE 8
#define PMU_PMEVTYPER_SEL_FLUSH_OFFSET 24
#define PMU_PMEVTYPER_SEL_FLUSH_SIZE 8

/** bit field defines for pmu_s#pmcntenset */
#define PMU_PMCNTENSET_CNTENSET_OFFSET 0
#define PMU_PMCNTENSET_CNTENSET_SIZE 32

/** bit field defines for pmu_s#pmcntenclr */
#define PMU_PMCNTENCLR_CNTENCLR_OFFSET 0
#define PMU_PMCNTENCLR_CNTENCLR_SIZE 32

/** bit field defines for pmu_s#pmintenset */
#define PMU_PMINTENSET_INTENSET_OFFSET 0
#define PMU_PMINTENSET_INTENSET_SIZE 32

/** bit field defines for pmu_s#pmintenclr */
#define PMU_PMINTENCLR_INTENCLR_OFFSET 0
#define PMU_PMINTENCLR_INTENCLR_SIZE 32

/** bit field defines for pmu_s#pmovsr */
#define PMU_PMOVSR_OVSR_OFFSET 0
#define PMU_PMOVSR_OVSR_SIZE 32

/** bit field defines for pmu_s#pmswinc */
#define PMU_PMSWINC_SWINC_OFFSET 0
#define PMU_PMSWINC_SWINC_SIZE 32

/** bit field defines for pmu_s#pmaxidest */
#define PMU_PMAXIDEST_ADDRESS_7DT0_OFFSET 0
#define PMU_PMAXIDEST_ADDRESS_7DT0_SIZE 8
#define PMU_PMAXIDEST_ADDRESS_11DT8_OFFSET 8
#define PMU_PMAXIDEST_ADDRESS_11DT8_SIZE 4
#define PMU_PMAXIDEST_ADDRESS_31DT12_OFFSET 12
#define PMU_PMAXIDEST_ADDRESS_31DT12_SIZE 20

/** bit field defines for pmu_s#pmfltsr */
#define PMU_PMFLTSR_COUNTER0_AXI_ERROR_OFFSET 0
#define PMU_PMFLTSR_COUNTER0_AXI_ERROR_SIZE 1
#define PMU_PMFLTSR_COUNTER0_FLUSH_OVERFLOW_OFFSET 1
#define PMU_PMFLTSR_COUNTER0_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTSR_COUNTER1_AXI_ERROR_OFFSET 2
#define PMU_PMFLTSR_COUNTER1_AXI_ERROR_SIZE 1
#define PMU_PMFLTSR_COUNTER1_FLUSH_OVERFLOW_OFFSET 3
#define PMU_PMFLTSR_COUNTER1_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTSR_COUNTER2_AXI_ERROR_OFFSET 4
#define PMU_PMFLTSR_COUNTER2_AXI_ERROR_SIZE 1
#define PMU_PMFLTSR_COUNTER2_FLUSH_OVERFLOW_OFFSET 5
#define PMU_PMFLTSR_COUNTER2_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTSR_COUNTER3_AXI_ERROR_OFFSET 6
#define PMU_PMFLTSR_COUNTER3_AXI_ERROR_SIZE 1
#define PMU_PMFLTSR_COUNTER3_FLUSH_OVERFLOW_OFFSET 7
#define PMU_PMFLTSR_COUNTER3_FLUSH_OVERFLOW_SIZE 1

/** bit field defines for pmu_s#pmfltenset */
#define PMU_PMFLTENSET_COUNTER0_AXI_ERROR_OFFSET 0
#define PMU_PMFLTENSET_COUNTER0_AXI_ERROR_SIZE 1
#define PMU_PMFLTENSET_COUNTER0_FLUSH_OVERFLOW_OFFSET 1
#define PMU_PMFLTENSET_COUNTER0_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTENSET_COUNTER1_AXI_ERROR_OFFSET 2
#define PMU_PMFLTENSET_COUNTER1_AXI_ERROR_SIZE 1
#define PMU_PMFLTENSET_COUNTER1_FLUSH_OVERFLOW_OFFSET 3
#define PMU_PMFLTENSET_COUNTER1_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTENSET_COUNTER2_AXI_ERROR_OFFSET 4
#define PMU_PMFLTENSET_COUNTER2_AXI_ERROR_SIZE 1
#define PMU_PMFLTENSET_COUNTER2_FLUSH_OVERFLOW_OFFSET 5
#define PMU_PMFLTENSET_COUNTER2_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTENSET_COUNTER3_AXI_ERROR_OFFSET 6
#define PMU_PMFLTENSET_COUNTER3_AXI_ERROR_SIZE 1
#define PMU_PMFLTENSET_COUNTER3_FLUSH_OVERFLOW_OFFSET 7
#define PMU_PMFLTENSET_COUNTER3_FLUSH_OVERFLOW_SIZE 1

/** bit field defines for pmu_s#pmfltenclr */
#define PMU_PMFLTENCLR_COUNTER0_AXI_ERROR_OFFSET 0
#define PMU_PMFLTENCLR_COUNTER0_AXI_ERROR_SIZE 1
#define PMU_PMFLTENCLR_COUNTER0_FLUSH_OVERFLOW_OFFSET 1
#define PMU_PMFLTENCLR_COUNTER0_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTENCLR_COUNTER1_AXI_ERROR_OFFSET 2
#define PMU_PMFLTENCLR_COUNTER1_AXI_ERROR_SIZE 1
#define PMU_PMFLTENCLR_COUNTER1_FLUSH_OVERFLOW_OFFSET 3
#define PMU_PMFLTENCLR_COUNTER1_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTENCLR_COUNTER2_AXI_ERROR_OFFSET 4
#define PMU_PMFLTENCLR_COUNTER2_AXI_ERROR_SIZE 1
#define PMU_PMFLTENCLR_COUNTER2_FLUSH_OVERFLOW_OFFSET 5
#define PMU_PMFLTENCLR_COUNTER2_FLUSH_OVERFLOW_SIZE 1
#define PMU_PMFLTENCLR_COUNTER3_AXI_ERROR_OFFSET 6
#define PMU_PMFLTENCLR_COUNTER3_AXI_ERROR_SIZE 1
#define PMU_PMFLTENCLR_COUNTER3_FLUSH_OVERFLOW_OFFSET 7
#define PMU_PMFLTENCLR_COUNTER3_FLUSH_OVERFLOW_SIZE 1

/** bit field defines for pmu_s#pmcfgr */
#define PMU_PMCFGR_N_OFFSET 0
#define PMU_PMCFGR_N_SIZE 8
#define PMU_PMCFGR_SIZE_OFFSET 8
#define PMU_PMCFGR_SIZE_SIZE 3
#define PMU_PMCFGR_CC_OFFSET 14
#define PMU_PMCFGR_CC_SIZE 1
#define PMU_PMCFGR_CCD_OFFSET 15
#define PMU_PMCFGR_CCD_SIZE 1
#define PMU_PMCFGR_EX_OFFSET 16
#define PMU_PMCFGR_EX_SIZE 1
#define PMU_PMCFGR_NA_OFFSET 17
#define PMU_PMCFGR_NA_SIZE 1
#define PMU_PMCFGR_WT_OFFSET 18
#define PMU_PMCFGR_WT_SIZE 1
#define PMU_PMCFGR_UEN_OFFSET 19
#define PMU_PMCFGR_UEN_SIZE 1

/** bit field defines for pmu_s#pmcr */
#define PMU_PMCR_CEN_OFFSET 0
#define PMU_PMCR_CEN_SIZE 1
#define PMU_PMCR_RST_OFFSET 1
#define PMU_PMCR_RST_SIZE 1
#define PMU_PMCR_CCR_OFFSET 2
#define PMU_PMCR_CCR_SIZE 1
#define PMU_PMCR_CCD_OFFSET 3
#define PMU_PMCR_CCD_SIZE 1
#define PMU_PMCR_EX_OFFSET 4
#define PMU_PMCR_EX_SIZE 1
#define PMU_PMCR_DP_OFFSET 5
#define PMU_PMCR_DP_SIZE 1
#define PMU_PMCR_NA_OFFSET 6
#define PMU_PMCR_NA_SIZE 1
#define PMU_PMCR_WT_OFFSET 7
#define PMU_PMCR_WT_SIZE 1

/** bit field defines for pmu_s#pmlar */
#define PMU_PMLAR_PMLAR_OFFSET 0
#define PMU_PMLAR_PMLAR_SIZE 32

/** bit field defines for pmu_s#pmlsr */
#define PMU_PMLSR_PMLSR_OFFSET 0
#define PMU_PMLSR_PMLSR_SIZE 32

/** bit field defines for pmu_s#pmauthstatus */
#define PMU_PMAUTHSTATUS_PMAUTHSTATUS_OFFSET 0
#define PMU_PMAUTHSTATUS_PMAUTHSTATUS_SIZE 32

/** bit field defines for pmu_s#pmdevid */
#define PMU_PMDEVID_NRCNT_OFFSET 0
#define PMU_PMDEVID_NRCNT_SIZE 8

/** bit field defines for pmu_s#pmdevtype */
#define PMU_PMDEVTYPE_PMDEVTYPE_OFFSET 0
#define PMU_PMDEVTYPE_PMDEVTYPE_SIZE 32

/** bit field defines for pmu_s#peripheralid4 */
#define PMU_PERIPHERALID4_PERIPHERALID4_OFFSET 0
#define PMU_PERIPHERALID4_PERIPHERALID4_SIZE 8

/** bit field defines for pmu_s#peripheralid5 */
#define PMU_PERIPHERALID5_PERIPHERALID5_OFFSET 0
#define PMU_PERIPHERALID5_PERIPHERALID5_SIZE 8

/** bit field defines for pmu_s#peripheralid6 */
#define PMU_PERIPHERALID6_PERIPHERALID6_OFFSET 0
#define PMU_PERIPHERALID6_PERIPHERALID6_SIZE 8

/** bit field defines for pmu_s#peripheralid7 */
#define PMU_PERIPHERALID7_PERIPHERALID7_OFFSET 0
#define PMU_PERIPHERALID7_PERIPHERALID7_SIZE 8

/** bit field defines for pmu_s#peripheralid0 */
#define PMU_PERIPHERALID0_PERIPHERALID0_OFFSET 0
#define PMU_PERIPHERALID0_PERIPHERALID0_SIZE 8

/** bit field defines for pmu_s#peripheralid1 */
#define PMU_PERIPHERALID1_PERIPHERALID1_OFFSET 0
#define PMU_PERIPHERALID1_PERIPHERALID1_SIZE 8

/** bit field defines for pmu_s#peripheralid2 */
#define PMU_PERIPHERALID2_PERIPHERALID2_OFFSET 0
#define PMU_PERIPHERALID2_PERIPHERALID2_SIZE 8

/** bit field defines for pmu_s#peripheralid3 */
#define PMU_PERIPHERALID3_PERIPHERALID3_OFFSET 0
#define PMU_PERIPHERALID3_PERIPHERALID3_SIZE 8

/** bit field defines for pmu_s#componentid0 */
#define PMU_COMPONENTID0_COMPONENTID0_OFFSET 0
#define PMU_COMPONENTID0_COMPONENTID0_SIZE 8

/** bit field defines for pmu_s#componentid1 */
#define PMU_COMPONENTID1_COMPONENTID1_OFFSET 0
#define PMU_COMPONENTID1_COMPONENTID1_SIZE 8

/** bit field defines for pmu_s#componentid2 */
#define PMU_COMPONENTID2_COMPONENTID2_OFFSET 0
#define PMU_COMPONENTID2_COMPONENTID2_SIZE 8

/** bit field defines for pmu_s#componentid3 */
#define PMU_COMPONENTID3_COMPONENTID3_OFFSET 0
#define PMU_COMPONENTID3_COMPONENTID3_SIZE 8

/* EOF pmu.h */
#endif