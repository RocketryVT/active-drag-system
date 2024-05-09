#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_iep.h>
#include "intc_map.h"
#include "resource_table_empty.h"

#define CYCLES_PER_SECOND 200000000
#define CYCLES_PER_PERIOD (CYCLES_PER_SECOND / 800)
#define DEFAULT_DUTY_CYCLE 0
#define P8_12 (1 << 14)
#define P8_12_n ~(1 << 14)
#define PRU0_DRAM 0x00000                 // Offset to DRAM
#define PRU_TIMER_PASSCODE 0x31138423
#define BURN_TIME 2 // seconds for motor burn

volatile uint32_t *pru0_dram = (uint32_t *) (PRU0_DRAM + 0x200);


volatile register uint32_t __R30;
volatile register uint32_t __R31;


void main(void) {
    uint32_t duty_cycle = DEFAULT_DUTY_CYCLE;
    uint32_t off_cycles = ((100 - duty_cycle) * CYCLES_PER_PERIOD / 100);
    uint32_t on_cycles = ((100 - (100 - duty_cycle)) * CYCLES_PER_PERIOD / 100);
    uint32_t off_count = off_cycles;
    uint32_t on_count = on_cycles;
    *pru0_dram = duty_cycle;

    /* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    while (1) {
        if (pru0_dram[1] == PRU_TIMER_PASSCODE) {
            /* Disable counter */
            CT_IEP.TMR_GLB_CFG_bit.CNT_EN = 0;

            /* Reset Count register */
            CT_IEP.TMR_CNT = 0x0;

            /* Clear overflow status register */
            CT_IEP.TMR_GLB_STS_bit.CNT_OVF = 0x1;

            /* Set compare value */
            CT_IEP.TMR_CMP0 = (BURN_TIME * CYCLES_PER_SECOND); // 10 seconds @ 200MHz

            /* Clear compare status */
            CT_IEP.TMR_CMP_STS_bit.CMP_HIT = 0xFF;

            /* Disable compensation */
            CT_IEP.TMR_COMPEN_bit.COMPEN_CNT = 0x0;

            /* Enable CMP0 and reset on event */
            CT_IEP.TMR_CMP_CFG_bit.CMP0_RST_CNT_EN = 0x1;
            CT_IEP.TMR_CMP_CFG_bit.CMP_EN = 0x1;

            /* Clear the status of all interrupts */
            CT_INTC.SECR0 = 0xFFFFFFFF;
            CT_INTC.SECR1 = 0xFFFFFFFF;

            /* Enable counter */
            CT_IEP.TMR_GLB_CFG = 0x11;
            break;
        }
    }

	/* Poll until R31.31 is set */
	do {
		while ((__R31 & 0x80000000) == 0) {
		}
		/* Verify that the IEP is the source of the interrupt */
	} while ((CT_INTC.SECR0 & (1 << 7)) == 0);

	/* Disable counter */
	CT_IEP.TMR_GLB_CFG_bit.CNT_EN = 0x0;

	/* Disable Compare0 */
	CT_IEP.TMR_CMP_CFG = 0x0;

	/* Clear Compare status */
	CT_IEP.TMR_CMP_STS = 0xFF;

	/* Clear the status of the interrupt */
	CT_INTC.SECR0 = (1 << 7);

    while (1) {
        if (on_count) {
            __R30 |= P8_12;		// Set the GPIO pin to 1
            on_count--;
            __delay_cycles(3);
        } else if (off_count) {
            __R30 &= P8_12_n;		// Clear the GPIO pin
            off_count--;
        } else {
            duty_cycle = *pru0_dram;
            off_count = ((100 - duty_cycle) * CYCLES_PER_PERIOD / 100);
            on_count = ((100 - (100 - duty_cycle)) * CYCLES_PER_PERIOD / 100);
        }
    }
}
