//See LICENSE for license details.
#include <gd32vf103.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "riscv_encoding.h"
#include "nmsis_core.h"

extern uint32_t disable_mcycle_minstret();
void _init()
{
	SystemInit();

	//ECLIC init
	ECLIC->CFG = 0U;
	ECLIC->MTH = 0U;
	for(int i=0; i<ECLIC_NUM_INTERRUPTS; i++) {
		ECLIC->CTRL[i].INTIP   = 0U;
		ECLIC->CTRL[i].INTIE   = 0U;
		ECLIC->CTRL[i].INTATTR = 0U;
		ECLIC->CTRL[i].INTCTRL = 0U;
	}
	__set_exc_entry(__RV_CSR_READ(CSR_MTVEC));

	//printf("After ECLIC mode enabled, the mtvec value is %x \n\n\r", read_csr(mtvec));

	// // It must be NOTED:
	//  //    * In the RISC-V arch, if user mode and PMP supported, then by default if PMP is not configured
	//  //      with valid entries, then user mode cannot access any memory, and cannot execute any instructions.
	//  //    * So if switch to user-mode and still want to continue, then you must configure PMP first
	//pmp_open_all_space();
	//switch_m2u_mode();
	
    /* Before enter into main, add the cycle/instret disable by default to save power,
    only use them when needed to measure the cycle/instret */
	disable_mcycle_minstret();
}

void _fini()
{
}
