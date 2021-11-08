//See LICENSE for license details.
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "riscv_encoding.h"
#include "nmsis_core.h"

__attribute__((weak)) uintptr_t handle_nmi()
{
  write(1, "nmi\n", 5);
  _exit(1);
  return 0;
}


__attribute__((weak)) uintptr_t handle_trap(uintptr_t mcause, uintptr_t sp)
{
  if((mcause & 0xFFF) == 0xFFF) {
      handle_nmi();
  }
  write(1, "trap\n", 5);
  //printf("In trap handler, the mcause is %d\n", mcause);
  //printf("In trap handler, the mepc is 0x%x\n", __RV_CSR_READ(mepc));
  //printf("In trap handler, the mtval is 0x%x\n", __RV_CSR_READ(mbadaddr));
  _exit(mcause);
  return 0;
}





