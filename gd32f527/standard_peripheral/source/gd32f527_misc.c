/*!
    \file    gd32f527_misc.c
    \brief   MISC driver

    \version 2026-01-31, V1.4.0, firmware for GD32F527
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f527_misc.h"

/*!
    \brief    set the priority group
    \param[in]  nvic_prigroup: the NVIC priority group
      \arg        NVIC_PRIGROUP_PRE0_SUB4:0 bits for pre-emption priority 4 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE1_SUB3:1 bits for pre-emption priority 3 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE2_SUB2:2 bits for pre-emption priority 2 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE3_SUB1:3 bits for pre-emption priority 1 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE4_SUB0:4 bits for pre-emption priority 0 bits for subpriority
    \param[out] none
    \retval     none
*/
void nvic_priority_group_set(uint32_t nvic_prigroup)
{
    /* set the priority group value */
    SCB->AIRCR = NVIC_AIRCR_VECTKEY_MASK | nvic_prigroup;
}

/*!
    \brief    enable NVIC interrupt request
    \param[in]  nvic_irq: the NVIC interrupt request, detailed in IRQn_Type
    \param[in]  nvic_irq_pre_priority: the pre-emption priority needed to set
    \param[in]  nvic_irq_sub_priority: the subpriority needed to set
    \param[out] none
    \retval     none
*/
void nvic_irq_enable(IRQn_Type nvic_irq, uint8_t nvic_irq_pre_priority,
                     uint8_t nvic_irq_sub_priority)
{
    uint32_t nvic_prigroup, nvic_priority;

    /* check current priority group */
    switch(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) {
    case NVIC_PRIGROUP_PRE0_SUB4:
    case NVIC_PRIGROUP_PRE1_SUB3:
    case NVIC_PRIGROUP_PRE2_SUB2:
    case NVIC_PRIGROUP_PRE3_SUB1:
    case NVIC_PRIGROUP_PRE4_SUB0:
        break;
    default:
        nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
        break;
    }

    /* get the priority group value */
    nvic_prigroup = NVIC_GetPriorityGrouping();

    /* encoding the pre-emption, subpriority priority */
    nvic_priority = NVIC_EncodePriority(nvic_prigroup, (uint32_t)nvic_irq_pre_priority, (uint32_t)nvic_irq_sub_priority);
    /* set priority */
    NVIC_SetPriority(nvic_irq, nvic_priority);

    /* enable the selected IRQ */
    NVIC_EnableIRQ(nvic_irq);
}

/*!
    \brief    disable NVIC interrupt request
    \param[in]  nvic_irq: the NVIC interrupt request, detailed in IRQn_Type
    \param[out] none
    \retval     none
*/
void nvic_irq_disable(IRQn_Type nvic_irq)
{
    /* disable the selected IRQ */
    NVIC_DisableIRQ(nvic_irq);
}

/*!
    \brief      initiates a system reset request to reset the MCU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_system_reset(void)
{
    NVIC_SystemReset();
}

/*!
    \brief    set the NVIC vector table base address
    \param[in]  nvic_vict_tab: the RAM or FLASH base address
      \arg        NVIC_VECTTAB_RAM: RAM base address
      \are        NVIC_VECTTAB_FLASH: Flash base address
    \param[in]  offset: Vector Table offset
    \param[out] none
    \retval     none
*/
void nvic_vector_table_set(uint32_t nvic_vict_tab, uint32_t offset)
{
    SCB->VTOR = nvic_vict_tab | (offset & NVIC_VECTTAB_OFFSET_MASK);
    __DSB();
}

/*!
    \brief    set the state of the low power mode
    \param[in]  lowpower_mode: the low power mode state
      \arg        SCB_LPM_SLEEP_EXIT_ISR: if chose this para, the system always enter low power
                    mode by exiting from ISR
      \arg        SCB_LPM_DEEPSLEEP: if chose this para, the system will enter the DEEPSLEEP mode
      \arg        SCB_LPM_WAKE_BY_ALL_INT: if chose this para, the lowpower mode can be woke up
                    by all the enable and disable interrupts
    \param[out] none
    \retval     none
*/
void system_lowpower_set(uint8_t lowpower_mode)
{
    SCB->SCR |= (uint32_t)lowpower_mode;
}

/*!
    \brief    reset the state of the low power mode
    \param[in]  lowpower_mode: the low power mode state
      \arg        SCB_LPM_SLEEP_EXIT_ISR: if chose this para, the system will exit low power
                    mode by exiting from ISR
      \arg        SCB_LPM_DEEPSLEEP: if chose this para, the system will enter the SLEEP mode
      \arg        SCB_LPM_WAKE_BY_ALL_INT: if chose this para, the lowpower mode only can be
                    woke up by the enable interrupts
    \param[out] none
    \retval     none
*/
void system_lowpower_reset(uint8_t lowpower_mode)
{
    SCB->SCR &= (~(uint32_t)lowpower_mode);
}

/*!
    \brief    set the systick clock source
    \param[in]  systick_clksource: the systick clock source needed to choose
      \arg        SYSTICK_CLKSOURCE_HCLK: systick clock source is from HCLK
      \arg        SYSTICK_CLKSOURCE_HCLK_DIV8: systick clock source is from HCLK/8
    \param[out] none
    \retval     none
*/

void systick_clksource_set(uint32_t systick_clksource)
{
    if(SYSTICK_CLKSOURCE_HCLK == systick_clksource) {
        /* set the systick clock source from HCLK */
        SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
    } else {
        /* set the systick clock source from HCLK/8 */
        SysTick->CTRL &= SYSTICK_CLKSOURCE_HCLK_DIV8;
    }
}

#if (__MPU_PRESENT == 1)

/*!
    \brief      enable the MPU
    \param[in]  MPU_Control: select a different MPU mode
      \arg        MPU_MODE_HFNMI_PRIVDEF_NONE: HFNMIENA and PRIVDEFENA are 0
      \arg        MPU_MODE_HARDFAULT_NMI: use the MPU for memory accesses by HardFault and NMI handlers only
      \arg        MPU_MODE_PRIV_DEFAULT: enables the default memory map as a background region for privileged access only
      \arg        MPU_MODE_HFNMI_PRIVDEF: HFNMIENA and PRIVDEFENA are 1
    \param[out] none
    \retval     none
*/
void mpu_enable(uint32_t MPU_Control)
{
    __DMB();
    /* enable the MPU */
    MPU->CTRL = MPU_Control | MPU_CTRL_ENABLE_Msk;
#ifdef SCB_SHCSR_MEMFAULTENA_Msk
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
#endif
    __DSB();
    __ISB();
}

/*!
    \brief      disable the MPU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mpu_disable(void)
{
    __DMB();
#ifdef SCB_SHCSR_MEMFAULTENA_Msk
    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;
#endif
    /* disable the MPU */
    MPU->CTRL  &= ~MPU_CTRL_ENABLE_Msk;
    __DSB();
    __ISB();
}

/*!
    \brief      initialize mpu_region_init_struct with the default values
    \param[in]  region_init_struct: pointer to a mpu_region_init_struct structure
    \param[out] none
    \retval     none
*/
void mpu_region_struct_para_init(mpu_region_init_struct *region_init_struct)
{
    region_init_struct->region_number        = MPU_REGION_NUMBER0;
    region_init_struct->region_base_address  = 0x00000000U;
    region_init_struct->region_limit_address = 0x00000000U;
    region_init_struct->instruction_exec     = MPU_INSTRUCTION_EXEC_PERMIT;
    region_init_struct->access_permission    = MPU_REGION_PRIVILEGED_RW;
    region_init_struct->shareability         = MPU_ACCESS_NOT_SHAREABLE;
    region_init_struct->attribute_index      = MPU_ATTRIBUTE_NUMBER0;
}

/*!
    \brief      initialize mpu_attribute_init_struct with the default values
    \param[in]  attribute_init_struct: pointer to a mpu_attribute_init_struct structure
    \param[out] none
    \retval     none
*/
void mpu_attribute_struct_para_init(mpu_attribute_init_struct *attribute_init_struct)
{
    attribute_init_struct->attribute_number = MPU_ATTRIBUTE_NUMBER0;
    attribute_init_struct->memory_type      = MPU_MEMORY_DEVICE;
    attribute_init_struct->inner_attributes = MPU_DEVICE_nGnRnE;
    attribute_init_struct->outer_attributes = MPU_OUTER_DEVICE;
}

/*!
    \brief      configure the MPU region
    \param[in]  region_init_struct: MPU initialization structure
                  region_number: region number
                                 MPU_REGION_NUMBERn (n=0,..,7)
                  region_base_address: region base address
                  region_limit_address: region limit address
                  access_permission: MPU_REGION_PRIVILEGED_RW, MPU_REGION_ALL_RW, MPU_AP_PRIV_RW_UNPRIV_RO, MPU_REGION_ALL_RO
                  instruction_exec: MPU_INSTRUCTION_EXEC_PERMIT, MPU_INSTRUCTION_EXEC_NOT_PERMIT
                  shareability: MPU_ACCESS_NOT_SHAREABLE, MPU_ACCESS_OUTER_SHAREABLE, MPU_ACCESS_INNER_SHAREABLE
                  attribute_index: attribute index
                                   MPU_ATTRIBUTE_NUMBERn (n=0,..,7)
    \param[out] none
    \retval     none
*/
void mpu_region_config(mpu_region_init_struct *region_init_struct)
{
    __DMB();
    /* select the Region number  */
    MPU->RNR = region_init_struct->region_number;
    /* disable the Region  */
    MPU->RLAR &= ~(MPU_RLAR_EN_Msk);
    MPU->RBAR = (((uint32_t)region_init_struct->region_base_address & 0xFFFFFFE0UL)   | 
                 ((uint32_t)region_init_struct->shareability << MPU_RBAR_SH_Pos)      |
                 ((uint32_t)region_init_struct->access_permission << MPU_RBAR_AP_Pos) |
                 ((uint32_t)region_init_struct->instruction_exec << MPU_RBAR_XN_Pos)) ;
    
    MPU->RLAR = (((uint32_t)region_init_struct->region_limit_address & 0xFFFFFFE0UL)      |
                 ((uint32_t)region_init_struct->attribute_index << MPU_RLAR_AttrIndx_Pos));
}

/*!
    \brief      configure the MPU attribute
    \param[in]  attribute_init_struct: MPU attribute initialization structure
                  attribute_number: attribute number
                                 MPU_ATTRIBUTE_NUMBERn (n=0,..,7)
                  memory_type: memory type
                                 MPU_MEMORY_DEVICE, MPU_MEMORY_NORMAL
                  outer_attributes: outer memory attributes
                                 MPU_OUTER_DEVICE
                                 MPU_NORMAL_OUTER_WT_TRAN_W_ALLOC
                                 MPU_NORMAL_OUTER_WT_TRAN_R_ALLOC
                                 MPU_NORMAL_OUTER_WT_TRAN_RW_ALLOC
                                 MPU_NORMAL_OUTER_NON_CACHEABLE
                                 MPU_NORMAL_OUTER_WB_TRAN_W_ALLOC
                                 MPU_NORMAL_OUTER_WB_TRAN_R_ALLOC
                                 MPU_NORMAL_OUTER_WB_TRAN_RW_ALLOC
                                 MPU_NORMAL_OUTER_WT_NON_TRAN_NO_ALLOC
                                 MPU_NORMAL_OUTER_WT_NON_TRAN_W_ALLOC
                                 MPU_NORMAL_OUTER_WT_NON_TRAN_R_ALLOC
                                 MPU_NORMAL_OUTER_WT_NON_TRAN_RW_ALLOC
                                 MPU_NORMAL_OUTER_WB_NON_TRAN_NO_ALLOC
                                 MPU_NORMAL_OUTER_WB_NON_TRAN_W_ALLOC
                                 MPU_NORMAL_OUTER_WB_NON_TRAN_R_ALLOC
                                 MPU_NORMAL_OUTER_WB_NON_TRAN_RW_ALLOC
                  inner_attributes: inner memory attributes
                                 MPU_NORMAL_INNER_WT_TRAN_W_ALLOC
                                 MPU_NORMAL_INNER_WT_TRAN_R_ALLOC
                                 MPU_NORMAL_INNER_WT_TRAN_RW_ALLOC
                                 MPU_NORMAL_INNER_NON_CACHEABLE
                                 MPU_NORMAL_INNER_WB_TRAN_W_ALLOC
                                 MPU_NORMAL_INNER_WB_TRAN_R_ALLOC
                                 MPU_NORMAL_INNER_WB_TRAN_RW_ALLOC
                                 MPU_NORMAL_INNER_WT_NON_TRAN_NO_ALLOC
                                 MPU_NORMAL_INNER_WT_NON_TRAN_W_ALLOC
                                 MPU_NORMAL_INNER_WT_NON_TRAN_R_ALLOC
                                 MPU_NORMAL_INNER_WT_NON_TRAN_RW_ALLOC
                                 MPU_NORMAL_INNER_WB_NON_TRAN_NO_ALLOC
                                 MPU_NORMAL_INNER_WB_NON_TRAN_W_ALLOC
                                 MPU_NORMAL_INNER_WB_NON_TRAN_R_ALLOC
                                 MPU_NORMAL_INNER_WB_NON_TRAN_RW_ALLOC
                                 MPU_DEVICE_nGnRnE(for Device)
                                 MPU_DEVICE_nGnRE(for Device)
                                 MPU_DEVICE_nGRE(for Device)
                                 MPU_DEVICE_GRE(for Device)
    \param[out] none
    \retval     none
*/
void mpu_attribute_config(mpu_attribute_init_struct *attribute_init_struct)
{
    uint32_t attr_values;
    uint8_t  attr_x_value;

    __DMB();
    if(attribute_init_struct->attribute_number < MPU_ATTRIBUTE_NUMBER4) {
        /* configure MPU_MAIR0 */
        attr_values = MPU->MAIR0;
        attr_values &=  ~(0xFFUL << (attribute_init_struct->attribute_number * 8U));
        if(MPU_MEMORY_DEVICE == attribute_init_struct->memory_type) {
            attr_x_value = (attribute_init_struct->inner_attributes & 0x0CU);
        }else{
            attr_x_value = attribute_init_struct->inner_attributes | attribute_init_struct->outer_attributes;
        }
        attr_values |= ((uint32_t)attr_x_value  << (attribute_init_struct->attribute_number * 8U));
        MPU->MAIR0 = attr_values;
    } else {
        /* configure MPU_MAIR0 */
        attr_values = MPU->MAIR1;
        attr_values &=  ~(0xFFUL << ((attribute_init_struct->attribute_number -4U) * 8U));
        if(MPU_MEMORY_DEVICE == attribute_init_struct->memory_type) {
            attr_x_value = (attribute_init_struct->inner_attributes & 0x0CU);
        }else{
            attr_x_value = attribute_init_struct->inner_attributes | attribute_init_struct->outer_attributes;
        }
        attr_values |= ((uint32_t)attr_x_value  << ((attribute_init_struct->attribute_number - 4U) * 8U));
        MPU->MAIR1 = attr_values;
    }
}

/*!
    \brief      enable the MPU region
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mpu_region_enable(void)
{
    MPU->RLAR |= MPU_RLAR_EN_Msk;
}

#endif /* __MPU_PRESENT */
