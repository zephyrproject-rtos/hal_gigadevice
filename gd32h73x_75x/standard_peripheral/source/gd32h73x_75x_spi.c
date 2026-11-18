/*!
    \file    gd32h73x_75x_spi.c
    \brief   SPI driver

    \version 2026-07-30, V1.7.0, firmware for GD32H73x_75x
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

#include "gd32h73x_75x_spi.h"

/* external clock value to ckin */
#define I2S_CKIN_VALUE                  ((uint32_t)0U)

/* I2S parameter initialization mask */
#define I2S_INIT_MASK                   ((uint32_t)0xFFFFF047U)

/*!
    \brief      reset SPI and I2S (API_ID(0x0001U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_i2s_deinit(uint32_t spi_periph)
{
    switch(spi_periph) {
    case SPI0:
        /* reset SPI0 and I2S0 */
        rcu_periph_reset_enable(RCU_SPI0RST);
        rcu_periph_reset_disable(RCU_SPI0RST);
        break;
    case SPI1:
        /* reset SPI1 and I2S1 */
        rcu_periph_reset_enable(RCU_SPI1RST);
        rcu_periph_reset_disable(RCU_SPI1RST);
        break;
    case SPI2:
        /* reset SPI2 and I2S2 */
        rcu_periph_reset_enable(RCU_SPI2RST);
        rcu_periph_reset_disable(RCU_SPI2RST);
        break;
    case SPI3:
        /* reset SPI3 */
        rcu_periph_reset_enable(RCU_SPI3RST);
        rcu_periph_reset_disable(RCU_SPI3RST);
        break;
    case SPI4:
        /* reset SPI4 */
        rcu_periph_reset_enable(RCU_SPI4RST);
        rcu_periph_reset_disable(RCU_SPI4RST);
        break;
    case SPI5:
        /* reset SPI5 and I2S5 */
        rcu_periph_reset_enable(RCU_SPI5RST);
        rcu_periph_reset_disable(RCU_SPI5RST);
        break;
    default:
        break;
    }
}

/*!
    \brief      initialize the parameters of SPI struct with default values (API_ID(0x0002U))
    \param[in]  spi_struct: SPI parameter structure
    \param[out] none
    \retval     none
*/
void spi_struct_para_init(spi_parameter_struct *spi_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(spi_struct)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else
#endif
    {
        /* configure the structure with default value */
        spi_struct->device_mode          = SPI_SLAVE;
        spi_struct->trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
        spi_struct->data_size            = SPI_DATASIZE_8BIT;
        spi_struct->nss                  = SPI_NSS_HARD;
        spi_struct->endian               = SPI_ENDIAN_MSB;
        spi_struct->clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        spi_struct->prescale             = SPI_PSC_2;
    }
}

/*!
    \brief      initialize SPI parameter (API_ID(0x0003U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  spi_struct: SPI parameter initialization stuct members of the structure
                            and the member values are shown as below:
                  device_mode: SPI_MASTER, SPI_SLAVE
                  trans_mode: SPI_TRANSMODE_FULLDUPLEX, SPI_TRANSMODE_RECEIVEONLY,
                              SPI_TRANSMODE_BDRECEIVE, SPI_TRANSMODE_BDTRANSMIT
                  data_size: SPI_DATASIZE_4BIT, SPI_DATASIZE_5BIT, SPI_DATASIZE_6BIT
                             SPI_DATASIZE_7BIT, SPI_DATASIZE_8BIT, SPI_DATASIZE_9BIT
                             SPI_DATASIZE_10BIT, SPI_DATASIZE_11BIT, SPI_DATASIZE_12BIT
                             SPI_DATASIZE_13BIT, SPI_DATASIZE_14BIT, SPI_DATASIZE_15BIT
                             SPI_DATASIZE_16BIT, SPI_DATASIZE_17BIT, SPI_DATASIZE_18BIT
                             SPI_DATASIZE_19BIT, SPI_DATASIZE_20BIT, SPI_DATASIZE_21BIT
                             SPI_DATASIZE_22BIT, SPI_DATASIZE_23BIT, SPI_DATASIZE_24BIT
                             SPI_DATASIZE_25BIT, SPI_DATASIZE_26BIT, SPI_DATASIZE_27BIT
                             SPI_DATASIZE_28BIT, SPI_DATASIZE_29BIT, SPI_DATASIZE_30BIT
                             SPI_DATASIZE_31BIT, SPI_DATASIZE_32BIT
                  nss: SPI_NSS_SOFT, SPI_NSS_HARD
                  endian: SPI_ENDIAN_MSB, SPI_ENDIAN_LSB
                  clock_polarity_phase: SPI_CK_PL_LOW_PH_1EDGE, SPI_CK_PL_HIGH_PH_1EDGE
                                        SPI_CK_PL_LOW_PH_2EDGE, SPI_CK_PL_HIGH_PH_2EDGE
                  prescale: SPI_PSC_n (n=2,4,8,16,32,64,128,256)
    \param[out] none
    \retval     none
*/
void spi_init(uint32_t spi_periph, spi_parameter_struct *spi_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(spi_struct)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif
    {
        uint32_t reg1 = 0U, reg2 = 0U;
        reg1 = SPI_CFG0(spi_periph);
        reg2 = SPI_CFG1(spi_periph);

        reg1 &= (uint32_t)(~(SPI_CFG0_DZ | SPI_CFG0_PSC));
        reg2 &= (uint32_t)(~(SPI_CFG1_MSTMOD | SPI_CFG1_RO | SPI_CFG1_BDOEN | SPI_CFG1_BDEN |
                    SPI_CFG1_NSSIM | SPI_CFG1_LF | SPI_CFG1_CKPH | SPI_CFG1_CKPL));

        /* select SPI as master or slave */
        reg2 |= (spi_struct->device_mode & SPI_CFG1_MSTMOD);
        /* select SPI transfer mode */
        reg2 |= (spi_struct->trans_mode & (SPI_CFG1_RO | SPI_CFG1_BDEN | SPI_CFG1_BDOEN));
        /* select SPI frame size */
        reg1 |= (spi_struct->data_size & SPI_CFG0_DZ);
        /* select SPI NSS use hardware or software */
        reg2 |= (spi_struct->nss & SPI_CFG1_NSSIM);
        /* select SPI LSB or MSB */
        reg2 |= (spi_struct->endian & SPI_CFG1_LF);
        /* select SPI polarity and phase */
        reg2 |= (spi_struct->clock_polarity_phase & (SPI_CFG1_CKPL | SPI_CFG1_CKPH));
        /* select SPI prescale to adjust transmit speed */
        reg1 |= (spi_struct->prescale & SPI_CFG0_PSC);

        /* write to SPI_CFG0 & SPI_CFG1 register */
        SPI_CFG0(spi_periph) = (uint32_t)reg1;
        SPI_CFG1(spi_periph) = (uint32_t)reg2;

        SPI_I2SCTL(spi_periph) &= (uint32_t)(~SPI_I2SCTL_I2SSEL);
    }
}

/*!
    \brief      enable SPI (API_ID(0x0004U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_enable(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_SPIEN;
}

/*!
    \brief      disable SPI (API_ID(0x0005U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_disable(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_SPIEN);
}

/*!
    \brief      initialize I2S parameter (API_ID(0x0006U))
    \param[in]  spi_periph: SPIx(x=0,1,2,5)
    \param[in]  i2s_mode: I2S operation mode
                only one parameter can be selected which is shown as below:
      \arg        I2S_MODE_SLAVETX: I2S slave transmit mode
      \arg        I2S_MODE_SLAVERX: I2S slave receive mode
      \arg        I2S_MODE_MASTERTX: I2S master transmit mode
      \arg        I2S_MODE_MASTERRX: I2S master receive mode
    \param[in]  i2s_standard: I2S standard
                only one parameter can be selected which is shown as below:
      \arg        I2S_STD_PHILIPS: I2S philips standard
      \arg        I2S_STD_MSB: I2S MSB standard
      \arg        I2S_STD_LSB: I2S LSB standard
      \arg        I2S_STD_PCMSHORT: I2S PCM short standard
      \arg        I2S_STD_PCMLONG: I2S PCM long standard
    \param[in]  i2s_ckpl: I2S idle state clock polarity
                only one parameter can be selected which is shown as below:
      \arg        I2S_CKPL_LOW: I2S clock polarity low level
      \arg        I2S_CKPL_HIGH: I2S clock polarity high level
    \param[out] none
    \retval     none
*/
void i2s_init(uint32_t spi_periph, uint32_t i2s_mode, uint32_t i2s_standard, uint32_t i2s_ckpl)
{
    uint32_t reg = 0U;
    reg = SPI_I2SCTL(spi_periph);
    reg &= I2S_INIT_MASK;

    /* enable I2S mode */
    reg |= (uint32_t)SPI_I2SCTL_I2SSEL;
    /* select I2S mode */
    reg |= (uint32_t)(i2s_mode & SPI_I2SCTL_I2SOPMOD);
    /* select I2S standard */
    reg |= (uint32_t)(i2s_standard & (SPI_I2SCTL_I2SSTD | SPI_I2SCTL_PCMSMOD));
    /* select I2S polarity */
    reg |= (uint32_t)(i2s_ckpl & SPI_I2SCTL_CKPL);

    /* write to SPI_I2SCTL register */
    SPI_I2SCTL(spi_periph) = (uint32_t)reg;
}

/*!
    \brief      configure I2S prescaler (API_ID(0x0007U))
    \param[in]  spi_periph: SPIx(x=0,1,2,5)
    \param[in]  i2s_audiosample: I2S audio sample rate
                only one parameter can be selected which is shown as below:
      \arg        I2S_AUDIOSAMPLE_8K: audio sample rate is 8KHz
      \arg        I2S_AUDIOSAMPLE_11K: audio sample rate is 11KHz
      \arg        I2S_AUDIOSAMPLE_16K: audio sample rate is 16KHz
      \arg        I2S_AUDIOSAMPLE_22K: audio sample rate is 22KHz
      \arg        I2S_AUDIOSAMPLE_32K: audio sample rate is 32KHz
      \arg        I2S_AUDIOSAMPLE_44K: audio sample rate is 44KHz
      \arg        I2S_AUDIOSAMPLE_48K: audio sample rate is 48KHz
      \arg        I2S_AUDIOSAMPLE_96K: audio sample rate is 96KHz
      \arg        I2S_AUDIOSAMPLE_192K: audio sample rate is 192KHz
    \param[in]  i2s_frameformat: I2S data length and channel length
                only one parameter can be selected which is shown as below:
      \arg        I2S_FRAMEFORMAT_DT16B_CH16B: I2S data length is 16 bit and channel length is 16 bit
      \arg        I2S_FRAMEFORMAT_DT16B_CH32B: I2S data length is 16 bit and channel length is 32 bit
      \arg        I2S_FRAMEFORMAT_DT24B_CH32B: I2S data length is 24 bit and channel length is 32 bit
      \arg        I2S_FRAMEFORMAT_DT32B_CH32B: I2S data length is 32 bit and channel length is 32 bit
    \param[in]  i2s_mckout: I2S master clock output
                only one parameter can be selected which is shown as below:
      \arg        I2S_MCKOUT_ENABLE: I2S master clock output enable
      \arg        I2S_MCKOUT_DISABLE: I2S master clock output disable
    \param[out] none
    \retval     none
*/
void i2s_psc_config(uint32_t spi_periph, uint32_t i2s_audiosample, uint32_t i2s_frameformat, uint32_t i2s_mckout)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(0U == i2s_audiosample) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x0007U), ERR_PARAM_INVALID);
    } else
#endif
    {
        uint32_t i2sdiv = 2U, i2sof = 0U;
        uint32_t clks = 0U;
        uint32_t i2sclock = 0U;
        uint32_t i2s_clk_sel = 0U;
        uint32_t spi0_2_clksel[5] = {CK_PLL0Q, CK_PLL1P, CK_PLL2P, I2S_CKIN_VALUE, CK_PER};
        uint32_t spi3_5_clksel[7] = {CK_APB2, CK_PLL1Q, CK_PLL2Q, CK_IRC64MDIV, CK_LPIRC4M, CK_HXTAL, I2S_CKIN_VALUE};

        /* get the I2S clock source */
        switch(spi_periph) {
        case SPI0:
            /* I2S0 clock source selection */
            i2s_clk_sel = RCU_CFG5 & RCU_CFG5_SPI0SEL;
            if(3U == i2s_clk_sel) {
                i2sclock = spi0_2_clksel[i2s_clk_sel];
            } else {
                i2sclock = rcu_clock_freq_get((rcu_clock_freq_enum)spi0_2_clksel[i2s_clk_sel]);
            }
            break;
        case SPI1:
            /* I2S1 clock source selection */
            i2s_clk_sel = (RCU_CFG5 & RCU_CFG5_SPI1SEL) >> 4U;
            if(3U == i2s_clk_sel) {
                i2sclock = spi0_2_clksel[i2s_clk_sel];
            } else {
                i2sclock = rcu_clock_freq_get((rcu_clock_freq_enum)spi0_2_clksel[i2s_clk_sel]);
            }
            break;
        case SPI2:
            /* I2S2 clock source selection */
            i2s_clk_sel = (RCU_CFG5 & RCU_CFG5_SPI2SEL) >> 8U;
            if(3U == i2s_clk_sel) {
                i2sclock = spi0_2_clksel[i2s_clk_sel];
            } else {
                i2sclock = rcu_clock_freq_get((rcu_clock_freq_enum)spi0_2_clksel[i2s_clk_sel]);
            }
            break;
        case SPI5:
            /* I2S5 clock source selection */
            i2s_clk_sel = (RCU_CFG5 & RCU_CFG5_SPI5SEL) >> 20U;
            if(6U == i2s_clk_sel) {
                i2sclock = spi3_5_clksel[i2s_clk_sel];
            } else {
                i2sclock = rcu_clock_freq_get((rcu_clock_freq_enum)spi3_5_clksel[i2s_clk_sel]);
            }
            break;
        default :
            break;
        }

        /* config the prescaler depending on the mclk output state, the frame format and audio sample rate */
        if(I2S_MCKOUT_ENABLE == i2s_mckout) {
            clks = (uint32_t)(((i2sclock / 256U) * 10U) / i2s_audiosample);
        } else {
            if(I2S_FRAMEFORMAT_DT16B_CH16B == i2s_frameformat) {
                clks = (uint32_t)(((i2sclock / 32U) * 10U) / i2s_audiosample);
            } else {
                clks = (uint32_t)(((i2sclock / 64U) * 10U) / i2s_audiosample);
            }
        }

        /* remove the floating point */
        clks   = (clks + 5U) / 10U;
        i2sof  = (clks & 0x00000001U);
        i2sdiv = ((clks - i2sof) / 2U);

        /* set the default values */
        if((i2sdiv < 2U) || (i2sdiv > 255U)) {
            i2sdiv = 2U;
            i2sof = 0U;
        }
        /* clear SPI_I2SCTL_DIV and SPI_I2SCTL_OF and SPI_I2SCTL_MCKOEN bits */
        SPI_I2SCTL(spi_periph) &= (uint32_t)(~(SPI_I2SCTL_DIV | SPI_I2SCTL_OF | SPI_I2SCTL_MCKOEN));
        /* configure SPI_I2SPSC */
        SPI_I2SCTL(spi_periph) |= (uint32_t)((i2sdiv << 16U) | (i2sof << 24U) | (i2s_mckout & SPI_I2SCTL_MCKOEN));

        /* clear SPI_I2SCTL_DTLEN and SPI_I2SCTL_CHLEN bits */
        SPI_I2SCTL(spi_periph) &= (uint32_t)(~(SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN));
        /* configure data frame format */
        SPI_I2SCTL(spi_periph) |= (uint32_t)(i2s_frameformat & (SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN));
    }
}

/*!
    \brief      enable I2S (API_ID(0x0008U))
    \param[in]  spi_periph: SPIx(x=0,1,2,5)
    \param[out] none
    \retval     none
*/
void i2s_enable(uint32_t spi_periph)
{
    SPI_I2SCTL(spi_periph) |= (uint32_t)SPI_I2SCTL_I2SEN;
}

/*!
    \brief      disable I2S (API_ID(0x0009U))
    \param[in]  spi_periph: SPIx(x=0,1,2,5)
    \param[out] none
    \retval     none
*/
void i2s_disable(uint32_t spi_periph)
{
    SPI_I2SCTL(spi_periph) &= (uint32_t)(~SPI_I2SCTL_I2SEN);
}

/*!
    \brief      SPI MOSI and MISO pin swap (API_ID(0x000AU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  io_cfg: SPI IO swap config
                only one parameter can be selected which is shown as below:
      \arg        SPI_IO_SWAP: SPI MOSI and MISO swap
      \arg        SPI_IO_NORMAL: SPI MOSI and MISO no swap
    \param[out] none
    \retval     none
*/
void spi_io_config(uint32_t spi_periph, uint32_t io_cfg)
{
    if(SPI_IO_SWAP == io_cfg) {
        SPI_CFG1(spi_periph) |= (uint32_t)SPI_CFG1_SWPMIO;
    } else if(SPI_IO_NORMAL == io_cfg) {
        SPI_CFG1(spi_periph) &= (uint32_t)(~SPI_CFG1_SWPMIO);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      set delay between active edge of NSS and start transfer or receive data in SPI master mode (API_ID(0x000BU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  delay_cycle: SPI_NSS_IDLENESS_00CYCLE, SPI_NSS_IDLENESS_01CYCLE, SPI_NSS_IDLENESS_02CYCLE,
                             SPI_NSS_IDLENESS_03CYCLE, SPI_NSS_IDLENESS_04CYCLE, SPI_NSS_IDLENESS_05CYCLE,
                             SPI_NSS_IDLENESS_06CYCLE, SPI_NSS_IDLENESS_07CYCLE, SPI_NSS_IDLENESS_08CYCLE,
                             SPI_NSS_IDLENESS_09CYCLE, SPI_NSS_IDLENESS_10CYCLE, SPI_NSS_IDLENESS_11CYCLE,
                             SPI_NSS_IDLENESS_12CYCLE, SPI_NSS_IDLENESS_13CYCLE, SPI_NSS_IDLENESS_14CYCLE,
                             SPI_NSS_IDLENESS_15CYCLE
    \param[out] none
    \retval     none
*/
void spi_nss_idleness_delay_set(uint32_t spi_periph, uint32_t delay_cycle)
{
    /* acquire SPI_CFG1 register */
    uint32_t reg = SPI_CFG1(spi_periph);
    reg &= (uint32_t)(~SPI_CFG1_MSSD);
    reg |= (uint32_t)(delay_cycle & SPI_CFG1_MSSD);
    /* assign regiser */
    SPI_CFG1(spi_periph) = reg;
}

/*!
    \brief      set SPI master data frame delay (API_ID(0x000CU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  delay_cycle: SPI_DATA_IDLENESS_00CYCLE, SPI_DATA_IDLENESS_01CYCLE, SPI_DATA_IDLENESS_02CYCLE,
                             SPI_DATA_IDLENESS_03CYCLE, SPI_DATA_IDLENESS_04CYCLE, SPI_DATA_IDLENESS_05CYCLE,
                             SPI_DATA_IDLENESS_06CYCLE, SPI_DATA_IDLENESS_07CYCLE, SPI_DATA_IDLENESS_08CYCLE,
                             SPI_DATA_IDLENESS_09CYCLE, SPI_DATA_IDLENESS_10CYCLE, SPI_DATA_IDLENESS_11CYCLE,
                             SPI_DATA_IDLENESS_12CYCLE, SPI_DATA_IDLENESS_13CYCLE, SPI_DATA_IDLENESS_14CYCLE,
                             SPI_DATA_IDLENESS_15CYCLE
    \param[out] none
    \retval     none
*/
void spi_data_frame_delay_set(uint32_t spi_periph, uint32_t delay_cycle)
{
    /* acquire SPI_CFG1 register */
    uint32_t reg = SPI_CFG1(spi_periph);
    reg &= (uint32_t)(~SPI_CFG1_MDFD);
    reg |= (uint32_t)(delay_cycle & SPI_CFG1_MDFD);
    /* assign regiser */
    SPI_CFG1(spi_periph) = reg;
}

/*!
    \brief      set SPI master mode rx clock delay (API_ID(0x000DU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  delay_unit: master mode receive clock delay
                             it can be 0-0x1f
    \param[out] none
    \retval     none
*/
void spi_master_receive_clock_delay_set(uint32_t spi_periph, uint32_t delay_unit)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_SPI_RX_CLK_NUM(delay_unit)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x000DU), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        SPI_RXDLYCK(spi_periph) &= (uint32_t)(~SPI_RXDLYCK_MRXDEN);
        SPI_RXDLYCK(spi_periph) &= (uint32_t)(~SPI_RXDLYCK_MRXD);
        SPI_RXDLYCK(spi_periph) |= (uint32_t)((delay_unit << 6U) & SPI_RXDLYCK_MRXD);
    }
}

/*!
    \brief      set SPI slave mode rx clock delay (API_ID(0x000EU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  delay_unit: slave mode receive clock delay
                             it can be 0-0x1f
    \param[out] none
    \retval     none
*/
void spi_slave_receive_clock_delay_set(uint32_t spi_periph, uint32_t delay_unit)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_SPI_RX_CLK_NUM(delay_unit)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x000EU), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        SPI_RXDLYCK(spi_periph) &= (uint32_t)(~SPI_RXDLYCK_SRXDEN);
        SPI_RXDLYCK(spi_periph) &= (uint32_t)(~SPI_RXDLYCK_SRXD);
        SPI_RXDLYCK(spi_periph) |= (uint32_t)(delay_unit & SPI_RXDLYCK_SRXD);
    }
}

/*!
    \brief      clear SPI master mode rx clock delay (API_ID(0x000FU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_master_receive_clock_delay_clear(uint32_t spi_periph)
{
    SPI_RXDLYCK(spi_periph) |= (uint32_t)SPI_RXDLYCK_MRXDEN;
}

/*!
    \brief      clear SPI slave mode rx clock delay (API_ID(0x0010U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_slave_receive_clock_delay_clear(uint32_t spi_periph)
{
    SPI_RXDLYCK(spi_periph) |= (uint32_t)SPI_RXDLYCK_SRXDEN;
}

/*!
    \brief      SPI NSS output control (API_ID(0x0011U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  nss_ctl: nss control bit
                only one parameter can be selected which is shown as below:
      \arg        SPI_NSS_HOLD_UNTIL_TRANS_END: SPI NSS remains active level until data transfer complete
      \arg        SPI_NSS_INVALID_PULSE: SPI data frames are interleaved with NSS invalid pulses
    \param[out] none
    \retval     none
*/
void spi_nss_output_control(uint32_t spi_periph, uint32_t nss_ctl)
{
    if(SPI_NSS_HOLD_UNTIL_TRANS_END == nss_ctl) {
        SPI_CFG1(spi_periph) &= (uint32_t)(~SPI_CFG1_NSSCTL);
    } else if(SPI_NSS_INVALID_PULSE == nss_ctl) {
        SPI_CFG1(spi_periph) |= (uint32_t)SPI_CFG1_NSSCTL;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      set SPI NSS active polarity (API_ID(0x0012U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  polarity: SPI NSS active level
                only one parameter can be selected which is shown as below:
      \arg        SPI_NSS_POLARITY_HIGH: SPI NSS high level is active
      \arg        SPI_NSS_POLARITY_LOW: SPI NSS low level is active
    \param[out] none
    \retval     none
*/
void spi_nss_polarity_set(uint32_t spi_periph, uint32_t polarity)
{
    if(SPI_NSS_POLARITY_HIGH == polarity) {
        SPI_CFG1(spi_periph) |= (uint32_t)SPI_CFG1_NSSIOPL;
    } else if(SPI_NSS_POLARITY_LOW == polarity) {
        SPI_CFG1(spi_periph) &= (uint32_t)(~SPI_CFG1_NSSIOPL);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      enable SPI NSS output (API_ID(0x0013U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_nss_output_enable(uint32_t spi_periph)
{
    SPI_CFG1(spi_periph) |= (uint32_t)SPI_CFG1_NSSDRV;
}

/*!
    \brief      disable SPI NSS output (API_ID(0x0014U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_nss_output_disable(uint32_t spi_periph)
{
    SPI_CFG1(spi_periph) &= (uint32_t)(~SPI_CFG1_NSSDRV);
}

/*!
    \brief      SPI NSS pin high level in software mode (API_ID(0x0015U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_nss_internal_high(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_NSSI;
}

/*!
    \brief      SPI NSS pin low level in software mode (API_ID(0x0016U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_nss_internal_low(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_NSSI);
}

/*!
    \brief      enable SPI DMA send or receive (API_ID(0x0017U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  spi_dma: SPI DMA mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_DMA_TRANSMIT: SPI transmit data use DMA
      \arg        SPI_DMA_RECEIVE: SPI receive data use DMA
    \param[out] none
    \retval     none
*/
void spi_dma_enable(uint32_t spi_periph, uint8_t spi_dma)
{
    if(SPI_DMA_TRANSMIT == spi_dma) {
        SPI_CFG0(spi_periph) |= (uint32_t)SPI_CFG0_DMATEN;
    } else if(SPI_DMA_RECEIVE == spi_dma) {
        SPI_CFG0(spi_periph) |= (uint32_t)SPI_CFG0_DMAREN;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      disable SPI DMA send or receive (API_ID(0x0018U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  spi_dma: SPI DMA mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_DMA_TRANSMIT: SPI transmit data use DMA
      \arg        SPI_DMA_RECEIVE: SPI receive data use DMA
    \param[out] none
    \retval     none
*/
void spi_dma_disable(uint32_t spi_periph, uint8_t spi_dma)
{
    if(SPI_DMA_TRANSMIT == spi_dma) {
        SPI_CFG0(spi_periph) &= (uint32_t)(~SPI_CFG0_DMATEN);
    } else if(SPI_DMA_RECEIVE == spi_dma) {
        SPI_CFG0(spi_periph) &= (uint32_t)(~SPI_CFG0_DMAREN);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure SPI/I2S data frame size (API_ID(0x0019U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  frame_size: SPI_DATASIZE_4BIT, SPI_DATASIZE_5BIT, SPI_DATASIZE_6BIT,
                            SPI_DATASIZE_7BIT, SPI_DATASIZE_8BIT, SPI_DATASIZE_9BIT,
                            SPI_DATASIZE_10BIT, SPI_DATASIZE_11BIT, SPI_DATASIZE_12BIT,
                            SPI_DATASIZE_13BIT, SPI_DATASIZE_14BIT, SPI_DATASIZE_15BIT,
                            SPI_DATASIZE_16BIT, SPI_DATASIZE_17BIT, SPI_DATASIZE_18BIT,
                            SPI_DATASIZE_19BIT, SPI_DATASIZE_20BIT, SPI_DATASIZE_21BIT,
                            SPI_DATASIZE_22BIT, SPI_DATASIZE_23BIT, SPI_DATASIZE_24BIT,
                            SPI_DATASIZE_25BIT, SPI_DATASIZE_26BIT, SPI_DATASIZE_27BIT,
                            SPI_DATASIZE_28BIT, SPI_DATASIZE_29BIT, SPI_DATASIZE_30BIT,
                            SPI_DATASIZE_31BIT, SPI_DATASIZE_32BIT
    \param[out] none
    \retval     none
*/
void spi_i2s_data_frame_size_config(uint32_t spi_periph, uint32_t frame_size)
{
    uint32_t reg = SPI_CFG0(spi_periph);
    /* confige SPI_CFG0_DZ bits */
    reg &= (uint32_t)(~SPI_CFG0_DZ);
    reg |= (uint32_t)(frame_size & SPI_CFG0_DZ);
    SPI_CFG0(spi_periph) = reg;
}

/*!
    \brief      SPI/I2S transmit data (API_ID(0x001AU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  data: 32-bit data
    \param[out] none
    \retval     none
*/
void spi_i2s_data_transmit(uint32_t spi_periph, uint32_t data)
{
    SPI_TDATA(spi_periph) = (uint32_t)data;
}

/*!
    \brief      SPI/I2S receive data (API_ID(0x001BU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     32-bit data
*/
uint32_t spi_i2s_data_receive(uint32_t spi_periph)
{
    uint32_t reval = 0U ;
    reval = ((uint32_t)SPI_RDATA(spi_periph));
    return reval;

}

/*!
    \brief      configure SPI bidirectional transfer direction (API_ID(0x001CU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  transfer_direction: SPI transfer direction
                only one parameter can be selected which is shown as below:
      \arg        SPI_BIDIRECTIONAL_TRANSMIT: SPI work in transmit-only mode
      \arg        SPI_BIDIRECTIONAL_RECEIVE: SPI work in receive-only mode
    \param[out] none
    \retval     none
*/
void spi_bidirectional_transfer_config(uint32_t spi_periph, uint32_t transfer_direction)
{
    if(SPI_BIDIRECTIONAL_TRANSMIT == transfer_direction) {
        /* set the transmit only mode */
        SPI_CFG1(spi_periph) |= (uint32_t)SPI_BIDIRECTIONAL_TRANSMIT;
    } else if(SPI_BIDIRECTIONAL_RECEIVE == transfer_direction) {
        /* set the receive only mode */
        SPI_CFG1(spi_periph) &= (uint32_t)(~SPI_CFG1_BDOEN);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      SPI/I2S master start transfer (API_ID(0x001DU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  transfer_start: transfer start bit
                only one parameter can be selected which is shown as below:
      \arg        SPI_TRANS_START: the master transmission is occurring,
                                   or has been temporarily suspended by automatic suspend
      \arg        SPI_TRANS_IDLE: the master transfer is idle status
    \param[out] none
    \retval     none
*/
void spi_master_transfer_start(uint32_t spi_periph, uint32_t transfer_start)
{
    if(SPI_TRANS_START == transfer_start) {
        SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_MSTART;
    } else if(SPI_TRANS_IDLE == transfer_start) {
        SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_MSTART);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief     configure SPI current data number (API_ID(0x001EU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  current_num: SPI transfer current data number
                             it can be 0-0xFFFF
    \param[out] none
    \retval     none
*/
void spi_current_data_num_config(uint32_t spi_periph, uint32_t current_num)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_SPI_TRANS_NUM(current_num)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x001EU), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        uint32_t reg = SPI_CTL1(spi_periph);
        /* confige SPI current data number */
        reg &= (uint32_t)(~BITS(0, 15));
        reg |= (uint32_t)(current_num & BITS(0, 15));
        SPI_CTL1(spi_periph) = reg;
    }
}

/*!
    \brief      configure SPI reload data number (API_ID(0x001FU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  reload_num: SPI transfer reload data number
                             it can be 0-0xFFFF
    \param[out] none
    \retval     none
*/
void spi_reload_data_num_config(uint32_t spi_periph, uint32_t reload_num)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_SPI_RELOAD_NUM(reload_num)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x001FU), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        uint32_t reg = SPI_CTL1(spi_periph);
        /* confige SPI reload data number */
        reg &= (uint32_t)(~BITS(16, 31));
        reg |= (uint32_t)((reload_num << 16U) & BITS(16, 31));
        SPI_CTL1(spi_periph) = reg;
    }
}

/*!
    \brief      set SPI CRC polynomial (API_ID(0x0020U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  crc_poly: CRC polynomial value
    \param[out] none
    \retval     none
*/
void spi_crc_polynomial_set(uint32_t spi_periph, uint32_t crc_poly)
{
    /* enable SPI CRC */
    SPI_CFG0(spi_periph) |= (uint32_t)SPI_CFG0_CRCEN;
    /* set SPI CRC polynomial */
    SPI_CRCPOLY(spi_periph) = (uint32_t)crc_poly;
}

/*!
    \brief      get SPI CRC polynomial (API_ID(0x0021U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     32-bit CRC polynomial
*/
uint32_t spi_crc_polynomial_get(uint32_t spi_periph)
{
    uint32_t reval = 0U ;
    reval = ((uint32_t)SPI_CRCPOLY(spi_periph));
    return reval;
}

/*!
    \brief      configure SPI CRC length (API_ID(0x0022U))
    \param[in]  spi_periph: (x=0,1,2,3,4,5)
    \param[in]  crc_size: SPI_CRCSIZE_4BIT, SPI_CRCSIZE_5BIT, SPI_CRCSIZE_6BIT,
                          SPI_CRCSIZE_7BIT, SPI_CRCSIZE_8BIT, SPI_CRCSIZE_9BIT, SPI_CRCSIZE_10BIT,
                          SPI_CRCSIZE_11BIT, SPI_CRCSIZE_12BIT, SPI_CRCSIZE_13BIT, SPI_CRCSIZE_14BIT,
                          SPI_CRCSIZE_15BIT, SPI_CRCSIZE_16BIT, SPI_CRCSIZE_17BIT, SPI_CRCSIZE_18BIT,
                          SPI_CRCSIZE_19BIT, SPI_CRCSIZE_20BIT, SPI_CRCSIZE_21BIT, SPI_CRCSIZE_22BIT,
                          SPI_CRCSIZE_23BIT, SPI_CRCSIZE_24BIT, SPI_CRCSIZE_25BIT, SPI_CRCSIZE_26BIT,
                          SPI_CRCSIZE_27BIT, SPI_CRCSIZE_28BIT, SPI_CRCSIZE_29BIT, SPI_CRCSIZE_30BIT,
                          SPI_CRCSIZE_31BIT, SPI_CRCSIZE_32BIT
    \param[out] none
    \retval     none
*/
void spi_crc_length_config(uint32_t spi_periph, uint32_t crc_size)
{
    uint32_t reg = SPI_CFG0(spi_periph);
    /* confige SPI_CFG0_CRCSZ bits */
    reg &= (uint32_t)(~SPI_CFG0_CRCSZ);
    reg |= (uint32_t)(crc_size & SPI_CFG0_CRCSZ);
    SPI_CFG0(spi_periph) = reg;
}

/*!
    \brief      turn on CRC function (API_ID(0x0023U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_crc_on(uint32_t spi_periph)
{
    SPI_CFG0(spi_periph) |= (uint32_t)SPI_CFG0_CRCEN;
}

/*!
    \brief      turn off CRC function (API_ID(0x0024U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_crc_off(uint32_t spi_periph)
{
    SPI_CFG0(spi_periph) &= (uint32_t)(~SPI_CFG0_CRCEN);
}

/*!
    \brief      get SPI CRC send value or receive value (API_ID(0x0025U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  crc: SPI crc value
                only one parameter can be selected which is shown as below:
      \arg        SPI_CRC_TX: get transmit crc value
      \arg        SPI_CRC_RX: get receive crc value
    \param[out] none
    \retval     32-bit CRC value
*/
uint32_t spi_crc_get(uint32_t spi_periph, uint8_t crc)
{
    uint32_t reval = 0U;
    if(SPI_CRC_TX == crc) {
        reval = ((uint32_t)(SPI_TCRC(spi_periph)));
    } else if(SPI_CRC_RX == crc) {
        reval = ((uint32_t)(SPI_RCRC(spi_periph)));
    } else {
        /* illegal parameters */
    }
    return reval;
}

/*!
    \brief      enable SPI CRC full size(33 bit or 17 bit) polynomial (API_ID(0x0026U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_crc_full_size_enable(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_CRCFS;
}

/*!
    \brief      disable SPI CRC full size(33 bit or 17 bit) polynomial (API_ID(0x0027U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_crc_full_size_disable(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_CRCFS);
}

/*!
    \brief      configure SPI TCRC init pattern (API_ID(0x0028U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  init_pattern: SPI crc value
                only one parameter can be selected which is shown as below:
      \arg        SPI_TCRC_INIT_1: use all 1 pattern
      \arg        SPI_TCRC_INIT_0: use all 0 pattern
    \param[out] none
    \retval     none
*/
void spi_tcrc_init_pattern(uint32_t spi_periph, uint32_t init_pattern)
{
    if(SPI_TCRC_INIT_1 == init_pattern) {
        SPI_CTL0(spi_periph) |= (uint32_t)(SPI_CTL0_TXCRCI);
    } else if(SPI_TCRC_INIT_0 == init_pattern) {
        SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_TXCRCI);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure SPI RCRC init pattern (API_ID(0x0029U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  init_pattern: SPI crc value
                only one parameter can be selected which is shown as below:
      \arg        SPI_RCRC_INIT_1: use all 1 pattern
      \arg        SPI_RCRC_INIT_0: use all 0 pattern
    \param[out] none
    \retval     none
*/
void spi_rcrc_init_pattern(uint32_t spi_periph, uint32_t init_pattern)
{
    if(SPI_RCRC_INIT_1 == init_pattern) {
        SPI_CTL0(spi_periph) |= (uint32_t)(SPI_CTL0_RXCRCI);
    } else if(SPI_RCRC_INIT_0 == init_pattern) {
        SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_RXCRCI);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      enable SPI TI mode (API_ID(0x002AU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_ti_mode_enable(uint32_t spi_periph)
{
    SPI_CFG1(spi_periph) |= (uint32_t)SPI_CFG1_TMOD;
}

/*!
    \brief      disable SPI TI mode (API_ID(0x002BU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_ti_mode_disable(uint32_t spi_periph)
{
    SPI_CFG1(spi_periph) &= (uint32_t)(~SPI_CFG1_TMOD);
}

/*!
    \brief      enable quad wire SPI (API_ID(0x002CU))
    \param[in]  spi_periph: SPIx(x=3,4)
    \param[out] none
    \retval     none
*/
void spi_quad_enable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) |= (uint32_t)SPI_QCTL_QMOD;
}

/*!
    \brief      disable quad wire SPI (API_ID(0x002DU))
    \param[in]  spi_periph: SPIx(x=3,4)
    \param[out] none
    \retval     none
*/
void spi_quad_disable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) &= (uint32_t)(~SPI_QCTL_QMOD);
}

/*!
    \brief      enable quad wire SPI write (API_ID(0x002EU))
    \param[in]  spi_periph: SPIx(x=3,4)
    \param[out] none
    \retval     none
*/
void spi_quad_write_enable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) &= (uint32_t)(~SPI_QCTL_QRD);
}

/*!
    \brief      enable quad wire SPI read (API_ID(0x002FU))
    \param[in]  spi_periph: SPIx(x=3,4)
    \param[out] none
    \retval     none
*/
void spi_quad_read_enable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) |= (uint32_t)SPI_QCTL_QRD;
}

/*!
    \brief      slave transmitter underrun detected operation (API_ID(0x0030U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  ur_ope: underrun operation
                only one parameter can be selected which is shown as below:
      \arg        SPI_CONFIG_REGISTER_PATTERN: slave send a constant value defined by the SPI_URDATA register
      \arg        SPI_CONFIG_LAST_RECEIVED: slave send the lastly data frame received from master
      \arg        SPI_CONFIG_LAST_TRANSMITTED: slave send its lastly transmitted data frame
    \param[out] none
    \retval     none
*/
void spi_underrun_operation(uint32_t spi_periph, uint32_t ur_ope)
{
    /* acquire SPI_CFG0 register */
    uint32_t reg = SPI_CFG0(spi_periph);
    reg &= (uint32_t)(~SPI_CFG0_TXUROP);
    reg |= (uint32_t)(ur_ope & SPI_CFG0_TXUROP);
    /* assign regiser */
    SPI_CFG0(spi_periph) = reg;
}

/*!
    \brief      configure slave transmitter underrun detected (API_ID(0x0031U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  ur_cfg: underrun config
                only one parameter can be selected which is shown as below:
      \arg        SPI_DETECT_BEGIN_DATA_FRAME: underrun detected at start of data frame (no bit 1 protection)
      \arg        SPI_DETECT_END_DATA_FRAME: underrun detected at end of last data frame
      \arg        SPI_DETECT_BEGIN_ACTIVE_NSS: underrun detected at start of NSS signal
    \param[out] none
    \retval     none
*/
void spi_underrun_config(uint32_t spi_periph, uint32_t ur_cfg)
{
    /* acquire SPI_CFG0 register */
    uint32_t reg = SPI_CFG0(spi_periph);
    reg &= (uint32_t)(~SPI_CFG0_TXURDT);
    reg |= (uint32_t)(ur_cfg & SPI_CFG0_TXURDT);
    /* assign regiser */
    SPI_CFG0(spi_periph) = reg;
}

/*!
    \brief      configure underrun data at slave mode (API_ID(0x0032U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  udata: underrun data
                       it can be 0-0xFFFFFFFF
    \param[out] none
    \retval     none
*/
void spi_underrun_data_config(uint32_t spi_periph, uint32_t udata)
{
    SPI_URDATA(spi_periph) = (uint32_t)udata;
}

/*!
    \brief      configure SPI suspend in receive mode (API_ID(0x0033U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  sus_mode: suspend mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_AUTO_SUSPEND: until the overrun condition is reached,
                                    the SPI stream is suspended in the full RxFIFO state
      \arg        SPI_CONTINUOUS: SPI stream/clock generation is continuous whether or not an overrun occurs
    \param[out] none
    \retval     none
*/
void spi_suspend_mode_config(uint32_t spi_periph, uint32_t sus_mode)
{
    if(SPI_AUTO_SUSPEND == sus_mode) {
        SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_MASP;
    } else if(SPI_CONTINUOUS == sus_mode) {
        SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_MASP);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      SPI master mode suspend request (API_ID(0x0034U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_suspend_request(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_MSPDR;
}

/*!
    \brief     enable SPI related IOs AF (API_ID(0x0035U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_related_ios_af_enable(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_IOAFEN);
}

/*!
    \brief      disable SPI related IOs AF (API_ID(0x0036U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_related_ios_af_disable(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_IOAFEN;
}

/*!
    \brief      SPI af gpio control (API_ID(0x0037U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  ctl: gpio control bit
                only one parameter can be selected which is shown as below:
      \arg        SPI_GPIO_CONTROL: SPI always control all associated GPIO
      \arg        SPI_GPIO_FREE: SPI do not control GPIO when disabled
    \param[out] none
    \retval     none
*/
void spi_af_gpio_control(uint32_t spi_periph, uint32_t ctl)
{
    if(SPI_GPIO_CONTROL == ctl) {
        SPI_CFG1(spi_periph) |= (uint32_t)SPI_CFG1_AFCTL;
    } else if(SPI_GPIO_FREE == ctl) {
        SPI_CFG1(spi_periph) &= (uint32_t)(~SPI_CFG1_AFCTL);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      enable SPI and I2S interrupt (API_ID(0x0038U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  interrupt: SPI/I2S interrupt
                one or more parameters can be selected which are shown as below:
      \arg        SPI_I2S_INT_RP: RP interrupt
      \arg        SPI_I2S_INT_TP: TP interrupt
      \arg        SPI_I2S_INT_DP: DP interrupt
      \arg        SPI_I2S_INT_ESTC: end of transfer or suspend or TxFIFO clear interrupt
      \arg        SPI_I2S_INT_TXF: transmission filled interrupt
      \arg        SPI_I2S_INT_TXURE: underrun error interrupt
      \arg        SPI_I2S_INT_RXORE: overrun error interrupt
      \arg        SPI_I2S_INT_CRCER: CRC error interrupt
      \arg        SPI_I2S_INT_FE: TI frame error interrupt
      \arg        SPI_I2S_INT_CONFE: mode error interrupt
      \arg        SPI_I2S_INT_TXSERF: TXSER reload interrupt
    \param[out] none
    \retval     none
*/
void spi_i2s_interrupt_enable(uint32_t spi_periph, uint32_t interrupt)
{
    SPI_INT(spi_periph) |= (uint32_t)(interrupt & SPI_I2S_INT_MASK);
}

/*!
    \brief      disable SPI and I2S interrupt (API_ID(0x0039U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  interrupt: SPI/I2S interrupt
                one or more parameters can be selected which are shown as below:
      \arg        SPI_I2S_INT_RP: RP interrupt
      \arg        SPI_I2S_INT_TP: TP interrupt
      \arg        SPI_I2S_INT_DP: DP interrupt
      \arg        SPI_I2S_INT_ESTC: end of transfer or suspend or TXFIFO clear interrupt
      \arg        SPI_I2S_INT_TXF: transmission filled interrupt
      \arg        SPI_I2S_INT_TXURE: underrun error interrupt
      \arg        SPI_I2S_INT_RXORE: overrun error interrupt
      \arg        SPI_I2S_INT_CRCER: CRC error interrupt
      \arg        SPI_I2S_INT_FE: TI frame error interrupt
      \arg        SPI_I2S_INT_CONFE: mode error interrupt
      \arg        SPI_I2S_INT_TXSERF: TXSER reload interrupt
    \param[out] none
    \retval     none
*/
void spi_i2s_interrupt_disable(uint32_t spi_periph, uint32_t interrupt)
{
    SPI_INT(spi_periph) &= (uint32_t)(~(interrupt & SPI_I2S_INT_MASK));
}

/*!
    \brief      get SPI and I2S interrupt flag status (API_ID(0x003AU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  interrupt: SPI/I2S interrupt flag status
                one or more parameters can be selected which are shown as below:
      \arg        SPI_I2S_INT_FLAG_RP: RP interrupt flag
      \arg        SPI_I2S_INT_FLAG_TP: TP interrupt flag
      \arg        SPI_I2S_INT_FLAG_DP: DP interrupt flag
      \arg        SPI_I2S_INT_FLAG_ET: end of transfer or receive interrupt flag
      \arg        SPI_I2S_INT_FLAG_TXF: transmission filled interrupt flag
      \arg        SPI_I2S_INT_FLAG_TXURERR: underrun error interrupt flag
      \arg        SPI_I2S_INT_FLAG_RXORERR: overrun error interrupt flag
      \arg        SPI_I2S_INT_FLAG_CRCERR: CRC error interrupt flag
      \arg        SPI_I2S_INT_FLAG_FERR: TI frame error interrupt flag
      \arg        SPI_I2S_INT_FLAG_CONFERR: mode error interrupt flag
      \arg        SPI_I2S_INT_FLAG_TXSERF: TXSER reload interrupt flag
      \arg        SPI_I2S_INT_FLAG_SPD: suspend interrupt flag
      \arg        SPI_I2S_INT_FLAG_TC: TxFIFO clear interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus spi_i2s_interrupt_flag_get(uint32_t spi_periph, uint32_t interrupt)
{
    FlagStatus reval = RESET;
    uint32_t int_flag = interrupt & SPI_I2S_INT_FLAG_MASK;
    uint32_t stat = SPI_STAT(spi_periph) & int_flag;
    uint32_t inten = SPI_INT(spi_periph);
    /*get SPI/I2S interrupt flag status */
    if((0U != (stat & SPI_I2S_INT_MASK & inten)) ||
        ((0U != (stat & (SPI_STAT_SPD | SPI_STAT_TC))) && (0U != (inten & SPI_INT_ESTCIE)))) {
        reval = SET;
    }
    return reval;
}

/*!
    \brief      get SPI and I2S flag status (API_ID(0x003BU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  flag: SPI/I2S flag status
                one or more parameters can be selected which are shown as below:
      \arg        SPI_FLAG_RP: RP flag
      \arg        SPI_FLAG_TP: TP flag
      \arg        SPI_FLAG_DP: DP flag
      \arg        SPI_FLAG_ET: end of transfer or receive flag
      \arg        SPI_FLAG_TXF: transmission filled flag
      \arg        SPI_FLAG_TXURERR: underrun error flag
      \arg        SPI_FLAG_RXORERR: overrun error flag
      \arg        SPI_FLAG_CRCERR: CRC error flag
      \arg        SPI_FLAG_FERR: TI frame error flag
      \arg        SPI_FLAG_CONFERR: mode error flag
      \arg        SPI_FLAG_TXSERF: TXSER reload flag
      \arg        SPI_FLAG_SPD: suspend flag
      \arg        SPI_FLAG_TC: TxFIFO clear flag
      \arg        SPI_FLAG_RWNE: the word of RXFIFO is not empty flag

      \arg        I2S_FLAG_RP: RP flag
      \arg        I2S_FLAG_TP: TP flag
      \arg        I2S_FLAG_DP: DP flag
      \arg        I2S_FLAG_ET: end of transfer or receive flag
      \arg        I2S_FLAG_TXF: transmission filled flag
      \arg        I2S_FLAG_TXURERR: underrun error flag
      \arg        I2S_FLAG_RXORERR: overrun error flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus spi_i2s_flag_get(uint32_t spi_periph, uint32_t flag)
{
    FlagStatus reval = RESET ;
    if(SPI_STAT(spi_periph) & (flag & SPI_I2S_FLAG_MASK)) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      clear SPI and I2S flag (API_ID(0x003CU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  flag: SPI/I2S flag status
                one or more parameters can be selected which are shown as below:
      \arg        SPI_STATC_ETC: clear the end of transfer flag
      \arg        SPI_STATC_TXFC: clear the send transmission filled flag
      \arg        SPI_STATC_TXURERRC: clear the transmission underrun error flag
      \arg        SPI_STATC_RXORERRC: clear the reception overrun error flag
      \arg        SPI_STATC_CRCERRC: clear the CRC error flag
      \arg        SPI_STATC_FERRC: clear the SPI TI format error flag
      \arg        SPI_STATC_CONFERRC: clear the configuration error flag
      \arg        SPI_STATC_TXSERFC: clear the TXSERF flag
      \arg        SPI_STATC_SPDC: clear the suspend flag
    \param[out] none
    \retval     none
*/
void spi_i2s_flag_clear(uint32_t spi_periph, uint32_t flag)
{
    SPI_STATC(spi_periph) |= (uint32_t)(flag & SPI_I2S_CLEAR_FLAG_MASK);
}

/*!
    \brief      get SPI and I2S RXFIFO packing level (API_ID(0x003DU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     2-bit RXFIFO packing data frame number
*/
uint32_t spi_i2s_rxfifo_plevel_get(uint32_t spi_periph)
{
    uint32_t reval = 0U ;
    reval = ((uint32_t)((SPI_STAT(spi_periph) & SPI_STAT_RPLVL) >> 13U));
    return reval;
}

/*!
    \brief      get SPI and I2S remaining data frames number in the TXSIZE session (API_ID(0x003EU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     32-bit value
*/
uint32_t spi_i2s_remain_data_num_get(uint32_t spi_periph)
{
    uint32_t reval = 0U ;
    reval = ((uint32_t)((SPI_STAT(spi_periph) & SPI_STAT_CTXSIZE) >> 16U));
    return reval;
}

/*!
    \brief      set SPI FIFO threshold level (API_ID(0x003FU))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[in]  fifo_thl: SPI_FIFO_TH_01DATA, SPI_FIFO_TH_02DATA, SPI_FIFO_TH_03DATA,
                          SPI_FIFO_TH_04DATA, SPI_FIFO_TH_05DATA, SPI_FIFO_TH_06DATA,
                          SPI_FIFO_TH_07DATA, SPI_FIFO_TH_08DATA, SPI_FIFO_TH_09DATA,
                          SPI_FIFO_TH_10DATA, SPI_FIFO_TH_11DATA, SPI_FIFO_TH_12DATA,
                          SPI_FIFO_TH_13DATA, SPI_FIFO_TH_14DATA, SPI_FIFO_TH_15DATA,
                          SPI_FIFO_TH_16DATA
    \param[out] none
    \retval     none
*/
void spi_fifo_threshold_level_set(uint32_t spi_periph, uint32_t fifo_thl)
{
    SPI_CFG0(spi_periph) &= (uint32_t)(~SPI_CFG0_FIFOLVL);
    SPI_CFG0(spi_periph) |= (uint32_t)(fifo_thl & SPI_CFG0_FIFOLVL);
}

/*!
    \brief      enable SPI word access (API_ID(0x0040U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_word_access_enable(uint32_t spi_periph)
{
    SPI_CFG0(spi_periph) |= (uint32_t)SPI_CFG0_WORDEN;
}

/*!
    \brief      disable SPI word access (API_ID(0x0041U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_word_access_disable(uint32_t spi_periph)
{
    SPI_CFG0(spi_periph) &= (uint32_t)(~SPI_CFG0_WORDEN);
}

/*!
    \brief      enable SPI byte access (API_ID(0x0042U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_byte_access_enable(uint32_t spi_periph)
{
    SPI_CFG0(spi_periph) |= (uint32_t)SPI_CFG0_BYTEN;
}

/*!
    \brief      disable SPI byte access (API_ID(0x0043U))
    \param[in]  spi_periph: SPIx(x=0,1,2,3,4,5)
    \param[out] none
    \retval     none
*/
void spi_byte_access_disable(uint32_t spi_periph)
{
    SPI_CFG0(spi_periph) &= (uint32_t)(~SPI_CFG0_BYTEN);
}
