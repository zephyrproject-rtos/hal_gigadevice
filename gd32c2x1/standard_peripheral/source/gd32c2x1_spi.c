/*!
    \file    gd32c2x1_spi.c
    \brief   SPI driver

    \version 2025-08-08, V1.1.0, firmware for gd32c2x1
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

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

#include "gd32c2x1_spi.h"

/* SPI/I2S parameter initialization mask */
#define SPI_INIT_MASK                   ((uint32_t)0x00003040U)  /*!< SPI init mask */
#define I2S_INIT_MASK                   ((uint32_t)0x0000F047U)  /*!< I2S init mask */
#define SPI_FIFO_INIT_MASK1             ((uint32_t)0x00003840U)  /*!< SPI parameter initialization mask1 */
#define SPI_FIFO_INIT_MASK2             ((uint32_t)0x0000F0FFU)  /*!< SPI parameter initialization mask2*/
#define SPI_BYTEN_MASK                  ((uint32_t)0x00001000U)  /*!< SPI access to FIFO mask */
#define SPI_TXLVL_MASK                  ((uint32_t)0x00001800U)  /*!< SPI TXFIFO mask */
#define SPI_RXLVL_MASK                  ((uint32_t)0x00000600U)  /*!< SPI RXFIFO mask */
/* default value */
#define SPI_I2SPSC_DEFAULT_VALUE        ((uint32_t)0x00000002U)  /*!< default value of SPI_I2SPSC register */
/* I2S clock source selection, multiplication and division mask */
#define I2S0_CLOCK_SEL                  ((uint32_t)0x0000C000U)  /*!< I2S clock source selection */
#define I2S_CLOCK_MUL_MASK              ((uint32_t)0x0000F000U)  /*!< I2S clock multiplication mask */
#define I2S_CLOCK_DIV_MASK              ((uint32_t)0x000000F0U)  /*!< I2S clock division mask */
#define I2S_STANDARD_MASK               ((uint32_t)0x000000B0U)  /*!< I2S standard mask */

#define SPI_TRANS_MODE_MASK             ((uint32_t)0x0000C400U)  /*!< SPI transfer mode mask */
#define SPI_I2S_INT_MASK                ((uint32_t)0x000000E0U)  /*!< SPI and I2S interrupt enable mask */

/*!
    \brief      reset SPI and I2S (API_ID(0x0001U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_i2s_deinit(uint32_t spi_periph)
{
    switch(spi_periph) {
    case SPI0:
        /* reset SPI0 */
        rcu_periph_reset_enable(RCU_SPI0RST);
        rcu_periph_reset_disable(RCU_SPI0RST);
        break;
    case SPI1:
        /* reset SPI1 and I2S1 */
        rcu_periph_reset_enable(RCU_SPI1RST);
        rcu_periph_reset_disable(RCU_SPI1RST);
        break;
    default :
        break;
    }
}

/*!
    \brief      initialize the parameters of SPI struct with default values (API_ID(0x0002U))
    \param[in]  none
    \param[out] spi_parameter_struct: the initialized struct spi_parameter_struct pointer
    \retval     none
*/
void spi_struct_para_init(spi_parameter_struct *spi_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(spi_struct)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure the structure with default value */
        spi_struct->device_mode          = SPI_SLAVE;
        spi_struct->trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
        spi_struct->frame_size           = SPI_FRAMESIZE_8BIT;
        spi_struct->nss                  = SPI_NSS_HARD;
        spi_struct->clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        spi_struct->prescale             = SPI_PSC_2;
        spi_struct->endian               = SPI_ENDIAN_MSB;
    }
}

/*!
    \brief      initialize SPI parameter (API_ID(0x0003U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  spi_struct: SPI parameter initialization stuct members of the structure
                            and the member values are shown as below:
                  device_mode: SPI_MASTER, SPI_SLAVE
                  trans_mode: SPI_TRANSMODE_FULLDUPLEX, SPI_TRANSMODE_RECEIVEONLY,
                              SPI_TRANSMODE_BDRECEIVE, SPI_TRANSMODE_BDTRANSMIT
                  frame_size: SPI_FRAMESIZE_4BIT, SPI_FRAMESIZE_5BIT
                              SPI_FRAMESIZE_6BIT, SPI_FRAMESIZE_7BIT
                              SPI_FRAMESIZE_8BIT, SPI_FRAMESIZE_9BIT
                              SPI_FRAMESIZE_10BIT, SPI_FRAMESIZE_11BIT
                              SPI_FRAMESIZE_12BIT, SPI_FRAMESIZE_13BIT
                              SPI_FRAMESIZE_14BIT, SPI_FRAMESIZE_15BIT
                              SPI_FRAMESIZE_16BIT
                  nss: SPI_NSS_SOFT, SPI_NSS_HARD
                  endian: SPI_ENDIAN_MSB, SPI_ENDIAN_LSB
                  clock_polarity_phase: SPI_CK_PL_LOW_PH_1EDGE, SPI_CK_PL_HIGH_PH_1EDGE
                                        SPI_CK_PL_LOW_PH_2EDGE, SPI_CK_PL_HIGH_PH_2EDGE
                  prescale: SPI_PSC_n (n=2,4,8,16,32,64,128,256)
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus spi_init(uint32_t spi_periph, spi_parameter_struct *spi_struct)
{
    ErrStatus reval = SUCCESS;
    uint32_t reg1;
    uint32_t reg2, reg3;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(spi_struct)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        reg1 = SPI_CTL0(spi_periph);
        reg1 &= SPI_INIT_MASK;

        reg2 = SPI_CTL0(spi_periph);
        reg2 &= SPI_FIFO_INIT_MASK1;

        reg3 = SPI_CTL1(spi_periph);
        reg3 &= SPI_FIFO_INIT_MASK2;

        if(SPI0 == spi_periph) {
            /* select SPI as master or slave */
            reg1 |= (spi_struct->device_mode & SPI_MASTER);
            /* select SPI transfer mode */
            reg1 |= (spi_struct->trans_mode & SPI_TRANS_MODE_MASK);
            /* select SPI NSS use hardware or software */
            reg1 |= (spi_struct->nss & SPI_NSS_SOFT);
            /* select SPI LSB or MSB */
            reg1 |= (spi_struct->endian & SPI_ENDIAN_LSB);
            /* select SPI polarity and phase */
            reg1 |= (spi_struct->clock_polarity_phase & SPI_CK_PL_HIGH_PH_2EDGE);
            /* select SPI prescale to adjust transmit speed */
            reg1 |= (spi_struct->prescale & SPI_PSC_256);
            /* select SPI frame size */
            /* check SPI0 frame size is 8bits/16bits or not*/
            if((SPI_FRAMESIZE_8BIT != spi_struct->frame_size) && (SPI_FRAMESIZE_16BIT != spi_struct->frame_size)) {
                reval = ERROR;
            } else {
                reg1 |= (uint32_t)(spi_struct->frame_size & SPI_CTL0_FF16);
            }

            /* write to SPI_CTL0 register */
            SPI_CTL0(spi_periph) = (uint32_t)reg1;

        } else {
            /* select SPI as master or slave */
            reg2 |= (spi_struct->device_mode & SPI_MASTER);
            /* select SPI transfer mode */
            reg2 |= (spi_struct->trans_mode & SPI_TRANS_MODE_MASK);
            /* select SPI NSS use hardware or software */
            reg2 |= (spi_struct->nss & SPI_NSS_SOFT);
            /* select SPI LSB or MSB */
            reg2 |= (spi_struct->endian & SPI_ENDIAN_LSB);
            /* select SPI polarity and phase */
            reg2 |= (spi_struct->clock_polarity_phase & SPI_CK_PL_HIGH_PH_2EDGE);
            /* select SPI prescale to adjust transmit speed */
            reg2 |= (spi_struct->prescale & SPI_PSC_256);
            /* write to SPI_CTL0 register */
            SPI_CTL0(spi_periph) = (uint32_t)reg2;

            /* select SPI data size */
            reg3 |= (uint32_t)(spi_struct->frame_size & SPI_CTL1_DZ);
            /* write to SPI_CTL1 register */
            SPI_CTL1(spi_periph) = (uint32_t)reg3;
        }
    }
    /* select SPI mode */
    SPI_I2SCTL(spi_periph) &= (uint32_t)(~SPI_I2SCTL_I2SSEL);
    return reval;
}

/*!
    \brief      enable SPI (API_ID(0x0004U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_enable(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_SPIEN;
}

/*!
    \brief      disable SPI (API_ID(0x0005U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_disable(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_SPIEN);
}

/*!
    \brief      initialize I2S parameter (API_ID(0x0006U))
    \param[in]  spi_periph: SPIx(x=0)
    \param[in]  i2s_mode: I2S operation mode
                only one parameter can be selected which is shown as below:
      \arg        I2S_MODE_SLAVETX: I2S slave transmit mode
      \arg        I2S_MODE_SLAVERX: I2S slave receive mode
      \arg        I2S_MODE_MASTERTX: I2S master transmit mode
      \arg        I2S_MODE_MASTERRX: I2S master receive mode
    \param[in]  i2s_standard: I2S standard
                only one parameter can be selected which is shown as below:
      \arg        I2S_STD_PHILLIPS: I2S phillips standard
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
    uint32_t reg;

    reg = SPI_I2SCTL(spi_periph);
    reg &= I2S_INIT_MASK;

    /* enable I2S mode */
    reg |= (uint32_t)SPI_I2SCTL_I2SSEL;
    /* select I2S mode */
    reg |= (uint32_t)(i2s_mode & I2S_MODE_MASTERRX);
    /* select I2S standard */
    reg |= (uint32_t)(i2s_standard & I2S_STANDARD_MASK);
    /* select I2S polarity */
    reg |= (uint32_t)(i2s_ckpl & I2S_CKPL_HIGH);

    /* write to SPI_I2SCTL register */
    SPI_I2SCTL(spi_periph) = (uint32_t)reg;
}

/*!
    \brief      configure I2S prescaler (API_ID(0x0007U))
    \param[in]  spi_periph: SPIx(x=0)
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
void i2s_psc_config(uint32_t spi_periph, uint32_t i2s_audiosample, uint32_t i2s_frameformat,
                    uint32_t i2s_mckout)
{
    uint32_t i2sdiv, i2sof;
    uint32_t clks;
    uint32_t i2sclock;

#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_I2S_AUDIOSAMPLE(i2s_audiosample)) {
        fw_debug_report_err(SPI_MODULE_ID, API_ID(0x0007U), ERR_PARAM_INVALID);
    } else
#endif
    {
        /* deinit SPI_I2SPSC register */
        SPI_I2SPSC(spi_periph) = SPI_I2SPSC_DEFAULT_VALUE;

        /* I2S1 clock source selection */
        clks = I2S0_CLOCK_SEL;

        if(0U != (RCU_CFG1 & clks)) {
            /* get RCU PLL2 clock multiplication factor */
            clks = (uint32_t)((RCU_CFG1 & I2S_CLOCK_MUL_MASK) >> 12U);

            if((clks > 5U) && (clks < 15U)) {
                /* multiplier is between 8 and 14 */
                clks += 2U;
            } else {
                if(15U == clks) {
                    /* multiplier is 20 */
                    clks = 20U;
                }
            }

            /* get the PREDV1 value */
            i2sclock = (uint32_t)(((RCU_CFG1 & I2S_CLOCK_DIV_MASK) >> 4U) + 1U);
            /* calculate i2sclock based on PLL2 and PREDV1 */
            i2sclock = (uint32_t)((HXTAL_VALUE / i2sclock) * clks * 2U);
        } else {
            /* get system clock */
            i2sclock = rcu_clock_freq_get(CK_SYS);
        }

        /* configure the prescaler depending on the mclk output state, the frame format and audio sample rate */
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
        i2sof  = (i2sof << 8U);

        /* set the default values */
        if((i2sdiv < 2U) || (i2sdiv > 255U)) {
            i2sdiv = 2U;
            i2sof = 0U;
        }

        /* configure SPI_I2SPSC */
        SPI_I2SPSC(spi_periph) = (uint32_t)((i2sdiv & SPI_I2SPSC_DIV) | (i2sof & SPI_I2SPSC_OF) | (i2s_mckout & I2S_MCKOUT_ENABLE));

        /* clear SPI_I2SCTL_DTLEN and SPI_I2SCTL_CHLEN bits */
        SPI_I2SCTL(spi_periph) &= (uint32_t)(~(SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN));
        /* configure data frame format */
        SPI_I2SCTL(spi_periph) |= (uint32_t)(i2s_frameformat & I2S_FRAMEFORMAT_DT32B_CH32B);
    }
}

/*!
    \brief      enable I2S (API_ID(0x0008U))
    \param[in]  spi_periph: SPIx(x=0)
    \param[out] none
    \retval     none
*/
void i2s_enable(uint32_t spi_periph)
{
    SPI_I2SCTL(spi_periph) |= (uint32_t)SPI_I2SCTL_I2SEN;
}

/*!
    \brief      disable I2S (API_ID(0x0009U))
    \param[in]  spi_periph: SPIx(x=0)
    \param[out] none
    \retval     none
*/
void i2s_disable(uint32_t spi_periph)
{
    SPI_I2SCTL(spi_periph) &= (uint32_t)(~SPI_I2SCTL_I2SEN);
}

/*!
    \brief      enable SPI NSS output (API_ID(0x000AU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_nss_output_enable(uint32_t spi_periph)
{
    SPI_CTL1(spi_periph) |= (uint32_t)SPI_CTL1_NSSDRV;
}

/*!
    \brief      disable SPI NSS output (API_ID(0x000BU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_nss_output_disable(uint32_t spi_periph)
{
    SPI_CTL1(spi_periph) &= (uint32_t)(~SPI_CTL1_NSSDRV);
}

/*!
    \brief      SPI NSS pin high level in software mode (API_ID(0x000CU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_nss_internal_high(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_SWNSS;
}

/*!
    \brief      SPI NSS pin low level in software mode (API_ID(0x000DU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_nss_internal_low(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_SWNSS);
}

/*!
    \brief      enable SPI DMA send or receive (API_ID(0x000EU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  dma: SPI DMA mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_DMA_TRANSMIT: SPI transmit data use DMA
      \arg        SPI_DMA_RECEIVE: SPI receive data use DMA
    \param[out] none
    \retval     none
*/
void spi_dma_enable(uint32_t spi_periph, uint8_t dma)
{
    if(SPI_DMA_TRANSMIT == dma) {
        SPI_CTL1(spi_periph) |= (uint32_t)SPI_CTL1_DMATEN;
    } else {
        SPI_CTL1(spi_periph) |= (uint32_t)SPI_CTL1_DMAREN;
    }
}

/*!
    \brief      disable SPI DMA send or receive (API_ID(0x000FU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  dma: SPI DMA mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_DMA_TRANSMIT: SPI transmit data use DMA
      \arg        SPI_DMA_RECEIVE: SPI receive data use DMA
    \param[out] none
    \retval     none
*/
void spi_dma_disable(uint32_t spi_periph, uint8_t dma)
{
    if(SPI_DMA_TRANSMIT == dma) {
        SPI_CTL1(spi_periph) &= (uint32_t)(~SPI_CTL1_DMATEN);
    } else {
        SPI_CTL1(spi_periph) &= (uint32_t)(~SPI_CTL1_DMAREN);
    }
}

/*!
    \brief      configure SPI1 total number of data to transmit by DMA is odd or not (API_ID(0x0010U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[in]  odd: odd bytes in TX DMA channel
                only one parameter can be selected which is shown as below:
      \arg        SPI_TXDMA_EVEN: number of byte in TX DMA channel is even
      \arg        SPI_TXDMA_ODD: number of byte in TX DMA channel is odd
    \param[out] none
    \retval     none
*/
void spi_transmit_odd_config(uint32_t spi_periph, uint16_t odd)
{
    /* clear SPI_CTL1_TXDMA_ODD bit */
    SPI_CTL1(spi_periph) &= (uint32_t)(~SPI_CTL1_TXDMA_ODD);
    /* configure SPI_CTL1_TXDMA_ODD bit */
    SPI_CTL1(spi_periph) |= (uint32_t)(odd & SPI_TXDMA_ODD);
}

/*!
    \brief      configure SPI1 total number of data to receive by DMA is odd or not (API_ID(0x0011U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[in]  odd: odd bytes in RX DMA channel
                only one parameter can be selected which is shown as below:
      \arg        SPI_RXDMA_EVEN: number of bytes in RX DMA channel is even
      \arg        SPI_RXDMA_ODD: number of bytes in RX DMA channel is odd
    \param[out] none
    \retval     none
*/
void spi_receive_odd_config(uint32_t spi_periph, uint16_t odd)
{
    /* clear SPI_CTL1_RXDMA_ODD bit */
    SPI_CTL1(spi_periph) &= (uint32_t)(~SPI_CTL1_RXDMA_ODD);
    /* configure SPI_CTL1_RXDMA_ODD bit */
    SPI_CTL1(spi_periph) |= (uint32_t)(odd & SPI_TXDMA_ODD);
}

/*!
    \brief      configure SPI0 access size to FIFO(8bit or 16bit) (API_ID(0x0012U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[in]  fifo_access_size: byte access enable
                only one parameter can be selected which is shown as below:
      \arg        SPI_HALFWORD_ACCESS: half-word access to FIFO
      \arg        SPI_BYTE_ACCESS: byte access to FIFO
    \param[out] none
    \retval     none
*/
void spi_fifo_access_size_config(uint32_t spi_periph, uint16_t fifo_access_size)
{
    /* clear SPI_CTL1_BYTEN bit */
    SPI_CTL1(spi_periph) &= (uint32_t)(~SPI_CTL1_BYTEN);
    /* configure SPI_CTL1_BYTEN bit */
    SPI_CTL1(spi_periph) |= (uint32_t)(fifo_access_size & SPI_BYTE_ACCESS);
}

/*!
    \brief      configure SPI data frame format (API_ID(0x0013U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  frame_format: SPI frame size
                only one parameter can be selected which is shown as below:
      \arg         SPI_FRAMESIZE_xBIT(x=4,5..16, for SPI1, x=8,16, for SPI0):SPI frame size is x bits
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus spi_i2s_data_frame_format_config(uint32_t spi_periph, uint16_t frame_format)
{
    ErrStatus reval = SUCCESS;
    uint32_t reg;

    if(SPI0 == spi_periph) {
        /* check SPI0 frame size is 8bits/16bits or not*/
        if((SPI_FRAMESIZE_8BIT != frame_format) && (SPI_FRAMESIZE_16BIT != frame_format)) {
            reval = ERROR;
        } else {
            /* clear SPI_CTL0_FF16 bit */
            SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_FF16);
            /* configure SPI_CTL0_FF16 bit */
            SPI_CTL0(spi_periph) |= ((uint32_t)frame_format & SPI_CTL0_FF16);
        }
    } else {
        reg = SPI_CTL1(spi_periph);
        /* clear SPI_CTL1_DZ bits */
        reg &= (uint32_t)(~SPI_CTL1_DZ);
        reg |= (uint32_t)(frame_format & SPI_CTL1_DZ);
        /* configure SPI_CTL1_DZ bits */
        SPI_CTL1(spi_periph) = reg;
    }
    return reval;
}


/*!
    \brief      configure SPI bidirectional transfer direction (API_ID(0x0014U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  transfer_direction: SPI transfer direction
                only one parameter can be selected which is shown as below:
      \arg        SPI_BIDIRECTIONAL_TRANSMIT: SPI work in transmit-only mode
      \arg        SPI_BIDIRECTIONAL_RECEIVE: SPI work in receive-only mode
    \retval     none
*/
void spi_bidirectional_transfer_config(uint32_t spi_periph, uint32_t transfer_direction)
{
    if(SPI_BIDIRECTIONAL_TRANSMIT == transfer_direction) {
        /* set the transmit only mode */
        SPI_CTL0(spi_periph) |= (uint32_t)SPI_BIDIRECTIONAL_TRANSMIT;
    } else {
        /* set the receive only mode */
        SPI_CTL0(spi_periph) &= SPI_BIDIRECTIONAL_RECEIVE;
    }
}

/*!
    \brief      SPI transmit data (API_ID(0x0015U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  data: 16-bit data
    \param[out] none
    \retval     none
*/
void spi_i2s_data_transmit(uint32_t spi_periph, uint16_t data)
{
    uint32_t reg, byten;

    if(SPI0 == spi_periph) {
        SPI_DATA(spi_periph) = (uint16_t)data;
    } else {
        /* get the access size to FIFO */
        byten = SPI_CTL1(spi_periph) & SPI_BYTEN_MASK;
        if(0U != byten) {
            reg = spi_periph + 0x0CU;
            *(uint8_t *)(reg) = (uint8_t)data;
        } else {
            SPI_DATA(spi_periph) = (uint16_t)data;
        }
    }
}

/*!
    \brief      SPI receive data (API_ID(0x0016U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     16-bit data
*/
uint16_t spi_i2s_data_receive(uint32_t spi_periph)
{
    uint16_t reval = 0U;

    uint32_t reg, byten;

    if(SPI0 == spi_periph) {
        reval = ((uint16_t)SPI_DATA(spi_periph));
    } else {
        /* get the access size to FIFO */
        byten = SPI_CTL1(spi_periph) & SPI_BYTEN_MASK;
        if(0U != byten) {
            reg = spi_periph + 0x0CU;
            reval = (uint16_t)(*(uint8_t *)(reg));
        } else {
            reval = ((uint16_t)SPI_DATA(spi_periph));
        }
    }

    return reval;
}

/*!
    \brief      set SPI CRC polynomial (API_ID(0x0017U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  crc_poly: CRC polynomial value
    \param[out] none
    \retval     none
*/
void spi_crc_polynomial_set(uint32_t spi_periph, uint16_t crc_poly)
{
    /* set SPI CRC polynomial */
    SPI_CRCPOLY(spi_periph) = (uint16_t)crc_poly;
}

/*!
    \brief      get SPI CRC polynomial (API_ID(0x0018U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     16-bit CRC polynomial
*/
uint16_t spi_crc_polynomial_get(uint32_t spi_periph)
{
    uint16_t reval = 0U;

    reval = ((uint16_t)SPI_CRCPOLY(spi_periph));
    return reval;
}

/*!
    \brief      set CRC length (API_ID(0x0019U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[in]  crc_length: CRC length
                only one parameter can be selected which is shown as below:
      \arg        SPI_CRC_8BIT: CRC length is 8 bits
      \arg        SPI_CRC_16BIT: CRC length is 16 bits
    \param[out] none
    \retval     none
*/
void spi_crc_length_set(uint32_t spi_periph, uint16_t crc_length)
{
    /* clear SPI_CTL0_CRCL bit */
    SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_CRCL);
    /* configure SPI_CTL0_CRCL bit */
    SPI_CTL0(spi_periph) |= (uint32_t)(crc_length & SPI_CRC_16BIT);
}

/*!
    \brief      turn on CRC function (API_ID(0x001AU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_crc_on(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_CRCEN;
}

/*!
    \brief      turn off CRC function (API_ID(0x001BU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_crc_off(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) &= (uint32_t)(~SPI_CTL0_CRCEN);
}

/*!
    \brief      SPI next data is CRC value (API_ID(0x001CU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_crc_next(uint32_t spi_periph)
{
    SPI_CTL0(spi_periph) |= (uint32_t)SPI_CTL0_CRCNT;
}

/*!
    \brief      get SPI CRC send value or receive value (API_ID(0x001DU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  crc: SPI crc value
                only one parameter can be selected which is shown as below:
      \arg        SPI_CRC_TX: get transmit crc value
      \arg        SPI_CRC_RX: get receive crc value
    \param[out] none
    \retval     16-bit CRC value
*/
uint16_t spi_crc_get(uint32_t spi_periph, uint8_t crc)
{
    uint16_t reval = 0U;

    if(SPI_CRC_TX == crc) {
        reval = ((uint16_t)(SPI_TCRC(spi_periph)));
    } else {
        reval = ((uint16_t)(SPI_RCRC(spi_periph)));
    }
    return reval;
}

/*!
    \brief      clear SPI CRC error flag status (API_ID(0x001EU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_crc_error_clear(uint32_t spi_periph)
{
    SPI_STAT(spi_periph) &= (uint32_t)(~SPI_FLAG_CRCERR);
}

/*!
    \brief      enable SPI TI mode (API_ID(0x001FU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_ti_mode_enable(uint32_t spi_periph)
{
    SPI_CTL1(spi_periph) |= (uint32_t)SPI_CTL1_TMOD;
}

/*!
    \brief      disable SPI TI mode (API_ID(0x0020U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_ti_mode_disable(uint32_t spi_periph)
{
    SPI_CTL1(spi_periph) &= (uint32_t)(~SPI_CTL1_TMOD);
}

/*!
    \brief      enable SPI NSS pulse mode (API_ID(0x0021U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_nssp_mode_enable(uint32_t spi_periph)
{
    SPI_CTL1(spi_periph) |= (uint32_t)SPI_CTL1_NSSP;
}

/*!
    \brief      disable SPI NSS pulse mode (API_ID(0x0022U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
void spi_nssp_mode_disable(uint32_t spi_periph)
{
    SPI_CTL1(spi_periph) &= (uint32_t)(~SPI_CTL1_NSSP);
}

/*!
    \brief      enable quad wire SPI (API_ID(0x0023U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[out] none
    \retval     none
*/
void spi_quad_enable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) |= (uint32_t)SPI_QCTL_QMOD;
}

/*!
    \brief      disable quad wire SPI (API_ID(0x0024U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[out] none
    \retval     none
*/
void spi_quad_disable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) &= (uint32_t)(~SPI_QCTL_QMOD);
}

/*!
    \brief      enable quad wire SPI write (API_ID(0x0025U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[out] none
    \retval     none
*/
void spi_quad_write_enable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) &= (uint32_t)(~SPI_QCTL_QRD);
}

/*!
    \brief      enable quad wire SPI read (API_ID(0x0026U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[out] none
    \retval     none
*/
void spi_quad_read_enable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) |= (uint32_t)SPI_QCTL_QRD;
}

/*!
    \brief      enable SPI_IO2 and SPI_IO3 pin output (API_ID(0x0027U))
    \param[in]  spi_periph: SPIx(x=1)
    \param[out] none
    \retval     none
*/
void spi_quad_io23_output_enable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) |= (uint32_t)SPI_QCTL_IO23_DRV;
}

/*!
   \brief      disable SPI_IO2 and SPI_IO3 pin output (API_ID(0x0028U))
   \param[in]  spi_periph: SPIx(x=1)
   \param[out] none
   \retval     none
*/
void spi_quad_io23_output_disable(uint32_t spi_periph)
{
    SPI_QCTL(spi_periph) &= (uint32_t)(~SPI_QCTL_IO23_DRV);
}

/*!
    \brief      clear SPI/I2S format error flag status (API_ID(0x0029U))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  flag: SPI/I2S frame format error flag
                one or more parameters can be selected which is shown as below:
      \arg        SPI_FLAG_FERR: SPI format error flag
      \arg        I2S_FLAG_FERR: I2S format error flag
    \param[out] none
    \retval     none
*/
void spi_i2s_format_error_clear(uint32_t spi_periph, uint32_t flag)
{
    SPI_STAT(spi_periph) = (uint32_t)(~flag);
}

/*!
    \brief      get SPI and I2S flag status (API_ID(0x002AU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  flag: SPI/I2S flag status
                only one parameter can be selected which are shown as below:
      \arg        SPI_FLAG_TBE: transmit buffer empty flag
      \arg        SPI_FLAG_RBNE: receive buffer not empty flag
      \arg        SPI_FLAG_TRANS: transmit on-going flag
      \arg        SPI_FLAG_RXORERR: receive overrun error flag
      \arg        SPI_FLAG_CONFERR: mode config error flag
      \arg        SPI_FLAG_CRCERR: CRC error flag
      \arg        SPI_FLAG_FERR: SPI format error interrupt flag
      \arg        I2S_FLAG_TBE: transmit buffer empty flag
      \arg        I2S_FLAG_RBNE: receive buffer not empty flag
      \arg        I2S_FLAG_CH: channel side flag
      \arg        I2S_FLAG_TXURERR: underrun error flag
      \arg        I2S_FLAG_TRANS: transmit on-going flag
      \arg        I2S_FLAG_RXORERR: overrun error flag
      \arg        I2S_FLAG_FERR: I2S format error interrupt flag
                only for SPI1:
      \arg        SPI_FLAG_TXLVL_EMPTY: SPI TXFIFO is empty
      \arg        SPI_FLAG_TXLVL_QUARTER_FULL: SPI TXFIFO is a quarter of full
      \arg        SPI_FLAG_TXLVL_HALF_FULL: SPI TXFIFO is a half of full
      \arg        SPI_FLAG_TXLVL_FULL: SPI TXFIFO is full
      \arg        SPI_FLAG_RXLVL_EMPTY: SPI RXFIFO is empty
      \arg        SPI_FLAG_RXLVL_QUARTER_FULL: SPI RXFIFO is a quarter of full
      \arg        SPI_FLAG_RXLVL_HALF_FULL: SPI RXFIFO is a half of full
      \arg        SPI_FLAG_RXLVL_FULL: SPI RXFIFO is full
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus spi_i2s_flag_get(uint32_t spi_periph, uint32_t flag)
{
    FlagStatus reval = RESET;
    uint32_t reg = SPI_STAT(spi_periph);

    switch(flag) {
#if defined(GD32C231) || defined(GD32C221)
    case SPI_FLAG_TXLVL_EMPTY:
        if(0U == (reg & SPI_TXLVL_MASK)) {
            reval = SET;
        }
        break;
    case SPI_FLAG_TXLVL_QUARTER_FULL:
    case SPI_FLAG_TXLVL_HALF_FULL:
    case SPI_FLAG_TXLVL_FULL:
        if(flag == (reg & SPI_TXLVL_MASK)) {
            reval = SET;
        }
        break;
    case SPI_FLAG_RXLVL_EMPTY:
        if(0U == (reg & SPI_RXLVL_MASK)) {
            reval = SET;
        }
        break;
    case SPI_FLAG_RXLVL_QUARTER_FULL:
    case SPI_FLAG_RXLVL_HALF_FULL:
    case SPI_FLAG_RXLVL_FULL:
        if(flag == (reg & SPI_RXLVL_MASK)) {
            reval = SET;
        }
        break;
#endif
    default:
        if(0U != (reg & flag)) {
            reval = SET;
        }
        break;
    }

    return reval;
}

/*!
    \brief      enable SPI and I2S interrupt (API_ID(0x002BU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  interrupt: SPI/I2S interrupt
                only one parameter can be selected which is shown as below:
      \arg        SPI_I2S_INT_TBE: transmit buffer empty interrupt
      \arg        SPI_I2S_INT_RBNE: receive buffer not empty interrupt
      \arg        SPI_I2S_INT_ERR: CRC error, configuration error,reception overrun error,
                                   transmission underrun error and format error interrupt
    \param[out] none
    \retval     none
*/
void spi_i2s_interrupt_enable(uint32_t spi_periph, uint8_t interrupt)
{
    SPI_CTL1(spi_periph) |= (uint32_t)(interrupt & SPI_I2S_INT_MASK);
}

/*!
    \brief      disable SPI and I2S interrupt (API_ID(0x002CU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  interrupt: SPI/I2S interrupt
                only one parameter can be selected which is shown as below:
      \arg        SPI_I2S_INT_TBE: transmit buffer empty interrupt
      \arg        SPI_I2S_INT_RBNE: receive buffer not empty interrupt
      \arg        SPI_I2S_INT_ERR: CRC error,configuration error,reception overrun error,
                                   transmission underrun error and format error interrupt
    \param[out] none
    \retval     none
*/
void spi_i2s_interrupt_disable(uint32_t spi_periph, uint8_t interrupt)
{
    SPI_CTL1(spi_periph) &= ~(uint32_t)(interrupt & SPI_I2S_INT_MASK);
}

/*!
    \brief      get SPI and I2S interrupt flag status (API_ID(0x002DU))
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  interrupt: SPI/I2S interrupt flag status
                only one parameter can be selected which is shown as below:
      \arg        SPI_I2S_INT_FLAG_TBE: transmit buffer empty interrupt flag
      \arg        SPI_I2S_INT_FLAG_RBNE: receive buffer not empty interrupt flag
      \arg        SPI_I2S_INT_FLAG_RXORERR: overrun interrupt flag
      \arg        SPI_INT_FLAG_CONFERR: config error interrupt flag
      \arg        SPI_INT_FLAG_CRCERR: CRC error interrupt flag
      \arg        I2S_INT_FLAG_TXURERR: underrun error interrupt flag
      \arg        SPI_I2S_INT_FLAG_FERR: format error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus spi_i2s_interrupt_flag_get(uint32_t spi_periph, uint8_t interrupt)
{
    FlagStatus reval = RESET;
    uint32_t reg1 = SPI_STAT(spi_periph);
    uint32_t reg2 = SPI_CTL1(spi_periph);

    switch(interrupt) {
    /* SPI/I2S transmit buffer empty interrupt */
    case SPI_I2S_INT_FLAG_TBE:
        reg1 = reg1 & SPI_STAT_TBE;
        reg2 = reg2 & SPI_CTL1_TBEIE;
        break;
    /* SPI/I2S receive buffer not empty interrupt */
    case SPI_I2S_INT_FLAG_RBNE:
        reg1 = reg1 & SPI_STAT_RBNE;
        reg2 = reg2 & SPI_CTL1_RBNEIE;
        break;
    /* SPI/I2S overrun interrupt */
    case SPI_I2S_INT_FLAG_RXORERR:
        reg1 = reg1 & SPI_STAT_RXORERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* SPI config error interrupt */
    case SPI_INT_FLAG_CONFERR:
        reg1 = reg1 & SPI_STAT_CONFERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* SPI CRC error interrupt */
    case SPI_INT_FLAG_CRCERR:
        reg1 = reg1 & SPI_STAT_CRCERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* underrun error interrupt flag */
    case I2S_INT_FLAG_TXURERR:
        reg1 = reg1 & SPI_STAT_TXURERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* SPI/I2S format error interrupt */
    case SPI_I2S_INT_FLAG_FERR:
        reg1 = reg1 & SPI_STAT_FERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    default :
        break;
    }
    /*get SPI/I2S interrupt flag status */
    if((0U != reg1) && (0U != reg2)) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}
