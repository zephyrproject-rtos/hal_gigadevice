/*!
    \file    gd32h73x_75x_enet.c
    \brief   ENET driver

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

#include "gd32h73x_75x_enet.h"
#include <stdlib.h>

#if defined(GD32H73X_75X)

/* init structure parameters for ENET initialization */
static enet_initpara_struct enet_initpara = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

/* array of register offset for debug information get */
static const uint16_t enet_reg_tab[] = {
    0x0000U, 0x0004U, 0x0008U, 0x000CU, 0x0010U, 0x0014U, 0x0018U, 0x001CU, 0x0028U, 0x002CU, 0x0034U,

    0x0038U, 0x003CU, 0x0040U, 0x0044U, 0x0048U, 0x004CU, 0x0050U, 0x0054U, 0x0058U, 0x005CU, 0x1080U,

    0x0100U, 0x0104U, 0x0108U, 0x010CU, 0x0110U, 0x014CU, 0x0150U, 0x0168U, 0x0194U, 0x0198U, 0x01C4U,

    0x0700U, 0x0704U, 0x0708U, 0x070CU, 0x0710U, 0x0714U, 0x0718U, 0x071CU, 0x0720U, 0x0728U, 0x072CU,

    0x1000U, 0x1004U, 0x1008U, 0x100CU, 0x1010U, 0x1014U, 0x1018U, 0x101CU, 0x1020U, 0x1024U, 0x1048U,

    0x104CU, 0x1050U, 0x1054U
};

/* initialize ENET peripheral with generally concerned parameters, call it by enet_init() */
static void enet_default_init(uint32_t enet_periph);
/* change subsecond to nanosecond */
static uint32_t enet_ptp_subsecond_2_nanosecond(uint32_t subsecond);

#define PHY_ADDR_REG_MASK                   (0x0000001FU)
#define FILTER_FEATURE_MASK                 (BITS(8,10) | BITS(2,4))
#define DEACTIVE_THRESHOLD_MASK             (0x00007000U)
#define ACTIVE_THRESHOLD_MASK               (0x00000700U)
#define FLOWCONTROL_FEATURE_MASK            (0x00000087U)
#define DMA_FEATURE_MASK                    (0x01000004U)
#define WUM_FEATURE_EN_MASK                 (0x00000207U)
#define WUM_FEATURE_DIS_MASK                (0x00000206U)
#define MSC_FEATURE_MASK                    (0x0000000EU)
#define PTP_FAETURE_MASK                    (0x00043911U)

/*!
    \brief      deinitialize the ENET, and reset structure parameters for ENET initialization (API_ID(0x0001U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_deinit(uint32_t enet_periph)
{
    switch(enet_periph) {
    case ENET0:
        /* reset ENET0 */
        rcu_periph_reset_enable(RCU_ENET0RST);
        rcu_periph_reset_disable(RCU_ENET0RST);
        enet_initpara_reset();
        break;
    case ENET1:
        /* reset ENET1 */
        rcu_periph_reset_enable(RCU_ENET1RST);
        rcu_periph_reset_disable(RCU_ENET1RST);
        enet_initpara_reset();
        break;
    default:
        break;
    }
}

/*!
    \brief      reset the ENET initpara struct, call it before using enet_initpara_config() (API_ID(0x0002U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void enet_initpara_reset(void)
{
    enet_initpara.option_enable = 0U;
    enet_initpara.forward_frame = 0U;
    enet_initpara.dmabus_mode = 0U;
    enet_initpara.dma_maxburst = 0U;
    enet_initpara.dma_arbitration = 0U;
    enet_initpara.store_forward_mode = 0U;
    enet_initpara.dma_function = 0U;
    enet_initpara.vlan_config = 0U;
    enet_initpara.flow_control = 0U;
    enet_initpara.hashtable_high = 0U;
    enet_initpara.hashtable_low = 0U;
    enet_initpara.framesfilter_mode = 0U;
    enet_initpara.halfduplex_param = 0U;
    enet_initpara.timer_config = 0U;
    enet_initpara.interframegap = 0U;
}

/*!
    \brief      configure the parameters which are usually less cared for initialization (API_ID(0x0003U))
                note -- this function must be called before enet_init(), otherwise
                configuration will be no effect
    \param[in]  option: different function option, which is related to several parameters,
                only one parameter can be selected which is shown as below, refer to enet_option_enum
      \arg        FORWARD_OPTION: choose to configure the frame forward related parameters
      \arg        DMABUS_OPTION: choose to configure the DMA bus mode related parameters
      \arg        DMA_MAXBURST_OPTION: choose to configure the DMA max burst related parameters
      \arg        DMA_ARBITRATION_OPTION: choose to configure the DMA arbitration related parameters
      \arg        STORE_OPTION: choose to configure the store forward mode related parameters
      \arg        DMA_OPTION: choose to configure the DMA descriptor related parameters
      \arg        VLAN_OPTION: choose to configure vlan related parameters
      \arg        FLOWCTL_OPTION: choose to configure flow control related parameters
      \arg        HASHH_OPTION: choose to configure hash high
      \arg        HASHL_OPTION: choose to configure hash low
      \arg        FILTER_OPTION: choose to configure frame filter related parameters
      \arg        HALFDUPLEX_OPTION: choose to configure halfduplex mode related parameters
      \arg        TIMER_OPTION: choose to configure time counter related parameters
      \arg        INTERFRAMEGAP_OPTION: choose to configure the inter frame gap related parameters
    \param[in]  para: the related parameters according to the option
                      all the related parameters should be configured which are shown as below
                      FORWARD_OPTION related parameters:
                      -  ENET_AUTO_PADCRC_DROP_ENABLE/ ENET_AUTO_PADCRC_DROP_DISABLE ;
                      -  ENET_TYPEFRAME_CRC_DROP_ENABLE/ ENET_TYPEFRAME_CRC_DROP_DISABLE ;
                      -  ENET_FORWARD_ERRFRAMES_ENABLE/ ENET_FORWARD_ERRFRAMES_DISABLE ;
                      -  ENET_FORWARD_UNDERSZ_GOODFRAMES_ENABLE/ ENET_FORWARD_UNDERSZ_GOODFRAMES_DISABLE .
                      DMABUS_OPTION related parameters:
                      -  ENET_ADDRESS_ALIGN_ENABLE/ ENET_ADDRESS_ALIGN_DISABLE ;
                      -  ENET_FIXED_BURST_ENABLE/ ENET_FIXED_BURST_DISABLE ;
                      -  ENET_MIXED_BURST_ENABLE/ ENET_MIXED_BURST_DISABLE ;
                      DMA_MAXBURST_OPTION related parameters:
                      -  ENET_RXDP_1BEAT/ ENET_RXDP_2BEAT/ ENET_RXDP_4BEAT/
                         ENET_RXDP_8BEAT/ ENET_RXDP_16BEAT/ ENET_RXDP_32BEAT/
                         ENET_RXDP_4xPGBL_4BEAT/ ENET_RXDP_4xPGBL_8BEAT/
                         ENET_RXDP_4xPGBL_16BEAT/ ENET_RXDP_4xPGBL_32BEAT/
                         ENET_RXDP_4xPGBL_64BEAT/ ENET_RXDP_4xPGBL_128BEAT ;
                      -  ENET_PGBL_1BEAT/ ENET_PGBL_2BEAT/ ENET_PGBL_4BEAT/
                         ENET_PGBL_8BEAT/ ENET_PGBL_16BEAT/ ENET_PGBL_32BEAT/
                         ENET_PGBL_4xPGBL_4BEAT/ ENET_PGBL_4xPGBL_8BEAT/
                         ENET_PGBL_4xPGBL_16BEAT/ ENET_PGBL_4xPGBL_32BEAT/
                         ENET_PGBL_4xPGBL_64BEAT/ ENET_PGBL_4xPGBL_128BEAT ;
                      -  ENET_RXTX_DIFFERENT_PGBL/ ENET_RXTX_SAME_PGBL ;
                      DMA_ARBITRATION_OPTION related parameters:
                      -  ENET_ARBITRATION_RXPRIORTX
                      -  ENET_ARBITRATION_RXTX_1_1/ ENET_ARBITRATION_RXTX_2_1/
                         ENET_ARBITRATION_RXTX_3_1/ ENET_ARBITRATION_RXTX_4_1/.
                      STORE_OPTION related parameters:
                      -  ENET_RX_MODE_STOREFORWARD/ ENET_RX_MODE_CUTTHROUGH ;
                      -  ENET_TX_MODE_STOREFORWARD/ ENET_TX_MODE_CUTTHROUGH ;
                      -  ENET_RX_THRESHOLD_64BYTES/ ENET_RX_THRESHOLD_32BYTES/
                         ENET_RX_THRESHOLD_96BYTES/ ENET_RX_THRESHOLD_128BYTES ;
                      -  ENET_TX_THRESHOLD_64BYTES/ ENET_TX_THRESHOLD_128BYTES/
                         ENET_TX_THRESHOLD_192BYTES/ ENET_TX_THRESHOLD_256BYTES/
                         ENET_TX_THRESHOLD_40BYTES/ ENET_TX_THRESHOLD_32BYTES/
                         ENET_TX_THRESHOLD_24BYTES/ ENET_TX_THRESHOLD_16BYTES .
                      DMA_OPTION related parameters:
                      -  ENET_FLUSH_RXFRAME_ENABLE/ ENET_FLUSH_RXFRAME_DISABLE ;
                      -  ENET_SECONDFRAME_OPT_ENABLE/ ENET_SECONDFRAME_OPT_DISABLE ;
                      -  ENET_ENHANCED_DESCRIPTOR/ ENET_NORMAL_DESCRIPTOR .
                      VLAN_OPTION related parameters:
                      -  ENET_VLANTAGCOMPARISON_12BIT/ ENET_VLANTAGCOMPARISON_16BIT ;
                      -  MAC_VLT_VLTI(regval) .
                      FLOWCTL_OPTION related parameters:
                      -  MAC_FCTL_PTM(regval) ;
                      -  ENET_ZERO_QUANTA_PAUSE_ENABLE/ ENET_ZERO_QUANTA_PAUSE_DISABLE ;
                      -  ENET_PAUSETIME_MINUS4/ ENET_PAUSETIME_MINUS28/
                         ENET_PAUSETIME_MINUS144/ENET_PAUSETIME_MINUS256 ;
                      -  ENET_MAC0_AND_UNIQUE_ADDRESS_PAUSEDETECT/ ENET_UNIQUE_PAUSEDETECT ;
                      -  ENET_RX_FLOWCONTROL_ENABLE/ ENET_RX_FLOWCONTROL_DISABLE ;
                      -  ENET_TX_FLOWCONTROL_ENABLE/ ENET_TX_FLOWCONTROL_DISABLE ;
                      -  ENET_ACTIVE_THRESHOLD_256BYTES/ ENET_ACTIVE_THRESHOLD_512BYTES ;
                      -  ENET_ACTIVE_THRESHOLD_768BYTES/ ENET_ACTIVE_THRESHOLD_1024BYTES ;
                      -  ENET_ACTIVE_THRESHOLD_1280BYTES/ ENET_ACTIVE_THRESHOLD_1536BYTES ;
                      -  ENET_ACTIVE_THRESHOLD_1792BYTES ;
                      -  ENET_DEACTIVE_THRESHOLD_256BYTES/ ENET_DEACTIVE_THRESHOLD_512BYTES ;
                      -  ENET_DEACTIVE_THRESHOLD_768BYTES/ ENET_DEACTIVE_THRESHOLD_1024BYTES ;
                      -  ENET_DEACTIVE_THRESHOLD_1280BYTES/ ENET_DEACTIVE_THRESHOLD_1536BYTES ;
                      -  ENET_DEACTIVE_THRESHOLD_1792BYTES .
                      HASHH_OPTION related parameters:
                      -  0x0~0xFFFF FFFFU
                      HASHL_OPTION related parameters:
                      -  0x0~0xFFFF FFFFU
                      FILTER_OPTION related parameters:
                      -  ENET_SRC_FILTER_NORMAL_ENABLE/ ENET_SRC_FILTER_INVERSE_ENABLE/
                         ENET_SRC_FILTER_DISABLE ;
                      -  ENET_DEST_FILTER_INVERSE_ENABLE/ ENET_DEST_FILTER_INVERSE_DISABLE ;
                      -  ENET_MULTICAST_FILTER_HASH_OR_PERFECT/ ENET_MULTICAST_FILTER_HASH/
                         ENET_MULTICAST_FILTER_PERFECT/ ENET_MULTICAST_FILTER_NONE ;
                      -  ENET_UNICAST_FILTER_EITHER/ ENET_UNICAST_FILTER_HASH/
                         ENET_UNICAST_FILTER_PERFECT ;
                      -  ENET_PCFRM_PREVENT_ALL/ ENET_PCFRM_PREVENT_PAUSEFRAME/
                         ENET_PCFRM_FORWARD_ALL/ ENET_PCFRM_FORWARD_FILTERED .
                      HALFDUPLEX_OPTION related parameters:
                      -  ENET_CARRIERSENSE_ENABLE/ ENET_CARRIERSENSE_DISABLE ;
                      -  ENET_RECEIVEOWN_ENABLE/ ENET_RECEIVEOWN_DISABLE ;
                      -  ENET_RETRYTRANSMISSION_ENABLE/ ENET_RETRYTRANSMISSION_DISABLE ;
                      -  ENET_BACKOFFLIMIT_10/ ENET_BACKOFFLIMIT_8/
                         ENET_BACKOFFLIMIT_4/ ENET_BACKOFFLIMIT_1 ;
                      -  ENET_DEFERRALCHECK_ENABLE/ ENET_DEFERRALCHECK_DISABLE .
                      TIMER_OPTION related parameters:
                      -  ENET_WATCHDOG_ENABLE/ ENET_WATCHDOG_DISABLE ;
                      -  ENET_JABBER_ENABLE/ ENET_JABBER_DISABLE ;
                      INTERFRAMEGAP_OPTION related parameters:
                      -  ENET_INTERFRAMEGAP_96BIT/ ENET_INTERFRAMEGAP_88BIT/
                         ENET_INTERFRAMEGAP_80BIT/ ENET_INTERFRAMEGAP_72BIT/
                         ENET_INTERFRAMEGAP_64BIT/ ENET_INTERFRAMEGAP_56BIT/
                         ENET_INTERFRAMEGAP_48BIT/ ENET_INTERFRAMEGAP_40BIT .
    \param[out] none
    \retval     none
*/
void enet_initpara_config(enet_option_enum option, uint32_t para)
{
    switch(option) {
    case FORWARD_OPTION:
        /* choose to configure forward_frame, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)FORWARD_OPTION;
        enet_initpara.forward_frame = para;
        break;
    case DMABUS_OPTION:
        /* choose to configure dmabus_mode, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)DMABUS_OPTION;
        enet_initpara.dmabus_mode = para;
        break;
    case DMA_MAXBURST_OPTION:
        /* choose to configure dma_maxburst, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)DMA_MAXBURST_OPTION;
        enet_initpara.dma_maxburst = para;
        break;
    case DMA_ARBITRATION_OPTION:
        /* choose to configure dma_arbitration, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)DMA_ARBITRATION_OPTION;
        enet_initpara.dma_arbitration = para;
        break;
    case STORE_OPTION:
        /* choose to configure store_forward_mode, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)STORE_OPTION;
        enet_initpara.store_forward_mode = para;
        break;
    case DMA_OPTION:
        /* choose to configure dma_function, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)DMA_OPTION;

#if (0U == SELECT_DESCRIPTORS_ENHANCED_MODE)
        para &= ~ENET_ENHANCED_DESCRIPTOR;
#endif /* 0U == SELECT_DESCRIPTORS_ENHANCED_MODE */

        enet_initpara.dma_function = para;
        break;
    case VLAN_OPTION:
        /* choose to configure vlan_config, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)VLAN_OPTION;
        enet_initpara.vlan_config = para;
        break;
    case FLOWCTL_OPTION:
        /* choose to configure flow_control, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)FLOWCTL_OPTION;
        enet_initpara.flow_control = para;
        break;
    case HASHH_OPTION:
        /* choose to configure hashtable_high, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)HASHH_OPTION;
        enet_initpara.hashtable_high = para;
        break;
    case HASHL_OPTION:
        /* choose to configure hashtable_low, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)HASHL_OPTION;
        enet_initpara.hashtable_low = para;
        break;
    case FILTER_OPTION:
        /* choose to configure framesfilter_mode, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)FILTER_OPTION;
        enet_initpara.framesfilter_mode = para;
        break;
    case HALFDUPLEX_OPTION:
        /* choose to configure halfduplex_param, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)HALFDUPLEX_OPTION;
        enet_initpara.halfduplex_param = para;
        break;
    case TIMER_OPTION:
        /* choose to configure timer_config, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)TIMER_OPTION;
        enet_initpara.timer_config = para;
        break;
    case INTERFRAMEGAP_OPTION:
        /* choose to configure interframegap, and save the configuration parameters */
        enet_initpara.option_enable |= (uint32_t)INTERFRAMEGAP_OPTION;
        enet_initpara.interframegap = para;
        break;
    default:
        break;
    }
}

/*!
    \brief      initialize ENET peripheral with generally concerned parameters and the less cared parameters (API_ID(0x0004U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  mediamode: PHY mode and mac loopback configurations, only one parameter can be selected
                           which is shown as below, refer to enet_mediamode_enum
      \arg        ENET_100M_FULLDUPLEX: 100Mbit/s, full-duplex
      \arg        ENET_100M_HALFDUPLEX: 100Mbit/s, half-duplex
      \arg        ENET_10M_FULLDUPLEX: 10Mbit/s, full-duplex
      \arg        ENET_10M_HALFDUPLEX: 10Mbit/s, half-duplex
      \arg        ENET_LOOPBACKMODE: MAC in loopback mode at the MII
    \param[in]  checksum: IP frame checksum offload function, only one parameter can be selected
                          which is shown as below, refer to enet_mediamode_enum
      \arg        ENET_NO_AUTOCHECKSUM: disable IP frame checksum function
      \arg        ENET_AUTOCHECKSUM_DROP_FAILFRAMES: enable IP frame checksum function
      \arg        ENET_AUTOCHECKSUM_ACCEPT_FAILFRAMES: enable IP frame checksum function, and the received frame
                                                       with only payload error but no other errors will not be dropped
    \param[in]  recept: frame filter function, only one parameter can be selected
                          which is shown as below, refer to enet_frmrecept_enum
      \arg        ENET_PROMISCUOUS_MODE: promiscuous mode enabled
      \arg        ENET_RECEIVEALL: all received frame are forwarded to application
      \arg        ENET_BROADCAST_FRAMES_PASS: the address filters pass all received broadcast frames
      \arg        ENET_BROADCAST_FRAMES_DROP: the address filters filter all incoming broadcast frames
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus enet_init(uint32_t enet_periph, enet_mediamode_enum mediamode, enet_chksumconf_enum checksum, enet_frmrecept_enum recept)
{
    uint32_t reg_value = 0U, reg_temp = 0U, temp = 0U;
    ErrStatus enet_state = ERROR;

    /* initialize ENET peripheral with generally concerned parameters */
    enet_default_init(enet_periph);

    /* 1st, configure mediamode */
    /* after configuring the PHY, use mediamode to configure registers */
    reg_value = ENET_MAC_CFG(enet_periph);
    /* configure ENET_MAC_CFG register */
    reg_value &= (~(ENET_MAC_CFG_SPD | ENET_MAC_CFG_DPM | ENET_MAC_CFG_LBM));
    reg_value |= (uint32_t)mediamode;
    ENET_MAC_CFG(enet_periph) = reg_value;

    /* 2st, configure checksum */
    if(RESET != ((uint32_t)checksum & ENET_CHECKSUMOFFLOAD_ENABLE)) {
        ENET_MAC_CFG(enet_periph) |= ENET_CHECKSUMOFFLOAD_ENABLE;

        reg_value = ENET_DMA_CTL(enet_periph);
        /* configure ENET_DMA_CTL register */
        reg_value &= ~ENET_DMA_CTL_DTCERFD;
        reg_value |= ((uint32_t)checksum & ENET_DMA_CTL_DTCERFD);
        ENET_DMA_CTL(enet_periph) = reg_value;
    }

    /* 3rd, configure recept */
    ENET_MAC_FRMF(enet_periph) |= (uint32_t)recept;

    /* 4th, configure different function options */
    /* configure forward_frame related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)FORWARD_OPTION)) {
        reg_temp = enet_initpara.forward_frame;

        reg_value = ENET_MAC_CFG(enet_periph);
        temp = reg_temp;
        /* configure ENET_MAC_CFG register */
        reg_value &= (~(ENET_MAC_CFG_TFCD | ENET_MAC_CFG_APCD));
        temp &= (ENET_MAC_CFG_TFCD | ENET_MAC_CFG_APCD);
        reg_value |= temp;
        ENET_MAC_CFG(enet_periph) = reg_value;

        reg_value = ENET_DMA_CTL(enet_periph);
        temp = reg_temp;
        /* configure ENET_DMA_CTL register */
        reg_value &= (~(ENET_DMA_CTL_FERF | ENET_DMA_CTL_FUF));
        temp &= ((ENET_DMA_CTL_FERF | ENET_DMA_CTL_FUF) << 2U);
        reg_value |= (temp >> 2U);
        ENET_DMA_CTL(enet_periph) = reg_value;
    }

    /* configure dmabus_mode related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)DMABUS_OPTION)) {
        temp = enet_initpara.dmabus_mode;

        reg_value = ENET_DMA_BCTL(enet_periph);
        /* configure ENET_DMA_BCTL register */
        reg_value &= ~(ENET_DMA_BCTL_AA | ENET_DMA_BCTL_FB \
                       | ENET_DMA_BCTL_FPBL | ENET_DMA_BCTL_MB);
        reg_value |= temp;
        ENET_DMA_BCTL(enet_periph) = reg_value;
    }

    /* configure dma_maxburst related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)DMA_MAXBURST_OPTION)) {
        temp = enet_initpara.dma_maxburst;

        reg_value = ENET_DMA_BCTL(enet_periph);
        /* configure ENET_DMA_BCTL register */
        reg_value &= ~(ENET_DMA_BCTL_RXDP | ENET_DMA_BCTL_PGBL | ENET_DMA_BCTL_UIP);
        reg_value |= temp;
        ENET_DMA_BCTL(enet_periph) = reg_value;
    }

    /* configure dma_arbitration related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)DMA_ARBITRATION_OPTION)) {
        temp = enet_initpara.dma_arbitration;

        reg_value = ENET_DMA_BCTL(enet_periph);
        /* configure ENET_DMA_BCTL register */
        reg_value &= ~(ENET_DMA_BCTL_RTPR | ENET_DMA_BCTL_DAB);
        reg_value |= temp;
        ENET_DMA_BCTL(enet_periph) = reg_value;
    }

    /* configure store_forward_mode related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)STORE_OPTION)) {
        temp = enet_initpara.store_forward_mode;

        reg_value = ENET_DMA_CTL(enet_periph);
        /* configure ENET_DMA_CTL register */
        reg_value &= ~(ENET_DMA_CTL_RSFD | ENET_DMA_CTL_TSFD | ENET_DMA_CTL_RTHC | ENET_DMA_CTL_TTHC);
        reg_value |= temp;
        ENET_DMA_CTL(enet_periph) = reg_value;
    }

    /* configure dma_function related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)DMA_OPTION)) {
        reg_temp = enet_initpara.dma_function;

        reg_value = ENET_DMA_CTL(enet_periph);
        temp = reg_temp;
        /* configure ENET_DMA_CTL register */
        reg_value &= (~(ENET_DMA_CTL_DAFRF | ENET_DMA_CTL_OSF));
        temp &= (ENET_DMA_CTL_DAFRF | ENET_DMA_CTL_OSF);
        reg_value |= temp;
        ENET_DMA_CTL(enet_periph) = reg_value;

        reg_value = ENET_DMA_BCTL(enet_periph);
        temp = reg_temp;
        /* configure ENET_DMA_BCTL register */
        reg_value &= (~ENET_DMA_BCTL_DFM);
        temp &= ENET_DMA_BCTL_DFM;
        reg_value |= temp;
        ENET_DMA_BCTL(enet_periph) = reg_value;
    }

    /* configure vlan_config related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)VLAN_OPTION)) {
        reg_temp = enet_initpara.vlan_config;

        reg_value = ENET_MAC_VLT(enet_periph);
        /* configure ENET_MAC_VLT register */
        reg_value &= ~(ENET_MAC_VLT_VLTI | ENET_MAC_VLT_VLTC);
        reg_value |= reg_temp;
        ENET_MAC_VLT(enet_periph) = reg_value;
    }

    /* configure flow_control related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)FLOWCTL_OPTION)) {
        reg_temp = enet_initpara.flow_control;

        reg_value = ENET_MAC_FCTL(enet_periph);
        temp = reg_temp;
        /* configure ENET_MAC_FCTL register */
        reg_value &= ~(ENET_MAC_FCTL_PTM | ENET_MAC_FCTL_DZQP | ENET_MAC_FCTL_PLTS \
                       | ENET_MAC_FCTL_UPFDT | ENET_MAC_FCTL_RFCEN | ENET_MAC_FCTL_TFCEN);
        temp &= (ENET_MAC_FCTL_PTM | ENET_MAC_FCTL_DZQP | ENET_MAC_FCTL_PLTS \
                 | ENET_MAC_FCTL_UPFDT | ENET_MAC_FCTL_RFCEN | ENET_MAC_FCTL_TFCEN);
        reg_value |= temp;
        ENET_MAC_FCTL(enet_periph) = reg_value;

        reg_value = ENET_MAC_FCTH(enet_periph);
        temp = reg_temp;
        /* configure ENET_MAC_FCTH register */
        reg_value &= ~(ENET_MAC_FCTH_RFA | ENET_MAC_FCTH_RFD);
        temp &= ((ENET_MAC_FCTH_RFA | ENET_MAC_FCTH_RFD) << 8U);
        reg_value |= (temp >> 8U);
        ENET_MAC_FCTH(enet_periph) = reg_value;
    }

    /* configure hashtable_high related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)HASHH_OPTION)) {
        ENET_MAC_HLH(enet_periph) = enet_initpara.hashtable_high;
    }

    /* configure hashtable_low related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)HASHL_OPTION)) {
        ENET_MAC_HLL(enet_periph) = enet_initpara.hashtable_low;
    }

    /* configure framesfilter_mode related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)FILTER_OPTION)) {
        reg_temp = enet_initpara.framesfilter_mode;

        reg_value = ENET_MAC_FRMF(enet_periph);
        /* configure ENET_MAC_FRMF register */
        reg_value &= ~(ENET_MAC_FRMF_SAFLT | ENET_MAC_FRMF_SAIFLT | ENET_MAC_FRMF_DAIFLT \
                       | ENET_MAC_FRMF_HMF | ENET_MAC_FRMF_HPFLT | ENET_MAC_FRMF_MFD \
                       | ENET_MAC_FRMF_HUF | ENET_MAC_FRMF_PCFRM);
        reg_value |= reg_temp;
        ENET_MAC_FRMF(enet_periph) = reg_value;
    }

    /* configure halfduplex_param related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)HALFDUPLEX_OPTION)) {
        reg_temp = enet_initpara.halfduplex_param;

        reg_value = ENET_MAC_CFG(enet_periph);
        /* configure ENET_MAC_CFG register */
        reg_value &= ~(ENET_MAC_CFG_CSD | ENET_MAC_CFG_ROD | ENET_MAC_CFG_RTD \
                       | ENET_MAC_CFG_BOL | ENET_MAC_CFG_DFC);
        reg_value |= reg_temp;
        ENET_MAC_CFG(enet_periph) = reg_value;
    }

    /* configure timer_config related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)TIMER_OPTION)) {
        reg_temp = enet_initpara.timer_config;

        reg_value = ENET_MAC_CFG(enet_periph);
        /* configure ENET_MAC_CFG register */
        reg_value &= ~(ENET_MAC_CFG_WDD | ENET_MAC_CFG_JBD);
        reg_value |= reg_temp;
        ENET_MAC_CFG(enet_periph) = reg_value;
    }

    /* configure interframegap related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)INTERFRAMEGAP_OPTION)) {
        reg_temp = enet_initpara.interframegap;

        reg_value = ENET_MAC_CFG(enet_periph);
        /* configure ENET_MAC_CFG register */
        reg_value &= ~ENET_MAC_CFG_IGBS;
        reg_value |= reg_temp;
        ENET_MAC_CFG(enet_periph) = reg_value;
    }

    enet_state = SUCCESS;

    return enet_state;
}

/*!
    \brief      reset all core internal registers located in CLK_TX and CLK_RX (API_ID(0x0005U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus enet_software_reset(uint32_t enet_periph)
{
    uint32_t timeout = 0U;
    ErrStatus enet_state = ERROR;
    uint32_t dma_flag;

    /* reset all core internal registers located in CLK_TX and CLK_RX */
    ENET_DMA_BCTL(enet_periph) |= ENET_DMA_BCTL_SWR;

    /* wait for reset operation complete */
    do {
        dma_flag = (ENET_DMA_BCTL(enet_periph) & ENET_DMA_BCTL_SWR);
        timeout++;
    } while((RESET != dma_flag) && (ENET_DELAY_TO != timeout));

    /* reset operation complete */
    if(RESET == (ENET_DMA_BCTL(enet_periph) & ENET_DMA_BCTL_SWR)) {
        enet_state = SUCCESS;
    }

    return enet_state;
}

/*!
    \brief      set MAC address (API_ID(0x0006U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  mac_addr: select which MAC address will be set,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_ADDRESS0: set MAC address 0 filter
      \arg        ENET_MAC_ADDRESS1: set MAC address 1 filter
      \arg        ENET_MAC_ADDRESS2: set MAC address 2 filter
      \arg        ENET_MAC_ADDRESS3: set MAC address 3 filter
    \param[in]  paddr: the buffer pointer which stores the MAC address
                  (little-endian store, such as MAC address is aa:bb:cc:dd:ee:22, the buffer is {22, ee, dd, cc, bb, aa})
    \param[out] none
    \retval     none
*/
void enet_mac_address_set(uint32_t enet_periph, enet_macaddress_enum mac_addr, uint8_t paddr[])
{
    REG32(ENET_ADDRH_BASE(enet_periph) + (uint32_t)mac_addr) = ENET_SET_MACADDRH(paddr);
    REG32(ENET_ADDRL_BASE(enet_periph) + (uint32_t)mac_addr) = ENET_SET_MACADDRL(paddr);
}

/*!
    \brief      get MAC address (API_ID(0x0007U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  mac_addr: select which MAC address will be get,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_ADDRESS0: get MAC address 0 filter
      \arg        ENET_MAC_ADDRESS1: get MAC address 1 filter
      \arg        ENET_MAC_ADDRESS2: get MAC address 2 filter
      \arg        ENET_MAC_ADDRESS3: get MAC address 3 filter
    \param[out] paddr: the buffer pointer which is stored the MAC address
                  (little-endian store, such as MAC address is aa:bb:cc:dd:ee:22, the buffer is {22, ee, dd, cc, bb, aa})
    \param[in]  bufsize: refer to the size of the buffer which stores the MAC address
      \arg        6 - 255
    \retval     ErrStatus: ERROR, SUCCESS
*/
ErrStatus enet_mac_address_get(uint32_t enet_periph, enet_macaddress_enum mac_addr, uint8_t paddr[], uint8_t bufsize)
{
    ErrStatus ret = ERROR;

    if(bufsize >= 6U) {
        paddr[0] = ENET_GET_MACADDR(enet_periph, mac_addr, 0U);
        paddr[1] = ENET_GET_MACADDR(enet_periph, mac_addr, 1U);
        paddr[2] = ENET_GET_MACADDR(enet_periph, mac_addr, 2U);
        paddr[3] = ENET_GET_MACADDR(enet_periph, mac_addr, 3U);
        paddr[4] = ENET_GET_MACADDR(enet_periph, mac_addr, 4U);
        paddr[5] = ENET_GET_MACADDR(enet_periph, mac_addr, 5U);
        ret = SUCCESS;
    }

    return ret;
}

/*!
    \brief      ENET Tx function enable (include MAC and DMA module) (API_ID(0x0008U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_tx_enable(uint32_t enet_periph)
{
    ENET_MAC_CFG(enet_periph) |= ENET_MAC_CFG_TEN;
    enet_txfifo_flush(enet_periph);
    ENET_DMA_CTL(enet_periph) |= ENET_DMA_CTL_STE;
}

/*!
    \brief      ENET Tx function disable (include MAC and DMA module) (API_ID(0x0009U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_tx_disable(uint32_t enet_periph)
{
    ENET_DMA_CTL(enet_periph) &= ~ENET_DMA_CTL_STE;
    enet_txfifo_flush(enet_periph);
    ENET_MAC_CFG(enet_periph) &= ~ENET_MAC_CFG_TEN;
}

/*!
    \brief      ENET Rx function enable (include MAC and DMA module) (API_ID(0x000AU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_rx_enable(uint32_t enet_periph)
{
    ENET_MAC_CFG(enet_periph) |= ENET_MAC_CFG_REN;
    ENET_DMA_CTL(enet_periph) |= ENET_DMA_CTL_SRE;
}

/*!
    \brief      ENET Rx function disable (include MAC and DMA module) (API_ID(0x000BU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_rx_disable(uint32_t enet_periph)
{
    ENET_DMA_CTL(enet_periph) &= ~ENET_DMA_CTL_SRE;
    ENET_MAC_CFG(enet_periph) &= ~ENET_MAC_CFG_REN;
}

/*!
    \brief      put registers value into the application buffer (API_ID(0x000CU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  type: register type which will be get, refer to enet_registers_type_enum,
                only one parameter can be selected which is shown as below
      \arg        ALL_MAC_REG: get the registers within the offset scope between ENET_MAC_CFG and ENET_MAC_FCTH
      \arg        ALL_MSC_REG: get the registers within the offset scope between ENET_MSC_CTL and ENET_MSC_RGUFCNT
      \arg        ALL_PTP_REG: get the registers within the offset scope between ENET_PTP_TSCTL and ENET_PTP_PPSCTL
      \arg        ALL_DMA_REG: get the registers within the offset scope between ENET_DMA_BCTL and ENET_DMA_CRBADDR
    \param[in]  num: the number of registers that the user wants to get
    \param[out] preg: the application buffer pointer for storing the register value
    \retval     none
*/
void enet_registers_get(uint32_t enet_periph, enet_registers_type_enum type, uint32_t preg[], uint32_t num)
{
    uint32_t offset = 0U, max = 0U, limit = 0U, i = 0U;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(preg)) {
        fw_debug_report_err(ENET_MODULE_ID, API_ID(0x000C), ERR_PARAM_POINTER);
    } else
#endif
    {
        offset = (uint32_t)type;
        max = (uint32_t)type + num;
        limit = sizeof(enet_reg_tab) / sizeof(uint16_t);

        /* prevent element in this array is out of range */
        if(max > limit) {
            max = limit;
        }

        for(; offset < max; offset++) {
            /* get value of the corresponding register */
            preg[i] = REG32((enet_periph) + enet_reg_tab[offset]);
            i++;
        }
    }
}

/*!
    \brief      get the ENET debug status from the debug register (API_ID(0x000DU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  mac_debug: ENET debug status,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_RECEIVER_NOT_IDLE: MAC receiver is not in idle state
      \arg        ENET_RX_ASYNCHRONOUS_FIFO_STATE: Rx asynchronous FIFO status
      \arg        ENET_RXFIFO_WRITING: RxFIFO is doing write operation
      \arg        ENET_RXFIFO_READ_STATUS: RxFIFO read operation status
      \arg        ENET_RXFIFO_STATE: RxFIFO state
      \arg        ENET_MAC_TRANSMITTER_NOT_IDLE: MAC transmitter is not in idle state
      \arg        ENET_MAC_TRANSMITTER_STATUS: status of MAC transmitter
      \arg        ENET_PAUSE_CONDITION_STATUS: pause condition status
      \arg        ENET_TXFIFO_READ_STATUS: TxFIFO read operation status
      \arg        ENET_TXFIFO_WRITING: TxFIFO is doing write operation
      \arg        ENET_TXFIFO_NOT_EMPTY: TxFIFO is not empty
      \arg        ENET_TXFIFO_FULL: TxFIFO is full
    \param[out] none
    \retval     value of the status users want to get
*/
uint32_t enet_debug_status_get(uint32_t enet_periph, uint32_t mac_debug)
{
    uint32_t temp_state = 0U;

    switch(mac_debug) {
    case ENET_RX_ASYNCHRONOUS_FIFO_STATE:
        temp_state = GET_MAC_DBG_RXAFS(ENET_MAC_DBG(enet_periph));
        break;
    case ENET_RXFIFO_READ_STATUS:
        temp_state = GET_MAC_DBG_RXFRS(ENET_MAC_DBG(enet_periph));
        break;
    case ENET_RXFIFO_STATE:
        temp_state = GET_MAC_DBG_RXFS(ENET_MAC_DBG(enet_periph));
        break;
    case ENET_MAC_TRANSMITTER_STATUS:
        temp_state = GET_MAC_DBG_SOMT(ENET_MAC_DBG(enet_periph));
        break;
    case ENET_TXFIFO_READ_STATUS:
        temp_state = GET_MAC_DBG_TXFRS(ENET_MAC_DBG(enet_periph));
        break;
    default:
        if(RESET != (ENET_MAC_DBG(enet_periph) & mac_debug)) {
            temp_state = 0x00000001U;
        }
        break;
    }
    return temp_state;
}

/*!
    \brief      enable the MAC address filter (API_ID(0x000EU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  mac_addr: select which MAC address will be enabled,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_ADDRESS1: enable MAC address 1 filter
      \arg        ENET_MAC_ADDRESS2: enable MAC address 2 filter
      \arg        ENET_MAC_ADDRESS3: enable MAC address 3 filter
    \param[out] none
    \retval     none
*/
void enet_address_filter_enable(uint32_t enet_periph, enet_macaddress_enum mac_addr)
{
    REG32(ENET_ADDRH_BASE(enet_periph) + mac_addr) |= ENET_MAC_ADDR1H_AFE;
}

/*!
    \brief      disable the MAC address filter (API_ID(0x000FU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  mac_addr: select which MAC address will be disabled,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_ADDRESS1: disable MAC address 1 filter
      \arg        ENET_MAC_ADDRESS2: disable MAC address 2 filter
      \arg        ENET_MAC_ADDRESS3: disable MAC address 3 filter
    \param[out] none
    \retval     none
*/
void enet_address_filter_disable(uint32_t enet_periph, enet_macaddress_enum mac_addr)
{
    REG32(ENET_ADDRH_BASE(enet_periph) + mac_addr) &= ~ENET_MAC_ADDR1H_AFE;
}

/*!
    \brief      configure the MAC address filter (API_ID(0x0010U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  mac_addr: select which MAC address will be configured,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_ADDRESS1: configure MAC address 1 filter
      \arg        ENET_MAC_ADDRESS2: configure MAC address 2 filter
      \arg        ENET_MAC_ADDRESS3: configure MAC address 3 filter
    \param[in]  addr_mask: select which MAC address bytes will be masked,
                one or more parameters can be selected which are shown as below
      \arg        ENET_ADDRESS_MASK_BYTE0: mask ENET_MAC_ADDR1L[7:0] bits
      \arg        ENET_ADDRESS_MASK_BYTE1: mask ENET_MAC_ADDR1L[15:8] bits
      \arg        ENET_ADDRESS_MASK_BYTE2: mask ENET_MAC_ADDR1L[23:16] bits
      \arg        ENET_ADDRESS_MASK_BYTE3: mask ENET_MAC_ADDR1L [31:24] bits
      \arg        ENET_ADDRESS_MASK_BYTE4: mask ENET_MAC_ADDR1H [7:0] bits
      \arg        ENET_ADDRESS_MASK_BYTE5: mask ENET_MAC_ADDR1H [15:8] bits
    \param[in]  filter_type: select which MAC address filter type will be selected,
                only one parameter can be selected which is shown as below
      \arg        ENET_ADDRESS_FILTER_SA: The MAC address is used to compare with the SA field of the received frame
      \arg        ENET_ADDRESS_FILTER_DA: The MAC address is used to compare with the DA field of the received frame
    \param[out] none
    \retval     none
*/
void enet_address_filter_config(uint32_t enet_periph, enet_macaddress_enum mac_addr, uint32_t addr_mask, uint32_t filter_type)
{
    uint32_t reg;

    /* get the address filter register value which is to be configured */
    reg = REG32(ENET_ADDRH_BASE(enet_periph) + mac_addr);

    /* clear and configure the address filter register */
    reg &= ~(ENET_MAC_ADDR1H_MB | ENET_MAC_ADDR1H_SAF);
    reg |= ((addr_mask & ENET_MAC_ADDR1H_MB) | (filter_type & ENET_ADDRESS_FILTER_SA));
    REG32(ENET_ADDRH_BASE(enet_periph) + mac_addr) = reg;
}

/*!
    \brief      PHY interface configuration (configure SMI clock and reset PHY chip) (API_ID(0x0011U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus enet_phy_config(uint32_t enet_periph)
{
    uint32_t ahbclk;
    uint32_t reg;
    ErrStatus enet_state = ERROR;

    /* clear the previous MDC clock */
    reg = ENET_MAC_PHY_CTL(enet_periph);
    reg &= ~ENET_MAC_PHY_CTL_CLR;

    /* get the HCLK frequency */
    ahbclk = rcu_clock_freq_get(CK_AHB);

    /* configure MDC clock according to HCLK frequency range */
    if(ENET_RANGE(ahbclk, 20000000U, 35000000U)) {
        reg |= ENET_MDC_HCLK_DIV16;
    } else if(ENET_RANGE(ahbclk, 35000000U, 60000000U)) {
        reg |= ENET_MDC_HCLK_DIV26;
    } else if(ENET_RANGE(ahbclk, 60000000U, 100000000U)) {
        reg |= ENET_MDC_HCLK_DIV42;
    } else if(ENET_RANGE(ahbclk, 100000000U, 150000000U)) {
        reg |= ENET_MDC_HCLK_DIV62;
    } else if(ENET_RANGE(ahbclk, 150000000U, 250000000U)) {
        reg |= ENET_MDC_HCLK_DIV102;
    } else if(ENET_RANGE(ahbclk, 250000000U, 300000000U)) {
        reg |= ENET_MDC_HCLK_DIV124;
    } else if((ENET_RANGE(ahbclk, 300000000U, 350000000U)) || (350000000U == ahbclk)) {
        reg |= ENET_MDC_HCLK_DIV142;
    } else {
    }
    /* only proceed if clock configuration is valid */
    if(ENET_RANGE(ahbclk, 20000000U, 350000000U) || (350000000U == ahbclk)) {
        ENET_MAC_PHY_CTL(enet_periph) = reg;
        enet_state = SUCCESS;
    }

    return enet_state;
}

/*!
    \brief      write to / read from a PHY register (API_ID(0x0012U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  direction: only one parameter can be selected which is shown as below
      \arg        ENET_PHY_WRITE: write data to phy register
      \arg        ENET_PHY_READ:  read data from phy register
    \param[in]  phy_address: 0x0 - 0x1F, the address of the PHY chip
    \param[in]  phy_reg: 0x0 - 0x1F, the register address of the PHY chip
    \param[in]  pvalue: the value will be written to the PHY register in ENET_PHY_WRITE direction
    \param[out] pvalue: the value will be read from the PHY register in ENET_PHY_READ direction
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus enet_phy_write_read(uint32_t enet_periph, enet_phydirection_enum direction, uint16_t phy_address, uint16_t phy_reg, uint16_t *pvalue)
{
    uint32_t reg, phy_flag;
    uint32_t timeout = 0U;
    ErrStatus enet_state = ERROR;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(pvalue)) {
        fw_debug_report_err(ENET_MODULE_ID, API_ID(0x0012), ERR_PARAM_POINTER);
    } else
#endif
    {
        /* configure ENET_MAC_PHY_CTL with write/read operation */
        reg = ENET_MAC_PHY_CTL(enet_periph);
        reg &= ~(ENET_MAC_PHY_CTL_PB | ENET_MAC_PHY_CTL_PW | ENET_MAC_PHY_CTL_PR | ENET_MAC_PHY_CTL_PA);
        reg |= (direction | MAC_PHY_CTL_PR((uint32_t)phy_reg & PHY_ADDR_REG_MASK) | MAC_PHY_CTL_PA((uint32_t)phy_address & PHY_ADDR_REG_MASK) |
                ENET_MAC_PHY_CTL_PB);

        /* if do the write operation, write value to the register */
        if(ENET_PHY_WRITE == direction) {
            ENET_MAC_PHY_DATA(enet_periph) = *pvalue;
        }

        /* do PHY write/read operation, and wait the operation complete */
        ENET_MAC_PHY_CTL(enet_periph) = reg;
        do {
            phy_flag = (ENET_MAC_PHY_CTL(enet_periph) & ENET_MAC_PHY_CTL_PB);
            timeout++;
        } while((RESET != phy_flag) && (ENET_DELAY_TO != timeout));

        /* write/read operation complete */
        if(RESET == (ENET_MAC_PHY_CTL(enet_periph) & ENET_MAC_PHY_CTL_PB)) {
            enet_state = SUCCESS;
        }

        /* if do the read operation, get value from the register */
        if(ENET_PHY_READ == direction) {
            *pvalue = (uint16_t)ENET_MAC_PHY_DATA(enet_periph);
        }
    }
    return enet_state;
}

/*!
    \brief      enable ENET forward feature (API_ID(0x0013U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of ENET forward mode,
                one or more parameters can be selected which are shown as below
      \arg        ENET_AUTO_PADCRC_DROP: the function of the MAC strips the Pad/FCS field on received frames
      \arg        ENET_TYPEFRAME_CRC_DROP: the function that FCS field(last 4 bytes) of frame will be dropped before forwarding
      \arg        ENET_FORWARD_ERRFRAMES: the function that all frame received with error except runt error are forwarded to memory
      \arg        ENET_FORWARD_UNDERSZ_GOODFRAMES: the function that forwarding undersized good frames
    \param[out] none
    \retval     none
*/
void enet_forward_feature_enable(uint32_t enet_periph, uint32_t feature)
{
    uint32_t mask;

    mask = (feature & (~(ENET_FORWARD_ERRFRAMES | ENET_FORWARD_UNDERSZ_GOODFRAMES)));
    ENET_MAC_CFG(enet_periph) |= mask;

    mask = (feature & (~(ENET_AUTO_PADCRC_DROP | ENET_TYPEFRAME_CRC_DROP)));
    ENET_DMA_CTL(enet_periph) |= (mask >> 2U);
}

/*!
    \brief      disable ENET forward feature (API_ID(0x0014U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of ENET forward mode,
                one or more parameters can be selected which are shown as below
      \arg        ENET_AUTO_PADCRC_DROP: the function of the MAC strips the Pad/FCS field on received frames
      \arg        ENET_TYPEFRAME_CRC_DROP: the function that FCS field(last 4 bytes) of frame will be dropped before forwarding
      \arg        ENET_FORWARD_ERRFRAMES: the function that all frame received with error except runt error are forwarded to memory
      \arg        ENET_FORWARD_UNDERSZ_GOODFRAMES: the function that forwarding undersized good frames
    \param[out] none
    \retval     none
*/
void enet_forward_feature_disable(uint32_t enet_periph, uint32_t feature)
{
    uint32_t mask;

    mask = (feature & (~(ENET_FORWARD_ERRFRAMES | ENET_FORWARD_UNDERSZ_GOODFRAMES)));
    ENET_MAC_CFG(enet_periph) &= ~mask;

    mask = (feature & (~(ENET_AUTO_PADCRC_DROP | ENET_TYPEFRAME_CRC_DROP)));
    ENET_DMA_CTL(enet_periph) &= ~(mask >> 2U);
}

/*!
    \brief      enable ENET filter feature (API_ID(0x0015U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of ENET filter mode,
                one or more parameters can be selected which are shown as below
      \arg        ENET_SRC_FILTER: filter source address function
      \arg        ENET_SRC_FILTER_INVERSE: inverse source address filtering result function
      \arg        ENET_DEST_FILTER_INVERSE: inverse DA filtering result function
      \arg        ENET_MULTICAST_FILTER_PASS: pass all multicast frames function
      \arg        ENET_MULTICAST_FILTER_HASH_MODE: HASH multicast filter function
      \arg        ENET_UNICAST_FILTER_HASH_MODE: HASH unicast filter function
      \arg        ENET_FILTER_MODE_EITHER: HASH or perfect filter function
    \param[out] none
    \retval     none
*/
void enet_filter_feature_enable(uint32_t enet_periph, uint32_t feature)
{
    ENET_MAC_FRMF(enet_periph) |= (feature & FILTER_FEATURE_MASK);
}

/*!
    \brief      disable ENET filter feature (API_ID(0x0016U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of ENET filter mode,
                one or more parameters can be selected which are shown as below
      \arg        ENET_SRC_FILTER: filter source address function
      \arg        ENET_SRC_FILTER_INVERSE: inverse source address filtering result function
      \arg        ENET_DEST_FILTER_INVERSE: inverse DA filtering result function
      \arg        ENET_MULTICAST_FILTER_PASS: pass all multicast frames function
      \arg        ENET_MULTICAST_FILTER_HASH_MODE: HASH multicast filter function
      \arg        ENET_UNICAST_FILTER_HASH_MODE: HASH unicast filter function
      \arg        ENET_FILTER_MODE_EITHER: HASH or perfect filter function
    \param[out] none
    \retval     none
*/
void enet_filter_feature_disable(uint32_t enet_periph, uint32_t feature)
{
    ENET_MAC_FRMF(enet_periph) &= ~(feature & FILTER_FEATURE_MASK);
}

/*!
    \brief      generate the pause frame, ENET will send pause frame after enable transmit flow control (API_ID(0x0017U))
                this function only use in full-duplex mode
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus enet_pauseframe_generate(uint32_t enet_periph)
{
    ErrStatus enet_state = ERROR;
    uint32_t temp = 0U;

    /* in full-duplex mode, must make sure this bit is 0 before writing register */
    temp = ENET_MAC_FCTL(enet_periph) & ENET_MAC_FCTL_FLCBBKPA;
    if(RESET == temp) {
        ENET_MAC_FCTL(enet_periph) |= ENET_MAC_FCTL_FLCBBKPA;
        enet_state = SUCCESS;
    }
    return enet_state;
}

/*!
    \brief      configure the pause frame detect type (API_ID(0x0018U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  detect: pause frame detect type,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC0_AND_UNIQUE_ADDRESS_PAUSEDETECT: besides the unique multicast address, MAC can also
                                                            use the MAC0 address to detecting pause frame
      \arg        ENET_UNIQUE_PAUSEDETECT: only the unique multicast address for pause frame which is specified
                                           in IEEE802.3 can be detected
    \param[out] none
    \retval     none
*/
void enet_pauseframe_detect_config(uint32_t enet_periph, uint32_t detect)
{
    ENET_MAC_FCTL(enet_periph) &= ~ENET_MAC_FCTL_UPFDT;
    ENET_MAC_FCTL(enet_periph) |= (detect & ENET_MAC_FCTL_UPFDT);
}

/*!
    \brief      configure the pause frame parameters (API_ID(0x0019U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  pausetime: pause time in transmit pause control frame
    \param[in]  pause_threshold: the threshold of the pause timer for retransmitting frames automatically,
                this value must make sure to be less than configured pause time, only one parameter can be
                selected which is shown as below
      \arg        ENET_PAUSETIME_MINUS4: pause time minus 4 slot times
      \arg        ENET_PAUSETIME_MINUS28: pause time minus 28 slot times
      \arg        ENET_PAUSETIME_MINUS144: pause time minus 144 slot times
      \arg        ENET_PAUSETIME_MINUS256: pause time minus 256 slot times
    \param[out] none
    \retval     none
*/
void enet_pauseframe_config(uint32_t enet_periph, uint32_t pausetime, uint32_t pause_threshold)
{
    ENET_MAC_FCTL(enet_periph) &= ~(ENET_MAC_FCTL_PTM | ENET_MAC_FCTL_PLTS);
    ENET_MAC_FCTL(enet_periph) |= (MAC_FCTL_PTM(pausetime) | (pause_threshold & ENET_MAC_FCTL_PLTS));
}

/*!
    \brief      configure the threshold of the flow control(deactive and active threshold) (API_ID(0x001AU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  deactive: the threshold of the deactive flow control, this value
                should always be less than active flow control value, only one
                parameter can be selected which is shown as below
      \arg        ENET_DEACTIVE_THRESHOLD_256BYTES: threshold level is 256 bytes
      \arg        ENET_DEACTIVE_THRESHOLD_512BYTES: threshold level is 512 bytes
      \arg        ENET_DEACTIVE_THRESHOLD_768BYTES: threshold level is 768 bytes
      \arg        ENET_DEACTIVE_THRESHOLD_1024BYTES: threshold level is 1024 bytes
      \arg        ENET_DEACTIVE_THRESHOLD_1280BYTES: threshold level is 1280 bytes
      \arg        ENET_DEACTIVE_THRESHOLD_1536BYTES: threshold level is 1536 bytes
      \arg        ENET_DEACTIVE_THRESHOLD_1792BYTES: threshold level is 1792 bytes
    \param[in]  active: the threshold of the active flow control, only one parameter
                can be selected which is shown as below
      \arg        ENET_ACTIVE_THRESHOLD_256BYTES: threshold level is 256 bytes
      \arg        ENET_ACTIVE_THRESHOLD_512BYTES: threshold level is 512 bytes
      \arg        ENET_ACTIVE_THRESHOLD_768BYTES: threshold level is 768 bytes
      \arg        ENET_ACTIVE_THRESHOLD_1024BYTES: threshold level is 1024 bytes
      \arg        ENET_ACTIVE_THRESHOLD_1280BYTES: threshold level is 1280 bytes
      \arg        ENET_ACTIVE_THRESHOLD_1536BYTES: threshold level is 1536 bytes
      \arg        ENET_ACTIVE_THRESHOLD_1792BYTES: threshold level is 1792 bytes
    \param[out] none
    \retval     none
*/
void enet_flowcontrol_threshold_config(uint32_t enet_periph, uint32_t deactive, uint32_t active)
{
    ENET_MAC_FCTH(enet_periph) = (((deactive & DEACTIVE_THRESHOLD_MASK) | (active & ACTIVE_THRESHOLD_MASK)) >> 8U);
}

/*!
    \brief      enable ENET flow control feature (API_ID(0x001BU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of ENET flow control mode
                one or more parameters can be selected which are shown as below
      \arg        ENET_ZERO_QUANTA_PAUSE: the automatic zero-quanta generation function
      \arg        ENET_TX_FLOWCONTROL: the flow control operation in the MAC
      \arg        ENET_RX_FLOWCONTROL: decoding function for the received pause frame and process it
      \arg        ENET_BACK_PRESSURE: back pressure operation in the MAC(only use in half-duplex mode)
    \param[out] none
    \retval     none
*/
void enet_flowcontrol_feature_enable(uint32_t enet_periph, uint32_t feature)
{
    feature = (feature & FLOWCONTROL_FEATURE_MASK);

    /* enable the zero-quanta pause function */
    if(RESET != (feature & ENET_ZERO_QUANTA_PAUSE)) {
        ENET_MAC_FCTL(enet_periph) &= ~ENET_ZERO_QUANTA_PAUSE;
    }

    feature &= ~ENET_ZERO_QUANTA_PAUSE;
    ENET_MAC_FCTL(enet_periph) |= feature;
}

/*!
    \brief      disable ENET flow control feature (API_ID(0x001CU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of ENET flow control mode
                one or more parameters can be selected which are shown as below
      \arg        ENET_ZERO_QUANTA_PAUSE: the automatic zero-quanta generation function
      \arg        ENET_TX_FLOWCONTROL: the flow control operation in the MAC
      \arg        ENET_RX_FLOWCONTROL: decoding function for the received pause frame and process it
      \arg        ENET_BACK_PRESSURE: back pressure operation in the MAC(only use in half-duplex mode)
    \param[out] none
    \retval     none
*/
void enet_flowcontrol_feature_disable(uint32_t enet_periph, uint32_t feature)
{
    feature = (feature & FLOWCONTROL_FEATURE_MASK);

    if(RESET != (feature & ENET_ZERO_QUANTA_PAUSE)) {
        ENET_MAC_FCTL(enet_periph) |= ENET_ZERO_QUANTA_PAUSE;
    }

    feature &= ~ENET_ZERO_QUANTA_PAUSE;
    ENET_MAC_FCTL(enet_periph) &= ~feature;
}

/*!
    \brief      get the dma transmit/receive process state (API_ID(0x001DU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  direction: choose the direction of dma process which users want to check,
                refer to enet_dmadirection_enum, only one parameter can be selected which is shown as below
      \arg        ENET_DMA_TX: dma transmit process
      \arg        ENET_DMA_RX: dma receive process
    \param[out] none
    \retval     state of dma process, the value range shows below:
                  ENET_RX_STATE_STOPPED, ENET_RX_STATE_FETCHING, ENET_RX_STATE_WAITING,
                  ENET_RX_STATE_SUSPENDED, ENET_RX_STATE_CLOSING, ENET_RX_STATE_QUEUING,
                  ENET_TX_STATE_STOPPED, ENET_TX_STATE_FETCHING, ENET_TX_STATE_WAITING,
                  ENET_TX_STATE_READING, ENET_TX_STATE_SUSPENDED, ENET_TX_STATE_CLOSING
*/
uint32_t enet_dmaprocess_state_get(uint32_t enet_periph, enet_dmadirection_enum direction)
{
    uint32_t reval;
    reval = (uint32_t)(ENET_DMA_STAT(enet_periph) & (uint32_t)direction);
    return reval;
}

/*!
    \brief      poll the DMA transmission/reception enable by writing any value to the
                ENET_DMA_TPEN/ENET_DMA_RPEN register, this will make the DMA to resume transmission/reception (API_ID(0x001EU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  direction: choose the direction of DMA process which users want to resume,
                refer to enet_dmadirection_enum, only one parameter can be selected which is shown as below
      \arg        ENET_DMA_TX: DMA transmit process
      \arg        ENET_DMA_RX: DMA receive process
    \param[out] none
    \retval     none
*/
void enet_dmaprocess_resume(uint32_t enet_periph, enet_dmadirection_enum direction)
{
    if(ENET_DMA_TX == direction) {
        ENET_DMA_TPEN(enet_periph) = 0U;
    } else {
        ENET_DMA_RPEN(enet_periph) = 0U;
    }
}

/*!
    \brief      flush the ENET transmit FIFO, and wait until the flush operation completes (API_ID(0x001FU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus enet_txfifo_flush(uint32_t enet_periph)
{
    uint32_t flush_state;
    uint32_t timeout = 0U;
    ErrStatus enet_state = ERROR;

    /* set the FTF bit for flushing transmit FIFO */
    ENET_DMA_CTL(enet_periph) |= ENET_DMA_CTL_FTF;
    /* wait until the flush operation completes */
    do {
        flush_state = ENET_DMA_CTL(enet_periph) & ENET_DMA_CTL_FTF;
        timeout++;
    } while((RESET != flush_state) && (timeout < ENET_DELAY_TO));
    /* return ERROR due to timeout */
    if(RESET == flush_state) {
        enet_state = SUCCESS;
    }

    return  enet_state;
}

/*!
    \brief      get the transmit/receive address of current descriptor, or current buffer, or descriptor table (API_ID(0x0020U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  addr_get: choose the address which users want to get, refer to enet_desc_reg_enum,
                only one parameter can be selected which is shown as below
      \arg        ENET_RX_DESC_TABLE: the start address of the receive descriptor table
      \arg        ENET_RX_CURRENT_DESC: the start descriptor address of the current receive descriptor read by
                                        the RxDMA controller
      \arg        ENET_RX_CURRENT_BUFFER: the current receive buffer address being read by the RxDMA controller
      \arg        ENET_TX_DESC_TABLE: the start address of the transmit descriptor table
      \arg        ENET_TX_CURRENT_DESC: the start descriptor address of the current transmit descriptor read by
                                        the TxDMA controller
      \arg        ENET_TX_CURRENT_BUFFER: the current transmit buffer address being read by the TxDMA controller
    \param[out] none
    \retval     address value
*/
uint32_t enet_current_desc_address_get(uint32_t enet_periph, enet_desc_reg_enum addr_get)
{
    uint32_t reval = 0U;

    reval = REG32((enet_periph) + (uint32_t)addr_get);
    return reval;
}

/*!
    \brief      get the number of missed frames during receiving (API_ID(0x0021U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] rxfifo_drop: pointer to the number of frames dropped by RxFIFO
    \param[out] rxdma_drop: pointer to the number of frames missed by the RxDMA controller
    \retval     none
*/
void enet_missed_frame_counter_get(uint32_t enet_periph, uint32_t *rxfifo_drop, uint32_t *rxdma_drop)
{
    uint32_t temp_counter = 0U;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameters */
    if(NOT_VALID_POINTER(rxfifo_drop)) {
        fw_debug_report_err(ENET_MODULE_ID, API_ID(0x0021), ERR_PARAM_POINTER);
    } else if(NOT_VALID_POINTER(rxdma_drop)) {
        fw_debug_report_err(ENET_MODULE_ID, API_ID(0x0021), ERR_PARAM_POINTER);
    } else
#endif
    {
        temp_counter = ENET_DMA_MFBOCNT(enet_periph);
        *rxfifo_drop = GET_DMA_MFBOCNT_MSFA(temp_counter);
        *rxdma_drop = GET_DMA_MFBOCNT_MSFC(temp_counter);
    }
}

/*!
    \brief      enable DMA feature (API_ID(0x0022U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of DMA mode,
                one or more parameters can be selected which are shown as below
      \arg        ENET_NO_FLUSH_RXFRAME: RxDMA does not flush frames function
      \arg        ENET_SECONDFRAME_OPT: TxDMA controller operate on second frame function
    \param[out] none
    \retval     none
*/
void enet_dma_feature_enable(uint32_t enet_periph, uint32_t feature)
{
    ENET_DMA_CTL(enet_periph) |= (feature & DMA_FEATURE_MASK);
}

/*!
    \brief      disable DMA feature (API_ID(0x0023U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of DMA mode,
                one or more parameters can be selected which are shown as below
      \arg        ENET_NO_FLUSH_RXFRAME: RxDMA does not flush frames function
      \arg        ENET_SECONDFRAME_OPT: TxDMA controller operate on second frame function
    \param[out] none
    \retval     none
*/
void enet_dma_feature_disable(uint32_t enet_periph, uint32_t feature)
{
    ENET_DMA_CTL(enet_periph) &= ~(feature & DMA_FEATURE_MASK);
}

#if SELECT_DESCRIPTORS_ENHANCED_MODE
/*!
    \brief      configure descriptor to work in enhanced mode (API_ID(0x0024U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_desc_select_enhanced_mode(uint32_t enet_periph)
{
    ENET_DMA_BCTL(enet_periph) |= ENET_DMA_BCTL_DFM;
}

#else

/*!
    \brief      configure descriptor to work in normal mode (API_ID(0x0025U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_desc_select_normal_mode(uint32_t enet_periph)
{
    ENET_DMA_BCTL(enet_periph) &= ~ENET_DMA_BCTL_DFM;
}

#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

/*!
    \brief      wakeup frame filter register pointer reset (API_ID(0x0026U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_wum_filter_register_pointer_reset(uint32_t enet_periph)
{
    ENET_MAC_WUM(enet_periph) |= ENET_MAC_WUM_WUFFRPR;
}

/*!
    \brief      set the remote wakeup frame registers (API_ID(0x0027U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  pdata: pointer to buffer data which is written to remote wakeup frame registers (8 words total)
    \param[out] none
    \retval     none
*/
void enet_wum_filter_config(uint32_t enet_periph, uint32_t pdata[])
{
    uint32_t num = 0U;

    /* configure ENET_MAC_RWFF register */
    for(num = 0U; num < ENET_WAKEUP_REGISTER_LENGTH; num++) {
        ENET_MAC_RWFF(enet_periph) = pdata[num];
    }
}

/*!
    \brief      enable wakeup management features (API_ID(0x0028U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: one or more parameters can be selected which are shown as below
      \arg        ENET_WUM_POWER_DOWN: power down mode
      \arg        ENET_WUM_MAGIC_PACKET_FRAME: enable a wakeup event due to magic packet reception
      \arg        ENET_WUM_WAKE_UP_FRAME: enable a wakeup event due to wakeup frame reception
      \arg        ENET_WUM_GLOBAL_UNICAST: any received unicast frame passed filter is considered to be a wakeup frame
    \param[out] none
    \retval     none
*/
void enet_wum_feature_enable(uint32_t enet_periph, uint32_t feature)
{
    ENET_MAC_WUM(enet_periph) |= (feature & WUM_FEATURE_EN_MASK);
}

/*!
    \brief      disable wakeup management features (API_ID(0x0029U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: one or more parameters can be selected which are shown as below
      \arg        ENET_WUM_MAGIC_PACKET_FRAME: enable a wakeup event due to magic packet reception
      \arg        ENET_WUM_WAKE_UP_FRAME: enable a wakeup event due to wakeup frame reception
      \arg        ENET_WUM_GLOBAL_UNICAST: any received unicast frame passed filter is considered to be a wakeup frame
    \param[out] none
    \retval     none
*/
void enet_wum_feature_disable(uint32_t enet_periph, uint32_t feature)
{
    ENET_MAC_WUM(enet_periph) &= ~(feature & WUM_FEATURE_DIS_MASK);
}

/*!
    \brief      reset the MAC statistics counters (API_ID(0x002AU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
void enet_msc_counters_reset(uint32_t enet_periph)
{
    /* reset all counters */
    ENET_MSC_CTL(enet_periph) |= ENET_MSC_CTL_CTR;
}

/*!
    \brief      enable the MAC statistics counter features (API_ID(0x002BU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: one or more parameters can be selected which are shown as below
      \arg        ENET_MSC_COUNTER_STOP_ROLLOVER: counter stop rollover
      \arg        ENET_MSC_RESET_ON_READ: reset on read
      \arg        ENET_MSC_COUNTERS_FREEZE: MSC counter freeze
    \param[out] none
    \retval     none
*/
void enet_msc_feature_enable(uint32_t enet_periph, uint32_t feature)
{
    ENET_MSC_CTL(enet_periph) |= (feature & MSC_FEATURE_MASK);
}

/*!
    \brief      disable the MAC statistics counter features (API_ID(0x002CU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: one or more parameters can be selected which are shown as below
      \arg        ENET_MSC_COUNTER_STOP_ROLLOVER: counter stop rollover
      \arg        ENET_MSC_RESET_ON_READ: reset on read
      \arg        ENET_MSC_COUNTERS_FREEZE: MSC counter freeze
    \param[out] none
    \retval     none
*/
void enet_msc_feature_disable(uint32_t enet_periph, uint32_t feature)
{
    ENET_MSC_CTL(enet_periph) &= ~(feature & MSC_FEATURE_MASK);
}

/*!
    \brief      configure MAC statistics counters preset mode (API_ID(0x002DU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  mode: MSC counters preset mode, refer to enet_msc_preset_enum,
                only one parameter can be selected which is shown as below
      \arg        ENET_MSC_PRESET_NONE: do not preset MSC counter
      \arg        ENET_MSC_PRESET_HALF: preset all MSC counters to almost-half(0x7FFF FFF0) value
      \arg        ENET_MSC_PRESET_FULL: preset all MSC counters to almost-full(0xFFFF FFF0) value
    \param[out] none
    \retval     none
*/
void enet_msc_counters_preset_config(uint32_t enet_periph, enet_msc_preset_enum mode)
{
    ENET_MSC_CTL(enet_periph) &= ENET_MSC_PRESET_MASK;
    ENET_MSC_CTL(enet_periph) |= (uint32_t)mode;
}

/*!
    \brief      get MAC statistics counter (API_ID(0x002EU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  counter: MSC counters which is selected, refer to enet_msc_counter_enum,
                only one parameter can be selected which is shown as below
      \arg        ENET_MSC_TX_SCCNT: MSC transmitted good frames after a single collision counter
      \arg        ENET_MSC_TX_MSCCNT: MSC transmitted good frames after more than a single collision counter
      \arg        ENET_MSC_TX_TGFCNT: MSC transmitted good frames counter
      \arg        ENET_MSC_RX_RFCECNT: MSC received frames with CRC error counter
      \arg        ENET_MSC_RX_RFAECNT: MSC received frames with alignment error counter
      \arg        ENET_MSC_RX_RGUFCNT: MSC received good unicast frames counter
    \param[out] none
    \retval     the MSC counter value
*/
uint32_t enet_msc_counters_get(uint32_t enet_periph, enet_msc_counter_enum counter)
{
    uint32_t reval;
    reval = REG32((enet_periph + (uint32_t)counter));
    return reval;
}

/*!
    \brief      enable the PTP features (API_ID(0x002FU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of ENET PTP mode
                one or more parameters can be selected which are shown as below
      \arg        ENET_RXTX_TIMESTAMP: timestamp function for transmit and receive frames
      \arg        ENET_PTP_TIMESTAMP_INT: timestamp interrupt trigger
      \arg        ENET_ALL_RX_TIMESTAMP: all received frames are taken snapshot
      \arg        ENET_NONTYPE_FRAME_SNAPSHOT: take snapshot when received non type frame
      \arg        ENET_IPV6_FRAME_SNAPSHOT: take snapshot for IPv6 frame
      \arg        ENET_IPV4_FRAME_SNAPSHOT: take snapshot for IPv4 frame
      \arg        ENET_PTP_FRAME_USE_MACADDRESS_FILTER: use MAC address1-3 to filter the PTP frame
    \param[out] none
    \retval     none
*/
void enet_ptp_feature_enable(uint32_t enet_periph, uint32_t feature)
{
    ENET_PTP_TSCTL(enet_periph) |= (feature & PTP_FAETURE_MASK);
}

/*!
    \brief      disable the PTP features (API_ID(0x0030U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  feature: the feature of ENET PTP mode
                one or more parameters can be selected which are shown as below
      \arg        ENET_RXTX_TIMESTAMP: timestamp function for transmit and receive frames
      \arg        ENET_PTP_TIMESTAMP_INT: timestamp interrupt trigger
      \arg        ENET_ALL_RX_TIMESTAMP: all received frames are taken snapshot
      \arg        ENET_NONTYPE_FRAME_SNAPSHOT: take snapshot when received non type frame
      \arg        ENET_IPV6_FRAME_SNAPSHOT: take snapshot for IPv6 frame
      \arg        ENET_IPV4_FRAME_SNAPSHOT: take snapshot for IPv4 frame
      \arg        ENET_PTP_FRAME_USE_MACADDRESS_FILTER: use MAC address1-3 to filter the PTP frame
    \param[out] none
    \retval     none
*/
void enet_ptp_feature_disable(uint32_t enet_periph, uint32_t feature)
{
    ENET_PTP_TSCTL(enet_periph) &= ~(feature & PTP_FAETURE_MASK);
}

/*!
    \brief      configure the PTP timestamp function (API_ID(0x0031U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  func: only one parameter can be selected which is shown as below
      \arg        ENET_CKNT_ORDINARY: type of ordinary clock node type for timestamp
      \arg        ENET_CKNT_BOUNDARY: type of boundary clock node type for timestamp
      \arg        ENET_CKNT_END_TO_END: type of end-to-end transparent clock node type for timestamp
      \arg        ENET_CKNT_PEER_TO_PEER: type of peer-to-peer transparent clock node type for timestamp
      \arg        ENET_PTP_ADDEND_UPDATE: addend register update
      \arg        ENET_PTP_SYSTIME_UPDATE: timestamp update
      \arg        ENET_PTP_SYSTIME_INIT: timestamp initialize
      \arg        ENET_PTP_FINEMODE: the system timestamp uses the fine method for updating
      \arg        ENET_PTP_COARSEMODE: the system timestamp uses the coarse method for updating
      \arg        ENET_SUBSECOND_DIGITAL_ROLLOVER: digital rollover mode
      \arg        ENET_SUBSECOND_BINARY_ROLLOVER: binary rollover mode
      \arg        ENET_SNOOPING_PTP_VERSION_2: version 2
      \arg        ENET_SNOOPING_PTP_VERSION_1: version 1
      \arg        ENET_EVENT_TYPE_MESSAGES_SNAPSHOT: only event type messages are taken snapshot
      \arg        ENET_ALL_TYPE_MESSAGES_SNAPSHOT: all type messages are taken snapshot except announce,
                                                   management and signaling message
      \arg        ENET_MASTER_NODE_MESSAGE_SNAPSHOT: snapshot is only take for master node message
      \arg        ENET_SLAVE_NODE_MESSAGE_SNAPSHOT: snapshot is only taken for slave node message
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus enet_ptp_timestamp_function_config(uint32_t enet_periph, enet_ptp_function_enum func)
{
    uint32_t temp_config = 0U, temp_state = 0U;
    uint32_t timeout = 0U;
    ErrStatus enet_state = SUCCESS;

    switch(func) {
    case ENET_CKNT_ORDINARY:
    case ENET_CKNT_BOUNDARY:
    case ENET_CKNT_END_TO_END:
    case ENET_CKNT_PEER_TO_PEER:
        ENET_PTP_TSCTL(enet_periph) &= ~ENET_PTP_TSCTL_CKNT;
        ENET_PTP_TSCTL(enet_periph) |= (uint32_t)func;
        break;
    case ENET_PTP_ADDEND_UPDATE:
        /* this bit must be read as zero before application set it */
        do {
            temp_state = ENET_PTP_TSCTL(enet_periph) & ENET_PTP_TSCTL_TMSARU;
            timeout++;
        } while((RESET != temp_state) && (timeout < ENET_DELAY_TO));
        /* return ERROR due to timeout */
        if(ENET_DELAY_TO == timeout) {
            enet_state = ERROR;
        } else {
            ENET_PTP_TSCTL(enet_periph) |= ENET_PTP_TSCTL_TMSARU;
        }
        break;
    case ENET_PTP_SYSTIME_UPDATE:
        /* both the TMSSTU and TMSSTI bits must be read as zero before application set this bit */
        do {
            temp_state = ENET_PTP_TSCTL(enet_periph) & (ENET_PTP_TSCTL_TMSSTU | ENET_PTP_TSCTL_TMSSTI);
            timeout++;
        } while((RESET != temp_state) && (timeout < ENET_DELAY_TO));
        /* return ERROR due to timeout */
        if(ENET_DELAY_TO == timeout) {
            enet_state = ERROR;
        } else {
            ENET_PTP_TSCTL(enet_periph) |= ENET_PTP_TSCTL_TMSSTU;
        }
        break;
    case ENET_PTP_SYSTIME_INIT:
        /* this bit must be read as zero before application set it */
        do {
            temp_state = ENET_PTP_TSCTL(enet_periph) & ENET_PTP_TSCTL_TMSSTI;
            timeout++;
        } while((RESET != temp_state) && (timeout < ENET_DELAY_TO));
        /* return ERROR due to timeout */
        if(ENET_DELAY_TO == timeout) {
            enet_state = ERROR;
        } else {
            ENET_PTP_TSCTL(enet_periph) |= ENET_PTP_TSCTL_TMSSTI;
        }
        break;
    default:
        temp_config = (uint32_t)func & (~BIT(31));
        if(RESET != ((uint32_t)func & BIT(31))) {
            ENET_PTP_TSCTL(enet_periph) |= temp_config;
        } else {
            ENET_PTP_TSCTL(enet_periph) &= ~temp_config;
        }
        break;
    }

    return enet_state;
}

/*!
    \brief      configure system time subsecond increment value (API_ID(0x0032U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  subsecond: the value will be added to the subsecond value of system time,
                this value must be between 0 and 0xFF
    \param[out] none
    \retval     none
*/
void enet_ptp_subsecond_increment_config(uint32_t enet_periph, uint32_t subsecond)
{
    ENET_PTP_SSINC(enet_periph) = PTP_SSINC_STMSSI(subsecond);
}

/*!
    \brief      adjusting the clock frequency only in fine update mode (API_ID(0x0033U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  add: the value will be added to the accumulator register to achieve time synchronization
    \param[out] none
    \retval     none
*/
void enet_ptp_timestamp_addend_config(uint32_t enet_periph, uint32_t add)
{
    ENET_PTP_TSADDEND(enet_periph) = add;
}

/*!
    \brief      initialize or add/subtract to second of the system time (API_ID(0x0034U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  sign: timestamp update positive or negative sign,
                only one parameter can be selected which is shown as below
      \arg        ENET_PTP_ADD_TO_TIME: timestamp update value is added to system time
      \arg        ENET_PTP_SUBSTRACT_FROM_TIME: timestamp update value is subtracted from system time
    \param[in]  second: initializing or adding/subtracting to second of the system time
    \param[in]  subsecond: the current subsecond of the system time
                with 0.46 ns accuracy if required accuracy is 20 ns
    \param[out] none
    \retval     none
*/
void enet_ptp_timestamp_update_config(uint32_t enet_periph, uint32_t sign, uint32_t second, uint32_t subsecond)
{
    ENET_PTP_TSUH(enet_periph) = second;
    ENET_PTP_TSUL(enet_periph) = sign | PTP_TSUL_TMSUSS(subsecond);
}

/*!
    \brief      configure the expected target time (API_ID(0x0035U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  second: the expected target second time
    \param[in]  nanosecond: the expected target nanosecond time (signed)
    \param[out] none
    \retval     none
*/
void enet_ptp_expected_time_config(uint32_t enet_periph, uint32_t second, uint32_t nanosecond)
{
    ENET_PTP_ETH(enet_periph) = second;
    ENET_PTP_ETL(enet_periph) = nanosecond;
}

/*!
    \brief      get the current system time (API_ID(0x0036U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] systime_struct: pointer to a enet_ptp_systime_struct structure which contains
                parameters of PTP system time
                members of the structure and the member values are shown as below:
                  second: 0x0 - 0xFFFF FFFF
                  nanosecond: 0x0 - 0x7FFF FFFF * 10^9 / 2^31
                  sign: ENET_PTP_TIME_POSITIVE, ENET_PTP_TIME_NEGATIVE
    \retval     none
*/
void enet_ptp_system_time_get(uint32_t enet_periph, enet_ptp_systime_struct *systime_struct)
{
    uint32_t temp_sec = 0U, temp_subs = 0U;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(systime_struct)) {
        fw_debug_report_err(ENET_MODULE_ID, API_ID(0x0036), ERR_PARAM_POINTER);
    } else
#endif
    {
        /* get the value of system time registers */
        temp_sec = (uint32_t)ENET_PTP_TSH(enet_periph);
        temp_subs = (uint32_t)ENET_PTP_TSL(enet_periph);

        /* get system time and construct the enet_ptp_systime_struct structure */
        systime_struct->second = temp_sec;
        systime_struct->nanosecond = GET_PTP_TSL_STMSS(temp_subs);
        systime_struct->nanosecond = enet_ptp_subsecond_2_nanosecond(systime_struct->nanosecond);
        systime_struct->sign = GET_PTP_TSL_STS(temp_subs);
    }
}

/*!
    \brief      configure the PPS output frequency (API_ID(0x0037U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  freq: PPS output frequency,
                only one parameter can be selected which is shown as below
      \arg        ENET_PPSOFC_1HZ: PPS output 1Hz frequency
      \arg        ENET_PPSOFC_2HZ: PPS output 2Hz frequency
      \arg        ENET_PPSOFC_4HZ: PPS output 4Hz frequency
      \arg        ENET_PPSOFC_8HZ: PPS output 8Hz frequency
      \arg        ENET_PPSOFC_16HZ: PPS output 16Hz frequency
      \arg        ENET_PPSOFC_32HZ: PPS output 32Hz frequency
      \arg        ENET_PPSOFC_64HZ: PPS output 64Hz frequency
      \arg        ENET_PPSOFC_128HZ: PPS output 128Hz frequency
      \arg        ENET_PPSOFC_256HZ: PPS output 256Hz frequency
      \arg        ENET_PPSOFC_512HZ: PPS output 512Hz frequency
      \arg        ENET_PPSOFC_1024HZ: PPS output 1024Hz frequency
      \arg        ENET_PPSOFC_2048HZ: PPS output 2048Hz frequency
      \arg        ENET_PPSOFC_4096HZ: PPS output 4096Hz frequency
      \arg        ENET_PPSOFC_8192HZ: PPS output 8192Hz frequency
      \arg        ENET_PPSOFC_16384HZ: PPS output 16384Hz frequency
      \arg        ENET_PPSOFC_32768HZ: PPS output 32768Hz frequency
    \param[out] none
    \retval     none
*/
void enet_ptp_pps_output_frequency_config(uint32_t enet_periph, uint32_t freq)
{
    ENET_PTP_PPSCTL(enet_periph) = (freq & ENET_PTP_PPSCTL_PPSOFC);
}

/*!
    \brief      get the ptp flag status (API_ID(0x0038U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  flag: ptp flag status to be checked
      \arg        ENET_PTP_ADDEND_UPDATE: addend register update
      \arg        ENET_PTP_SYSTIME_UPDATE: timestamp update
      \arg        ENET_PTP_SYSTIME_INIT: timestamp initialize
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus enet_ptp_flag_get(uint32_t enet_periph, uint32_t flag)
{
    FlagStatus bitstatus = RESET;

    if((uint32_t)RESET != (ENET_PTP_TSCTL(enet_periph) & flag)) {
        bitstatus = SET;
    }

    return bitstatus;
}

/*!
    \brief      get the ENET MAC/MSC/PTP/DMA status flag (API_ID(0x0039U))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  enet_flag: ENET status flag, refer to enet_flag_enum,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_FLAG_MPKR: magic packet received flag
      \arg        ENET_MAC_FLAG_WUFR: wakeup frame received flag
      \arg        ENET_MAC_FLAG_FLOWCONTROL: flow control status flag
      \arg        ENET_MAC_FLAG_WUM: WUM status flag
      \arg        ENET_MAC_FLAG_MSC: MSC status flag
      \arg        ENET_MAC_FLAG_MSCR: MSC receive status flag
      \arg        ENET_MAC_FLAG_MSCT: MSC transmit status flag
      \arg        ENET_MAC_FLAG_TMST: time stamp trigger status flag
      \arg        ENET_PTP_FLAG_TSSCO: timestamp second counter overflow flag
      \arg        ENET_PTP_FLAG_TTM: target time match flag
      \arg        ENET_MSC_FLAG_RFCE: received frames CRC error flag
      \arg        ENET_MSC_FLAG_RFAE: received frames alignment error flag
      \arg        ENET_MSC_FLAG_RGUF: received good unicast frames flag
      \arg        ENET_MSC_FLAG_TGFSC: transmitted good frames single collision flag
      \arg        ENET_MSC_FLAG_TGFMSC: transmitted good frames more single collision flag
      \arg        ENET_MSC_FLAG_TGF: transmitted good frames flag
      \arg        ENET_DMA_FLAG_TS: transmit status flag
      \arg        ENET_DMA_FLAG_TPS: transmit process stopped status flag
      \arg        ENET_DMA_FLAG_TBU: transmit buffer unavailable status flag
      \arg        ENET_DMA_FLAG_TJT: transmit jabber timeout status flag
      \arg        ENET_DMA_FLAG_RO: receive overflow status flag
      \arg        ENET_DMA_FLAG_TU: transmit underflow status flag
      \arg        ENET_DMA_FLAG_RS: receive status flag
      \arg        ENET_DMA_FLAG_RBU: receive buffer unavailable status flag
      \arg        ENET_DMA_FLAG_RPS: receive process stopped status flag
      \arg        ENET_DMA_FLAG_RWT: receive watchdog timeout status flag
      \arg        ENET_DMA_FLAG_ET: early transmit status flag
      \arg        ENET_DMA_FLAG_FBE: fatal bus error status flag
      \arg        ENET_DMA_FLAG_ER: early receive status flag
      \arg        ENET_DMA_FLAG_AI: abnormal interrupt summary flag
      \arg        ENET_DMA_FLAG_NI: normal interrupt summary flag
      \arg        ENET_DMA_FLAG_EB_DMA_ERROR: DMA error flag
      \arg        ENET_DMA_FLAG_EB_TRANSFER_ERROR: transfer error flag
      \arg        ENET_DMA_FLAG_EB_ACCESS_ERROR: access error flag
      \arg        ENET_DMA_FLAG_MSC: MSC status flag
      \arg        ENET_DMA_FLAG_WUM: WUM status flag
      \arg        ENET_DMA_FLAG_TST: timestamp trigger status flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus enet_flag_get(uint32_t enet_periph, enet_flag_enum enet_flag)
{
    FlagStatus status = RESET;
    if(RESET != (ENET_REG_VAL(enet_periph, enet_flag) & BIT(ENET_BIT_POS(enet_flag)))) {
        status = SET;
    } else {
        status = RESET;
    }
    return status;
}

/*!
    \brief      clear the ENET DMA status flag (API_ID(0x003AU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  enet_flag: ENET DMA flag clear, refer to enet_flag_clear_enum
                only one parameter can be selected which is shown as below
      \arg        ENET_DMA_FLAG_TS_CLR: transmit status flag clear
      \arg        ENET_DMA_FLAG_TPS_CLR: transmit process stopped status flag clear
      \arg        ENET_DMA_FLAG_TBU_CLR: transmit buffer unavailable status flag clear
      \arg        ENET_DMA_FLAG_TJT_CLR: transmit jabber timeout status flag clear
      \arg        ENET_DMA_FLAG_RO_CLR: receive overflow status flag clear
      \arg        ENET_DMA_FLAG_TU_CLR: transmit underflow status flag clear
      \arg        ENET_DMA_FLAG_RS_CLR: receive status flag clear
      \arg        ENET_DMA_FLAG_RBU_CLR: receive buffer unavailable status flag clear
      \arg        ENET_DMA_FLAG_RPS_CLR: receive process stopped status flag clear
      \arg        ENET_DMA_FLAG_RWT_CLR: receive watchdog timeout status flag clear
      \arg        ENET_DMA_FLAG_ET_CLR: early transmit status flag clear
      \arg        ENET_DMA_FLAG_FBE_CLR: fatal bus error status flag clear
      \arg        ENET_DMA_FLAG_ER_CLR: early receive status flag clear
      \arg        ENET_DMA_FLAG_AI_CLR: abnormal interrupt summary flag clear
      \arg        ENET_DMA_FLAG_NI_CLR: normal interrupt summary flag clear
    \param[out] none
    \retval     none
*/
void enet_flag_clear(uint32_t enet_periph, enet_flag_clear_enum enet_flag)
{
    /* write 1 to the corresponding bit in ENET_DMA_STAT, to clear it */
    ENET_REG_VAL(enet_periph, enet_flag) = BIT(ENET_BIT_POS(enet_flag));
}

/*!
    \brief      enable ENET MAC/MSC/DMA interrupt (API_ID(0x003BU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  enet_int: ENET interrupt,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_INT_WUMIM: WUM interrupt mask
      \arg        ENET_MAC_INT_TMSTIM: timestamp trigger interrupt mask
      \arg        ENET_MSC_INT_RFCEIM: received frame CRC error interrupt mask
      \arg        ENET_MSC_INT_RFAEIM: received frames alignment error interrupt mask
      \arg        ENET_MSC_INT_RGUFIM: received good unicast frames interrupt mask
      \arg        ENET_MSC_INT_TGFSCIM: transmitted good frames single collision interrupt mask
      \arg        ENET_MSC_INT_TGFMSCIM: transmitted good frames more single collision interrupt mask
      \arg        ENET_MSC_INT_TGFIM: transmitted good frames interrupt mask
      \arg        ENET_DMA_INT_TIE: transmit interrupt enable
      \arg        ENET_DMA_INT_TPSIE: transmit process stopped interrupt enable
      \arg        ENET_DMA_INT_TBUIE: transmit buffer unavailable interrupt enable
      \arg        ENET_DMA_INT_TJTIE: transmit jabber timeout interrupt enable
      \arg        ENET_DMA_INT_ROIE: receive overflow interrupt enable
      \arg        ENET_DMA_INT_TUIE: transmit underflow interrupt enable
      \arg        ENET_DMA_INT_RIE: receive interrupt enable
      \arg        ENET_DMA_INT_RBUIE: receive buffer unavailable interrupt enable
      \arg        ENET_DMA_INT_RPSIE: receive process stopped interrupt enable
      \arg        ENET_DMA_INT_RWTIE: receive watchdog timeout interrupt enable
      \arg        ENET_DMA_INT_ETIE: early transmit interrupt enable
      \arg        ENET_DMA_INT_FBEIE: fatal bus error interrupt enable
      \arg        ENET_DMA_INT_ERIE: early receive interrupt enable
      \arg        ENET_DMA_INT_AIE: abnormal interrupt summary enable
      \arg        ENET_DMA_INT_NIE: normal interrupt summary enable
    \param[out] none
    \retval     none
*/
void enet_interrupt_enable(uint32_t enet_periph, enet_int_enum enet_int)
{
    if(DMA_INTEN_REG_OFFSET == ((uint32_t)enet_int >> 6U)) {
        /* ENET_DMA_INTEN register interrupt */
        ENET_REG_VAL(enet_periph, enet_int) |= BIT(ENET_BIT_POS(enet_int));
    } else {
        /* other INTMSK register interrupt */
        ENET_REG_VAL(enet_periph, enet_int) &= ~BIT(ENET_BIT_POS(enet_int));
    }
}

/*!
    \brief      disable ENET MAC/MSC/DMA interrupt (API_ID(0x003CU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  enet_int: ENET interrupt,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_INT_WUMIM: WUM interrupt mask
      \arg        ENET_MAC_INT_TMSTIM: timestamp trigger interrupt mask
      \arg        ENET_MSC_INT_RFCEIM: received frame CRC error interrupt mask
      \arg        ENET_MSC_INT_RFAEIM: received frames alignment error interrupt mask
      \arg        ENET_MSC_INT_RGUFIM: received good unicast frames interrupt mask
      \arg        ENET_MSC_INT_TGFSCIM: transmitted good frames single collision interrupt mask
      \arg        ENET_MSC_INT_TGFMSCIM: transmitted good frames more single collision interrupt mask
      \arg        ENET_MSC_INT_TGFIM: transmitted good frames interrupt mask
      \arg        ENET_DMA_INT_TIE: transmit interrupt enable
      \arg        ENET_DMA_INT_TPSIE: transmit process stopped interrupt enable
      \arg        ENET_DMA_INT_TBUIE: transmit buffer unavailable interrupt enable
      \arg        ENET_DMA_INT_TJTIE: transmit jabber timeout interrupt enable
      \arg        ENET_DMA_INT_ROIE: receive overflow interrupt enable
      \arg        ENET_DMA_INT_TUIE: transmit underflow interrupt enable
      \arg        ENET_DMA_INT_RIE: receive interrupt enable
      \arg        ENET_DMA_INT_RBUIE: receive buffer unavailable interrupt enable
      \arg        ENET_DMA_INT_RPSIE: receive process stopped interrupt enable
      \arg        ENET_DMA_INT_RWTIE: receive watchdog timeout interrupt enable
      \arg        ENET_DMA_INT_ETIE: early transmit interrupt enable
      \arg        ENET_DMA_INT_FBEIE: fatal bus error interrupt enable
      \arg        ENET_DMA_INT_ERIE: early receive interrupt enable
      \arg        ENET_DMA_INT_AIE: abnormal interrupt summary enable
      \arg        ENET_DMA_INT_NIE: normal interrupt summary enable
    \param[out] none
    \retval     none
*/
void enet_interrupt_disable(uint32_t enet_periph, enet_int_enum enet_int)
{
    if(DMA_INTEN_REG_OFFSET == ((uint32_t)enet_int >> 6U)) {
        /* ENET_DMA_INTEN register interrupt */
        ENET_REG_VAL(enet_periph, enet_int) &= ~BIT(ENET_BIT_POS(enet_int));
    } else {
        /* other INTMSK register interrupt */
        ENET_REG_VAL(enet_periph, enet_int) |= BIT(ENET_BIT_POS(enet_int));
    }
}

/*!
    \brief      get ENET MAC/MSC/DMA interrupt flag (API_ID(0x003DU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  int_flag: ENET interrupt flag,
                only one parameter can be selected which is shown as below
      \arg        ENET_MAC_INT_FLAG_WUM: WUM status flag
      \arg        ENET_MAC_INT_FLAG_MSC: MSC status flag
      \arg        ENET_MAC_INT_FLAG_MSCR: MSC receive status flag
      \arg        ENET_MAC_INT_FLAG_MSCT: MSC transmit status flag
      \arg        ENET_MAC_INT_FLAG_TMST: time stamp trigger status flag
      \arg        ENET_MSC_INT_FLAG_RFCE: received frames CRC error flag
      \arg        ENET_MSC_INT_FLAG_RFAE: received frames alignment error flag
      \arg        ENET_MSC_INT_FLAG_RGUF: received good unicast frames flag
      \arg        ENET_MSC_INT_FLAG_TGFSC: transmitted good frames single collision flag
      \arg        ENET_MSC_INT_FLAG_TGFMSC: transmitted good frames more single collision flag
      \arg        ENET_MSC_INT_FLAG_TGF: transmitted good frames flag
      \arg        ENET_DMA_INT_FLAG_TS: transmit status flag
      \arg        ENET_DMA_INT_FLAG_TPS: transmit process stopped status flag
      \arg        ENET_DMA_INT_FLAG_TBU: transmit buffer unavailable status flag
      \arg        ENET_DMA_INT_FLAG_TJT: transmit jabber timeout status flag
      \arg        ENET_DMA_INT_FLAG_RO: receive overflow status flag
      \arg        ENET_DMA_INT_FLAG_TU: transmit underflow status flag
      \arg        ENET_DMA_INT_FLAG_RS: receive status flag
      \arg        ENET_DMA_INT_FLAG_RBU: receive buffer unavailable status flag
      \arg        ENET_DMA_INT_FLAG_RPS: receive process stopped status flag
      \arg        ENET_DMA_INT_FLAG_RWT: receive watchdog timeout status flag
      \arg        ENET_DMA_INT_FLAG_ET: early transmit status flag
      \arg        ENET_DMA_INT_FLAG_FBE: fatal bus error status flag
      \arg        ENET_DMA_INT_FLAG_ER: early receive status flag
      \arg        ENET_DMA_INT_FLAG_AI: abnormal interrupt summary flag
      \arg        ENET_DMA_INT_FLAG_NI: normal interrupt summary flag
      \arg        ENET_DMA_INT_FLAG_MSC: MSC status flag
      \arg        ENET_DMA_INT_FLAG_WUM: WUM status flag
      \arg        ENET_DMA_INT_FLAG_TST: timestamp trigger status flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus enet_interrupt_flag_get(uint32_t enet_periph, enet_int_flag_enum int_flag)
{
    FlagStatus status = RESET;
    if(RESET != (ENET_REG_VAL(enet_periph, int_flag) & BIT(ENET_BIT_POS(int_flag)))) {
        status = SET;
    } else {
        status = RESET;
    }
    return status;
}

/*!
    \brief      clear ENET DMA interrupt flag (API_ID(0x003EU))
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[in]  int_flag_clear: clear ENET interrupt flag,
                only one parameter can be selected which is shown as below
      \arg        ENET_DMA_INT_FLAG_TS_CLR: transmit status flag
      \arg        ENET_DMA_INT_FLAG_TPS_CLR: transmit process stopped status flag
      \arg        ENET_DMA_INT_FLAG_TBU_CLR: transmit buffer unavailable status flag
      \arg        ENET_DMA_INT_FLAG_TJT_CLR: transmit jabber timeout status flag
      \arg        ENET_DMA_INT_FLAG_RO_CLR: receive overflow status flag
      \arg        ENET_DMA_INT_FLAG_TU_CLR: transmit underflow status flag
      \arg        ENET_DMA_INT_FLAG_RS_CLR: receive status flag
      \arg        ENET_DMA_INT_FLAG_RBU_CLR: receive buffer unavailable status flag
      \arg        ENET_DMA_INT_FLAG_RPS_CLR: receive process stopped status flag
      \arg        ENET_DMA_INT_FLAG_RWT_CLR: receive watchdog timeout status flag
      \arg        ENET_DMA_INT_FLAG_ET_CLR: early transmit status flag
      \arg        ENET_DMA_INT_FLAG_FBE_CLR: fatal bus error status flag
      \arg        ENET_DMA_INT_FLAG_ER_CLR: early receive status flag
      \arg        ENET_DMA_INT_FLAG_AI_CLR: abnormal interrupt summary flag
      \arg        ENET_DMA_INT_FLAG_NI_CLR: normal interrupt summary flag
    \param[out] none
    \retval     none
*/
void enet_interrupt_flag_clear(uint32_t enet_periph, enet_int_flag_clear_enum int_flag_clear)
{
    /* write 1 to the corresponding bit in ENET_DMA_STAT, to clear it */
    ENET_REG_VAL(enet_periph, int_flag_clear) = BIT(ENET_BIT_POS(int_flag_clear));
}

/*!
    \brief      initialize ENET peripheral with generally concerned parameters, call it by enet_init()
    \param[in]  enet_periph: ENETx(x=0,1)
    \param[out] none
    \retval     none
*/
static void enet_default_init(uint32_t enet_periph)
{
    uint32_t reg_value = 0U;

    /* MAC */
    /* configure ENET_MAC_CFG register */
    reg_value = ENET_MAC_CFG(enet_periph);
    reg_value &= MAC_CFG_MASK;
    reg_value |= ENET_WATCHDOG_ENABLE | ENET_JABBER_ENABLE | ENET_INTERFRAMEGAP_96BIT \
                 | ENET_SPEEDMODE_10M | ENET_MODE_HALFDUPLEX | ENET_LOOPBACKMODE_DISABLE \
                 | ENET_CARRIERSENSE_ENABLE | ENET_RECEIVEOWN_ENABLE \
                 | ENET_RETRYTRANSMISSION_ENABLE | ENET_BACKOFFLIMIT_10 \
                 | ENET_DEFERRALCHECK_DISABLE \
                 | ENET_TYPEFRAME_CRC_DROP_DISABLE \
                 | ENET_AUTO_PADCRC_DROP_DISABLE \
                 | ENET_CHECKSUMOFFLOAD_DISABLE;
    ENET_MAC_CFG(enet_periph) = reg_value;

    /* configure ENET_MAC_FRMF register */
    ENET_MAC_FRMF(enet_periph) = ENET_SRC_FILTER_DISABLE | ENET_DEST_FILTER_INVERSE_DISABLE \
                                 | ENET_MULTICAST_FILTER_PERFECT | ENET_UNICAST_FILTER_PERFECT \
                                 | ENET_PCFRM_PREVENT_ALL | ENET_BROADCASTFRAMES_ENABLE \
                                 | ENET_PROMISCUOUS_DISABLE | ENET_RX_FILTER_ENABLE;

    /* configure ENET_MAC_HLH, ENET_MAC_HLL register */
    ENET_MAC_HLH(enet_periph) = 0x00000000U;

    ENET_MAC_HLL(enet_periph) = 0x00000000U;

    /* configure ENET_MAC_FCTL, ENET_MAC_FCTH register */
    reg_value = ENET_MAC_FCTL(enet_periph);
    reg_value &= MAC_FCTL_MASK;
    reg_value |= MAC_FCTL_PTM(0) | ENET_ZERO_QUANTA_PAUSE_DISABLE \
                 | ENET_PAUSETIME_MINUS4 | ENET_UNIQUE_PAUSEDETECT \
                 | ENET_RX_FLOWCONTROL_DISABLE | ENET_TX_FLOWCONTROL_DISABLE;
    ENET_MAC_FCTL(enet_periph) = reg_value;

    /* configure ENET_MAC_VLT register */
    ENET_MAC_VLT(enet_periph) = ENET_VLANTAGCOMPARISON_16BIT | MAC_VLT_VLTI(0);

    /* DMA */
    /* configure ENET_DMA_CTL register */
    reg_value = ENET_DMA_CTL(enet_periph);
    reg_value &= DMA_CTL_MASK;
    reg_value |= ENET_TCPIP_CKSUMERROR_DROP | ENET_RX_MODE_STOREFORWARD \
                 | ENET_FLUSH_RXFRAME_ENABLE | ENET_TX_MODE_STOREFORWARD \
                 | ENET_TX_THRESHOLD_64BYTES | ENET_RX_THRESHOLD_64BYTES \
                 | ENET_SECONDFRAME_OPT_DISABLE;
    ENET_DMA_CTL(enet_periph) = reg_value;

    /* configure ENET_DMA_BCTL register */
    reg_value = ENET_DMA_BCTL(enet_periph);
    reg_value &= DMA_BCTL_MASK;
    reg_value = ENET_ADDRESS_ALIGN_ENABLE | ENET_ARBITRATION_RXTX_2_1 \
                | ENET_RXDP_32BEAT | ENET_PGBL_32BEAT | ENET_RXTX_DIFFERENT_PGBL \
                | ENET_FIXED_BURST_ENABLE | ENET_MIXED_BURST_DISABLE \
                | ENET_NORMAL_DESCRIPTOR;
    ENET_DMA_BCTL(enet_periph) = reg_value;
}

/*!
    \brief      change subsecond to nanosecond
    \param[in]  subsecond: subsecond value
    \param[out] none
    \retval     the nanosecond value
*/
static uint32_t enet_ptp_subsecond_2_nanosecond(uint32_t subsecond)
{
    uint64_t val = subsecond * 1000000000ULL;
    val >>= 31U;
    return (uint32_t)val;
}

#endif /* GD32H73X_75X */
