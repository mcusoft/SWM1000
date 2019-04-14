#include "main.h"
#include "port.h"
#include "cont_frame.h"

#define CONT_FRAME_PERIOD 124800
#define CONT_FRAME_DURATION_MS 120000

static dwt_config_t cont_frame_config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC64,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 64 - 64)  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static dwt_txconfig_t cont_frame_txconfig = {
    0xC2,            /* PG delay. */
    0x07274767,      /* TX power. */
};

static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};

void DW1000_Cont_Frame_Config(void)
{
    dwt_configure(&cont_frame_config);
    dwt_configuretxrf(&cont_frame_txconfig);
	dwt_setleds(DWT_LEDS_ENABLE);
    
    dwt_configcontinuousframemode(CONT_FRAME_PERIOD);
    
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */
    dwt_starttx(DWT_START_TX_IMMEDIATE);
}

void DW1000_Cont_Frame_Handle(void)
{
    while (1)
    { };
}

