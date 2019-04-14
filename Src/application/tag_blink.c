#include "main.h"
#include "port.h"
#include "dw_main.h"
#include "tag_blink.h"

#define TX_DELAY_MS 1000

uint32_t blink_timer;

blink_msg_frame tag_blink_msg;
//DECA_STUTAS_TypeDef deca_status;

static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static uint32_t ulong2littleEndian(uint32_t prm);

void DW1000_Blink_Config(void)
{
	dwt_softreset();
	spi_set_rate_low();
	if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
	{
		Error_Handler();
	}
	spi_set_rate_high();
	dwt_configure(&config);
	//dwt_setleds(DWT_LEDS_ENABLE|DWT_LEDS_INIT_BLINK);
	dwt_setleds(DWT_LEDS_ENABLE);

	uint8 eui64[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xCA, 0xDE};

	uint32 id= ulong2littleEndian(dwt_getpartid());
	memcpy(eui64, &id, sizeof(uint32));
	memcpy(tag_blink_msg.tagID, eui64, 8);

	tag_blink_msg.frameCtrl = 0xC5;
}

void DW1000_Blink_Handle(void)
{
	if((HAL_GetTick() - blink_timer) >= TX_DELAY_MS)
	{
		dwt_writetxdata(sizeof(tag_blink_msg), (uint8 *)(&tag_blink_msg), 0) ;
		dwt_writetxfctrl(sizeof(tag_blink_msg), 0, 0);
		dwt_starttx(DWT_START_TX_IMMEDIATE);

		while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
		{ };
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		tag_blink_msg.seqNum++;

		blink_timer = HAL_GetTick();
	}
}

static uint32_t ulong2littleEndian(uint32_t prm)
{
	union {
	    uint32_t  ui;
	    uint8	uc[sizeof(uint32)];
	} v;

	v.ui = 1;	//check endian
	if(v.uc[0] == 1) {
		v.ui=prm;	//little
	} else {
		v.ui=prm;	//
		SWAP(v.uc[0], v.uc[3]);
		SWAP(v.uc[1], v.uc[2]);
	}
	return v.ui;
}

