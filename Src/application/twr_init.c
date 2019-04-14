#include "main.h"
#include "port.h"
#include "dw_main.h"
#include "twr_init.h"
#include "uart.h"

#define RNG_DELAY_MS		100
#define MAX_SLAVE_NUMBER	1
#define	MASTER_ADDR			0x01

uint8 slave_addr = 1;

//The table below specifies the default TX spectrum configuration parameters... this has been tuned for DW EVK hardware units
static const tx_struct_t txSpectrumConfig[8] =   {
        {  0x0,  {  0x0       ,  0x0          }  },
        {  0xc9, {  0x15355575,  0x07274767   }  },
        {  0xc2, {  0x15355575,  0x07274767   }  },
        {  0xc5, {  0x0f2f4f6f,  0x2b4b6b8b   }  },
        {  0x95, {  0x1f1f3f5f,  0x3a5a7a9a   }  },
        {  0xc0, {  0x0E082848,  0x25456585   }  },
        {  0x0,  {  0x0       ,  0x0          }  },
        {  0x93, {  0x32527292,  0x5171B1d1   }  }
};

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
#if 1
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
#else
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* Use non-standard SFD (Boolean) */
    DWT_BR_6M8,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
#endif

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
//#define TX_ANT_DLY 16425
//#define RX_ANT_DLY 16425

/* Frames used in the ranging process. See NOTE 2 below. */
#if 0
//static uint8 tx_init_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x20, 0, 0};
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x2A, 0, 0, 0, 0, 0, 0};
//static uint8 tx_end_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x28, 0, 0};
#else
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 0, 'M', MASTER_ADDR, 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'M', MASTER_ADDR, 'S', 0, 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 0, 'M', MASTER_ADDR, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'M', MASTER_ADDR, 'S', 0, 0x2A, 0, 0, 0, 0, 0, 0};
#endif
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_DEST_ADDR_IDX 6
#define ALL_MSG_SRC_ADDR_IDX 8

#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

#define REPORT_MSG_POLL_RX_TS_IDX 10
#define REPORT_MSG_RESP_TX_TS_IDX 14
#define REPORT_MSG_FINAL_RX_TS_IDX 18

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ¦Ìs and 1 ¦Ìs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 150
#define POLL_TX_TO_RESP_RX_DLY_UUS 850

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
//#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3500
/* Receive response timeout. See NOTE 5 below. */
//#define RESP_RX_TIMEOUT_UUS 2700
#define RESP_RX_TIMEOUT_UUS 8500
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
//#define PRE_TIMEOUT 8
#define PRE_TIMEOUT 16

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */

//typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof_dtu;
static double tof;
static double distance;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void report_msg_get_ts(const uint8 *ts_field, double *ts);

void TWR_Init_Config(void)
{
	dwt_txconfig_t	configTx;
	
	dwt_softreset();

	spi_set_rate_low();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		Error_Handler();
	}
	spi_set_rate_high();

	dwt_configure(&config) ;
	dwt_setsmarttxpower(0);
	dwt_setleds(DWT_LEDS_ENABLE);

	configTx.PGdly = txSpectrumConfig[config.chan].PG_DELAY ;
	configTx.power = txSpectrumConfig[config.chan].tx_pwr[config.prf - DWT_PRF_16M];
	dwt_configuretxrf(&configTx);

	/* Apply default antenna delay value. See NOTE 1 below. */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
	
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
}

void TWR_Init_Handle(void)
{
    /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
	tx_poll_msg[ALL_MSG_DEST_ADDR_IDX] = slave_addr;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };

    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;

        /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Check that the frame is the expected response from the companion "DS TWR responder" example.
         * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		if(rx_buffer[ALL_MSG_SRC_ADDR_IDX] == slave_addr)
		{
	        rx_buffer[ALL_MSG_SN_IDX] = 0;
	        rx_resp_msg[ALL_MSG_SRC_ADDR_IDX] = slave_addr;
	        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
	        {
	            uint32 final_tx_time;
	            int ret;

	            /* Retrieve poll transmission and response reception timestamp. */
	            poll_tx_ts = get_tx_timestamp_u64();
	            resp_rx_ts = get_rx_timestamp_u64();

	            /* Compute final message transmission time. See NOTE 9 below. */
	            final_tx_time = (resp_rx_ts + (uint64)(RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
	            dwt_setdelayedtrxtime(final_tx_time);

	            /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
	            final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

	            /* Write all timestamps in the final message. See NOTE 10 below. */
	            final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
	            final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
	            final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

	            /* Write and send final message. See NOTE 7 below. */
	            tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				tx_final_msg[ALL_MSG_DEST_ADDR_IDX] = slave_addr;
	            dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
	            dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);

				dwt_setrxtimeout(0);
				ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

	            /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
	            if (ret == DWT_SUCCESS)
	            {
					while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	                { };

					if (status_reg & SYS_STATUS_RXFCG)
					{
						/* A frame has been received, read it into the local buffer. */
				        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				        if (frame_len <= RX_BUF_LEN)
				        {
				            dwt_readrxdata(rx_buffer, frame_len, 0);
				        }
						/* Clear good RX frame event in the DW1000 status register. */
	           			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
						if(rx_buffer[ALL_MSG_SRC_ADDR_IDX] == slave_addr)
						{
							rx_buffer[ALL_MSG_SN_IDX] = 0;
							rx_report_msg[ALL_MSG_SRC_ADDR_IDX] = slave_addr;
							if (memcmp(rx_buffer, rx_report_msg, ALL_MSG_COMMON_LEN) == 0)
							{
								report_msg_get_ts(&rx_buffer[ALL_MSG_COMMON_LEN], &tof_dtu);
								tof = tof_dtu * DWT_TIME_UNITS;
								distance = tof * SPEED_OF_LIGHT;

								if(distance < 0)
								{
									Uart_ERROR();
								}
								else
								{
									Uart_Print_Dist(distance, MASTER_ADDR, slave_addr);
								}
							}
						}
					}
					else
					{
	                    /* Clear RX error/timeout events in the DW1000 status register. */
	                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

	                    /* Reset RX to properly reinitialise LDE operation. */
	                    dwt_rxreset();
					}
					
					/* Increment frame sequence number after transmission of the final message (modulo 256). */
					frame_seq_nb++;
					dwt_forcetrxoff();
					dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	            }
	        }
		}
    }
    else
    {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
    }

    /* Execute a delay between ranging exchanges. */
    HAL_Delay(RNG_DELAY_MS);
	slave_addr++;
	if(slave_addr > MAX_SLAVE_NUMBER)
	{
		slave_addr = 1;
	}
}

static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

static void report_msg_get_ts(const uint8 *ts_field, double *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

