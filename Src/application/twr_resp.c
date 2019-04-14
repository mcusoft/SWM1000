#include "main.h"
#include "port.h"
#include "dw_main.h"
#include "twr_resp.h"
#include "uart.h"

#define	SLAVE_ADDR			0x01
uint8 master_addr;

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
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 tx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x2A, 0, 0, 0, 0, 0, 0};
#else
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', SLAVE_ADDR, 'M', 0, 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'M', 0, 'S', SLAVE_ADDR, 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', SLAVE_ADDR, 'M', 0, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 tx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'M', 0, 'S', SLAVE_ADDR, 0x2A, 0, 0, 0, 0, 0, 0};
#endif
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
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

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ¦Ìs and 1 ¦Ìs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
//#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
#define POLL_RX_TO_RESP_TX_DLY_UUS 3500
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 700

/* Receive final timeout. See NOTE 5 below. */
//#define FINAL_RX_TIMEOUT_UUS 3300
#define FINAL_RX_TIMEOUT_UUS 4800
//#define PRE_TIMEOUT 8
#define PRE_TIMEOUT 16
/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

//static uint32 dist_timer;
//static double sum_distance;
//static double avg_distance;
//static uint32 num_counter;
/* String used to display measured distance on LCD screen (16 characters maximum). */
//char dist_str[16] = {0};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void report_msg_set_ts(uint8 *ts_field, uint64 ts);

void TWR_Resp_Config(void)
{
	dwt_txconfig_t	configTx;

	dwt_softreset();

	spi_set_rate_low();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		Error_Handler();
	}
	spi_set_rate_high();

    dwt_configure(&config);
	dwt_setsmarttxpower(0);
	dwt_setleds(DWT_LEDS_ENABLE);

	configTx.PGdly = txSpectrumConfig[config.chan].PG_DELAY ;
	configTx.power = txSpectrumConfig[config.chan].tx_pwr[config.prf - DWT_PRF_16M];
	dwt_configuretxrf(&configTx);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

	dwt_setpreambledetecttimeout(PRE_TIMEOUT);
}

void TWR_Resp_Handle(void)
{
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(0);

	/* Activate reception immediately. */
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

	/* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{ };

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32 frame_len;

		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUFFER_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is a poll sent by "DS TWR initiator" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		master_addr = rx_buffer[ALL_MSG_SRC_ADDR_IDX];

		rx_buffer[ALL_MSG_SRC_ADDR_IDX] = 0;
		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 resp_tx_time;
			int ret;

			/* Retrieve poll reception timestamp. */
			poll_rx_ts = get_rx_timestamp_u64();

			/* Set send time for response. See NOTE 8 below. */
			resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(resp_tx_time);

			/* Set expected delay and timeout for final message reception. */
			dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
			dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

			/* Write and send the response message. See NOTE 9 below.*/
			tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
			tx_resp_msg[ALL_MSG_DEST_ADDR_IDX] = master_addr;
			dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
			dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

			if (ret == DWT_ERROR)
			{
               return;
			}
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
			{ };

			/* Increment frame sequence number after transmission of the response message (modulo 256). */
			frame_seq_nb++;

			if (status_reg & SYS_STATUS_RXFCG)
			{
				/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUF_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len, 0);
				}

				/* Check that the frame is a final message sent by "DS TWR initiator" example.
				 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
				rx_buffer[ALL_MSG_SN_IDX] = 0;
				if(rx_buffer[ALL_MSG_SRC_ADDR_IDX] == master_addr)
				{
					rx_buffer[ALL_MSG_SRC_ADDR_IDX] = 0;
				}
				if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
				{
					uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
					uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
					double Ra, Rb, Da, Db;
					int64 tof_dtu;

					/* Retrieve response transmission and final reception timestamps. */
					resp_tx_ts = get_tx_timestamp_u64();
					final_rx_ts = get_rx_timestamp_u64();

					/* Get timestamps embedded in the final message. */
					final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

					/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
					poll_rx_ts_32 = (uint32)poll_rx_ts;
					resp_tx_ts_32 = (uint32)resp_tx_ts;
					final_rx_ts_32 = (uint32)final_rx_ts;
					Ra = (double)(resp_rx_ts - poll_tx_ts);
					Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
					Da = (double)(final_tx_ts - resp_rx_ts);
					Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
					tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					if(distance < 0)
					{
						Uart_ERROR();
					}
					else
					{
						report_msg_set_ts(&tx_report_msg[ALL_MSG_COMMON_LEN], tof_dtu);
						tx_report_msg[ALL_MSG_DEST_ADDR_IDX] = master_addr;
						dwt_writetxdata(sizeof(tx_report_msg), tx_report_msg, 0);
						dwt_writetxfctrl(sizeof(tx_report_msg), 0, 0);

						dwt_starttx(DWT_START_TX_IMMEDIATE);
						while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
	                	{ };
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
						dwt_forcetrxoff();

#if 0
						sum_distance += distance;
						num_counter++;
						if((HAL_GetTick() - dist_timer) >= 1000)
						{
							avg_distance = sum_distance/num_counter;
							sum_distance = 0;
							num_counter = 0;
							/* Display computed distance on LCD. */
							sprintf(dist_str, "DIST: %3.2fM", avg_distance);
							//sprintf(string_buff,"$DIST,M%d,S%d,%2.2f\r\n", addr_master, addr_slave, count_data);
							//OLED_dispMsg(0, 32, (char*)dist_str, &Font_8x16);
							dist_timer = HAL_GetTick();
						}
#endif
						Uart_Print_Dist(distance, master_addr, SLAVE_ADDR);
					}
				}
			}
			else
			{
				/* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

                    /* Reset RX to properly reinitialise LDE operation. */
                    dwt_rxreset();
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

static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

static void report_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

