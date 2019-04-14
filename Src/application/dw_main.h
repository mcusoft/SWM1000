#ifndef _DW_MAIN_H_
#define _DW_MAIN_H_

#include "Deca_device_api.h"
#include "deca_regs.h"

typedef signed long long int64;
typedef unsigned long long uint64;

#ifndef SWAP
#define SWAP(a,b) {a^=b;b^=a;a^=b;}
#endif /* SWAP */

#ifndef MIN
#define MIN(a,b)	(((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a,b)	(((a) < (b)) ? (b) : (a))
#endif /* MAX */

typedef enum
{
	DECA_BLINK,
	DECA_TWR_INIT,
	DECA_TWR_RESP,
	DECA_CONT_FRAME,
	DECA_STATUS_MAX
} DECA_STUTAS_TypeDef;

typedef struct {
    uint8_t PG_DELAY;

    //TX POWER
    //31:24     BOOST_0.125ms_PWR
    //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
    //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
    //7:0       DEFAULT_PWR-TX_DATA_PWR
    uint32_t tx_pwr[2]; //
}tx_struct_t;

void dw_main(void);
void DW1000_Init(void);
void DW1000_Config(void);
void DW1000_Handle(void);

#endif

