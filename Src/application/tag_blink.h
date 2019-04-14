#ifndef __TAG_BLINK_H_
#define __TAG_BLINK_H_

typedef struct
{
    uint8 frameCtrl;                         		//  frame control bytes 00
    uint8 seqNum;                               	//  sequence_number 01
    uint8 tagID[8];           						//  02-09 64 bit addresses
    uint8 fcs[2] ;                              	//  10-11  we allow space for the CRC as it is logically part of the message. However DW1000 TX calculates and adds these bytes.
} blink_msg_frame;


void DW1000_Blink_Config(void);
void DW1000_Blink_Handle(void);


#endif
