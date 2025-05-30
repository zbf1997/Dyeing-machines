#ifndef __CAN_H
#define __CAN_H
#include "stm32f4xx.h" 

#ifndef TRUE
  /** Value is true (boolean_t type) */
  #define TRUE        ((boolean_t) 1u)
#endif

#ifndef FALSE
  /** Value is false (boolean_t type) */
  #define FALSE       ((boolean_t) 0u)
#endif  

typedef uint8_t      boolean_t;


#define  CAN_FILTER_EXTID_H(EXTID)       ((uint16_t)  (((EXTID<<3)|0x04)>>16) )
#define  CAN_FILTER_EXTID_L(EXTID)       ((uint16_t)  (( EXTID<<3)|0x04) )
#define  CAN_STD_ID_H_MASK_DONT_CARE     0x0000
#define  CAN_STD_ID_L_MASK_DONT_CARE     0x0000



extern void CAN_INIT(void);
extern uint8_t canCRC_ATM(uint8_t *buf,uint8_t len);
extern void CanTransfer(uint8_t *buf,uint8_t len);
extern CanRxMsg CanRxBuf;
extern CanTxMsg CanTxBuf;
extern uint16_t  CAN_ID;
extern boolean_t CAN_RxDone;

/*********************PMC007 CanOpen***********/
void CANOpen_Transfer_Write(uint8_t NodeID, uint8_t CanOpearType, uint16_t Address, uint8_t SubAddress, uint32_t value);
void CANOpen_Transfer_Read(uint8_t NodeID, uint8_t CanOpearType, uint16_t Address, uint8_t SubAddress, uint32_t value);

typedef enum {
	READ   = 0x40,//读取参数
	WRITE1 = 0x2F,//发送的参数数据（不包括命令字和索引）一个字节
	WRITE2 = 0x2B,//两个字节
	WRITE3 = 0x27,//三个字节
	WRITE4 = 0x23//四个字节
}CAN_OPERA_TYPE;

typedef enum {
	READ1 = 0x4F,
	READ2 = 0x4B,
	READ3 = 0x47,
	READ4 = 0x43,
	WRITE_SUCCESS = 0x60,
	WRITE_FAIL = 0x80
}CAN_RESPONSE_TYPE;


#endif
