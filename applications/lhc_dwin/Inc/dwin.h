/*USING_MALLOCUSING_MALLOC
 * Dwin.h
 *
 *  Created on: Nov 19, 2020
 *      Author: play
 */

#ifndef INC_DWIN_H_
#define INC_DWIN_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include "dwin_cfg.h"
#include "tool.h"
#if (DWIN_USING_RTOS == 1)
// #include "cmsis_os.h"
// #include "shell_port.h"
#else
// #include <rtthread.h>
// #include <rtdbg.h>
#endif

/*****************************************************迪文屏幕特殊指令区*******************************************/
/*迪文屏幕写指令*/
#define DWIN_WRITE_CMD 0x82
/*迪文屏幕读指令*/
#define DWIN_READ_CMD 0x83
/*迪文屏幕页面切换指令*/
#define DWIN_PAGE_CHANGE_CMD 0x84
/*迪文屏幕触摸指令*/
#define DWIN_TOUCH_CMD 0xD4

	typedef enum
	{
		err_frame_head = 0,	 /*数据帧头错误*/
		err_check_code,		 /*校验码错误*/
		err_event,			 /*事件错误*/
		err_max_upper_limit, /*超出数据上限*/
		err_min_lower_limit, /*低于数据下限*/
		err_other,			 /*其他错误*/
		dwin_ok,
	} dwin_result;

	typedef struct Dwin_HandleTypeDef *pDwinHandle;
	typedef struct Dwin_HandleTypeDef DwinHandle;

	struct Dwin_HandleTypeDef
	{
		uint8_t Id;
		void (*Dw_Transmit)(pDwinHandle);
		void (*Dw_Recive)(void *);
		void (*Dw_Write)(pDwinHandle, uint16_t, uint8_t *, uint16_t);
		void (*Dw_Read)(pDwinHandle, uint16_t, uint8_t);
		void (*Dw_Page)(pDwinHandle, uint16_t);
		void (*Dw_Poll)(pDwinHandle);
		void (*Dw_Error)(pDwinHandle, dwin_result, uint8_t);
		void (*Dw_Delay)(uint32_t);
		struct
		{
			void *pHandle;
		} Master;
		struct
		{
			/*预留外部数据结构接口*/
			void *pHandle;
			Event_Map *pMap;
			uint16_t Events_Size;
		} Slave;
		UartHandle Uart;
	} __attribute__((aligned(4)));

/*带上(pd)解决多级指针解引用问题：(*pd)、(**pd)*/
#define dwin_tx_size(pd) ((pd)->Uart.tx.size)
#define dwin_rx_size(pd) ((pd)->Uart.rx.size)
#define dwin_tx_count(pd) ((pd)->Uart.tx.count)
#define dwin_rx_count(pd) ((pd)->Uart.rx.count)
#define dwin_tx_buf ((pd)->Uart.tx.pbuf)
#define dwin_rx_buf ((pd)->Uart.rx.pbuf)

/*83指令返回数据以一个字为基础*/
#define DWIN_WORD 1U
#define DWIN_DWORD 2U
/*获取迪文屏幕数据*/
#define Get_Dwin_Data(__pbuf, __s, __size)                                   \
	((__size) < 2U ? ((__pbuf[__s] << 8U) |                                  \
					  (__pbuf[__s + 1U]))                                    \
				   : ((__pbuf[__s] << 24U) |                                 \
					  (__pbuf[__s + 1U] << 16U) | (__pbuf[__s + 2U] << 8U) | \
					  (__pbuf[__s + 3U])))

#if (DWIN_USING_MALLOC == 0)
#define Init_Dwin_Object(id) (static DwinHandle Connect_Str(Dwin_Object, id))
#define Get_Dwin_Object(id) (&Connect_Str(Dwin_Object, id))
#else
#define dwin_handler(obj) (obj->Dw_Poll(obj))
extern void Create_DwinObject(pDwinHandle *pd, pDwinHandle ps);
#endif
#ifdef __cplusplus
}
#endif
#endif /* INC_DWIN_H_ */
