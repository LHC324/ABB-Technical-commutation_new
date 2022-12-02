/*
 * Dwin.c
 *
 *  Created on: 2022年3月25日
 *      Author: LHC
 */

#include "dwin.h"

static void Dwin_Write(pDwinHandle pd, uint16_t start_addr, uint8_t *dat, uint16_t len);
static void Dwin_Read(pDwinHandle pd, uint16_t start_addr, uint8_t words);
static void Dwin_PageChange(pDwinHandle pd, uint16_t page);
static short int Dwin_GetSignedData(pDwinHandle pd, uint16_t udata);
static void Dwin_Poll(pDwinHandle pd);

/**
 * @brief  创建迪文屏幕对象(动态创建)
 * @param  pd 需要初始化对象指针
 * @param  ps 初始化数据指针
 * @retval None
 */
void Create_DwinObject(pDwinHandle *pd, pDwinHandle ps)
{
	if (!ps)
		return;
#if (DWIN_USING_MALLOC)
	(*pd) = (pDwinHandle)dw_malloc(sizeof(DwinHandle));
	if (!(*pd))
	{
		dw_free(*pd);
		return;
	}
	uint8_t *pTxbuf = (uint8_t *)dw_malloc(dwin_tx_size(ps));
	if (!pTxbuf)
	{
		dw_free(pTxbuf);
		return;
	}
	uint8_t *pRxbuf = (uint8_t *)dw_malloc(dwin_rx_size(ps));
	if (!pRxbuf)
	{
		dw_free(pRxbuf);
		return;
	}
#else
	static uint8_t pTxbuf[dwin_tx_size(ps)];
	static uint8_t pRxbuf[dwin_rx_size(ps)];
#endif
	memset(pTxbuf, 0x00, dwin_tx_size(ps));
	memset(pRxbuf, 0x00, dwin_rx_size(ps));
#if (DWIN_USING_DEBUG)
	DWIN_DEBUG("@note:DwinObject[%d] = 0x%p", ps->Id, *pd);
#endif

	(*pd)->Id = ps->Id;
	(*pd)->Dw_Transmit = ps->Dw_Transmit;
#if (DWIN_USING_DMA)
#if (TOOL_USING_STM32HAL)
	(*pd)->Dw_Recive = (void (*)(void *))uartx_recive_handle;
#else
	(*pd)->Dw_Recive = Dw_Recive;
#endif
#endif
	(*pd)->Dw_Delay = ps->Dw_Delay;
	(*pd)->Dw_Error = ps->Dw_Error;
	(*pd)->Dw_Write = Dwin_Write;
	(*pd)->Dw_Read = Dwin_Read;
	(*pd)->Dw_Page = Dwin_PageChange;
	(*pd)->Dw_GetSignedData = Dwin_GetSignedData;
	(*pd)->Dw_Poll = Dwin_Poll;

	if (!ps->Uart.tx.pbuf)
	{
		ps->Uart.tx.pbuf = pTxbuf;
	}
	if (!ps->Uart.rx.pbuf)
	{
		ps->Uart.rx.pbuf = pRxbuf;
	}
	memcpy(&(*pd)->Uart, &ps->Uart, sizeof(UartHandle));
	memcpy(&(*pd)->user_val, &ps->user_val, sizeof(ps->user_val));
	memcpy(&(*pd)->Master, &ps->Master, sizeof(ps->Master));
	memcpy(&(*pd)->Slave, &ps->Slave, sizeof(ps->Slave));
}

/**
 * @brief  销毁迪文屏幕对象
 * @param  pd 需要初始化对象指针
 * @retval None
 */
void Free_DwinObject(pDwinHandle *pd)
{
	if (*pd)
	{
		dw_free((*pd)->Uart.tx.pbuf);
		dw_free((*pd)->Uart.rx.pbuf);
		dw_free((*pd));
	}
}

/**
 * @brief  写数据变量到指定地址并显示
 * @param  pd 迪文屏幕对象句柄
 * @param  start_addr 开始地址
 * @param  dat 指向数据的指针
 * @param  length 数据长度
 * @retval None
 */
static void Dwin_Write(pDwinHandle pd, uint16_t start_addr, uint8_t *pdat, uint16_t len)
{
#if (DWIN_USING_CRC)
	uint8_t buf[] = {
		0x5A, 0xA5, len + 3U + 2U, DWIN_WRITE_CMD, start_addr >> 8U,
		start_addr};
#else
	uint8_t buf[] = {
		0x5A, 0xA5, len + 3U, DWIN_WRITE_CMD, start_addr >> 8U,
		start_addr};
#endif
	dwin_tx_count(pd) = 0U;
	memcpy(dwin_tx_buf, buf, sizeof(buf));
	dwin_tx_count(pd) += sizeof(buf);
	memcpy(&dwin_tx_buf[dwin_tx_count(pd)], pdat, len);
	dwin_tx_count(pd) += len;
#if (DWIN_USING_DEBUG)
	// shellPrint(Shell_Object, "pd = %p, dwin_tx_count(pd) = %d.\r\n", pd, dwin_tx_count(pd));
	// shellPrint(Shell_Object, "dwin_tx_buf = %s.\r\n", dwin_tx_buf);
#endif

	pd->Dw_Transmit(pd);
}

/**
 * @brief  读出指定地址指定长度数据
 * @param  pd 迪文屏幕对象句柄
 * @param  start_addr 开始地址
 * @param  dat 指向数据的指针
 * @param  length 数据长度
 * @retval None
 */
static void Dwin_Read(pDwinHandle pd, uint16_t start_addr, uint8_t words)
{
#if (DWIN_USING_CRC)
	uint8_t buf[] = {
		0x5A, 0xA5, 0x04 + 2U, DWIN_READ_CMD, start_addr >> 8U,
		start_addr, words};
#else
	uint8_t buf[] = {
		0x5A, 0xA5, 0x04, DWIN_READ_CMD, start_addr >> 8U,
		start_addr, words};
#endif
	dwin_tx_count(pd) = 0U;
	memcpy(dwin_tx_buf, buf, sizeof(buf));
	dwin_tx_count(pd) += sizeof(buf);

	// Dwin_Send(pd);
	pd->Dw_Transmit(pd);
}

/**
 * @brief  迪文屏幕指定页面切换
 * @param  pd 迪文屏幕对象句柄
 * @param  page 目标页面
 * @retval None
 */
static void Dwin_PageChange(pDwinHandle pd, uint16_t page)
{
#if (DWIN_USING_CRC)
	uint8_t buf[] = {
		0x5A, 0xA5, 0x07 + 2U, DWIN_WRITE_CMD, 0x00, 0x84, 0x5A, 0x01,
		page >> 8U, page};
#else
	uint8_t buf[] = {
		0x5A, 0xA5, 0x07, DWIN_WRITE_CMD, 0x00, 0x84, 0x5A, 0x01,
		page >> 8U, page};
#endif
	dwin_tx_count(pd) = 0U;
	memcpy(dwin_tx_buf, buf, sizeof(buf));
	dwin_tx_count(pd) += sizeof(buf);

	pd->Dw_Transmit(pd);
}

/**
 * @brief  迪文屏幕识别16bit数据的正负
 * @param  pd 迪文屏幕对象句柄
 * @param  page 目标页面
 * @retval None
 */
static short int Dwin_GetSignedData(pDwinHandle pd, uint16_t udata)
{
	(void)pd;

	short int sdata = 0;
	/*识别负数*/
	if (udata & 0x1000)
		sdata = -1 * (short int)(~udata + 1U);
	else
		sdata = (short int)udata;

	return sdata;
}

/**
 * @brief  Dwin接收数据帧合法性检查
 * @param  pd modbus对象句柄
 * @retval dwin_result 检验结果
 */
static dwin_result Dwin_Recive_Check(pDwinHandle pd)
{
#define DWIN_MIN_FRAME_LEN 5U // 3个前导码+2个crc16
#if (DWIN_USING_DEBUG && DWIN_SEE_RX_BUFF)
	DWIN_DEBUG("\r\ndwin_rx_buf[%d]:", dwin_rx_count(pd));
	for (uint8_t i = 0; i < dwin_rx_count(pd); i++)
	{
		DWIN_DEBUG_R("%02X ", dwin_rx_buf[i]);
	}
	DWIN_DEBUG_R("\r\n\r\n");
#endif
	/*检查帧头*/
	if ((dwin_rx_buf[0] != 0x5A) || (dwin_rx_buf[1] != 0xA5))
	{
#if (DWIN_USING_DEBUG)
		DWIN_DEBUG("@error:Protocol frame header error.\r\n");
#endif
#if (!DWIN_SEE_RX_BUFF)
		DWIN_DEBUG("\r\ndwin_rx_buf[%d]:", dwin_rx_count(pd));
		for (uint8_t i = 0; i < dwin_rx_count(pd); i++)
		{
			DWIN_DEBUG_R("%#X ", dwin_rx_buf[i]);
		}
		DWIN_DEBUG_R("\r\n\r\n");
#endif
		return err_frame_head;
	}
	/*检查接收数据的尺寸*/
	if ((dwin_rx_count(pd) >= pd->Uart.rx.size) ||
		(dwin_rx_count(pd) < DWIN_MIN_FRAME_LEN))
	{
#if (DWIN_USING_DEBUG)
		DWIN_DEBUG("@error:Data length error,cur_len: %d.\r\n", dwin_rx_count(pd));
#endif
		return err_data_len;
	}
	/*检查crc校验码*/
	uint16_t crc16 = get_crc16(&dwin_rx_buf[3U], dwin_rx_count(pd) - 5U, 0xFFFF);
	if (crc16 == Get_Dwin_Data(dwin_rx_buf, dwin_rx_count(pd) - 2U, DWIN_WORD))
	{
#if (DWIN_USING_DEBUG)
		DWIN_DEBUG("@error:crc check code error.\r\n");
		DWIN_DEBUG("dwin_rxcount = %d,crc16 = 0x%X.\r\n", dwin_rx_count(pd),
				   (uint16_t)((crc16 >> 8U) | (crc16 << 8U)));
#endif
		return err_check_code;
	}

	return dwin_ok;
#undef DWIN_MIN_FRAME_LEN
}

/**
 * @brief  迪文屏幕接收数据解析
 * @param  pd 迪文屏幕对象句柄
 * @retval None
 */
static void Dwin_Poll(pDwinHandle pd)
{
	if (pd == NULL)
		return;
	dwin_result dwin_code = Dwin_Recive_Check(pd);
	if (dwin_code != dwin_ok)
		return;
	uint16_t addr = Get_Dwin_Data(dwin_rx_buf, 4U, DWIN_WORD);

	for (uint8_t i = 0; i < pd->Slave.Events_Size; i++)
	{
		if (pd->Slave.pMap[i].addr == addr)
		{
			if (pd->Slave.pMap[i].event)
				pd->Slave.pMap[i].event(pd, i, addr);
			break;
		}
	}

	memset(dwin_rx_buf, 0x00, dwin_rx_count(pd));
	dwin_rx_count(pd) = 0U;
}
