/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef PACKAGES_DWIN_INC_DWIN_CFG_H_
#define PACKAGES_DWIN_INC_DWIN_CFG_H_
#ifdef __cplusplus
extern "C"
{
#endif
/*****************************************************功能配置区*******************************************/
/*迪文屏幕使用RTOS[0:不使用RTOS;1:Freertos;2:rt_thread]*/
#define DWIN_USING_RTOS 2
/*迪文屏幕动态内存分配[0:不使用；1：使用]*/
#define DWIN_USING_MALLOC 1
/*迪文屏幕DMA选项*/
#define DWIN_USING_DMA 1
/*迪文屏幕CRC校验*/
#define DWIN_USING_CRC 1
/*迪文屏幕调试选项*/
#define DWIN_USING_DEBUG 2
/*迪文屏幕调试输出终端选择:[0:不使用调试终端；1：leeter shell; 2:finish shell]*/
#define DWIN_USING_SHELL 2
/*迪文屏幕数据缓冲区尺寸*/
#define DWIN_RX_BUF_SIZE 128
#define DWIN_TX_BUF_SIZE 512

#if (DWIN_USING_MALLOC)
#define dw_malloc rt_malloc
#define dw_free rt_free
#endif

#if (DWIN_USING_DEBUG == 1)
/*迪文屏幕调试接口*/
#define DWIN_DEBUG (fmt, ...) shellPrint(Shell_Object, fmt, ##__VA_ARGS__)
#elif (DWIN_USING_DEBUG == 2)
#undef DBG_TAG
#define DBG_TAG "dwin"
#define DBG_LVL DBG_LOG
/*必须位于DBG_SECTION_NAME之后*/
#include <rtdbg.h>
#define DWIN_DEBUG_R dbg_raw
#define DWIN_DEBUG LOG_D
#else
#define DWIN_DEBUG
#endif

#ifdef __cplusplus
}
#endif
#endif /* PACKAGES_DWIN_INC_DWIN_CFG_H_ */
