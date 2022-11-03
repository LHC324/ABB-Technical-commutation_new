/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_CFG_H_
#define PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_CFG_H_
#ifdef __cplusplus
extern "C"
{
#endif
/*****************************************************功能配置区*******************************************/
/*small_modbus使用RTOS[0:不使用RTOS;1:Freertos;2:rt_thread]*/
#define SMODBUS_USING_RTOS 2
/*small_modbus动态内存分配[0:不使用；1：使用]*/
#define SMODBUS_USING_MALLOC 1
/*small_modbusDMA选项*/
#define SMODBUS_USING_DMA 1
/*small_modbusCRC校验*/
#define SMODBUS_USING_CRC 1
/*small_modbus调试选项*/
#define SMODBUS_USING_DEBUG 2
/*small_modbus调试输出终端选择:[0:不使用调试终端；1：leeter shell; 2:finish shell]*/
#define SMODBUS_USING_SHELL 2
/*small_modbus数据缓冲区尺寸*/
#define SMODBUS_RX_BUF_SIZE 128
#define SMODBUS_TX_BUF_SIZE 128

#if (SMODBUS_USING_MALLOC)
#define smd_malloc rt_malloc
#define smd_free rt_free
#endif

/*协议栈参数配置*/
#define SMALL_MODBUS_MASTER_ADDR 0x01
#define SMALL_MODBUS_SLAVE_ADDR 0x02
/*寄存器池尺寸*/
#define SMODBUS_REG_POOL_SIZE 128U

#define COIL_OFFSET (1)
#define INPUT_COIL_OFFSET (10001)
#define INPUT_REGISTER_OFFSET (30001)
#define HOLD_REGISTER_OFFSET (40001)
/*启用主机模式*/
#define SMODBUS_USING_MASTER
/*启用线圈*/
#define SMODBUS_USING_COIL
/*启用输入线圈*/
#define SMODBUS_USING_INPUT_COIL
/*启用输入寄存器*/
#define SMODBUS_USING_INPUT_REGISTER
/*启用保持寄存器*/
#define SMODBUS_USING_HOLD_REGISTER

/*包含对应操作系统接口:tool.h中包含对应api*/
#if (SMODBUS_USING_RTOS == 1)

#elif (SMODBUS_USING_RTOS == 2)

#else

#endif

/*检查关联寄存器组是否同时启用*/
#if defined(SMODBUS_USING_COIL) && !defined(SMODBUS_USING_INPUT_COIL)
#error Input coil not defined!
#elif !defined(SMODBUS_USING_COIL) && defined(SMODBUS_USING_INPUT_COIL)
#error Coil not defined!
#elif defined(SMODBUS_USING_INPUT_REGISTER) && !defined(SMODBUS_USING_HOLD_REGISTER)
#error Holding register not defined!
#elif !defined(SMODBUS_USING_INPUT_REGISTER) && defined(SMODBUS_USING_HOLD_REGISTER)
#error The input register is not defined!
#endif

#if (SMODBUS_USING_DEBUG == 1)
/*small modbus调试接口*/
#define SMODBUS_DEBUG (fmt, ...) shellPrint(Shell_Object, fmt, ##__VA_ARGS__)
#elif (SMODBUS_USING_DEBUG == 2)
#undef DBG_TAG
#define DBG_TAG "smd"
#define DBG_LVL DBG_LOG
/*必须位于DBG_SECTION_NAME之后*/
#include <rtdbg.h>
#define SMODBUS_DEBUG_R dbg_raw
#define SMODBUS_DEBUG_D LOG_D
#define SMODBUS_DEBUG_I LOG_I
#define SMODBUS_DEBUG_W LOG_W
#define SMODBUS_DEBUG_E LOG_E
#else
#define SMODBUS_DEBUG
#endif

#ifdef __cplusplus
}
#endif
#endif /* PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_CFG_H_ */
