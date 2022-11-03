/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_PORT_H_
#define PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_PORT_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "small_modbus.h"

    extern pModbusHandle Modbus_Object;

#if (SMODBUS_USING_RTOS == 2)
    extern int rt_small_modbus_init(void);
#else
extern void MX_ModbusInit(void);
#endif

#ifdef __cplusplus
}
#endif
#endif /* PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_PORT_H_ */
