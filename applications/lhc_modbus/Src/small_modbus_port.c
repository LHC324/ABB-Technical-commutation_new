/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 *  @verbatim
 *  使用： 1、用户需要完善"rt_small_modbus_init/MX_ModbusInit"、"Modbus_Send"函数
 *         2、用户需要定义"ModbusPools"寄存器池
 *         3、"rt_small_modbus_init/MX_ModbusInit"初始化时需要明确指定"UartHandle"参数
 *         4、"Modbus_CallBack"函数用户按需编写
 *         5、按需配置"small_modbus_cfg.h"
 */
#include "small_modbus_port.h"
#include <rtdevice.h>

#define __init_modbus(__name, __type, __master_id, __slave_id,                    \
                      __callback, __lock, __unlock, __ota_update, __error_handle, \
                      __transmit, __uart, __pools, __user_handle)                 \
    MdbusHandle __name##_small_modbus = {                                         \
        .type = __type,                                                           \
        .Master.id = __master_id,                                                 \
        .Slave.id = __slave_id,                                                   \
        .Mod_CallBack = __callback,                                               \
        .Mod_Lock = __lock,                                                       \
        .Mod_Unlock = __unlock,                                                   \
        .Mod_Ota = __ota_update,                                                  \
        .Mod_Error = __error_handle,                                              \
        .Mod_Transmit = __transmit,                                               \
        .Uart = __uart,                                                           \
        .pPools = &__pools,                                                       \
        .Slave.pHandle = __user_handle,                                           \
    };

/*定义Modbus对象*/
pModbusHandle Modbus_Object;
static ModbusPools Spool;
/* 指向互斥量的指针 */
static rt_mutex_t modbus_mutex = RT_NULL;

static void Modbus_CallBack(pModbusHandle pd, Function_Code code);
static void lhc_console_transfer(pModbusHandle pd);
static void Modbus_ErrorHadle(pModbusHandle pd, Lhc_Modbus_State_Code error_code);
#if (SMODBUS_USING_RTOS)
static void Modbus_Lock(void);
static void Modbus_UnLock(void);
#endif
static void Modbus_Send(pModbusHandle pd, enum Using_Crc crc);

#if (SMODBUS_USING_RTOS == 2)
#if (SMODBUS_USING_MALLOC)
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

/**
 * @brief  初始化Modbus协议库
 * @retval None
 */
int rt_small_modbus_init(void)
{
    /* 创建一个动态互斥量 */
    modbus_mutex = rt_mutex_create("modbus_mutex", RT_IPC_FLAG_PRIO);
    UartHandle small_modbus_uart = {
        .huart = &huart1,
        .phdma = &hdma_usart1_rx,
#if (SMODBUS_USING_RTOS)
        .semaphore = NULL,
#else
        .recive_finish_flag = false,
#endif
        .tx = {
            .wmode = uart_using_it,
            .size = SMODBUS_TX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
        .rx = {
            .wmode = uart_using_dma,
            .size = SMODBUS_RX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
    };
    __init_modbus(temp, Smd_Slave, SMALL_MODBUS_MASTER_ADDR, SMALL_MODBUS_SLAVE_ADDR,
                  Modbus_CallBack, Modbus_Lock, Modbus_UnLock, lhc_console_transfer, Modbus_ErrorHadle,
                  Modbus_Send, small_modbus_uart, Spool, NULL);
#if (SMODBUS_USING_RTOS == 2U)
    // temp_small_modbus.dev_name = "uart1";
    temp_small_modbus.dev = NULL;
    temp_small_modbus.old_console = NULL;
#endif
    Create_ModObject(&Modbus_Object, &temp_small_modbus);
    return 0;
}
/*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
// INIT_COMPONENT_EXPORT(rt_small_modbus_init);
// INIT_ENV_EXPORT(rt_small_modbus_init);
INIT_DEVICE_EXPORT(rt_small_modbus_init);

/*主机使用example*/
/*void small_modbus_master_request(void)
{
    pModbusHandle pd = Modbus_Object;
    Request_HandleTypeDef request = {
        .code = ReadHoldReg,
        .reg_start_addr = 0x0000,
        .reg_len = 0x01,
    };

    if (pd)
    {
        memcpy(&pd->Master.request_data, &request, sizeof(request));
        pd->Mod_Request(pd);
    }
}*/
#else
int rt_small_modbus_init(void)
{
    MdbusHandle temp_small_modbus = {
        /*User init info*/
    };
    Create_ModObject(&Modbus_Object, &temp_small_modbus);
    return 0;
}
#endif

#else
void MX_ModbusInit(void)
{
}

#endif

/**
 * @brief  modbus协议栈外部回调函数
 * @param  pd 需要初始化对象指针
 * @param  code 功能码
 * @retval None
 */
static void Modbus_CallBack(pModbusHandle pd, Function_Code code)
{
}

#if (SMODBUS_USING_RTOS)
/**
 * @brief  modbus协议栈加锁函数
 * @param  pd 需要初始化对象指针
 * @param  code 功能码
 * @retval None
 */
static void Modbus_Lock(void)
{
    rt_mutex_take(modbus_mutex, RT_WAITING_FOREVER);
}

/**
 * @brief  modbus协议栈解锁函数
 * @param  pd 需要初始化对象指针
 * @param  code 功能码
 * @retval None
 */
static void Modbus_UnLock(void)
{
    rt_mutex_release(modbus_mutex);
}
#endif

/**
 * @brief  modbus协议栈关闭目标串口dma模式
 * @param  pd modbus协议站句柄
 * @retval None
 */
// static void modbus_close_uart_dma_revc_mode(pModbusHandle pd)
// {
//     if (NULL == pd || pd->Uart.huart)
//         return;
//     /*关闭空闲中断*/
//     __HAL_UART_DISABLE_IT((UART_HandleTypeDef *)pd->Uart.huart, UART_IT_IDLE);
// }

/**
 * @brief  modbus协议栈进行控制台转移
 * @param  pd modbus协议站句柄
 * @retval None
 */
static void lhc_console_transfer(pModbusHandle pd)
{
    extern void finsh_set_device(const char *device_name);
    rt_err_t ret = RT_EOK;
    if (NULL == pd || NULL == pd->Uart.huart ||
        NULL == pd->dev)
        return;

    ret = rt_device_close(pd->dev);
    if (RT_EOK == ret)
    {
        rt_device_set_rx_indicate(pd->dev, RT_NULL); // 置空当前串口回调函数
        pd->old_console = rt_console_set_device(pd->dev->parent.name);
        finsh_set_device(pd->dev->parent.name);
    }

    /*以流模式打开串口*/
    // ret = rt_device_open(p_dev, RT_DEVICE_FLAG_TX_BLOCKING |
    //                                 RT_DEVICE_FLAG_RX_NON_BLOCKING |
    //                                 RT_DEVICE_FLAG_STREAM);
    rt_thread_t thread_handle = rt_thread_self();
    if (RT_EOK == ret && thread_handle)
    {
        rt_kprintf("@note: enter finsh mode.\r\n");
        rt_thread_suspend(thread_handle);
    }
}

/**
 * @brief  modbus协议栈接收帧错误处理
 * @param  pd 需要初始化对象指针
 * @param  error_code 错误码
 * @retval None
 */
static void Modbus_ErrorHadle(pModbusHandle pd, Lhc_Modbus_State_Code error_code)
{
}

/**
 * @brief  Modbus协议发送
 * @param  pd 需要初始化对象指针
 * @retval None
 */
static void Modbus_Send(pModbusHandle pd, enum Using_Crc crc)
{
#if (SMODBUS_USING_RTOS == 2U)
    if (NULL == pd || NULL == pd->dev)
#else
    if (NULL == pd || NULL == pd->Uart.huart)
#endif
        return;

    if (crc == UsedCrc)
    {
        uint16_t crc16 = get_crc16(smd_tx_buf, smd_tx_count(pd), 0xffff);

        memcpy(&smd_tx_buf[smd_tx_count(pd)], (uint8_t *)&crc16, sizeof(crc16));
        smd_tx_count(pd) += sizeof(crc16);
    }

#if (SMODBUS_USING_RTOS == 2U)
    rt_device_write(pd->dev, 0, smd_tx_buf, smd_tx_count(pd));
#else
    switch (pd->Uart.tx.wmode)
    {
    case uart_using_it:
    {
        HAL_UART_Transmit((UART_HandleTypeDef *)pd->Uart.huart, smd_tx_buf, smd_tx_count(pd), 0x100);
    }
    break;
    case uart_using_dma:
    {
        HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pd->Uart.huart, smd_tx_buf, smd_tx_count(pd));
        while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pd->Uart.huart, UART_FLAG_TC) == RESET)
        {
        }
    }
    break;
    default:
        break;
    }
#endif
}
