/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "main.h"
#include "dwin_port.h"
#include "small_modbus_port.h"
#include "io_signal.h"
#include "ad9833.h"
#include "test.h"
#ifdef DBG_TAG
#undef DBG_TAG
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#endif

/*三角函数：https://zhuanlan.zhihu.com/p/361839484
https://blog.csdn.net/Gou_Hailong/article/details/122830552*/
/*ADC超频采样AC信号：https://blog.csdn.net/qq_34022877/article/details/121941236*/

typedef enum
{
    using_semaphore = 0x00,
    unusing_semaphore,
} semaphore_state;

/*rt_thread 线程池*/
typedef struct rt_thread_pools
{
    rt_thread_t thread_handle;
    const char name[RT_NAME_MAX];
    void *parameter;
    rt_uint32_t stack_size;
    rt_uint8_t priority;
    rt_uint32_t tick;
    void (*thread)(void *parameter);
    semaphore_state sema_state;
    rt_sem_t semaphore;
} rt_thread_pools_t;

typedef struct
{
    // rt_thread_t thread_handle_t;
    rt_thread_pools_t *pools;
    rt_uint32_t rt_thread_numbers;
} rt_thread_pool_map_t;

/*线程入口函数声明区*/
static void modbus_slave_thread_entry(void *parameter);
static void dwin_recive_thread_entry(void *parameter);
static void sampling_thread_entry(void *parameter);
static void contrl_thread_entry(void *parameter);
static void test_poll_thread_entry(void *parameter);
static void report_thread_entry(void *parameter);

#define __init_rt_thread_pools(__name, __handle, __param, __statck_size,    \
                               __prio, __tick, __fun, __sema_state, __sema) \
    {                                                                       \
        .name = __name,                                                     \
        .thread_handle = __handle,                                          \
        .parameter = __param,                                               \
        .stack_size = __statck_size,                                        \
        .priority = __prio,                                                 \
        .tick = __tick,                                                     \
        .thread = __fun,                                                    \
        .sema_state = __sema_state,                                         \
        .semaphore = __sema,                                                \
    }

static rt_thread_pools_t thread_pools[] = {
    __init_rt_thread_pools("md_slave_thread", RT_NULL, RT_NULL, 1024U, 0x0F,
                           10, modbus_slave_thread_entry, using_semaphore, RT_NULL),
    __init_rt_thread_pools("dwin_thread", RT_NULL, RT_NULL, 1024U, 0x0F,
                           10, dwin_recive_thread_entry, using_semaphore, RT_NULL),
    __init_rt_thread_pools("sampling_thread", RT_NULL, RT_NULL, 1024U, 0x05,
                           50, sampling_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("contrl_thread", RT_NULL, RT_NULL, 512U, 0x0F,
                           10, contrl_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("measure_thread", RT_NULL, RT_NULL, 2048U, 0x06,
                           10, test_poll_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("measure_exe", RT_NULL, RT_NULL, 2048U, 0x10,
                           10, report_thread_entry, unusing_semaphore, RT_NULL),

};

rt_thread_pool_map_t rt_thread_pool_map = {
    // .thread_handle_t = RT_NULL,
    .pools = thread_pools,
    .rt_thread_numbers = sizeof(thread_pools) / sizeof(rt_thread_pools_t),
};

/**
 * @brief	rt thread 线程池初始化
 * @details
 * @param	none
 * @retval  none
 */
// static int rt_thread_pools_init(rt_thread_pool_map_t *p_rt_thread_map)
static void rt_thread_pools_init(rt_thread_pool_map_t *p_rt_thread_map)
{
    rt_thread_pools_t *p_rt_thread_pools = p_rt_thread_map->pools;
    if (p_rt_thread_map->rt_thread_numbers && p_rt_thread_pools)
    {
        // rt_thread_t thread_handle = RT_NULL;
        /*从线程池中初始化线程*/
        for (rt_thread_pools_t *p = p_rt_thread_pools;
             p < p_rt_thread_pools + p_rt_thread_map->rt_thread_numbers; ++p)
        {
            if (p->thread)
            {
                /*线程自生参数信息传递给线程入口函数*/
                p->parameter = p;
                p->thread_handle = rt_thread_create(p->name, p->thread, p->parameter,
                                                    p->stack_size, p->priority, p->tick);
            }
            /* 创建一个动态信号量，初始值是 0 */
            if (p->sema_state == using_semaphore)
                p->semaphore = rt_sem_create("sempx", 0, RT_IPC_FLAG_PRIO);
            if (p->thread_handle)
                rt_thread_startup(p->thread_handle);
        }
    }
    //    return 0;
}
/*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
// INIT_COMPONENT_EXPORT(rt_thread_pools_init);
// INIT_APP_EXPORT(rt_thread_pools_init);
// INIT_ENV_EXPORT(rt_thread_pools_init);

/**
 * @brief	rt_thread 初始化目标串口的串口空闲中断+DMA接收
 * @details 乒乓缓冲实现方式：https://www.cnblogs.com/puyu9495/p/15914090.html
 * @param	puart uartx句柄
 * @param   p_pool 线程池句柄
 * @retval  none
 */
static void rt_thread_hal_uartx_dma_info_init(rt_thread_pools_t *p_pool, pUartHandle puart)
{
    /*初始化目标串口DMA配置*/
    if (puart && puart->huart && puart->phdma && puart->rx.pbuf)
    {
        __HAL_UART_ENABLE_IT((UART_HandleTypeDef *)puart->huart, UART_IT_IDLE);
        /*DMA buffer must point to an entity address!!!*/
        HAL_UART_Receive_DMA((UART_HandleTypeDef *)puart->huart, puart->rx.pbuf, puart->rx.size);
        puart->rx.count = 0;
        rt_memset(puart->rx.pbuf, 0x00, puart->rx.size);
    }
    if (p_pool && p_pool->semaphore)
        puart->semaphore = p_pool->semaphore;
}

/* defined the LED0 pin: PB1 */
// #define LED0_PIN GET_PIN(B, 1)

int main(void)
{
    // int count = 1;
    // /* set LED0 pin mode to output */
    // rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    // while (count++)
    // {
    //     rt_pin_write(LED0_PIN, PIN_HIGH);
    //     rt_thread_mdelay(500);
    //     rt_pin_write(LED0_PIN, PIN_LOW);
    //     rt_thread_mdelay(500);
    // }
    extern ADC_HandleTypeDef hadc1;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc_buffer, ADC_DMA_SIZE);
    /*初始化线程池*/
    rt_thread_pools_init(&rt_thread_pool_map);
    for (;;)
    {
        HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}

/**
 * @brief	modbus 从机接收线程
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void modbus_slave_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    /*初始化modbus接口信号量*/
    // extern UartHandle small_modbus_uart;
    rt_thread_hal_uartx_dma_info_init(p_rt_thread_pool, &Modbus_Object->Uart);

    for (;;)
    {
        /* 永久方式等待信号量，获取到信号量则解析modbus协议*/
        if (p_rt_thread_pool->semaphore &&
            rt_sem_take(p_rt_thread_pool->semaphore, RT_WAITING_FOREVER) == RT_EOK)
        {
            small_modbus_handler(Modbus_Object);
            /*远程升级*/
            LOG_D("small modbus recive a data.");
        }
    }
}

/**
 * @brief	dwin 线程解析接收的数据
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void dwin_recive_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    rt_thread_hal_uartx_dma_info_init(p_rt_thread_pool, &Dwin_Object->Uart);

    for (;;)
    {
        /* 永久方式等待信号量*/
        if (p_rt_thread_pool->semaphore &&
            rt_sem_take(p_rt_thread_pool->semaphore, RT_WAITING_FOREVER) == RT_EOK)
        {
            dwin_handler(Dwin_Object);
            // LOG_D("dwin recive a data.");
        }
    }
}

/**
 * @brief	采样线程定时采样数据
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void sampling_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    UNUSED(p_rt_thread_pool);

    for (;;)
    {
        Read_Io_Handle();
        rt_thread_mdelay(500);
    }
}

/**
 * @brief	控制线程定时动作
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void contrl_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    UNUSED(p_rt_thread_pool);

    for (;;)
    {
        Write_Io_Handle();
        ad9833_out_target_wave();
        rt_thread_mdelay(100);
    }
}

/**
 * @brief	测试系统事件处理
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void test_poll_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    UNUSED(p_rt_thread_pool);

    for (;;)
    {
        extern void test_poll(void);
        // test_poll();
        rt_thread_mdelay(500);
    }
}

/**
 * @brief   定时上报数据到屏幕
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void report_thread_entry(void *parameter)
{
#define VAL_UINT8_T_NUM 4U
#define VAL_UINT16_T_NUM 16U
#define VAL_FLOAT_NUM 40U //浮点数据数量（不包括历史数据）
#define BUF_SIE (VAL_UINT8_T_NUM + VAL_UINT16_T_NUM * 2U + VAL_FLOAT_NUM * 4U)
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    pModbusHandle pd = Modbus_Object;
    pDwinHandle pw = Dwin_Object;
    // pmeasure_handle pm = &measure_object;
    ptesthandle pt = &test_object;
    uint8_t out_coil[32U];
    uint8_t buf[BUF_SIE];
    UNUSED(p_rt_thread_pool);

    for (;;)
    {
        if (pd && pw && pt->pre)
        {
            memset(buf, 0x00, sizeof(buf));
            /*数字输出:32路*/
            pd->Mod_Operatex(pd, Coil, Read, 0x00, out_coil, sizeof(out_coil));
            for (uint8_t i = 0; i < sizeof(out_coil); ++i)
            {
                uint8_t site = i / 8U;

                site = !site ? 1U : site == 1U ? 0U
                                : site == 2U   ? 3U
                                               : 2U;
                buf[site] |= out_coil[i] << i % 8U;
            }
            /*波形选项*/
            buf[5] = pt->ac.wave_param.wave_mode;
            /*电压模式*/
            buf[7] = pt->pre->cur_power;
            /*wifi开关*/
            buf[9] = 0x01;
            /*继电器策略*/
            buf[11] = pt->cur_tactics;
            /*通道判定结果*/
            buf[13] = (uint8_t)pt->test_result;
            /*通道告警*/
            buf[15] = 0x00;

            /*32bit采样数据+电压电流偏差*/
            memcpy(&buf[VAL_UINT16_T_NUM * 2U], pt->data.val_buf, sizeof(pt->data.val_buf));
            memcpy(&buf[VAL_UINT16_T_NUM * 2U + 16U * sizeof(float)], pt->data.offset_buf,
                   sizeof(pt->data.offset_buf));
            /*频率*/
            // memcpy(&buf[VAL_UINT16_T_NUM * 2U + 32U * 4U], &pt->ac.wave_param.frequency, 4U);
            // /*系统允许电压/电流偏差*/
            // memcpy(&buf[VAL_UINT16_T_NUM * 2U + 32U * 4U + 4U], &pt->comm_param, 8U);
            // /*相位*/
            // memcpy(&buf[VAL_UINT16_T_NUM * 2U + 32U * 4U + 12U], &pt->ac.wave_param.phase, 2U);
            // /*频率寄存器、相位寄存器、幅度、波形*/
            // memcpy(&buf[VAL_UINT16_T_NUM * 2U + 32U * 4U + 14U], &pt->ac.wave_param.fre_sfr,
            //        VAL_UINT8_T_NUM);
            float temp_buf[8] = {
                (float)pt->ac.wave_param.frequency,
                (float)pt->ac.wave_param.fre_sfr,
                (float)pt->ac.wave_param.phase,
                (float)pt->ac.wave_param.phase_sfr,
                (float)pt->ac.wave_param.range,
                (float)pt->cur_segment,
                (float)pt->comm_param.voltage_offset,
                (float)pt->comm_param.current_offset,
            };

            memcpy(&buf[VAL_UINT16_T_NUM * 2U + 32U * 4U], temp_buf, sizeof(temp_buf));
            /*4字节数据交换*/
            for (uint8_t i = 0; i < VAL_FLOAT_NUM; ++i)
                endian_swap(&buf[VAL_UINT16_T_NUM * sizeof(uint16_t) + i * sizeof(float)], 0, sizeof(float));

            pw->Dw_Write(pw, DWIN_DI_OUTPUT_ADDR, buf, sizeof(buf));
        }
        rt_thread_mdelay(1000);
    }
#undef VAL_UINT8_T_NUM
#undef VAL_UINT16_T_NUM
#undef VAL_FLOAT_NUM
#undef BUF_SIE
}
