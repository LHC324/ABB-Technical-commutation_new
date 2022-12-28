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
#include <fal.h>
#include <dfs_fs.h>
#include "dwin_port.h"
#include "small_modbus_port.h"
#include "io_signal.h"
#include "ad9833.h"
#include "test.h"
#include "minIni.h"
#include <at.h>
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
static void adc_start_conv_thread_entry(void *parameter);
static void sampling_thread_entry(void *parameter);
static void control_thread_entry(void *parameter);
static void test_poll_thread_entry(void *parameter);
static void report_thread_entry(void *parameter);
static void at_fsm_thread_entry(void *parameter);

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
/*finsh控制台卡死问题：https://www.cnblogs.com/jzcn/p/16450021.html*/
static rt_thread_pools_t thread_pools[] = {
    __init_rt_thread_pools("md_slave_thread", RT_NULL, RT_NULL, 1024U, 0x11,
                           10, modbus_slave_thread_entry, using_semaphore, RT_NULL),
    __init_rt_thread_pools("dwin_thread", RT_NULL, RT_NULL, 2048U, 0x0F,
                           10, dwin_recive_thread_entry, using_semaphore, RT_NULL),
    __init_rt_thread_pools("adc_trig_thread", RT_NULL, &test_object, 512U, 0x03,
                           10, adc_start_conv_thread_entry, using_semaphore, RT_NULL),
    __init_rt_thread_pools("sampling_thread", RT_NULL, &test_object, 2048U, 0x05,
                           100, sampling_thread_entry, using_semaphore, RT_NULL),
    __init_rt_thread_pools("control_thread", RT_NULL, RT_NULL, 512U, 0x0F,
                           10, control_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("test_thread", RT_NULL, RT_NULL, 2048U, 0x05,
                           10, test_poll_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("report_thread", RT_NULL, RT_NULL, 2048U, 0x10,
                           10, report_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("at_fsm_thread", RT_NULL, &test_object, 2048U, 0x12,
                           10, at_fsm_thread_entry, unusing_semaphore, RT_NULL),

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
                // p->parameter = p;
                p->thread_handle = rt_thread_create(p->name, p->thread, p,
                                                    p->stack_size, p->priority, p->tick);
            }
            /* 创建一个动态信号量，初始值是 0 */
            if (p->sema_state == using_semaphore)
            {
                char *psemaphore_name = rt_malloc(RT_NAME_MAX);
                if (psemaphore_name)
                {
                    rt_strncpy(psemaphore_name, p->name, RT_NAME_MAX); // 最大拷贝RT_NAME_MAX
                    p->semaphore = rt_sem_create(strncat(psemaphore_name, "_sem", 5), 0, RT_IPC_FLAG_PRIO);
                }
                rt_free(psemaphore_name);
            }

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

/**
 * @Function    ota_app_vtor_reconfig
 * @note rt-thread ota 升级指导https://getiot.tech/rtt/rt-thread-ota.html
 * @note 官网文档 https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/application-note/system/rtboot/an0028-rtboot
 * @note qboot使用 https://blog.csdn.net/victor_zy/article/details/122844572
 * @Description Set Vector Table base location to the start addr of app(RT_APP_PART_ADDR).
 */
static int ota_app_vtor_reconfig(void)
{
#define NVIC_VTOR_MASK 0x3FFFFF80
    /* Set the Vector Table base location by user application firmware definition */
    SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;

    return 0;
}
INIT_BOARD_EXPORT(ota_app_vtor_reconfig);

/* defined the LED0 pin: PB1 */
// #define LED0_PIN GET_PIN(B, 1)

static void timer1_callback(void *parameter);
static void test_timer_callback(void *parameter);
static void see_sys_info(void);
// static rt_thread_t modbus_rtu_thread_handle = RT_NULL;
rt_sem_t adc_semaphore = RT_NULL;      // adc转换完成同步信号量
rt_sem_t trig_semaphore = RT_NULL;     // 触发adc开始转换同步信号量
rt_sem_t continue_semaphore = RT_NULL; // 继续同步信号量
// rt_sem_t at_stop_semaphore = RT_NULL;  // 停止at解析线程信号量

/**
 * @brief	挂载文件系统到rt_thread
 * @details
 * @note    littlefs文件系统使用教程：https://club.rt-thread.org/ask/article/a5c8b007eed2584e.html
 * @param	None
 * @retval  none
 */
void mount_file_system(void)
{
/* 定义要使用的分区名字 */
#define FS_PARTITION_NAME "filesystem"
    struct rt_device *mtd_dev = RT_NULL;
    /* 生成 mtd 设备 */
    mtd_dev = fal_mtd_nor_device_create(FS_PARTITION_NAME);
    if (mtd_dev == RT_NULL)
    {
        LOG_E("Can't create a mtd device on '%s' partition.", FS_PARTITION_NAME);
    }
    else
    {
        /* 挂载 littlefs */
        if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
        {
            LOG_I("Filesystem initialized!");
        }
        else
        {
            /* 格式化文件系统 */
            dfs_mkfs("lfs", FS_PARTITION_NAME);
            /* 挂载 littlefs */
            if (dfs_mount("filesystem", "/", "lfs", 0, 0) == 0)
            {
                LOG_I("Filesystem initialized!");
            }
            else
            {
                LOG_E("Failed to initialize filesystem!");
            }
        }
    }
#undef FS_PARTITION_NAME
}

extern comm_val_t *get_comm_val(uint16_t index);

static struct ini_data_t ini_val_group[] = {
    __INIT_INI_VAL("run", "flag", u32, 0, 0x00),
    __INIT_INI_VAL("run", "freq_mode", u16, 0, 0x01),
    __INIT_INI_VAL("ad9833", "frequency register", u16, 0, 0x03),
    __INIT_INI_VAL("ad9833", "phase", u16, 0, 0x04),
    __INIT_INI_VAL("ad9833", "phase register", u16, 0, 0x05),
    __INIT_INI_VAL("ad9833", "range", u16, 155, 0x06),
    __INIT_INI_VAL("ad9833", "wave", u16, 0, 0x07),
    __INIT_INI_VAL("run", "start", u16, 1, 0x08),
    __INIT_INI_VAL("run", "end", u16, 7, 0x09),
    __INIT_INI_VAL("run", "voltage offset", f32, 0.5, 0x0A),
    __INIT_INI_VAL("run", "current offset", f32, 5, 0x0B),
    __INIT_INI_VAL("run", "file_size", u32, 137, 0x0C),
};
#define INI_VAL_NUM() (sizeof(ini_val_group) / sizeof(ini_val_group[0]))
/**
 * @brief  获取目标变量在'ini_val_group’位置
 * @param  index 索引值
 * @retval None
 */
struct ini_data_t *get_target_val_handle(uint16_t index)
{
    for (struct ini_data_t *pi = ini_val_group;
         (pi < ini_val_group + INI_VAL_NUM()); ++pi) //(index < INI_VAL_NUM()) &&
    {
        if (pi->index == index) // 通过比较index得到目标变量
        {
            return pi;
        }
    }

    return NULL;
}

/**
 * @brief	从.ini文件中读取目标参数
 * @details
 * @param	pi ini文件中数据句柄
 * @param   pv 变量结构
 * @param   str_size 目标数据是字符串时，指定其长度
 * @retval  none
 */
static void read_data_from_ini_file(struct ini_data_t *pi,
                                    comm_val_t *pv,
                                    uint16_t str_size)
{
#define INI_FILE_DIR "/config/config.ini"

    if (NULL == pi || NULL == pv)
        return;

    const char *p_name = text_name[pv->type];
    void *ptr = NULL;
    uint8_t c_size = BY_DATA_TYPE_GET_SIZE(pv->type);
    void *pdata = pv->val;

    switch (pv->type)
    {
    case co_string:
        if (str_size)
            ini_gets(pi->section_name, pi->key_name, pi->def_val.string, (char *)pdata, str_size, INI_FILE_DIR);
        LOG_I("[R <- S] section: %s,val: %s,[%s]: %s.", pi->section_name, pi->key_name, p_name, (char *)pdata);
        break;
    case co_bool:
    {
        bool data = ini_getbool(pi->section_name, pi->key_name, pi->def_val.b1, INI_FILE_DIR);
        ptr = &data;
        LOG_I("[R <- S] section: %s,val: %s,[%s]: %d.", pi->section_name, pi->key_name, p_name, data);
    }
    break;
    case co_uint8: // case中定义变量：https://blog.csdn.net/scutth/article/details/6894975
    case co_int16:
    case co_uint16:
    case co_int32:
    case co_uint32:
    case co_long:
    case co_ulong:
    {
        long data = ini_getl(pi->section_name, pi->key_name, pi->def_val.l32, INI_FILE_DIR);
        ptr = &data;
        LOG_I("[R <- S] section: %s,val: %s,[%s]: %ld.", pi->section_name, pi->key_name, p_name, data);
    }
    break;
    case co_float:
    {
        float data = ini_getf(pi->section_name, pi->key_name, (float)pi->def_val.f32, INI_FILE_DIR);
        ptr = &data;
        LOG_I("[R <- S] section: %s,val: %s,[%s]: %f.", pi->section_name, pi->key_name, p_name, data);
    }
    break;
    default:
        break;
    }
    if (ptr && c_size < co_type_max)
        memcpy(pdata, ptr, c_size);
}

/**
 * @brief	写入数据到.ini文件
 * @details
 * @param	pi ini文件中数据句柄
 * @param   pv 变量结构
 * @retval  none
 */
static void write_data_to_ini_file(struct ini_data_t *pi,
                                   comm_val_t *pv)
{
    int code;

    if (NULL == pi || NULL == pv)
        return;

    void *pdata = pv->val;

    const char *p_name = text_name[pv->type];

    switch (pv->type)
    {
    case co_string:
        code = ini_puts(pi->section_name, pi->key_name, (const char *)pdata, INI_FILE_DIR);
        LOG_I("[R -> S] section: %s,val: %s,[%s]: %s.", pi->section_name, pi->key_name, p_name, (char *)pdata);
        break;
    case co_bool:
    case co_uint8:
    case co_int16:
    case co_uint16:
    case co_int32:
    case co_uint32:
    case co_long:
    case co_ulong:
    {
        uint8_t c_size = BY_DATA_TYPE_GET_SIZE(pv->type);
        long temp_data = 0;
        /*解决小内存类型变量，在使用大内存指针时，访问越界，造成的数据错误*/
        // temp_data = c_size < 2U ? temp_data & 0xFFF0 : c_size < 4U ? temp_data & 0xFFF0
        //                                                            : temp_data;
        memcpy(&temp_data, pdata, c_size);
        code = ini_putl(pi->section_name, pi->key_name, temp_data, INI_FILE_DIR);
        LOG_I("[R -> S] section: %s,val: %s,[%s]: %ld.", pi->section_name, pi->key_name, p_name, temp_data);
    }
    break;
    case co_float:
        code = ini_putf(pi->section_name, pi->key_name, *(float *)pdata, INI_FILE_DIR);
        LOG_I("[R -> S] section: %s,val: %s,[%s]: %d.", pi->section_name, pi->key_name, p_name, *(float *)pdata);
        break;
    default:
        break;
    }

    if (code)
    {
        LOG_I("@note: Data written successfully.");
    }
    else
    {
        LOG_I("@note: Data write failed,code: %d.", code);
    }
}

/**
 * @brief	获取系统参数
 * @details
 * @param	None
 * @retval  none
 */
static void get_system_param(void)
{
    for (struct ini_data_t *pi = ini_val_group;
         pi && pi < ini_val_group + INI_VAL_NUM(); ++pi)
    {
        comm_val_t *pv = get_comm_val(pi->index);
        if (pv)
            read_data_from_ini_file(pi, pv, 0);
    }
}

/*启用minIni文件数据写入检查：minIni内部自带检查机制*/
#define USING_WRITE_PARAM_CHECK 0
#if (USING_WRITE_PARAM_CHECK)
/**
 * @brief	检查系统参数的变化
 * @details
 * @param	pi ini文件中数据句柄
 * @param   pv 变量结构
 * @retval  none
 */
bool check_system_param(struct ini_data_t *pi,
                        comm_val_t *pv)
{
    bool result = false;

    if (NULL == pi || NULL == pv)
        return result;

    union_data_t cur_data;
    uint8_t c_size = BY_DATA_TYPE_GET_SIZE(pv->type);
    memset(&cur_data, 0x00, sizeof(cur_data)); // 栈上分配的地址，必须初始化
    comm_val_t c_val = {
        .val = &cur_data.string,
        .type = pv->type,
    };

    /*先读取文件中目标参数：若本次修改与上次不同，更新.ini*/
    read_data_from_ini_file(pi, &c_val, 0);
    switch (pv->type)
    {
    case co_string:
    {
        if (strcmp(cur_data.string, (const char *)pv->val))
            result = true;
    }
    break;
    case co_bool:
    case co_uint8:
    case co_int16:
    case co_uint16:
    case co_int32:
    case co_uint32:
    case co_long:
    case co_ulong:
    {
        long data = 0;
        memcpy(&data, pv->val, c_size);
        // LOG_D("@note: data: %ld, l32: %ld.", data, cur_data.l32);
        if (cur_data.l32 != data)
            result = true;
    }
    break;
    case co_float:
    {
        float data = *(float *)pv->val;
        if (cur_data.l32 != data)
            result = true;
    }
    break;
    default:
        break;
    }

    return result;
}
#endif

/**
 * @brief	存储系统参数
 * @details
 * @param   pv 变量结构
 * @param	index 索引值
 * @retval  none
 */
void set_system_param(comm_val_t *pv, uint16_t index)
{
    struct ini_data_t *pi = get_target_val_handle(index);
    // comm_val_t *pv = get_comm_val(index);
    if (NULL == pi || NULL == pv)
        return;

#if (USING_WRITE_PARAM_CHECK)
    if (check_system_param(pi, pv))
#endif
        write_data_to_ini_file(pi, pv);
#if (USING_WRITE_PARAM_CHECK)
    else
    {
        LOG_I("@note: The current parameters have not changed.");
    }
#endif
}

/**
 * @brief	rt_thread main线程
 * @details
 * @param	None
 * @retval  none
 */
int main(void)
{
    rt_timer_t timer = RT_NULL;

    see_sys_info();
    /*fal层使用：https://github.com/RT-Thread-packages/fal/blob/master/README_ZH.md#falflash-%E6%8A%BD%E8%B1%A1%E5%B1%82*/
    fal_init();
    mount_file_system();
    /*初始化系统参数*/
    get_system_param();
    extern void test_get_csv_file_cur_line(void);
    test_get_csv_file_cur_line();
    // #define AT_DEVICE_NAME "uart1"
    // at_client_init(AT_DEVICE_NAME, 256U);
    // #undef AT_DEVICE_NAME
    /*初始化线程池*/
    rt_thread_pools_init(&rt_thread_pool_map);
    /* 创建定时器1  周期定时器 */
    timer = rt_timer_create("timer1",
                            timer1_callback,
                            RT_NULL,
                            2000,
                            RT_TIMER_FLAG_PERIODIC);
    /* 启动timer1定时器 */
    if (timer != RT_NULL)
        rt_timer_start(timer);
    /* 创建测试对象定时器  周期定时器 */
    timer = rt_timer_create("test_timer",
                            test_timer_callback,
                            RT_NULL,
                            1000,
                            RT_TIMER_FLAG_PERIODIC);
    /* 启动timer1定时器 */
    if (timer != RT_NULL)
        rt_timer_start(timer);
    // extern UART_HandleTypeDef huart1;
    // __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    for (;;)
    {
        HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}

/**
 * @brief	rt_thread 软件定时器回调函数
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void timer1_callback(void *parameter)
{
    UNUSED(parameter);
    extern void start_adc_conv(void);
    // start_adc_conv();
}

/**
 * @brief	rt_thread 测试对象定时器回调函数
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void test_timer_callback(void *parameter)
{
    extern void test_timer_poll(void);
    test_timer_poll();
}

/**
 * @brief	modbus 从机接收回调函数
 * @details
 * @param	dev 设备句柄
 * @param   size 当前尺寸
 * @retval  None
 */
static rt_err_t modbus_rtu_rx_ind(rt_device_t dev, rt_size_t size)
{
    pModbusHandle pd = Modbus_Object;

    // uartx_irq_recive(pd, Mod);
    if (pd && pd->Uart.semaphore)
    {
        pd->Uart.rx.count = size;
        rt_sem_release((rt_sem_t)pd->Uart.semaphore);
    }

    return RT_EOK;
}

/**
 * @brief	modbus 从机接收线程
 * @details rt thread V1和V2串口驱动说明：https://club.rt-thread.org/ask/article/8e1d18464219fae7.html
 * @param	parameter:线程初始参数
 * @retval  None
 */
void modbus_slave_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    // modbus_rtu_thread_handle = p_rt_thread_pool->thread_handle;
    pModbusHandle pd = Modbus_Object;
    rt_device_t p_dev = RT_NULL;

    if (pd)
    {
        /*确认目标串口设备存在*/
        p_dev = rt_device_find(LHC_MODBUS_DEVICE_NAME);
        if (p_dev)
        {
            // 记录当前设备指针if (RT_NULL == pd->dev)
            pd->dev = p_dev;
            /*初始化modbus接口信号量*/
            // rt_thread_hal_uartx_dma_info_init(p_rt_thread_pool, &Modbus_Object->Uart); // 和finsh线程冲突
            pd->Uart.semaphore = p_rt_thread_pool->semaphore;
            rt_device_open(p_dev, RT_DEVICE_FLAG_TX_BLOCKING | RT_DEVICE_FLAG_RX_NON_BLOCKING);
            /*挂接目标接收中断函数*/
            rt_device_set_rx_indicate(p_dev, modbus_rtu_rx_ind);
        }
        else
            rt_kprintf("@error: Target device [%s] not found.^_^\r\n", LHC_MODBUS_DEVICE_NAME);
    }
    for (;;)
    {
        /* 永久方式等待信号量，获取到信号量则解析modbus协议*/
        if (p_rt_thread_pool->semaphore &&
            rt_sem_take(p_rt_thread_pool->semaphore, RT_WAITING_FOREVER) == RT_EOK)
        {
            rt_device_read(p_dev, 0, pd->Uart.rx.pbuf, pd->Uart.rx.count);
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
 * @brief	采样ac信号时，过零点回调函数
 * @details
 * @param	args: 附加参数
 * @retval  None
 */
void gpio_zero_crossing_callback(void *args)
{
    test_t *pt = (test_t *)args;

    if (pt == NULL)
        return;
    /*开始信号无效*/
    if (!__GET_FLAG(pt->flag, test_start_signal))
        return;
    /*还未检测到过零信号*/
    if (!__GET_FLAG(pt->flag, test_zero_crossing_signal))
    {
        __SET_FLAG(pt->flag, test_zero_crossing_signal);
        /*触发adc同步采样当前ac信号*/
        release_semaphore(trig_semaphore);
    }
}

rt_base_t pin_number;
/**
 * @brief	adc开始转换线程
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void adc_start_conv_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    // UNUSED(p_rt_thread_pool);
    // trig_semaphore = rt_sem_create("trig_sem", 0, RT_IPC_FLAG_PRIO);
    trig_semaphore = p_rt_thread_pool->semaphore; // 信号量扩展到全局
    /*得到电压过零检测引脚编号*/
    pin_number = rt_pin_get("PA.5");
    rt_pin_mode(pin_number, PIN_MODE_INPUT); // 过0引脚为输入模式
    /* 绑定中断，上升沿模式，回调函数名为gpio_zero_crossing_callback */
    rt_pin_attach_irq(pin_number, PIN_IRQ_MODE_RISING,
                      gpio_zero_crossing_callback, p_rt_thread_pool->parameter);
    /* 使能中断 */
    rt_pin_irq_enable(pin_number, PIN_IRQ_ENABLE);
    /* 脱离中断回调函数 */
    // rt_pin_detach_irq(KEY0_PIN_NUM);

    for (;;)
    {
        /* 永久方式等待信号量*/
        if (p_rt_thread_pool->semaphore &&
            rt_sem_take(p_rt_thread_pool->semaphore, RT_WAITING_FOREVER) == RT_EOK)
        {
            extern void start_adc_conv(void);
            start_adc_conv();
        }
    }
}

/**
 * @brief	采样数据发送到VOFA终端
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
#define USING_VOFA 0
#if (USING_VOFA)
void vofa_send_data(void)
{
    extern UART_HandleTypeDef huart3;

    uint8_t buf[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, 0x80, 0x7f};
    float fbuf[] = {0, 0, 0};

    // for (uint16_t j = 0; j < sizeof(adc_buf[0]) / sizeof(adc_buf[0][0]) / 2U; ++j)
    for (uint16_t j = 0; j < sizeof(adc_buf[0]) / sizeof(adc_buf[0][0]); ++j)
    {
        for (uint8_t i = 0; i < sizeof(adc_buf) / sizeof(adc_buf[0]); ++i)
        {
            if (i == 0)
            {
                adc_buf[1][j] = adc_buf[0][j] >> 16U;
                adc_buf[0][j] &= 0x0000FFFF;
            }
            // memcpy(&buf[i * sizeof(float)], &adc_group[i].buf[j], sizeof(float));
            // buf[i * sizeof(float)] = i * 0.5F;
            fbuf[i] = (float)adc_buf[i][j] * 3.3F / 4096.0F;
            memcpy(&buf[i * sizeof(float)], &fbuf[i], sizeof(float));
        }
        // memcpy(&buf[0], &fbuf[0], sizeof(fbuf));
        HAL_UART_Transmit(&huart1, buf, sizeof(buf), 0xffff);
    }
}
#endif

/**
 * @brief	采样线程定时采样数据
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void sampling_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    // UNUSED(p_rt_thread_pool);
    // adc_semaphore = rt_sem_create("adc_sem", 0, RT_IPC_FLAG_PRIO);
    adc_semaphore = p_rt_thread_pool->semaphore; // 信号量扩展到全局
    test_t *pt = (test_t *)p_rt_thread_pool->parameter;
    continue_semaphore = rt_sem_create("conti_sem", 0, RT_IPC_FLAG_PRIO);

    for (;;)
    {
        // Read_Io_Handle();
        // rt_thread_mdelay(500);

        /* 永久方式等待信号量*/
        // if (rt_sem_take(adc_semaphore, RT_WAITING_FOREVER) == RT_EOK && adc_semaphore)
        if (p_rt_thread_pool->semaphore &&
            rt_sem_take(p_rt_thread_pool->semaphore, RT_WAITING_FOREVER) == RT_EOK)
        {
            extern void stop_adc_conv(void);
            stop_adc_conv();

#if (USING_VOFA)
            UNUSED(pt);
            vofa_send_data();
#else
            if (pt)
            {
                /*调度器上锁，上锁后不再切换到其他线程，仅响应中断*/
                // rt_enter_critical();
                test_data_handle(pt);
                /*调度器解锁*/
                // rt_exit_critical();
                /*开发者模式下：不再释放continue信号量*/
                if ((continue_semaphore != RT_NULL) &&
                    !__GET_FLAG(pt->flag, test_developer_signal))
                    release_semaphore(continue_semaphore);
            }
#endif
        }
    }
}

/**
 * @brief	控制线程定时动作
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void control_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    UNUSED(p_rt_thread_pool);

    for (;;)
    {
        Write_Io_Handle();
        ad9833_out_target_wave();
        extern void test_over_current_check(void);
        test_over_current_check();
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
        test_poll();
        rt_thread_mdelay(300);
    }
}

/**
 * @brief   获取wifi状态
 * @details
 * @param	None
 * @retval  None
 */
static uint8_t get_wifi_state(void)
{
    static Gpiox_info wifi_gpio[] = {
        {.pGPIOx = WIFI_READY_GPIO_Port, .Gpio_Pin = WIFI_READY_Pin},
        {.pGPIOx = WIFI_LINK_GPIO_Port, .Gpio_Pin = WIFI_LINK_Pin},
    };
    uint8_t wifi_state = 0;
    for (Gpiox_info *p = wifi_gpio;
         p < wifi_gpio + sizeof(wifi_gpio) / sizeof(wifi_gpio[0]); ++p)
    {
        uint8_t site = p - wifi_gpio;
        uint8_t bit = HAL_GPIO_ReadPin((GPIO_TypeDef *)p->pGPIOx, p->Gpio_Pin) ? 0 : 1;
        wifi_state |= bit << site;
    }
    return wifi_state;
}

/**
 * @brief   从modbus寄存器池拼装数字类型数据到buf
 * @note    size 推荐为8的整数倍
 * @details
 * @param	pd modbus句柄
 * @param   reg_type 寄存器类型
 * @param   buf  数据转载缓冲区
 * @param   size 缓冲区尺寸
 * @retval  None
 */
static void form_modbus_get_digital_data_to_buf(pModbusHandle pd,
                                                Regsiter_Type reg_type,
                                                uint8_t *buf,
                                                uint8_t size)
{
    if (pd == NULL || buf == NULL || !size)
        return;

    uint8_t coils[size];
    pd->Mod_Operatex(pd, reg_type, Read, 0x00, coils, size);

    for (uint8_t i = 0; i < size; ++i)
    {
        uint8_t byte = i / 8U;
        uint8_t site = !(byte % 2U) ? byte + 1U : byte - 1U;
        buf[site] |= coils[i] << i % 8U;
    }
}

/**
 * @brief   对16bit数据进行交换并追加到buf
 * @note    size 推荐为8的整数倍
 * @details
 * @param	buf_16 16bit缓冲区
 * @param   size   字节数
 * @param   buf_8  数据转载缓冲区
 * @retval  None
 */
static void swap_16bit_data_to_buf(uint16_t *buf_16,
                                   uint8_t size,
                                   uint8_t *buf_8)
{
#define __SWP16(A) ((((uint16_t)(A)&0xff00) >> 8) | \
                    (((uint16_t)(A)&0x00ff) << 8))

    if (NULL == buf_16 || NULL == buf_8 ||
        !size || (size % sizeof(uint16_t)))
        return;

    for (uint8_t i = 0; i < size / sizeof(uint16_t); ++i)
        buf_16[i] = __SWP16(buf_16[i]);

    memcpy(buf_8, buf_16, size);
}

/**
 * @brief   对16bit数据进行交换并追加到buf
 * @note    size 推荐为8的整数倍
 * @details
 * @param	buf_16 16bit缓冲区
 * @param   size   字节数
 * @param   buf_8  数据转载缓冲区
 * @retval  None
 */
// static void swap_32bit_data_to_buf(uint16_t *buf_16,
//                                    uint8_t size,
//                                    uint8_t *buf_8)
// {
// #define __SWP16(A) ((((uint16_t)(A)&0xff00) >> 8) | \
//                     (((uint16_t)(A)&0x00ff) << 8))

//     if (buf_16 == NULL || buf_8 == NULL || !size)
//         return;

//     for (uint8_t i = 0; i < size; ++i)
//         buf_16[i] = __SWP16(buf_16[i]);

//     memcpy(buf_8, buf_16, size);
// }

/**
 * @brief   定时上报数据到屏幕
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void report_thread_entry(void *parameter)
{
#define VAL_UINT16_T_NUM 22U
#define VAL_FLOAT_NUM 42U
#define BUF_SIE (VAL_UINT16_T_NUM * 2U + VAL_FLOAT_NUM * 4U)
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    pModbusHandle pd = Modbus_Object;
    pDwinHandle pw = Dwin_Object;
    ptest_t pt = &test_object;
    // uint8_t out_coil[32U];
    /*写入小端序，发送大端*/
    uint8_t buf[BUF_SIE] = {
        0x00, 0x00,
        0x00, 0x00,
        0x00, pt->user_freq,
        0x00, pt->ac.wave_param.wave_mode,
        /*wifi start*/
        0x00, 0x00, // wifi模块状态
        0x00, 0x00, // wifi恢复出厂设置按钮
        0x00, 0x02, // wifi模式
        0x00, 0x01, // wifi串口波特率
        0x00, 0x01, // wifi串口数据位
        0x00, 0x00, // wifi串口停止位
        0x00, 0x00, // wifi串口校验位
        0x00, 0x00, // wifi数据导出按钮
        /*wifi end*/

        0x00, 0x00, // 通道校正按钮
        0x00, 0x00, // 数据保存按钮
        0x00, 0x00, // 参数保存按钮
        0x00, 0x00, // 网络时间获取按钮
        0x00, 0x00, // 屏幕时间下发0
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00, // 屏幕时间下发3
        0x00, 0x00, // 空闲地址
        0x00, 0x00, // 开发者模式按钮
        0x00, pt->ac.wave_param.fre_sfr,
        pt->ac.wave_param.phase >> 8U, pt->ac.wave_param.phase,
        0x00, pt->ac.wave_param.phase_sfr,
        0x00, pt->ac.wave_param.range,
        0x00, pt->cur_group.start,
        0x000, pt->cur_group.end,
        0x00, 0x00

    };
    UNUSED(p_rt_thread_pool);
    if (pw)
    {
        rt_thread_mdelay(3000); // 解决首次上电迪文屏幕不接收参数问题
        // pw->Dw_Page(pw, MAIN_PAGE); //切换到登录页面
        // rt_thread_mdelay(10);
        // 系统参数刷新
        pw->Dw_Write(pw, DWIN_OPERATE_SHIFT_ADDR, buf, 28 * sizeof(uint16_t));
        rt_thread_mdelay(50);
        /*主动请求迪文屏幕更新本地RTC时间*/
        pw->Dw_Read(pw, DWIN_SYSTEM_READ_RTC_ADDR, 0x04);
        rt_thread_mdelay(50);
    }
    for (;;)
    {
        if (pd && pw && pt->pre)
        {
            uint16_t buf_16[] = {pt->test_result, pt->cartoon.over_current};
            memset(buf, 0x00, sizeof(buf));
            /*数字输出:32路*/
            form_modbus_get_digital_data_to_buf(pd, Coil, &buf[0], 32U);
            /*获取wifi模块状态*/
            buf[2] |= (get_wifi_state() & 0x03) << 6U;
            //            dbg_raw("\r\nwifi_state:%#x.\r\n", buf[2]);
            swap_16bit_data_to_buf(buf_16, sizeof(buf_16), &buf[4]);

            // /*通道判定结果*/
            // buf[5] = pt->test_result;
            // /*过流动画*/
            // buf[7] = pt->cartoon.over_current;
            /*3路模拟输入、1路模拟输出*/
            /*频率*/
            memcpy(&buf[VAL_UINT16_T_NUM * 2U + 4U * sizeof(float)], &pt->ac.wave_param.frequency,
                   sizeof(pt->ac.wave_param.frequency));
            /*设置位电压偏差率、电流偏差率*/
            memcpy(&buf[VAL_UINT16_T_NUM * 2U + 5U * sizeof(float)], &pt->comm_param, sizeof(pt->comm_param));
            /*7组32bit采样数据+实际电压电流偏差*/
            memcpy(&buf[VAL_UINT16_T_NUM * 2U + 7U * sizeof(float)], pt->data.p, pt->data.size * sizeof(test_data_t));
            /*4字节数据交换*/
            for (uint8_t i = 0; i < VAL_FLOAT_NUM; ++i)
                endian_swap(&buf[VAL_UINT16_T_NUM * sizeof(uint16_t) + i * sizeof(float)], 0, sizeof(float));

            pw->Dw_Write(pw, DWIN_DI_OUTPUT_ADDR, buf, sizeof(buf));
        }
        rt_thread_mdelay(1000);
    }
#undef VAL_UINT16_T_NUM
#undef VAL_FLOAT_NUM
#undef BUF_SIE
}

/**
 * @brief  at模块重启
 * @details
 * @param	resp at模块响应帧
 * @param   other 其他信息
 * @retval  None
 */
static void at_restart(void *resp, void *other)
{
    if (RT_NULL == resp || RT_NULL == other)
        return;

    test_t *pt = (test_t *)other;

    rt_kprintf("@note: at module enter transport mode or restart....\r\n");

    pt->at_id = at_noll_id;
    pt->at_state = at_exit_transparent;
}

/**
 * @brief  获取at模块串口参数
 * @details
 * @note    可变参数宏使用：https://www.cnblogs.com/gogly/articles/2416833.html
 * @param	resp at模块响应帧
 * @param   other 其他信息
 * @retval  None
 */
static void at_get_uart_param(void *resp, void *other)
{
    uint32_t baudrate, databits, stopbits;
    char parity[8], control[8];
    if (RT_NULL == resp || RT_NULL == other)
        return;

    test_t *pt = (test_t *)other;

    sscanf((char *)resp, "%*[^=]=%d,%d,%d,%s,%s", &baudrate, &databits,
           &stopbits, parity, control);
    rt_kprintf("baudrate: %d, databits: %d, stopbits: %d, parity: %d, control: %d\r\n",
               baudrate, databits, stopbits, parity, control);

    /*可以连续处理多个at事件*/
    pt->at_id = at_entm;
    // pt->at_state = at_exit_transparent;
}

/**
 * @brief  通过at模块获取网络时间
 * @details
 * @note    sscsnf使用正则表达式：https://www.cnblogs.com/lanjianhappy/p/7171341.html
 * @param	resp at模块响应帧
 * @param   other 其他信息
 * @retval  None
 */
static void at_get_ntp_time(void *resp, void *other)
{
    if (RT_NULL == resp || RT_NULL == other)
        return;

    test_t *pt = (test_t *)other;
    char buf[8];
    struct tm tm_new, *p = &tm_new;

    sscanf((char *)resp, "%*[^=]=%d-%d-%d  %d:%d:%d  %s",
           &p->tm_year, &p->tm_mon, &p->tm_mday, &p->tm_hour, &p->tm_min, &p->tm_sec, buf);
    rt_kprintf("%d-%d-%d  %d:%d:%d  %s\r\n",
               p->tm_year, p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec, buf);

    pt->at_id = at_entm;
}

static at_cmd at_table[] = {
    {.psend = "+++", .precv = "a"},
    {.psend = "a", .precv = "+OK"},
    {.psend = "AT+Z", .event = at_restart},
    {.psend = "AT+ENTM", .event = at_restart},
    {.psend = "AT+UART", .event = at_get_uart_param},
    {.psend = "AT+NTPTM", .event = at_get_ntp_time},
};

/**
 * @brief   根据当前id
 * @details
 * @param	id at事件id
 * @retval  NULL/target cmd handle
 */
static at_cmd_t get_target_at_event(enum at_cmd_id id)
{
    if (id > at_enter_cmd2 && id < at_max_id)
        return &at_table[id];

    return NULL;
}

/**
 * @brief   at进入临时指令线
 * @details
 * @param client 目标客户端句柄
 * @param cmd_table 客户端指令表
 * @retval  操作的结果
 */
static rt_err_t at_enter_transparent_mode(at_client_t client,
                                          at_cmd_t cmd_table)
{
    rt_err_t ret = RT_EOK;
    char at_recv_buf[8U];

    for (at_cmd_t p_cmd = cmd_table; client && cmd_table && (p_cmd < cmd_table + 2U);
         ++p_cmd)
    {
        at_client_obj_send(client, p_cmd->psend, rt_strlen(p_cmd->psend));
        rt_memset(at_recv_buf, 0x00, sizeof(at_recv_buf));
        at_client_obj_recv(client, at_recv_buf, sizeof(at_recv_buf), 500);
        if (RT_NULL == strstr(at_recv_buf, p_cmd->precv))
        {
            rt_kprintf("cur %s, string: %s not found. ^_^\r\n", at_recv_buf, p_cmd->precv);
            ret = RT_ERROR;
            break;
        }
    }

    return ret;
}

/**
 * @brief   at状态机线程
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void at_fsm_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    // at_stop_semaphore = p_rt_thread_pool->semaphore;
    test_t *pt = (test_t *)p_rt_thread_pool->parameter;
    pModbusHandle pd = Modbus_Object;
    rt_base_t level;
    rt_err_t (*modbus_rtu_rx_ind)(rt_device_t dev, rt_size_t size) = RT_NULL;
    rt_err_t ret;
    at_client_t client = RT_NULL;
    // at_response_t resp = RT_NULL;
    char *resp = RT_NULL;

    if (pd && pd->dev)
    {
        at_client_init(pd->dev->parent.name, 128U); // 初始化client
        client = at_client_get(pd->dev->parent.name);
        if (client && client->parser) // rt-thread的一个线程无法挂起另外一个线程，用不到urc数据解析
        {
            resp = client->recv_line_buf;
            rt_thread_delete(client->parser);
        }
    }

    for (;;)
    {
        if (RT_NULL == pd || RT_NULL == pd->dev ||
            RT_NULL == pt || RT_NULL == client) // RT_NULL == at_semaphore ||
            pt->at_state = at_standy;

        switch (pt->at_state)
        {
        case at_enter_transparent:
        {
            /* backup modbus device RX indicate */
            // {
            //     level = rt_hw_interrupt_disable();
            //     modbus_rtu_rx_ind = pd->dev->rx_indicate;
            //     rt_device_set_rx_indicate(pd->dev, at_config_rx_ind);
            //     rt_hw_interrupt_enable(level);
            // }

            /* backup modbus device RX indicate */
            modbus_rtu_rx_ind = pd->dev->rx_indicate;
            if (client)
            {
                extern void set_at_client_rx(at_client_t client);
                set_at_client_rx(client);
                ret = at_enter_transparent_mode(client, at_table);
            }
            if (ret != RT_EOK)
            {
                rt_kprintf("@error: an error occurred while entering command mode. ^_^\r\n");
                // __RESET_FLAG(pt->flag, test_at_flag); // 未收到响应，导致失败
            }
            // else
            //     rt_thread_startup(client->parser);

            // else
            // {
            //     at_client_t client = at_client_get_first();
            //     if (client)
            //     {
            //         extern void set_at_client_rx(at_client_t client);
            //         level = rt_hw_interrupt_disable();
            //         set_at_client_rx(client);
            //         rt_hw_interrupt_enable(level);
            //         rt_thread_startup(client->parser);
            //     }
            // }
            /* 创建响应结构体，设置最大支持响应数据长度为 128字节，响应数据行数无限制，超时时间为 2 秒 */
            // resp = at_create_resp(128, 0, rt_tick_from_millisecond(1000));
            // if (!resp)
            // {
            //     LOG_E("@note: No memory for response structure!");
            // }
            pt->at_state = (RT_EOK == ret) ? at_continue : at_exe_failed;

            if (__GET_FLAG(pt->flag, test_at_flag)) // 进入at cli模式
                pt->at_state = at_cli;
        }
        break;
        case at_exit_transparent:
        {
            /* restore modbus device RX indicate */
            {
                level = rt_hw_interrupt_disable();
                rt_device_set_rx_indicate(pd->dev, modbus_rtu_rx_ind);
                rt_hw_interrupt_enable(level);
            }
            /* 删除服务器响应结构体 */
            // at_delete_resp(resp);
            /*发出暂停at解析线程信号量*/
            // if (at_stop_semaphore)
            //     rt_sem_release(at_stop_semaphore);
            pt->at_state = at_standy;
        }
        break;
        case at_continue:
        {
            at_cmd_t p_at = get_target_at_event(pt->at_id);
            if (RT_NULL == p_at || RT_NULL == p_at->event ||
                RT_NULL == p_at->psend || RT_NULL == resp)
                goto __exit;

            /* 发送数据到服务器，并接收响应数据存放在 resp 结构体中 */
            at_obj_exec_cmd(client, RT_NULL, p_at->psend);
            rt_memset(resp, 0x00, client->recv_bufsz);
            at_client_obj_recv(client, resp, client->recv_bufsz, 500);
            p_at->event(resp, pt);
            // pt->at_state = at_standy;
        }
        break;
        case at_exe_failed:
        case at_complete:
        {
        __exit:
            pt->at_state = at_exit_transparent;
        }
        break;
        case at_standy:
        case at_cli:
        default:
        {
            // if ((at_cli == pt->at_state) && !__GET_FLAG(pt->flag, test_at_flag)) // 退出at cli模式
            //     pt->at_state = at_exit_transparent;
            rt_thread_mdelay(100);
        }
        break;
        }
    }
}

#ifdef FINSH_USING_MSH
#include <finsh.h>

/**
 * @brief   查看系统信息
 * @details
 * @param	None
 * @retval  None
 */
static void see_sys_info(void)
{
#define CURRENT_HARDWARE_VERSION "1.0.0"
#define CURRENT_SOFT_VERSION "1.3.22"
    rt_kprintf("@note:Current Hardware version: %s , software version: %s.\n",
               CURRENT_HARDWARE_VERSION, CURRENT_SOFT_VERSION);
#undef CURRENT_HARDWARE_VERSION
#undef CURRENT_SOFT_VERSION
}
MSH_CMD_EXPORT(see_sys_info, View system information.);

/**
 * @brief  finsh进行控制台还原
 * @param  pd modbus协议站句柄
 * @retval None
 */
void re_console(void)
{
    rt_err_t ret = RT_EOK;
    pModbusHandle pd = Modbus_Object;
    rt_device_t console = pd->old_console; // rt_console_get_device()

    if ((RT_NULL == pd) || (RT_NULL == pd->dev) || (RT_NULL == console))
        return;
    ret = rt_device_close(pd->dev);
    if (RT_EOK == ret)
    {
        finsh_set_device(console->parent.name);
        rt_console_set_device(console->parent.name);
        ret = rt_device_open(pd->dev, RT_DEVICE_FLAG_TX_BLOCKING | RT_DEVICE_FLAG_RX_NON_BLOCKING);
        if (RT_EOK == ret)
            rt_device_set_rx_indicate(pd->dev, modbus_rtu_rx_ind); // 重新设置空当前串口回调函数

        rt_kprintf("@note: enter modbus_rtu mode.\r\n");
    }
}
MSH_CMD_EXPORT(re_console, finsh console restore.);
#endif
