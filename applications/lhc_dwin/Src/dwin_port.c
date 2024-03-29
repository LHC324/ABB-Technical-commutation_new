/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 *  @verbatim
 *      使用： 1、用户需要完善"rt_dwin_init/MX_DwinInit"、"dwin_send"函数
 *             2、用户需要初始化"Dwin_ObjMap[]"迪文事件组
 *             3、"rt_dwin_init/MX_DwinInit"初始化时需要明确指定"UartHandle"参数
 *             4、"dwin_error_handle"函数用户按需编写
 *             5、按需配置"dwin_cfg.h"
 */
#include "dwin_port.h"
#include <sys/time.h>
#include <rtdevice.h>
// #include "flash.h"
#include "test.h"

/*用户函数声明区*/
static void dwin_send(pDwinHandle pd);
static void dwin_error_handle(pDwinHandle pd, dwin_result err_code, uint8_t site, void *pdata);
static void dwin_password_handle(pDwinHandle pd, uint8_t site, uint16_t addr);

static void dwin_data_enrty(pDwinHandle pd, uint8_t site, uint16_t addr);
static void dwin_cur_mode_distinguish(pDwinHandle pd, uint8_t site, uint16_t addr);
static void dwin_start_or_end_test(pDwinHandle pd, uint8_t site, uint16_t addr);
static void dwin_overcurrent_reset(pDwinHandle pd, uint8_t site, uint16_t addr);
static void dwin_set_wifi_module_param(pDwinHandle pd, uint8_t site, uint16_t addr);
static void dwin_rtc_handle(pDwinHandle pd, uint8_t site, uint16_t addr);
static void dwin_data_handle(pDwinHandle pd, uint8_t site, uint16_t addr);
static void dwin_developer_mode(pDwinHandle pd, uint8_t site, uint16_t addr);

/*迪文响应线程*/
Event_Map Dwin_ObjMap[] = {
    /*dds模块参数设置*/
    {.addr = DWIN_SET_USER_FRE_ADDR, .upper = high_freq, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_DEVE_FRE_ADDR, .upper = 12.5e6f, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_FRE_REG_ADDR, .upper = 1.0F, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_PHASE_ADDR, .upper = 360.0F, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_PHASE_REG_ADDR, .upper = 1.0F, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_RANGE_ADDR, .upper = 255.0F, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_DEVE_WAVE_ADDR, .upper = ad9833_squ, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    /*后台参数设置:开始、结束组号；电压、电流偏差率*/
    {.addr = DWIN_SET_START_GROUP_ADDR, .upper = 7.0F, .lower = 1.0F, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_END_GROUP_ADDR, .upper = 7.0F, .lower = 1.0F, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_VOLTAGE_SHIF, .upper = 100.0F, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    {.addr = DWIN_SET_CURRENT_SHIF, .upper = 100.0F, .lower = 0, .event = (_dwin_func)dwin_data_enrty},
    /*按钮响应事件*/
    {.addr = DWIN_OPERATE_SHIFT_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_cur_mode_distinguish},
    {.addr = DWIN_START_TEST_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_start_or_end_test},
    {.addr = DWIN_OVERCURRNRT_RESET_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_overcurrent_reset},
    /*WIFI模块参数设置*/
    {.addr = DWIN_WIFI_SWITCH_ADDR, .upper = 1, .lower = 0, .event = (_dwin_func)dwin_set_wifi_module_param},
    {.addr = DWIN_WIFI_REFACTORY_ADDR, .upper = 1, .lower = 0, .event = (_dwin_func)dwin_set_wifi_module_param},
    {.addr = DWIN_WIFI_WORK_MODE_ADDR, .upper = 1, .lower = 0, .event = (_dwin_func)dwin_set_wifi_module_param},
    {.addr = DWIN_WIFI_SET_BAUD_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_set_wifi_module_param},
    {.addr = DWIN_WIFI_SET_DATA_BIT_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_set_wifi_module_param},
    {.addr = DWIN_WIFI_SET_STOP_BIT_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_set_wifi_module_param},
    {.addr = DWIN_WIFI_SET_CHECK_BIT_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_set_wifi_module_param},
    {.addr = DWIN_WIFI_DATA_EXPORT_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_set_wifi_module_param},
    /*rtc类处理*/
    {.addr = DWIN_SYSTEM_READ_RTC_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_rtc_handle},
    {.addr = DWIN_REQUEST_TIMES_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_rtc_handle},
    {.addr = DWIN_GET_NET_TIMES_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_rtc_handle},
    /*数据操作类处理*/
    {.addr = DWIN_USER_DATA_CLEAN_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_data_handle},
    {.addr = DWIN_SAVE_DATA_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_data_handle},
    {.addr = DWIN_SAVE_PARAM_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_data_handle},
    /*系统登录参数设置*/
    {.addr = DWIN_USER_NAME_ADDR, .upper = 9999, .lower = 0, .event = (_dwin_func)dwin_password_handle},
    {.addr = DWIN_USER_PASSWORD_ADDR, .upper = 9999, .lower = 0, .event = (_dwin_func)dwin_password_handle},
    {.addr = DWIN_LOGIN_OPERATE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_password_handle},
    /*开发者模式*/
    {.addr = DWIN_DEVELOPER_MODE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (_dwin_func)dwin_developer_mode},
};

#define Dwin_EventSize (sizeof(Dwin_ObjMap) / sizeof(Event_Map))

/*定义迪文屏幕对象*/
pDwinHandle Dwin_Object;

/**
 * @brief  通过index得到comm_val_t句柄
 * @param  index 索引值
 * @retval comm_val_t *
 */
comm_val_t *get_comm_val(uint16_t index)
{
#define EX_S &test_object
#define VAL(_x) (EX_S._x)
    /*挂接到公共数据处理的外部变量*/
    static comm_val_t struct_val_table[] = {
        {VAL(flag), co_uint32},
        {VAL(user_freq), co_uint16},
        {VAL(ac.wave_param.frequency), co_float},
        {VAL(ac.wave_param.fre_sfr), co_uint16},
        {VAL(ac.wave_param.phase), co_uint16},
        {VAL(ac.wave_param.phase_sfr), co_uint16},
        {VAL(ac.wave_param.range), co_uint16},
        {VAL(ac.wave_param.wave_mode), co_uint16},
        {VAL(cur_group.start), co_uint16},
        {VAL(cur_group.end), co_uint16},
        {VAL(comm_param.voltage_offset), co_float},
        {VAL(comm_param.current_offset), co_float},
        {VAL(file.cur_size), co_uint32},
    };
#define STRUCT_VAL_NUM() (sizeof(struct_val_table) / sizeof(struct_val_table[0]))

    if (index < STRUCT_VAL_NUM())
        return &struct_val_table[index];
#undef EX_S
    return NULL;
}

#define __INIT_DWIN_VAL(_adr, _in, _si, _ra)                  \
    {                                                         \
        .addr = _adr, .index = _in, .site = _si, .ratio = _ra \
    }

/*定义数据变量绑定表最少是2Byte的整数倍，迪文屏幕地址是16bit*/
static dwin_val_glue_t dwin_val_table[] = {
    /*dds参数*/
    // {.addr = DWIN_SET_USER_FRE_ADDR, .val = &test_object.user_freq, .type = dw_uint16_t, .site = 0x07},
    // {.addr = DWIN_SET_DEVE_FRE_ADDR, .val = &test_object.ac.wave_param.frequency, .type = dw_float, .site = 0x07, .ratio = DWIN_PARAM_OFFSET_SIT},
    // {.addr = DWIN_SET_FRE_REG_ADDR, .val = &test_object.ac.wave_param.fre_sfr, .type = dw_uint16_t, .site = 0x07},
    // {.addr = DWIN_SET_PHASE_ADDR, .val = &test_object.ac.wave_param.phase, .type = dw_uint16_t, .site = 0x07},
    // {.addr = DWIN_SET_PHASE_REG_ADDR, .val = &test_object.ac.wave_param.phase_sfr, .type = dw_uint16_t, .site = 0x07},
    // {.addr = DWIN_SET_RANGE_ADDR, .val = &test_object.ac.wave_param.range, .type = dw_uint16_t, .site = 0x07},
    // {.addr = DWIN_SET_DEVE_WAVE_ADDR, .val = &test_object.ac.wave_param.wave_mode, .type = dw_uint16_t, .site = 0x07},
    // /*后台参数设置:开始、结束组号；电压、电流偏差率*/
    // {.addr = DWIN_SET_START_GROUP_ADDR, .val = &test_object.cur_group.start, .type = dw_uint16_t, .site = 0x07},
    // {.addr = DWIN_SET_END_GROUP_ADDR, .val = &test_object.cur_group.end, .type = dw_uint16_t, .site = 0x07},
    // {.addr = DWIN_SET_VOLTAGE_SHIF, .val = &test_object.comm_param.voltage_offset, .type = dw_float, .site = 0x07, .ratio = DWIN_PARAM_OFFSET_SIT},
    // {.addr = DWIN_SET_CURRENT_SHIF, .val = &test_object.comm_param.current_offset, .type = dw_float, .site = 0x07, .ratio = DWIN_PARAM_OFFSET_SIT},

    /*dds参数*/
    __INIT_DWIN_VAL(DWIN_SET_USER_FRE_ADDR, 0x01, 0x07, 1),
    __INIT_DWIN_VAL(DWIN_SET_DEVE_FRE_ADDR, 0x02, 0x07, DWIN_PARAM_OFFSET_SIT),
    __INIT_DWIN_VAL(DWIN_SET_FRE_REG_ADDR, 0x03, 0x07, 1),
    __INIT_DWIN_VAL(DWIN_SET_PHASE_ADDR, 0x04, 0x07, 1),
    __INIT_DWIN_VAL(DWIN_SET_PHASE_REG_ADDR, 0x05, 0x07, 1),
    __INIT_DWIN_VAL(DWIN_SET_RANGE_ADDR, 0x06, 0x07, 1),
    __INIT_DWIN_VAL(DWIN_SET_DEVE_WAVE_ADDR, 0x07, 0x07, 1),
    /*后台参数设置:开始、结束组号；电压、电流偏差率*/
    __INIT_DWIN_VAL(DWIN_SET_START_GROUP_ADDR, 0x08, 0x07, 1),
    __INIT_DWIN_VAL(DWIN_SET_END_GROUP_ADDR, 0x09, 0x07, 1),
    __INIT_DWIN_VAL(DWIN_SET_VOLTAGE_SHIF, 0x0A, 0x07, DWIN_PARAM_OFFSET_SIT),
    __INIT_DWIN_VAL(DWIN_SET_CURRENT_SHIF, 0x0B, 0x07, DWIN_PARAM_OFFSET_SIT),
};

/**
 * @brief  迪文屏幕初始化
 * @param  None
 * @retval None
 */
#if (DWIN_USING_RTOS == 2)
#if (DWIN_USING_MALLOC)
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;

int rt_dwin_init(void)
{
    UartHandle dwin_uart = {
        .huart = &huart2,
        .phdma = &hdma_usart2_rx,
#if (DWIN_USING_RTOS)
        .semaphore = NULL,
#else
        .recive_finish_flag = false,
#endif
        .tx = {
            .wmode = uart_using_dma,
            .size = DWIN_TX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
        .rx = {
            .wmode = uart_using_dma,
            .size = DWIN_RX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
    };
    DwinHandle temp_dwin = {
        .Id = 0x00,
        .user_val = {
            .ptable = dwin_val_table,
            .num = sizeof(dwin_val_table) / sizeof(dwin_val_table[0]),
        },
        .Slave.pMap = Dwin_ObjMap,
        .Slave.Events_Size = Dwin_EventSize,
        .Uart = dwin_uart,
        // .Slave.pHandle = &measure_storage_object,
        .Slave.pHandle = &test_object,
        .Dw_Delay = (void (*)(uint32_t))rt_thread_mdelay,
        .Dw_Transmit = dwin_send,
        .Dw_Error = dwin_error_handle,
    };

    Create_DwinObject(&Dwin_Object, &temp_dwin);
    return 0;
}
/*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
// INIT_COMPONENT_EXPORT(rt_dwin_init);
// INIT_ENV_EXPORT(rt_dwin_init);
INIT_DEVICE_EXPORT(rt_dwin_init);
#else
void rt_dwin_init(void)
{
    Init_Dwin_Object(0);
    pDwinHandle pd = Get_Dwin_Object(0);
    DwinHandle temp_dwin = {
        /*User init info*/
    };
    Create_DwinObject(&pd, &temp_dwin);
}
#endif

#else
void MX_DwinInit(void)
{
}
#endif

/**
 * @brief  迪文屏幕获取目标变量地址
 * @param  pd 迪文屏幕句柄
 * @param  addr 目标变量地址
 * @retval 目标地址信息
 */
static dwin_val_glue_t *dwin_get_target_val_addr(pDwinHandle pd, uint16_t addr)
{
    dwin_val_glue_t *ptarget = NULL;

    if (NULL == pd || NULL == pd->user_val.ptable)
        goto __exit;

    for (dwin_val_glue_t *p = pd->user_val.ptable;
         p < pd->user_val.ptable + pd->user_val.num; ++p)
    {
        if (addr == p->addr)
            ptarget = p;
    }

__exit:
    return ptarget;
}

/**
 * @brief  带CRC的发送数据帧
 * @param  pd 迪文屏幕句柄
 * @retval None
 */
static void dwin_send(pDwinHandle pd)
{
#if (DWIN_USING_CRC == 1U)
    uint16_t crc16 = get_crc16(&dwin_tx_buf[3U], dwin_tx_count(pd) - 3U, 0xffff);

    memcpy(&dwin_tx_buf[dwin_tx_count(pd)], (uint8_t *)&crc16, sizeof(crc16));
    dwin_tx_count(pd) += sizeof(crc16);
#endif

    switch (pd->Uart.tx.wmode)
    {
    case uart_using_it:
    {
        HAL_UART_Transmit((UART_HandleTypeDef *)pd->Uart.huart, dwin_tx_buf, dwin_tx_count(pd), 0xffff);
    }
    break;
#if (DWIN_USING_DMA)
    case uart_using_dma:
    {
        HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pd->Uart.huart, dwin_tx_buf, dwin_tx_count(pd));
        /*https://blog.csdn.net/mickey35/article/details/80186124*/
        /*https://blog.csdn.net/qq_40452910/article/details/80022619*/
        while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pd->Uart.huart, UART_FLAG_TC) == RESET)
        {
            if (pd->Dw_Delay)
                pd->Dw_Delay(1);
        }
    }
    break;
#endif
    default:
        break;
    }
}

/**
 * @brief  迪文屏幕错误处理
 * @param  pd 迪文屏幕对象句柄
 * @param  err_code 错误代码
 * @param  site 当前对象出错位置
 * @param  pdata 当前出错数据
 * @retval None
 */
static void dwin_error_handle(pDwinHandle pd,
                              dwin_result err_code,
                              uint8_t site,
                              void *pdata)
{
    // #define ERROR_NOTE_PAGE 30U
    //  TYPEDEF_STRUCT tdata = (error_code == BELOW_LOWER_LIMIT) ? pd->Slave.pMap[site].lower : pd->Slave.pMap[site].upper;
    //  uint16_t tarry[] = {0, 0, 0};
    // #if (DWIN_USING_DEBUG)
    //  if (error_code == BELOW_LOWER_LIMIT)
    //  {
    //      DWIN_DEBUG(Shell_Object, "Error: Below lower limit %.3f.\r\n", tdata);
    //  }
    //  else
    //  {
    //      tarry[0] = 0x0100;
    //      DWIN_DEBUG(Shell_Object, "Error: Above upper limit %.3f.\r\n", tdata);
    //  }
    // #endif
    //  Endian_Swap((uint8_t *)&tdata, 0U, sizeof(TYPEDEF_STRUCT));
    //  /*设置错误时将显示上下限*/
    //  pd->Dw_Write(pd, pd->Slave.pMap[site].addr, (uint8_t *)&tdata, sizeof(TYPEDEF_STRUCT));
    //  pd->Dw_Delay(NEXT_DELAT_TIMES);
    //  /*切换到提示页面*/
    //  pd->Dw_Page(pd, ERROR_NOTE_PAGE);
    //  pd->Dw_Delay(NEXT_DELAT_TIMES);
    //  memcpy(&tarry[1], (void *)&tdata, sizeof(tdata));
    //  pd->Dw_Write(pd, NOTE_PAGE_ADDR, (uint8_t *)&tarry, sizeof(tarry));
}

/**
 * @brief  迪文屏幕设置目标类型数据
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
dwin_data_t dwin_set_target_data(void *const psource,
                                 void *const pdest,
                                 const comm_data_type s_type,
                                 const comm_data_type d_type)
{
    dwin_data_t data = {
        .data = 0,
        .size = 0,
        .result = dwin_ok,
    };

    if (NULL == psource || NULL == pdest)
    {
        data.result = err_other;
        goto __exit;
    }

    switch (s_type)
    {
    case co_uint16:
    {
        switch (d_type)
        {
        case co_uint16:
            *(uint16_t *)pdest = *(uint16_t *)psource;
            break;
        case co_float:
            *(float *)pdest = *(uint16_t *)psource;
            // data.data.f32 = *(float *)pdest;
            // data.size = sizeof(float);
            break;
        default:
            break;
        }
        data.data.us16 = *(uint16_t *)psource;
        data.size = sizeof(uint16_t);
    }
    break;
    case co_float:
    {
        switch (d_type)
        {
        case co_uint16:
            *(uint16_t *)pdest = *(float *)psource;
            data.data.us16 = *(uint16_t *)pdest;
            data.size = sizeof(uint16_t);
            break;
        case co_float:
            *(float *)pdest = *(float *)psource;
            data.data.f32 = *(float *)pdest;
            data.size = sizeof(float);
            break;
        default:
            break;
        }
    }
    break;
    default:
        break;
    }
__exit:
    return data;
}

/**
 * @brief  迪文屏幕检测变量数据范围
 * @param  pd 迪文屏幕对象句柄
 * @param  paddr 目标地址
 * @param  type 目标数据类型
 * @param  site 记录当前Map中位置
 * @retval None
 */
dwin_data_t dwin_check_scope(pDwinHandle pd,
                             void *const pdata,
                             comm_data_type type,
                             uint8_t site)
{
    dwin_data_t data = {
        .data = 0,
        .size = 0,
        .result = dwin_ok,
    };
    float cur_val = 0;

    /*数据类型错误*/
    if (type >= co_type_max)
    {
        data.result = err_data_type;
        goto __exit;
    }

    data = dwin_set_target_data(pdata, &cur_val, type, co_float);

    if (cur_val < pd->Slave.pMap[site].lower)
    {
        data = dwin_set_target_data(&pd->Slave.pMap[site].lower, pdata, co_float, type);
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("Error: [%.2f]Below lower limit %.2f.\r\n",
                   cur_val, pd->Slave.pMap[site].lower);
#endif
        data.result = err_min_lower_limit;
    }

    if (cur_val > pd->Slave.pMap[site].upper)
    {
        data = dwin_set_target_data(&pd->Slave.pMap[site].upper, pdata, co_float, type);
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("Error: [%.2f]Above upper limit %.2f.\r\n",
                   cur_val, pd->Slave.pMap[site].upper);
#endif
        data.result = err_max_upper_limit;
    }

__exit:
    return data;
}

/**
 * @brief  迪文屏幕数据类型解析器
 * @param  pd 迪文屏幕句柄
 * @param  pt 数据信息
 * @param  len 数据长度
 * @retval 数据信息
 */
static dwin_result dwin_data_type_parser(dwin_val_glue_t *pt,
                                         uint8_t *pdata,
                                         uint8_t len)
{
    dwin_result result = dwin_ok;
    float ratio = 0;
    void *ptr = NULL;
    comm_val_t *pc = get_comm_val(pt->index);
    // extern const char *text_name[co_type_max + 1U];

    if (NULL == pt || NULL == pdata || NULL == pc)
        return err_other;

    const char *p_name = text_name[pc->type];
    uint8_t c_size = BY_DATA_TYPE_GET_SIZE(pc->type);

    /*仅支持一位小数*/
    ratio = !pt->ratio
                ? 1.0F
            : pt->ratio > DWIN_PARAM_OFFSET_SIT ? DWIN_PARAM_OFFSET_SIT
                                                : pt->ratio;

    switch (pc->type)
    {
    case co_uint8: // case中定义变量：https://blog.csdn.net/scutth/article/details/6894975
    case co_int8:
    case co_uint16:
    case co_int16:
    case co_int32:
    case co_long:
    {
        int data = (int)Get_Dwin_Data(pdata, 0, len) / (int)ratio;
        ptr = &data;
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("note: [%s] data: %d.\r\n", p_name, data);
#endif
    }
    break;
    case co_float:
    {
        float data = (float)Get_Dwin_Data(pdata, 0, len) / ratio;
        ptr = &data;
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("note: [%s] data: %f.\r\n", p_name, data);
#endif
    }
    break;
    default:
        break;
    }
    if (ptr && c_size < co_type_max)
        memcpy(pc->val, ptr, c_size);

    return result;
}

/**
 * @brief  密码处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_password_handle(pDwinHandle pd,
                                 uint8_t site,
                                 uint16_t addr)
{
#define USER_NAMES 0x07E6
#define USER_PASSWORD 0x0522
#define TARGET_PAGE_NUMBER 0x01
    uint16_t data = Get_Dwin_Data(dwin_rx_buf, 7U, dwin_rx_buf[6U]);
    static uint16_t user_name = 0x0000, user_code = 0x0000, error = 0x0000;
    uint8_t jump_flag = 0;
    uint16_t default_name = USER_NAMES, defalut_code = USER_PASSWORD;
    // dwin_result dwin_code = dwin_check_scope(pd, (float *)&data, site);

    // /*数据超限*/
    // if (dwin_code != dwin_ok)
    //     goto __exit;
    /*输入阶段*/
    jump_flag = (addr == DWIN_USER_NAME_ADDR) ? user_name = data, 0x01U
                                              : (addr == DWIN_USER_PASSWORD_ADDR
                                                 ? user_code = data,
           0x01U : 0x00U);

    if (jump_flag)
        goto __exit;
    /*登录识别地址错误*/
    if (addr != DWIN_LOGIN_OPERATE_ADDR)
        goto __exit;
    /*登录*/
    if (data == DWIN_SURE_CODE)
    {
        /*密码用户名正确*/
        if ((user_name == default_name) && (user_code == defalut_code))
        { /*清除错误信息*/
            error = 0x0000;
            pd->Dw_Page(pd, TARGET_PAGE_NUMBER);
#if defined(DWIN_USING_DEBUG)
            DWIN_DEBUG("success: The password is correct!\r\n");
#endif
        }
        else
        {
            /*用户名、密码错误*/
            if ((user_name != default_name) && (user_code != defalut_code))
            {
                error = 0x0300;
#if defined(DWIN_USING_DEBUG)
                DWIN_DEBUG("error: Wrong user name and password!\r\n");
#endif
            }
            /*用户名错误*/
            else if (user_name != default_name)
            {
                error = 0x0100;
#if defined(DWIN_USING_DEBUG)
                DWIN_DEBUG("error: User name error!\r\n");
#endif
            }
            /*密码错误*/
            else
            {
                error = 0x0200;

#if defined(DWIN_USING_DEBUG)
                DWIN_DEBUG("error: User password error!\r\n");
#endif
            }
        }
    }
    else /*注销*/
    {
        error = 0x0000;
        user_name = user_code = 0x0000;
        uint32_t temp_value = 0x0000;
        pd->Dw_Write(pd, DWIN_USER_NAME_ADDR, (uint8_t *)&temp_value, sizeof(temp_value));
#if defined(DWIN_USING_DEBUG)
        DWIN_DEBUG("success: Clear Error Icon!\r\n");
#endif
    }
    pd->Dw_Write(pd, DWIN_ERROR_NOTE_ADDR, (uint8_t *)&error, sizeof(error));

__exit:
#if defined(DWIN_USING_DEBUG)
    DWIN_DEBUG("site:%#x,addr:%#x,data:%d,user_name = %d, user_code = %d.\r\n",
               site, addr, data, user_name, user_code);
#endif
#undef USER_NAME
#undef USER_PASSWORD
#undef TARGET_PAGE_NUMBER
}

/**
 * @brief  迪文屏幕数据存储到系统区
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_save_param(pDwinHandle pd,
                            comm_val_t *pv,
                            uint16_t index)
{
    if (NULL == pd || NULL == pv)
        return;

    extern void set_system_param(comm_val_t * pv, uint16_t index);
    set_system_param(pv, index);
}

/**
 * @brief  迪文屏幕数据录入
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_data_enrty(pDwinHandle pd,
                            uint8_t site,
                            uint16_t addr)
{
    dwin_val_glue_t *pt = dwin_get_target_val_addr(pd, addr);
    comm_val_t *pv = get_comm_val(pt->index);
    dwin_result dwin_code = dwin_ok;
    dwin_data_t data = {0};
    uint8_t *psite = NULL;

    if (NULL == pd || NULL == pt || NULL == pv)
        return;
    uint8_t *pdata = &dwin_rx_buf[pt->site], len = pd->Uart.rx.pbuf[6U];

    dwin_code = dwin_data_type_parser(pt, pdata, len);
    /*数据类型错误*/
    if (dwin_code != dwin_ok)
    {
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@error:Non-existent data type.\r\n");
#endif
        goto __exit;
    }

    // float data = (float)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]) / DWIN_PARAM_OFFSET_SIT;
    // static float temp_data = 0;
    // temp_data = *(uint16_t *)pt->val;
    data = dwin_check_scope(pd, pv->val, pv->type, site);

    /*数据超限*/
    // if (dwin_code != dwin_ok)
    // {
    //     goto __exit;
    // }

    dwin_save_param(pd, pv, pt->index); // 参数存储

    if (pv->type > co_int8)
    {
        if (pv->type < co_uint32)
        {
            psite = (uint8_t *)&data.data.us16;
        }
        else
            psite = (uint8_t *)&data.data.ui32;
        if (psite)
            endian_swap(psite, 0, data.size);
    }
    // else
    //     psite = (uint8_t *)&data.data.uc8;

__exit:
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,addr:%#x,exe'dwin_data_enrty'.\r\n",
               site, addr);
#endif
    // dwin_data_echo(pd, addr, (uint8_t *)&data, c_size);
    /*确认数据回传到屏幕:*/
    if (psite)
        pd->Dw_Write(pd, addr, psite, data.size);
}

/**
 * @brief  模式区分地址
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_cur_mode_distinguish(pDwinHandle pd,
                                      uint8_t site,
                                      uint16_t addr)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptest_t pt = (ptest_t)pd->Slave.pHandle;
    if (NULL == pt)
        return;
    if (data == DWIN_SURE_CODE)
    {
        __SET_FLAG(pt->flag, test_mode);
    }
    if (data == DWIN_CANCEL_CODE)
    {
        __RESET_FLAG(pt->flag, test_mode);
    }
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,addr:%#x,key_code:%#x,test_mode:%#x, exe'dwin_cur_mode_distinguish'.\r\n",
               site, addr, data, __GET_FLAG(pt->flag, test_mode));
#endif
}

/**
 * @brief  测试系统清除检验结果
 * @param  pt 测试系统对象句柄
 * @retval None
 */
static void dwin_clear_test_result(ptest_t pt)
{
    if (NULL == pt)
        return;
    for (uint8_t i = 0; i < TEST_MAX_NUM; i++)
        __RESET_FLAG(pt->test_result, i);
}

/**
 * @brief  开始/结束检测处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_start_or_end_test(pDwinHandle pd,
                                   uint8_t site,
                                   uint16_t addr)
{
    ptest_t pt = (ptest_t)pd->Slave.pHandle;
    if (NULL == pt)
        return;

    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    // dwin_result dwin_code = dwin_check_scope(pd, (float *)&data, site);

    // /*数据超限*/
    // if (dwin_code != dwin_ok)
    //     goto __exit;
    /*变量地址错误*/
    if (addr != DWIN_START_TEST_ADDR)
        goto __exit;

    switch (data)
    {
    case DWIN_SURE_CODE:
        if (__GET_FLAG(pt->flag, test_start_signal)) // 如果已经开始，将无法再次开始
            goto __exit;

        __SET_FLAG(pt->flag, test_start_signal);
        /*复位上一次检测结果*/
        dwin_clear_test_result(pt);
        break;
    case DWIN_CANCEL_CODE:
        __RESET_FLAG(pt->flag, test_start_signal);
        break;
    // case 0xF2:
    //     pt->comm_param.voltage_offset = data;
    //     break;
    // case 0xF3:
    //     pt->comm_param.current_offset = data;
    //     break;
    default:
        break;
    }
__exit:
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,addr:%#x,data:%#x,exe'dwin_start_or_end_test'.\r\n",
               site, addr, data);
#endif
}

/**
 * @brief  过流复位
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_overcurrent_reset(pDwinHandle pd,
                                   uint8_t site,
                                   uint16_t addr)
{
    ptest_t pt = (ptest_t)pd->Slave.pHandle;
    if (NULL == pt)
        return;

    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);

    if (data != DWIN_SURE_CODE)
        goto __exit;

    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // 清除硬件标志
    __RESET_FLAG(pt->flag, test_overcurrent_signal); // 并复位标志位
    // pt->cartoon.over_current = 0;                         // 清除过流动画图标

__exit:
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,addr:%#x,data:%#x,exe'dwin_overcurrent_reset'.\r\n",
               site, addr, data);
#endif
}

/**
 * @brief  设置WiFi模块参数
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_set_wifi_module_param(pDwinHandle pd,
                                       uint8_t site,
                                       uint16_t addr)
{
    ptest_t pt = (ptest_t)pd->Slave.pHandle;
    if (NULL == pt)
        return;

    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    // dwin_result dwin_code = dwin_check_scope(pd, (float *)&data, site);

    // /*数据超限*/
    // if (dwin_code != dwin_ok)
    //     goto __exit;

    switch (addr)
    {
    case DWIN_WIFI_SWITCH_ADDR: // wifi模块使能信号
        if (data == 0)
            HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port, WIFI_RESET_Pin, GPIO_PIN_SET);
        if (data == 1)
            HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port, WIFI_RESET_Pin, GPIO_PIN_RESET);
        break;
    case DWIN_WIFI_REFACTORY_ADDR: // wifi模块恢复出厂设置
        if (data == DWIN_SURE_CODE)
        {
            // HAL_GPIO_WritePin(WIFI_RELOAD_GPIO_Port, WIFI_RELOAD_Pin, GPIO_PIN_RESET);
            // if (pd->Dw_Delay)
            //     pd->Dw_Delay(4000);
            // HAL_GPIO_WritePin(WIFI_RELOAD_GPIO_Port, WIFI_RELOAD_Pin, GPIO_PIN_SET);
        }
        break;
    case DWIN_WIFI_WORK_MODE_ADDR: // 设置wifi模块工作模式

        break;
    case DWIN_WIFI_SET_BAUD_ADDR: // 设置wifi模块串口波特率

        break;
    case DWIN_WIFI_SET_DATA_BIT_ADDR: // 设置wifi模块串口数据位

        break;
    case DWIN_WIFI_SET_STOP_BIT_ADDR: // 设置wifi模块串口停止位

        break;
    case DWIN_WIFI_SET_CHECK_BIT_ADDR: // 设置wifi模块串口校验位

        break;
    case DWIN_WIFI_DATA_EXPORT_ADDR: // WiFi导出数据

        break;
    default:
        break;
    }

//__exit:
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,addr:%#x,data:%#x,exe'dwin_set_wifi_module_param'.\r\n",
               site, addr, data);
#endif
}

/**
 * @brief  触发at指令执行器执行目标指令
 * @note   带busy检测，busy状态失败
 * @param  pt 检测系统句柄
 * @param  id 目标at时间id
 * @retval None
 */
bool at_start_exe_cmd(ptest_t pt, enum at_cmd_id id)
{
    // 确保at执行器空闲模式才触发
    if (pt && id < at_max_id && (at_standy == pt->at_state))
    {
        pt->at_id = id;
        pt->at_state = at_enter_transparent;
        return true;
    }
    return false;
}

/**
 * @brief  时间更新操作
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_rtc_handle(pDwinHandle pd,
                            uint8_t site,
                            uint16_t addr)
{
    /* set time and date */
    time_t now = {0};
    struct timeval tv = {0};
    struct timezone tz = {0};

    struct tm tm_new = {
        .tm_year = (int)(pd->Uart.rx.pbuf[7] + 2000) - 1900,
        .tm_mon = pd->Uart.rx.pbuf[8] - 1,
        .tm_mday = pd->Uart.rx.pbuf[9],
        .tm_wday = pd->Uart.rx.pbuf[10],
        .tm_hour = pd->Uart.rx.pbuf[11],
        .tm_min = pd->Uart.rx.pbuf[12],
        .tm_sec = pd->Uart.rx.pbuf[13],
    };
    ptest_t pt = (ptest_t)pd->Slave.pHandle;
    if (NULL == pt)
        return;

    switch (addr)
    {
    case DWIN_SYSTEM_READ_RTC_ADDR:
    case DWIN_REQUEST_TIMES_ADDR: // dwin屏幕rtc下发
    {
        /* converts the local time into the calendar time. */
        now = mktime(&tm_new);
        rt_err_t err = set_timestamp(now);
        if (err != RT_EOK)
        {
            rt_kprintf("set date failed. %d\n", err);
            return;
        }
        gettimeofday(&tv, &tz);
        /* output current time */
        rt_kprintf("local time: %.*s", 25, ctime(&now));
        rt_kprintf("timestamps: %ld\n", (long)tv.tv_sec);
        rt_kprintf("timezone: UTC%c%d\n", -tz.tz_minuteswest > 0 ? '+' : '-', -tz.tz_minuteswest / 60);
    }
    break;
    case DWIN_GET_NET_TIMES_ADDR: // 获取网络时间
    {
        at_start_exe_cmd(pt, at_ntp_time);
    }
    break;
    default:
        break;
    }

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,addr:%#x,exe'dwin_rtc_handle'.\r\n",
               site, addr);
#endif
}

/**
 * @brief  数据类操作
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_data_handle(pDwinHandle pd,
                             uint8_t site,
                             uint16_t addr)
{
    ptest_t pt = (ptest_t)pd->Slave.pHandle;
    if (NULL == pt)
        return;

    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    // dwin_result dwin_code = dwin_check_scope(pd, (float *)&data, site);

    // /*数据超限*/
    // if (dwin_code != dwin_ok)
    //     goto __exit;

    switch (addr)
    {
    case DWIN_USER_DATA_CLEAN_ADDR: // 数据清除
    {
        /*清空前台检测数据(测量过程中不允许清除)*/
        if (!__GET_FLAG(pt->flag, test_start_signal) && pt->data.p)
        {
            memset(pt->data.p, 0x00, pt->data.size * sizeof(test_data_t));
            /*复位上一次检测结果*/
            dwin_clear_test_result(pt);
        }
    }
    break;
    case DWIN_SAVE_DATA_ADDR: // 保存数据
    {
    }
    break;
    case DWIN_SAVE_PARAM_ADDR: // 保存参数
    {
    }
    break;
    default:
        break;
    }
//__exit:
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,addr:%#x,data:%#x,exe'dwin_data_handle'.\r\n",
               site, addr, data);
#endif
}

/**
 * @brief  迪文屏进入开发者模式
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @param  addr 变量地址
 * @retval None
 */
static void dwin_developer_mode(pDwinHandle pd,
                                uint8_t site,
                                uint16_t addr)
{
#define DEVELOPER_PAGE 44
#define ENTRY_TIMEOUT 5U
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptest_t pt = (ptest_t)pd->Slave.pHandle;
    static uint8_t count = 0;
    static uint8_t step_table[] = {0xF0, 0xF1, 0xF2, 0xF3};

    if (NULL == pt || (tim_id_invalid > TEST_SOFT_TIMER_NUM))
        return;

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,addr:%#x,exe'dwin_developer_mode',key_code:%#x,count:%#x.\r\n",
               site, addr, data, count);
#endif

    // test_timer_t *ptimer = &pt->timer[tim_id_invalid];
    test_timer_t *ptimer = get_soft_timer_handle(pt->timer, tim_id_invalid);

    if (ptimer->flag)
    {
        ptimer->flag = false;
        count = 0;
    }

    /*刷新超时定时器*/
    ptimer->count = ENTRY_TIMEOUT;
    if (step_table[count] == data)
    {
        count++;
    }
    else
    {
        count = 0;
        return;
    }

    if (data == step_table[sizeof(step_table) - 1U])
    {
        ptimer->flag = false;
        count = 0;
        /*切换到开发者页面*/
        pd->Dw_Page(pd, DEVELOPER_PAGE);
    }

#undef DEVOLOPER_PAGE
}

/**
 * @brief  设置波形类型
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
// static void Dwin_Set_Wave_Type(pDwinHandle pd, uint8_t site)
// {
//     uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
//     ptest_t pt = (ptest_t)pd->Slave.pHandle;
//     if (pt == NULL)
//         return;

//     if (data < ad9833_null_signal)
//     {
//         pt->ac.wave_param.wave_mode = (ad9833_wave)data; //(ad9833_wave)(data - 1U)
//     }

// #if (DWIN_USING_DEBUG)
//     DWIN_DEBUG("@note:key_code:%#x,exe'Dwin_Set_Wave_Type'.\r\n", data);
// #endif
// }

/**
 * @brief  设置电压类型
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
// static void Dwin_Set_Voltage_Mode(pDwinHandle pd, uint8_t site)
// {
//     uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
//     ptest_t pt = (ptest_t)pd->Slave.pHandle;
//     if (pt == NULL)
//         return;

//     if (data < null_out)
//         pt->pre->cur_power = (relay_power_type)data;

// #if (DWIN_USING_DEBUG)
//     DWIN_DEBUG("@note:key_code:%#x,exe'Dwin_Set_Voltage_Mode'.\r\n", data);
// #endif
// }

/**
 * @brief  设置继电器策略
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
// static void Dwin_Set_Relay_Tactics(pDwinHandle pd, uint8_t site)
// {
//     uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
//     ptest_t pt = (ptest_t)pd->Slave.pHandle;
//     if (pt == NULL)
//         return;

//     if (data < custom_action)
//         pt->cur_tactics = (relay_tactics_type)data;

// #if (DWIN_USING_DEBUG)
//     DWIN_DEBUG("@note:key_code:%#x,exe'Dwin_Set_Relay_Tactics'.\r\n", data);
// #endif
// }
