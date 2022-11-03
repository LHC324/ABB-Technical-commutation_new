/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 *  @verbatim
 *      使用： 1、用户需要完善"rt_dwin_init/MX_DwinInit"、"Dwin_Send"函数
 *             2、用户需要初始化"Dwin_ObjMap[]"迪文事件组
 *             3、"rt_dwin_init/MX_DwinInit"初始化时需要明确指定"UartHandle"参数
 *             4、"Dwin_ErrorHandle"函数用户按需编写
 *             5、按需配置"dwin_cfg.h"
 */
#include "dwin_port.h"
//#include "measure.h"
//#include "flash.h"
#include "test.h"

/*用户函数声明区*/
static void Dwin_Send(pDwinHandle pd);
static void Dwin_ErrorHandle(pDwinHandle pd, dwin_result err_code, uint8_t site);
static void Password_Handle(pDwinHandle pd, uint8_t site);

static void Dwin_Set_Back_Param(pDwinHandle pd, uint8_t site);
static void Dwin_Cur_Mode_Distinguish(pDwinHandle pd, uint8_t site);
static void Dwin_Auto_Test(pDwinHandle pd, uint8_t site);
static void Dwin_End_Test(pDwinHandle pd, uint8_t site);
static void Dwin_Save_Data(pDwinHandle pd, uint8_t site);
static void Dwin_Save_Param(pDwinHandle pd, uint8_t site);
static void Dwin_Set_Wave_Type(pDwinHandle pd, uint8_t site);
static void Dwin_Set_Voltage_Mode(pDwinHandle pd, uint8_t site);
static void Dwin_Set_Relay_Tactics(pDwinHandle pd, uint8_t site);

typedef void (*dwin_struct__)(void *, uint8_t);
/*迪文响应线程*/
Event_Map Dwin_ObjMap[] = {
    /*后台参数设置*/
    {.addr = DWIN_SET_FRE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Set_Back_Param},
    {.addr = DWIN_SET_FRE_REG_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Set_Back_Param},
    {.addr = DWIN_SET_PHASE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Set_Back_Param},
    {.addr = DWIN_SET_PHASE_REG_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Set_Back_Param},
    {.addr = DWIN_SET_RANGE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Set_Back_Param},
    {.addr = DWIN_SET_SEGMENT_ADDDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Set_Back_Param},
    {.addr = DWIN_SET_VOLTAGE_SHIF, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Set_Back_Param},
    {.addr = DWIN_SET_CURRENT_SHIF, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Set_Back_Param},
    /*系统登录参数设置*/
    {.addr = DWIN_USER_NAME_ADDR, .upper = 9999, .lower = 0, .event = (dwin_struct__)Password_Handle},
    {.addr = DWIN_USER_PASSWORD_ADDR, .upper = 9999, .lower = 0, .event = (dwin_struct__)Password_Handle},
    {.addr = DWIN_LOGIN_SURE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Password_Handle},
    {.addr = DWIN_CANCEL_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Password_Handle},
    /*按钮响应事件*/
    {.addr = DWIN_OPERATE_SHIFT_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Cur_Mode_Distinguish},
    {.addr = DWIN_END_TEST_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_End_Test},
    {.addr = DWIN_SAVE_DATA_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Save_Data},
    {.addr = DWIN_SAVE_PARAM_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Save_Param},
    {.addr = DWIN_AUTO_TEST_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Auto_Test},
    /*图标变量*/
    {.addr = DWIN_WAVE_MODE_ADDR, .upper = 2.0F, .lower = 0, .event = (dwin_struct__)Dwin_Set_Wave_Type},
    {.addr = DWIN_VOLTAGE_MODE_ADDR, .upper = 1.0F, .lower = 0, .event = (dwin_struct__)Dwin_Set_Voltage_Mode},
    {.addr = DWIN_RELAY_TACTICS_ADDR, .upper = 1.0F, .lower = 0, .event = (dwin_struct__)Dwin_Set_Relay_Tactics},
};

#define Dwin_EventSize (sizeof(Dwin_ObjMap) / sizeof(Event_Map))

/*定义迪文屏幕对象*/
pDwinHandle Dwin_Object;

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
            .size = DWIN_TX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
        .rx = {
            .size = DWIN_RX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
    };
    DwinHandle temp_dwin = {
        .Id = 0x00,
        .Slave.pMap = Dwin_ObjMap,
        .Slave.Events_Size = Dwin_EventSize,
        .Uart = dwin_uart,
        // .Slave.pHandle = &measure_storage_object,
        .Slave.pHandle = &test_object,
        .Dw_Delay = (void (*)(uint32_t))rt_thread_mdelay,
        .Dw_Transmit = Dwin_Send,
        .Dw_Error = Dwin_ErrorHandle,
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
 * @brief  带CRC的发送数据帧
 * @param  pd 迪文屏幕句柄
 * @retval None
 */
static void Dwin_Send(pDwinHandle pd)
{
#if (DWIN_USING_CRC == 1U)
    uint16_t crc16 = get_crc16(&dwin_tx_buf[3U], dwin_tx_count(pd) - 3U, 0xffff);

    memcpy(&dwin_tx_buf[dwin_tx_count(pd)], (uint8_t *)&crc16, sizeof(crc16));
    dwin_tx_count(pd) += sizeof(crc16);
#endif

#if (DWIN_USING_DMA)
    HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pd->Uart.huart, dwin_tx_buf, dwin_tx_count(pd));
    /*https://blog.csdn.net/mickey35/article/details/80186124*/
    /*https://blog.csdn.net/qq_40452910/article/details/80022619*/
    while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pd->Uart.huart, UART_FLAG_TC) == RESET)
    {
        if (pd->Dw_Delay)
            pd->Dw_Delay(1);
    }
#else
    HAL_UART_Transmit((UART_HandleTypeDef *)pd->Uart.huart, dwin_tx_buf, dwin_tx_count, 0xffff);
#endif
}

/**
 * @brief  迪文屏幕错误处理
 * @param  pd 迪文屏幕对象句柄
 * @param  err_code 错误代码
 * @param  site 当前对象出错位置
 * @retval None
 */
static void Dwin_ErrorHandle(pDwinHandle pd, dwin_result err_code, uint8_t site)
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
 * @brief  密码处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Password_Handle(pDwinHandle pd, uint8_t site)
{
#define USER_NAMES 0x07E6
#define USER_PASSWORD 0x0522
#define PAGE_NUMBER 0x01
    uint16_t data = Get_Dwin_Data(dwin_rx_buf, 7U, dwin_rx_buf[6U]);
    uint16_t addr = pd->Slave.pMap[site].addr;
    static uint16_t user_name = 0x0000, user_code = 0x0000, error = 0x0000;
    // Save_HandleTypeDef *ps = &Save_Flash;
    uint16_t default_name = USER_NAMES, defalut_code = USER_PASSWORD;

    if ((data >= pd->Slave.pMap[site].lower) && (data <= pd->Slave.pMap[site].upper))
    {
        addr == DWIN_USER_NAME_ADDR
            ? user_name = data
            : (addr == DWIN_USER_PASSWORD_ADDR
                   ? user_code = data
                   : 0U);

        if ((addr == DWIN_LOGIN_SURE_ADDR) && (data == DWIN_SURE_CODE))
        { /*密码用户名正确*/
            if ((user_name == default_name) && (user_code == defalut_code))
            { /*清除错误信息*/
                error = 0x0000;
                pd->Dw_Page(pd, PAGE_NUMBER);
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
        if ((addr == DWIN_CANCEL_ADDR) && (data == DWIN_CANCEL_CODE))
        {
            error = 0x0000;
            user_name = user_code = 0x0000;
            uint32_t temp_value = 0x0000;
            pd->Dw_Write(pd, DWIN_USER_NAME_ADDR, (uint8_t *)&temp_value, sizeof(temp_value));
            // pd->Dw_Write(pd, DWIN_ERROR_NOTE_ADDR, (uint8_t *)&error, sizeof(error));
#if defined(DWIN_USING_DEBUG)
            DWIN_DEBUG("success: Clear Error Icon!\r\n");
#endif
        }
        pd->Dw_Write(pd, DWIN_ERROR_NOTE_ADDR, (uint8_t *)&error, sizeof(error));
    }

#if defined(DWIN_USING_DEBUG)
    DWIN_DEBUG("data = %d,user_name = %d, user_code = %d.\r\n", data, user_name, user_code);
#endif
}

/**
 * @brief  迪文屏幕设置后台参数
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Set_Back_Param(pDwinHandle pd, uint8_t site)
{
#define DWIN_PARAM_OFFSET_SIT 10U
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;
    float data = (float)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]) / DWIN_PARAM_OFFSET_SIT;
    // uint8_t base_addr = site - DWIN_SET_FRE_ADDR;
    switch (site)
    {
    case 0:
        pt->ac.wave_param.frequency = data;
        break;
    case 1:
        pt->ac.wave_param.fre_sfr = data;
        break;
    case 2:
        pt->ac.wave_param.phase = data;
        break;
    case 3:
        pt->ac.wave_param.phase_sfr = data;
        break;
    case 4:
        pt->ac.wave_param.range = data;
        break;
    case 5:
        pt->cur_segment = data;
        break;
    case 6:
        pt->comm_param.voltage_offset = data;
        break;
    case 7:
        pt->comm_param.current_offset = data;
        break;
    default:
        break;
    }
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:site:%#x,data:%#x,exe'Dwin_Set_Back_Param'.\r\n",
               site, data);
#endif
}

/**
 * @brief  模式区分地址
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Cur_Mode_Distinguish(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;
    if (data == DWIN_SURE_CODE)
    {
        __RESET_FLAG(pt->flag, test_mode);
    }
    if (data == DWIN_CANCEL_CODE)
    {
        __SET_FLAG(pt->flag, test_mode);
    }
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:key_code:%#x,test_mode:%#x, exe'Dwin_Cur_Mode_Distinguish'.\r\n",
               data, __GET_FLAG(pt->flag, test_mode));
#endif
}

/**
 * @brief  自动测试按钮处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Auto_Test(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;
    if (data == DWIN_SURE_CODE)
    {
        __SET_FLAG(pt->flag, test_start_signal);
    }
    if (data == DWIN_CANCEL_CODE)
    {
        __RESET_FLAG(pt->flag, test_start_signal);
    }
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:key_code:%#x,test_start_signal:%#x,exe'Dwin_Auto_Test'.\r\n",
               data, __GET_FLAG(pt->flag, test_start_signal));
#endif
}

/**
 * @brief  结束测试按钮处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_End_Test(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;
    if (data == DWIN_SURE_CODE)
    {
        __SET_FLAG(pt->flag, test_end_signal);
    }
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:key_code:%#x,test_end_signal:%#x,exe'Dwin_End_Test'\r\n",
               data, __GET_FLAG(pt->flag, test_end_signal));
#endif
}

/**
 * @brief  保存数据按钮处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Save_Data(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;

    if (data == DWIN_SURE_CODE)
    {
    }
    if (data == DWIN_CANCEL_CODE)
    {
        memset(pt->data.val_buf, 0x00, sizeof(pt->data.val_buf));
    }
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:key_code:%#x,exe'Dwin_Save_Data'.\r\n", data);
#endif
}

/**
 * @brief  保存数据按钮处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Save_Param(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;

    if (data == DWIN_SURE_CODE)
    {
        /*系统参数存储到flash*/
    }
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:key_code:%#x,exe'Dwin_Save_Param'.\r\n", data);
#endif
}

/**
 * @brief  设置波形类型
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Set_Wave_Type(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;

    if (data < ad9833_null_signal)
    {
        pt->ac.wave_param.wave_mode = (ad9833_wave)data; //(ad9833_wave)(data - 1U)
    }

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:key_code:%#x,exe'Dwin_Set_Wave_Type'.\r\n", data);
#endif
}

/**
 * @brief  设置电压类型
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Set_Voltage_Mode(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;

    if (data < null_out)
        pt->pre->cur_power = (relay_power_type)data;

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:key_code:%#x,exe'Dwin_Set_Voltage_Mode'.\r\n", data);
#endif
}

/**
 * @brief  设置继电器策略
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Set_Relay_Tactics(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    ptesthandle pt = (ptesthandle)pd->Slave.pHandle;
    if (pt == NULL)
        return;

    if (data < custom_action)
        pt->cur_tactics = (relay_tactics_type)data;

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:key_code:%#x,exe'Dwin_Set_Relay_Tactics'.\r\n", data);
#endif
}
