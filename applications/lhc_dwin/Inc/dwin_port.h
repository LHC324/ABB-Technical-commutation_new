/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef PACKAGES_DWIN_INC_DWIN_PORT_H_
#define PACKAGES_DWIN_INC_DWIN_PORT_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "dwin.h"

    extern pDwinHandle Dwin_Object;
#if (DWIN_USING_RTOS == 2)
    extern int rt_dwin_init(void);
#else
extern void MX_DwinInit(void);
#endif

/*迪文屏幕页面*/
#define MAIN_PAGE 0x03
#define DIGITAL_INPUT_PAGE 0x04
#define DIFITAL_OUTPUT_PAGE 0x05
#define ANALOG_INPUT_PAGE 0x06
#define ANALOG_OUTPUT_PAGE 0x07
#define NONE_PAGE 0x08
#define COMMUNICATION_PAGE 0x0F
#define ERROR_PAGE 0x10
#define RESET_POEWR_NOTE_PAGE 28U
#define NEXT_DELAT_TIMES 50U

#define DWIN_USER_NAME_ADDR 0x3000     //用户名
#define DWIN_USER_PASSWORD_ADDR 0x3001 //用户密码
#define DWIN_LOGIN_SURE_ADDR 0x3002    //登录确认地址
#define DWIN_CANCEL_ADDR 0x3003        //注销地址
#define DWIN_ERROR_NOTE_ADDR 0x3004    //错误图标提示地址
#define DWIN_AUTO_TEST_ADDR 0x3005     //迪文自动测试地址
#define DWIN_END_TEST_ADDR 0x3006      //迪文结束测试地址
#define DWIN_SAVE_DATA_ADDR 0x3007     //迪文保存数据地址
#define DWIN_SAVE_PARAM_ADDR 0x3008    //迪文保存参数地址
#define DWIN_OPERATE_SHIFT_ADDR 0x3009 //迪文手动/自动模式区分地址

#define DWIN_SURE_CODE 0x00F1   //设置确认键值
#define DWIN_CANCEL_CODE 0x00F0 //注销键值

#define DWIN_DI_OUTPUT_ADDR 0x1000            //数字输出开始地址
#define DWIN_WAVE_MODE_ADDR 0x1002            //迪文波形模式地址
#define DWIN_VOLTAGE_MODE_ADDR 0x1003         //电压模式地址
#define DWIN_RELAY_TACTICS_ADDR 0x1005        //继电器策略
#define DWIN_SAMPLING_CURRENT_CH0_ADDR 0x1010 // 0通道电流采样地址
#define DWIN_SET_FRE_ADDR 0x1050              //设置频率
#define DWIN_SET_FRE_REG_ADDR 0x1052          //设置频率寄存器
#define DWIN_SET_PHASE_ADDR 0x1054            //设置相位
#define DWIN_SET_PHASE_REG_ADDR 0x1056        //设置相位寄存器
#define DWIN_SET_RANGE_ADDR 0x1058            //设置幅度
#define DWIN_SET_SEGMENT_ADDDR 0x105A         //设置段号
#define DWIN_SET_VOLTAGE_SHIF 0x105C          //电压偏差率
#define DWIN_SET_CURRENT_SHIF 0x105E          //电流偏差率

#ifdef __cplusplus
}
#endif
#endif /* PACKAGES_DWIN_INC_DWIN_PORT_H_ */
