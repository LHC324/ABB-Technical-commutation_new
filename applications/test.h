/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _TEST_H_
#define _TEST_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include "ad9833.h"
#include "relay.h"

#define TEST_MAX_NUM 8U //测试最大段数

    enum system_flag_group
    {
        test_start_signal = 0, //启动测试信号
        test_end_signal,       //强制结束信号
        test_filed_signal,     //测试失败信号
        test_finsh_signal,     //测试完成信号
        test_first_flag,       //首次开始标志
        test_mode,             /*测试模式:bit0 0为自动，1为手动*/
    };
    enum test_stage
    {
        test_standby = 0, //待机
        test_start,       //测试开始
        test_under,       //测试中
        test_auto,        //自动测试
        test_manual,      //手动测试
        test_dc,          // dc测试
        test_ac,          // ac测试
        test_filed,       //测试失败
        test_finish,      //测试完成
    };
    typedef struct test_handletypedef testhandle;
    typedef struct test_handletypedef *ptesthandle;
    struct test_handletypedef
    {
        uint32_t flag; //系统标志位
        // relay_power_type cur_power;     //电源类型
        pre_handle pre;                 //继电器操作组指针
        enum test_stage cur_stage;      //系统当前测试阶段
        relay_tactics_type cur_tactics; //当前策略
        uint8_t cur_segment;            //当前段号
        uint16_t test_result;           //测试结果
        struct
        {
            float output_voltage; //理论输出电压值
        } dc;                     //系统直流测试模式

        struct
        {
            ad9833_wave_out_handletypedef wave_param; //波形参数
            uint16_t fre_table[TEST_MAX_NUM];         //频率表
            uint16_t *pnext_fre;                      //自动模式下一测试频率
        } ac;                                         //系统交流测试模式
        struct
        {
            float voltage_offset, current_offset; //电压偏差率,电流偏差率
        } comm_param;

        struct
        {
            float val_buf[TEST_MAX_NUM * 2U];    //电压/电流
            float offset_buf[TEST_MAX_NUM * 2U]; //电压/电流偏差值
        } data;
    };

    typedef struct test_eventhandletypedef test_event;
    typedef struct test_eventhandletypedef *ptest_event;
    struct test_eventhandletypedef
    {
        enum test_stage stage;
        void (*test_event)(ptesthandle);
    };
    extern testhandle test_object;
    extern void test_poll(void);
#ifdef __cplusplus
}
#endif
#endif /* _TEST_H_ */
