#include "test.h"
#include <rtthread.h>
#include "small_modbus_port.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "test"
#define DBG_LVL DBG_LOG
#define USING_TEST_DEBUG 1
#define TEST_DEBUG_R dbg_raw
#define TEST_DEBUG_D LOG_D

testhandle test_object = {
    .pre = &relay_object,
    .comm_param = {
        .current_offset = 0.05f,
        .voltage_offset = 0.05f,
    },
    .dc.output_voltage = 36.0F,
    .ac = {
        .fre_table = {550U, 1250U, 750U, 2500U, 3750U, 6250U, 7500U, 10000U},
        .pnext_fre = NULL, //&fre_table[0]
        .wave_param = {
            .frequency = 0,
            .fre_sfr = 0,
            .phase_sfr = 0,
            .range = 30,
            .wave_mode = ad9833_sin,
        },
    },
};

#define __RESET_SOME_SYSTEM_FLAG(__pt)                 \
    do                                                 \
    {                                                  \
        __RESET_FLAG((__pt)->flag, test_start_signal); \
        __RESET_FLAG((__pt)->flag, test_first_flag);   \
    } while (0)

/**
 * @brief  获取当前测试状态
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static enum test_stage get_cur_test_state(ptesthandle pt)
{
    if (pt == NULL)
        return test_standby;

    if (!__GET_FLAG(pt->flag, test_start_signal))
        return test_standby;

    if (!__GET_FLAG(pt->flag, test_first_flag))
    {
        __SET_FLAG(pt->flag, test_first_flag);
        if (!__GET_FLAG(pt->flag, test_mode))

            return test_auto;
        else
            return test_manual;
    }

    if (__GET_FLAG(pt->flag, test_filed_signal))
    {
        __RESET_SOME_SYSTEM_FLAG(pt);
        return test_filed;
    }

    if (__GET_FLAG(pt->flag, test_end_signal))
    {
        __RESET_SOME_SYSTEM_FLAG(pt);
        return test_standby;
    }

    if (__GET_FLAG(pt->flag, test_finsh_signal))
    {
        __RESET_SOME_SYSTEM_FLAG(pt);
        return test_finish;
    }

    return pt->cur_stage;
}
/**
 * @brief  波形参数写入保持寄存器
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void write_wave_param(ptesthandle pt)
{
    pModbusHandle pd = Modbus_Object;
    ad9833_wave_out_handletypedef *pad = &pt->ac.wave_param;
    if (pt == NULL || pd == NULL)
        return;

    /*写入保持寄存器*/
    if (!pd->Mod_Operatex(pd, HoldRegister, Write, AD9833_PARAM_START_ADDR,
                          (uint8_t *)pad, sizeof(pt->ac.wave_param)))
    {
#if (USING_TEST_DEBUG)
        TEST_DEBUG_D("@error:Hold register write failed!\r\n");
#endif
    }
}

static void test_at_standby(ptesthandle pt);
static void test_at_auto(ptesthandle pt);
static void test_at_manual(ptesthandle pt);
static void test_at_filed(ptesthandle pt);
static void test_at_finish(ptesthandle pt);

static test_event test_event_group[] = {
    {test_standby, test_at_standby},
    {test_auto, test_at_auto},
    {test_manual, test_at_manual},
    {test_filed, test_at_filed},
    {test_finish, test_at_finish},
};

/**
 * @brief  根据启动信号执行测试流程
 * @note
 * @param None
 * @retval None
 */
void test_poll(void)
{
#define TEST_EVENT_SIZE() (sizeof(test_event_group) / sizeof(test_event_group[0]))

    ptesthandle pt = &test_object;

    pt->cur_stage = get_cur_test_state(pt);
    /*波形参数写入参数池*/
    write_wave_param(pt);
    for (ptest_event p = test_event_group;
         p < test_event_group + TEST_EVENT_SIZE(); ++p)
    {
        if (p->stage == pt->cur_stage)
            if (p->test_event)
                p->test_event(pt);
    }
}

/**
 * @brief  测试处于待机模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_standby(ptesthandle pt)
{
}

/**
 * @brief  测试处于自动模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_auto(ptesthandle pt)
{
    if (pt->pre)
        relay_poll(pt->pre, pt->cur_tactics);
}

/**
 * @brief  测试处于手动模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_manual(ptesthandle pt)
{
    // /**/
    // pt->cur_tactics = pos_sque_action;
    if (pt->pre)
        relay_poll(pt->pre, pt->cur_tactics);
}

/**
 * @brief  测试处于失败模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_filed(ptesthandle pt)
{
}

/**
 * @brief  测试处于完成模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_finish(ptesthandle pt)
{
}
