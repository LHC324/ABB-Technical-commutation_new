#include "relay.h"
#include "main.h"
#include <rtthread.h>
#include "io_signal.h"
#include "small_modbus_port.h"
#include "test.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "relay"
#define DBG_LVL DBG_LOG
#define USING_RELAY_DEBUG 1
#define RELAY_DEBUG_R dbg_raw
#define RELAY_DEBUG_D LOG_D

static void relay_any_tactics_action(pre_handle pre, relay_tactics_type cur_type);
// static void relay_manual_tactics_action(pre_handle pre, relay_tactics_type cur_type);
static void relay_callback(pre_handle pre,
                           unsigned char site,
                           unsigned char target_count);
/*继电器动作策略组*/
relay_action_group rela_act_group[] = {
    {.action_type = pos_sque_action, .relay_operate = relay_any_tactics_action},
    {.action_type = rev_ord_action, .relay_operate = relay_any_tactics_action},
    {.action_type = pos_and_rev_action, .relay_operate = NULL},
    {.action_type = custom_action, .relay_operate = NULL},
};
/*继电器池*/
// uint8_t relay_coil[SIGNAL_IO_DO_MAX];
relay_handle relay_object = {
    .pag = rela_act_group,
    .action_group_size = sizeof(rela_act_group) / sizeof(rela_act_group[0]),
    // .pcoil = &Modbus_Object->pPools->Coils,
    .coil_size = SIGNAL_IO_DO_MAX,
    //    .pmodbus = Modbus_Object,
    .next_time = RELAY_ACTION_NEXT_TIMES,
    .relay_delay = (void (*)(unsigned int))rt_thread_mdelay,
    .relay_callback = relay_callback,
};

/**
 * @brief   继电器组初始化
 * @details
 * @param   None
 * @retval  None
 */
int rt_relay_init(void)
{
    relay_object.pmodbus = Modbus_Object;
#if (USING_RELAY_DEBUG)
    RELAY_DEBUG_D("@note:relay_object = 0x%p\r\n", &relay_object);
#endif
    return 0;
}
/*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
INIT_DEVICE_EXPORT(rt_relay_init);

/**
 * @brief   继电器策略执行回调函数
 * @details
 * @param   pre 继电器组对象
 * @param   site 当前点
 * @param
 * @retval  None
 */
void relay_callback(pre_handle pre,
                    unsigned char site,
                    unsigned char target_count)
{
    ptesthandle pt = &test_object;
    pModbusHandle pd = Modbus_Object;
    float *pdata = pt->data.val_buf;
    if (pd == NULL)
        return;

    if (site < sizeof(pt->data.val_buf) / sizeof(pt->data.val_buf[0]))
    {
        /*读取输入寄存器:电流、电压*/
        pd->Mod_Operatex(pd, InputRegister, Read, site, (uint8_t *)&pdata[site], sizeof(float));
        pd->Mod_Operatex(pd, InputRegister, Read, 8U + site, (uint8_t *)&pdata[8U + site], sizeof(float));

        // if (!pd->Mod_Operatex(pd, InputRegister, Read, AD9833_PARAM_START_ADDR,
        //                       (uint8_t *)pad, sizeof(pt->ac.wave_param)))
        //         {
        // #if (USING_RELAY_DEBUG)
        //             RELAY_DEBUG_D("@error:Input Register read failed!\r\n");
        // #endif
        //         }
    }
    if ((site + 1U) == target_count)
    {
        __SET_FLAG(pt->flag, test_finsh_signal);
    }
}

/**
 * @brief   继电器策略执行
 * @details
 * @param   pre 继电器组对象
 * @param   cur_type 当前动作类型
 * @retval  None
 */
void relay_poll(pre_handle pre, relay_tactics_type cur_type)
{
    if (pre == NULL || pre->pag == NULL)
        return;
    for (relay_action_group *p = pre->pag;
         p < pre->pag + pre->action_group_size; ++p)
    {
        if (p->action_type == cur_type)
        {
            if (p->relay_operate)
                p->relay_operate(pre, cur_type);
        }
    }
}

/**
 * @brief   通过继电器输出目标电压类型
 * @details
 * @param   pre 继电器组对象
 * @retval  执行结果
 */
static relay_error_code set_relay_coil(pre_handle pre,
                                       uint8_t site,
                                       relay_state state)
{
    pModbusHandle pd = (pModbusHandle)pre->pmodbus;
    if (pre == NULL || pd == NULL ||
        pd->pPools == NULL)
        return re_p_is_null;
    if (site > pre->coil_size)
        return re_coil_over;
    // *(pre->pcoil + site) = (uint8_t)state;
    pd->pPools->Coils[site] = (uint8_t)state;
    /*数据写回输出寄存器池*/
    return re_ok;
}

/**
 * @brief   通过继电器输出目标电压类型
 * @details
 * @param   pre 继电器组对象
 * @retval  执行结果
 */
static relay_error_code set_relay_target_power(pre_handle pre,
                                               relay_power_type power,
                                               relay_state state)
{
#define K19_DC 9U
#define K29_AC 19U

    relay_error_code result = re_ok;
    result = power < ac_out
                 ? set_relay_coil(pre, K19_DC, state)
             : power < null_out ? set_relay_coil(pre, K29_AC, state)
                                : re_power_err;
    return result;
#undef K19_DC
#undef K29_AC
}

/**
 * @brief   设置继电器下一个动作周期
 * @details
 * @param   pre 继电器组对象
 * @param   times 目标时间
 * @retval  执行结果
 */
static relay_error_code set_coil_next_action_cycle(pre_handle pre,
                                                   uint32_t times)
{
    if (pre->relay_delay == NULL)
        return re_p_is_null;
    if (times)
        pre->relay_delay(times);
    return re_ok;
}

struct relay_tactics
{
    uint8_t tactics[2U][ACTION_MAX_GROUP_NUM * 2U];
    uint8_t tactics_count;
};
struct relay_matrix
{
    struct relay_tactics tactics;
    uint8_t matrix_size;
    relay_error_code result;
};

/*继电器策略组*/
const struct relay_tactics relay_matrix_group[] = {
    {
        .tactics = {
            {0, 1, 2, 3, 4, 5, 6, 7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
            {1, 2, 3, 4, 5, 6, 7, 8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        },
        .tactics_count = ACTION_MAX_GROUP_NUM,
    }, /*正向递进vs1->vs2*/
    {
        .tactics = {
            {1, 2, 3, 4, 5, 6, 7, 8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
            {0, 1, 2, 3, 4, 5, 6, 7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        },
        .tactics_count = ACTION_MAX_GROUP_NUM,
    }, /*反向递进vs2->vs1*/
    {
        .tactics = {
            {0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8},
            {1, 0, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7, 6, 8, 7},
        },
        .tactics_count = ACTION_MAX_GROUP_NUM * 2U,
    }, /*先正向后逆向*/
    {
        .tactics = {
            {1, 0, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7, 6, 8, 7},
            {0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8},
        },
        .tactics_count = ACTION_MAX_GROUP_NUM * 2U,
    }, /*先逆向后正向*/
};

// struct relay_matrix matrix_tactics = {
//     .ptactics = relay_matrix_group,
//     .matrix_size = sizeof(relay_matrix_group) / sizeof(relay_matrix_group[0]),
// };

#define __RESET_RELAY(__pre, __size)                                      \
    do                                                                    \
    {                                                                     \
        memset((__pre)->pcoil, 0x00, (__size));                           \
        memset(&((__pre)->pcoil[SIGNAL_IO_DO_MAX / 2U]), 0x00, (__size)); \
    } while (0)
/**
 * @brief   复位动作继电器组(不包括电源组)
 * @details
 * @param   pre 继电器组对象
 * @param   size 继电器数量
 * @retval  执行结果
 */
static relay_error_code reset_relay(pre_handle pre,
                                    uint8_t size)
{
    if (size > pre->coil_size)
        return re_coil_over;
    memset(pre->pcoil, 0x00, size);
    memset(&pre->pcoil[SIGNAL_IO_DO_MAX / 2U], 0x00, size);
    return re_ok;
}

/**
 * @brief   通过继电器模式获得对应策略
 * @details
 * @param   pre 继电器组对象
 * @param   relay_mode 当前动作模式组
 * @retval  执行结果
 */
static struct relay_matrix get_tactics_by_relay_mode(pre_handle pre,
                                                     relay_tactics_type relay_mode)
{
    ptesthandle pt = &test_object;
    struct relay_matrix target_matrix = {
        .matrix_size = sizeof(relay_matrix_group) / sizeof(relay_matrix_group[0]),
        .result = re_ok,
    };
    if (relay_mode > target_matrix.matrix_size)
        target_matrix.result = re_praram_err;
    else
        target_matrix.tactics = relay_matrix_group[(uint8_t)relay_mode]; /*访问flash*/
                                                                         /*手动模式：执行单组*/
    if (__GET_FLAG(pt->flag, test_mode))
    {
        if (pt->cur_segment < ACTION_MAX_GROUP_NUM)
        {
            uint8_t site0 = relay_mode < rev_ord_action
                                ? pt->cur_segment - 1U
                            : relay_mode < pos_and_rev_action
                                ? pt->cur_segment
                                : 0U;
            uint8_t site1 = relay_mode < rev_ord_action
                                ? pt->cur_segment
                            : relay_mode < pos_and_rev_action
                                ? pt->cur_segment - 1U
                                : 0U;
            target_matrix.tactics.tactics[0][0] = site0;
            target_matrix.tactics.tactics[1][0] = site1;
            target_matrix.tactics.tactics_count = 1U;
        }
    }

    return target_matrix;
}

/**
 * @brief   继电器动作执行器
 * @details
 * @param   pre 继电器组对象
 * @param   relay_mode 当前动作模式组
 * @retval  执行结果
 */
static relay_error_code set_coil_action_exe(pre_handle pre,
                                            relay_tactics_type relay_mode)
{
    struct relay_matrix cur_matrix = get_tactics_by_relay_mode(pre, relay_mode);
    relay_error_code result = re_ok;
    if (cur_matrix.result != re_ok)
    {
        result = re_call_err;
        goto __exit;
    }

    for (uint8_t i = 0; i < cur_matrix.tactics.tactics_count; ++i)
    {
        result = set_coil_next_action_cycle(pre, pre->next_time);
        if (result != re_ok)
            break;

        result = set_relay_coil(pre, cur_matrix.tactics.tactics[0][i], relay_open);
        if (result != re_ok)
            break;
        result = set_relay_coil(pre, cur_matrix.tactics.tactics[1][i], relay_open);
        if (result != re_ok)
            break;

        result = set_coil_next_action_cycle(pre, pre->next_time);
        if (result != re_ok)
            break;
        /*回调函数采集数据、计算偏差*/
        if (pre->relay_callback) /*回调函数中传入一些继电器策略*/
            pre->relay_callback(pre, i, cur_matrix.tactics.tactics_count);

        /*除电源外，其他继电器全部关闭*/
        // __RESET_RELAY(pre, ACTION_MAX_GROUP_NUM);
        reset_relay(pre, ACTION_MAX_GROUP_NUM);
    }

__exit:
    /*意外错误或者测试完毕，关闭所有继电器（包括电源）*/
    if (result != re_ok)
    {
        reset_relay(pre, ACTION_MAX_GROUP_NUM + 1U);
        // __RESET_RELAY(pre, ACTION_MAX_GROUP_NUM + 1U);
    }

    return result;
}

// typedef struct
// {
//     // relay_tactics_type relay_mode;
//     uint8_t k1x_start_addr, k2x_start_addr;
//     // uint8_t k1x_end_addr, k2x_end_addr;
//     uint8_t coil_act_group_num; /*继电器动作的组数*/
// } coil_head_info;

// struct coil_operate_info
// {
//     coil_head_info ope_info;
//     relay_error_code result;
// };

// /*数据按照relay_mode的顺序排列*/
// static coil_head_info coil_info[] = {
//     {
//         // .relay_mode = pos_sque_action,
//         .k1x_start_addr = 0U,
//         .k2x_start_addr = 15U,
//         .coil_act_group_num = ACTION_MAX_GROUP_NUM,
//     },
//     {
//         .k1x_start_addr = 1U,
//         .k2x_start_addr = 14U,
//         .coil_act_group_num = ACTION_MAX_GROUP_NUM,
//     },
// };

// /**
//  * @brief   获取操作继电器的一些信息
//  * @details
//  * @param   pre 继电器组对象
//  * @param   relay_mode 当前动作模式组
//  * @retval  执行结果
//  */
// static struct coil_operate_info get_coil_operate_info(pre_handle pre,
//                                                       relay_tactics_type relay_mode)
// {
//     /*数据按照relay_mode的顺序排列*/
//     coil_head_info coil_info[] = {
//         {
//             // .relay_mode = pos_sque_action,
//             .k1x_start_addr = 0U,
//             .k2x_start_addr = 15U,
//             .coil_act_group_num = ACTION_MAX_GROUP_NUM,
//         },
//         {
//             .k1x_start_addr = 1U,
//             .k2x_start_addr = 14U,
//             .coil_act_group_num = ACTION_MAX_GROUP_NUM,
//         },
//     };
//     struct coil_operate_info c_oper_info = {
//         .result = re_ok,
//     };

//     if (relay_mode < sizeof(coil_info) / sizeof(coil_info[0]) &&
//         relay_mode < null_action)
//         c_oper_info.ope_info = coil_info[(uint8_t)relay_mode];
//     else
//         c_oper_info.result = re_find_null;

//     return c_oper_info;
// }

// /**
//  * @brief   继电器动作执行器
//  * @details
//  * @param   pre 继电器组对象
//  * @param   relay_mode 当前动作模式组
//  * @retval  执行结果
//  */
// // static relay_error_code relay_positive_action(pre_handle pre,
// //                                               relay_tactics_type relay_mode)
// // {
// // }

// /**
//  * @brief   继电器动作执行器
//  * @details
//  * @param   pre 继电器组对象
//  * @param   relay_mode 当前动作模式组
//  * @retval  执行结果
//  */
// static relay_error_code set_coil_action_exe(pre_handle pre,
//                                             relay_tactics_type relay_mode)
// {
//     struct coil_operate_info info = get_coil_operate_info(pre, relay_mode);
//     relay_error_code result = re_ok;
//     if (info.result != re_ok)
//     {
//         result = re_call_err;
//         goto __exit;
//     }
//     uint8_t exe_count = info.ope_info.coil_act_group_num;
//     for (uint8_t i = 0; i < exe_count; ++i)
//     {
//         result = set_coil_next_action_cycle(pre, pre->next_time);
//         if (result != re_ok)
//             break;

//         result = set_relay_coil(pre, info.ope_info.k1x_start_addr + i, relay_open);
//         if (result != re_ok)
//             break;
//         result = set_relay_coil(pre, info.ope_info.k2x_start_addr + i, relay_open);
//         if (result != re_ok)
//             break;

//         result = set_coil_next_action_cycle(pre, pre->next_time);
//         if (result != re_ok)
//             break;
//         /*回调函数采集数据、计算偏差*/
//         if (pre->relay_callback) /*回调函数中传入一些继电器策略*/
//             pre->relay_callback(pre);
//         // result = set_relay_coil(pre, i, relay_close);
//         // if (i < exe_count - 1U)
//         //     result = set_relay_coil(pre, base_addr + i, relay_close);
//         /*除电源外，其他继电器全部关闭*/
//         __RESET_RELAY(pre, ACTION_MAX_GROUP_NUM);
//         // memcpy(pre->pcoil, 0x00, ACTION_MAX_GROUP_NUM);
//         // memcpy(&pre->pcoil[SIGNAL_IO_DO_MAX / 2U], 0x00, ACTION_MAX_GROUP_NUM);
//     }

// __exit:
//     /*意外错误或者测试完毕，关闭所有继电器（包括电源）*/
//     if (result != re_ok)
//     {
//         __RESET_RELAY(pre, ACTION_MAX_GROUP_NUM + 1U);
//     }

//     return result;
// }

/**
 * @brief   继电器策略执行
 * @details
 * @param   pre 继电器组对象
 * @param   cur_type 当前动作类型
 * @retval  None
 */
static void relay_any_tactics_action(pre_handle pre, relay_tactics_type cur_type)
{
    /*根据电源类型打开目标电源*/
    set_relay_target_power(pre, pre->cur_power, relay_open);
    set_coil_action_exe(pre, cur_type);
}

/**
 * @brief   继电器手动模式时策略执行
 * @details
 * @param   pre 继电器组对象
 * @param   cur_type 当前动作类型
 * @retval  None
 */
// static void relay_manual_tactics_action(pre_handle pre, relay_tactics_type cur_type)
// {
//     /*根据电源类型打开目标电源*/
//     set_relay_target_power(pre, pre->cur_power, relay_open);
// }
