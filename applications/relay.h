/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _RELAY_H_
#define _RELAY_H_
#ifdef __cplusplus
extern "C"
{
#endif
/*继电器动作最大组数*/
#define ACTION_MAX_GROUP_NUM 8U
/*继电器动作的下一次时间*/
#define RELAY_ACTION_NEXT_TIMES 1000U
    typedef enum
    {
        re_ok = 0,     /*无错误*/
        re_p_is_null,  /*空指针*/
        re_coil_over,  /*继电器池越界*/
        re_power_err,  /*电源类型错误*/
        re_act_err,    /*继电器操作类型错误*/
        re_praram_err, /*继电器api接口参数错误*/
        re_find_null,  /*查找对象为空*/
        re_call_err,   /*函数调用返回错误*/
        re_other_err,  /*其他错误*/
    } relay_error_code;

    typedef enum
    {
        relay_close = 0,
        relay_open,
    } relay_state;
    typedef struct relay_handletypedef relay_handle;
    typedef struct relay_handletypedef *pre_handle;
    typedef enum
    {
        pos_sque_action = 0, /*正序动作*/
        rev_ord_action,      /*逆序动作*/
        pos_and_rev_action,  /*正序逆序交替进行*/
        custom_action,       /*自定义动作*/
        null_action,         /*不动作*/
    } relay_tactics_type;
    typedef enum
    {
        dc_out = 0, /*直流输出*/
        ac_out,     /*交流输出*/
        null_out,
    } relay_power_type;
    typedef struct
    {
        relay_tactics_type action_type;
        void (*relay_operate)(pre_handle, relay_tactics_type);
    } relay_action_group;
    struct relay_handletypedef
    {
        relay_power_type cur_power;
        relay_action_group *pag;
        unsigned short action_group_size;
        unsigned char *pcoil;
        unsigned char coil_size;
        unsigned int next_time;  /*继电器动作的下一次时间*/
        unsigned char cur_group; /*自定义时使用*/

        void *pmodbus;
        void (*relay_delay)(unsigned int);
        void (*relay_callback)(pre_handle, unsigned char, unsigned char);
    };

    extern relay_handle relay_object;
    extern void relay_poll(pre_handle pre, relay_tactics_type cur_type);

#ifdef __cplusplus
}
#endif
#endif /* _RELAY_H_ */
