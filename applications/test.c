#include "test.h"
#include <rtthread.h>
#include "small_modbus_port.h"
#include "io_signal.h"
#include "dfs_file.h" /* 当需要使用文件操作时，需要包含这个头文件 */
#include "fcntl.h"
#include "unistd.h"
#include "minIni.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "test"
#define DBG_LVL DBG_LOG
#define USING_TEST_DEBUG 1
#define TEST_DEBUG_R dbg_raw
#define TEST_DEBUG_D LOG_D

static test_data_t test_data[TEST_MAX_NUM];

test_t test_object = {
    .pre = &relay_object,
    .comm_param = {
        .current_offset = 5.0f,
        .voltage_offset = 5.0f,
    },
    .freq_table = {0, 5e1f, 1e4f, 125e5f},
    .dc.output_voltage = 36.0F,
    .ac = {

        // .pnext_fre = NULL, //&fre_table[0]
        .wave_param = {
            .frequency = 0,
            .fre_sfr = 0,
            .phase_sfr = 0,
            .range = 30,
            .wave_mode = ad9833_sin,
        },
    },
    .cur_group = {
        .start = 0x01,
        .end = 0x07,
    },
    //    .cur_exe_count = 0,
    .data = {
        .size = sizeof(test_data) / sizeof(test_data[0]),
        .p = &test_data[0],
    },
};

/**
 * @brief	获取目标软件定时器句柄
 * @details
 * @param	pe 软件定时器对象
 * @param   id 目标软件定时器id
 * @retval  none
 */
test_timer_t *get_soft_timer_handle(test_timer_t *pe, test_timer id)
{
    if (pe == NULL || id > tim_id_max)
        return NULL;

    return &pe[id];
}

/**
 * @brief	软件定时器轮询
 * @details
 * @param	pt 校准系统句柄
 * @retval  none
 */
static void soft_timer_poll(ptest_t pt)
{
    if (pt == NULL)
        return;
    for (test_timer_t *p = pt->timer;
         p < pt->timer + sizeof(pt->timer) / sizeof(pt->timer[0]); ++p)
    {
        if (!(p->count))
            p->flag = true;
        else
            p->count--;
    }
}

/**
 * @brief	测试项定时器轮询系统
 * @details
 * @param	none
 * @retval  none
 */
void test_timer_poll(void)
{
    soft_timer_poll(&test_object);
}

#define __RESET_SOME_SYSTEM_FLAG(__pt)                 \
    do                                                 \
    {                                                  \
        __RESET_FLAG((__pt)->flag, test_start_signal); \
        __RESET_FLAG((__pt)->flag, test_first_flag);   \
        __RESET_FLAG((__pt)->flag, test_filed_signal); \
        __RESET_FLAG((__pt)->flag, test_finsh_signal); \
    } while (0)

/**
 * @brief  获取当前测试状态
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static enum test_stage get_cur_test_state(ptest_t pt)
{
    if (NULL == pt || NULL == pt->pre)
        return test_standby;

    /*检测到过流信号*/
    if (__GET_FLAG(pt->flag, test_overcurrent_signal))
    {
        return test_filed;
    }

    if (__GET_FLAG(pt->flag, test_developer_signal)) // 优先检测是否进入开发者模式
    {
        // __RESET_FLAG(pt->flag, test_developer_signal);

        __RESET_FLAG(pt->flag, test_start_signal);
        __RESET_FLAG(pt->flag, test_first_flag);
        __RESET_FLAG(pt->flag, test_filed_signal);
        __RESET_FLAG(pt->flag, test_finsh_signal);
        return test_developer;
    }

    if (!__GET_FLAG(pt->flag, test_start_signal)) // 结束信号
    {
        __RESET_FLAG(pt->flag, test_first_flag);
        return test_standby;
    }

    if (!__GET_FLAG(pt->flag, test_first_flag))
    {
        __SET_FLAG(pt->flag, test_first_flag);
        if (!__GET_FLAG(pt->flag, test_mode))
        {
            // pt->cur_group.end = TEST_MAX_NUM; // 自动模式时：开始/结束段不受手动运行
            relay_group temp_group =
                {
                    .start = 1U,
                    .end = TEST_MAX_NUM,
                };
            pt->pre->cur_group = temp_group; // 传递一个临时参数
            return test_auto;
        }
        else
        {
            pt->pre->cur_group = pt->cur_group;
            pt->pre->cur_exe_count = pt->cur_group.start - 1U; // 手动模式：存在一个组选择问题
            return test_manual;
        }
    }

    if (__GET_FLAG(pt->flag, test_filed_signal))
    {
        __RESET_FLAG(pt->flag, test_start_signal);
        __RESET_FLAG(pt->flag, test_first_flag);
        __RESET_FLAG(pt->flag, test_filed_signal);
        return test_filed;
    }

    // if (__GET_FLAG(pt->flag, test_end_signal))
    // {
    //     __RESET_SOME_SYSTEM_FLAG(pt);
    //     return test_standby;
    // }

    if (__GET_FLAG(pt->flag, test_finsh_signal))
    {
        __RESET_FLAG(pt->flag, test_start_signal);
        __RESET_FLAG(pt->flag, test_first_flag);
        __RESET_FLAG(pt->flag, test_finsh_signal);
        return test_finish;
    }

    return pt->cur_stage;
}

/**
 * @brief  波形参数写入保持寄存器
 * @note
 * @param pt 测试对象句柄
 * @param cur_freq 当前频率代号
 * @retval None
 */
static void test_write_wave_param(ptest_t pt, ad9833_out_t *pad)
{
    pModbusHandle pd = (pModbusHandle)pt->pmodbus;

    if (NULL == pt || NULL == pd || NULL == pad)
        return;

    // if (pad->frequency > pt->ac.fre_table[max_freq]) // 检查dds输出频率是否超限
    //     pad->frequency = 0;

    /*写入保持寄存器*/
    if (!pd->Mod_Operatex(pd, HoldRegister, Write, AD9833_PARAM_START_ADDR,
                          (uint8_t *)pad, sizeof(ad9833_out_t)))
    {
#if (USING_TEST_DEBUG)
        TEST_DEBUG_D("@error:Hold register write failed!\r\n");
#endif
    }
}

/**
 * @brief  过流检测
 * @note
 * @param None
 * @retval None
 */
void test_over_current_check(void)
{
    ptest_t pt = &test_object;

    if (HAL_GPIO_ReadPin(AI_SHORT_GPIO_Port, AI_SHORT_Pin) == GPIO_PIN_SET)
    {
        __SET_FLAG(pt->flag, test_overcurrent_signal);
        pt->cartoon.over_current = 1; // 开启过流动画图标
    }
    if (!__GET_FLAG(pt->flag, test_overcurrent_signal))
    {
        HAL_GPIO_WritePin(AI_RESET_GPIO_Port, AI_RESET_Pin, GPIO_PIN_RESET); // 清除硬件标志
        pt->cartoon.over_current = 0;                                        // 清除过流动画图标
    }
}

/**
 * @brief  过流处理
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
// static void test_over_current_handle(ptest_t pt)
// {
//     if (pt == NULL)
//         return;

//     if (!__GET_FLAG(pt->flag, test_overcurrent_signal))
//     {
//         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // 清除硬件标志
//         pt->cartoon.over_current = 0;                         // 清除过流动画图标
//     }
// }

/**
 * @brief  选择测试频率
 * @note
 * @param pt       测试对象句柄
 * @param cur_freq 当前频率
 * @retval 目标频率
 */
float test_select_output_freq(ptest_t pt, uint8_t site)
{
    if (NULL == pt)
        return 0;

    if (pt->user_freq > max_freq)
        return 0;

    if (site >= FREQ_MAX_NUM)
        return 0;

    pt->ac.wave_param.frequency = pt->freq_table[site]; // 更新显示频率

    return pt->ac.wave_param.frequency;
}

/**
 * @brief  获取一个波形参数句柄
 * @note
 * @param None
 * @retval 波形句柄
 */
ad9833_out_t *test_get_wave_handle(void)
{
    static ad9833_out_t test_wave;

    return &test_wave;
}

/**
 * @brief  设置一个运行时波形参数
 * @note
 * @param pt       测试对象句柄
 * @param cur_freq 当前频率
 * @retval 波形句柄
 */
ad9833_out_t *test_set_run_wave(ptest_t pt, float cur_freq)
{
    if (NULL == pt)
        return NULL;

    ad9833_out_t *pwave = test_get_wave_handle();

    if (cur_freq > 5000) // 自动调整幅值以适应功率放大板：保证不削顶
        pt->ac.wave_param.range = 50;
    else if (cur_freq > 1000)
        pt->ac.wave_param.range = 30;
    else
        pt->ac.wave_param.range = 20;

    memcpy(pwave, &pt->ac.wave_param, sizeof(ad9833_out_t)); // 系统参数加载到运行区
    pwave->frequency = cur_freq;

    return pwave;
}

static void test_at_standby(ptest_t pt);
static void test_at_auto(ptest_t pt);
static void test_at_manual(ptest_t pt);
static void test_at_developer(ptest_t pt);
static void test_at_filed(ptest_t pt);
static void test_at_finish(ptest_t pt);

static test_event_t test_event_group[] = {
    {test_standby, test_at_standby},
    {test_auto, test_at_auto},
    {test_manual, test_at_manual},
    {test_developer, test_at_developer},
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

    ptest_t pt = &test_object;
    pModbusHandle pd = (pModbusHandle)Modbus_Object;

    if (NULL == pd)
        return;

    if (pt->pre)
    {
        pt->pmodbus = pd;
        pt->pre->coil.p = Modbus_Object->pPools->Coils; // 挂接操作线圈
        // pt->pre->cur_group = pt->cur_group;                            // 传递组号
        pt->pre->timer.p = pt->timer;                                  // 传递软件定时器
        pt->pre->timer.num = sizeof(pt->timer) / sizeof(pt->timer[0]); // 传递软件定时器数量
    }

    // test_over_current_check();           // 随时检测是否发生过流：应该在测试过程中检测
    pt->cur_stage = get_cur_test_state(pt);
    for (ptest_event_t p = test_event_group;
         p < test_event_group + TEST_EVENT_SIZE(); ++p)
    {
        if (p->stage == pt->cur_stage)
            if (p->test_event)
            {
                test_write_wave_param(pt, test_get_wave_handle()); // 执行操作前刷新波形参数
                p->test_event(pt);
                break;
            }
    }
}

#define CSV_FILE_NAME "data.csv"
#define CSV_TABLE_HEADER "sn,timestamp(s),voltage(v),currnet(mA),v_offset(%),c_offset(%),notes\
    \n0,1668839363,18.0,120,0.05,0.05,sample                        \n"
#define STR_FORMAT "%d,%ld,%f,%f,%f,%f,real\n" // 序号、时间戳、电压、电流、电压偏差率、电流偏差率
#define CSV_FILE_MAX_SIZE 48 * 1024U
#define CSV_BUF_SIZE 64
/*单个文件数据开始回滚最大记录条数*/
#define CSV_MAX_RECORD ((CSV_FILE_MAX_SIZE - sizeof(CSV_TABLE_HEADER)) / CSV_BUF_SIZE)
/**
 * @brief	检查csv文件
 * @details
 * @param   pt 测试对象句柄
 * @param   p_file_info 文件信息指针
 * @retval  None
 */
static int test_check_csv(test_t *pt, struct stat *p_file_info)
{
    int ret;

    if (NULL == pt || NULL == p_file_info)
        return RT_ERROR;

    ret = stat(CSV_FILE_NAME, p_file_info);
#if (USING_TEST_DEBUG)
    if (ret == RT_EOK)
    {
        TEST_DEBUG_R("\r\n'data.csv' real size: %dByte, count size: %dByte.\n",
                     p_file_info->st_size, pt->file.cur_size);
    }
    else
        TEST_DEBUG_R("\r\n'data.csv' file not fonud ^_^.\n");
#endif

    return ret;
}

/**
 * @brief	测试系统数据存储到csv文件
 * @details
 * @param   pt 测试对象句柄
 * @param   pdata 目标数据指针
 * @retval  None
 */
static void test_data_save_to_csv(test_t *pt, test_data_t *pdata)
{
    int fd;
    struct timeval tv = {0};
    struct timezone tz = {0};
    char write_buf[CSV_BUF_SIZE];
    struct stat file_info;

    if (NULL == pt || NULL == pdata)
        return;

    fd = open(CSV_FILE_NAME, O_WRONLY | O_APPEND); // 数据以追加方式写入
    if (test_check_csv(pt, &file_info) != RT_EOK)  // 检查目标文件是否存在
        return;

    if (++pt->file.csv_line_count > CSV_MAX_RECORD) // 文件记录条数已达最大
    {
        pt->file.cur_size = sizeof(CSV_TABLE_HEADER);
        pt->file.csv_line_count = 0;
    }
    else
    { /*文件尚未发生回滚，意外错误导致的写入位置不同步：直接覆盖*/
        if (file_info.st_size < CSV_FILE_MAX_SIZE &&
            pt->file.cur_size != file_info.st_size)
        {
            pt->file.cur_size = sizeof(CSV_TABLE_HEADER);
        }
    }

    off_t offset = lseek(fd, pt->file.cur_size, SEEK_SET); // 重新定位文件读写指针
    if (offset != pt->file.cur_size)
    {
#if (USING_TEST_DEBUG)
        TEST_DEBUG_R("@error: lseek failed, target offset: %ld, actual offset: %ld ^_^.\n",
                     pt->file.cur_size, offset);
#endif
        pt->file.cur_size = sizeof(CSV_TABLE_HEADER);
        return;
    }
    else
    {
#if (USING_TEST_DEBUG)
        TEST_DEBUG_R("@note: lseek success, cur offset: %ld .\n",
                     offset);
#endif
    }

    uint32_t count = pt->file.csv_line_count;

    if (fd >= 0)
    {
        gettimeofday(&tv, &tz); // 获取时间
        rt_memset(write_buf, 0x00, sizeof(write_buf));
        /*预测buf尺寸：https://c-faq-chn.sourceforge.net/ccfaq/node210.html*/
        int str_size = rt_snprintf(NULL, 0, STR_FORMAT, count, tv.tv_sec, pdata->voltage, pdata->current,
                                   pdata->voltage_offset, pdata->current_offset);
        // char write_buf[str_size];
        // char *write_buf = rt_malloc(str_size);
        // if (write_buf)
        {
            rt_sprintf(write_buf, STR_FORMAT, count, tv.tv_sec, pdata->voltage, pdata->current,
                       pdata->voltage_offset, pdata->current_offset);
            write(fd, write_buf, sizeof(write_buf));
            close(fd);
            // rt_free(write_buf);

            // pt->file.csv_line_count++;
            pt->file.cur_size += sizeof(write_buf);

            // #define FILE_SIZE_INDEX 0x0C
            //             extern comm_val_t *get_comm_val(uint16_t index);
            //             extern void set_system_param(comm_val_t * pv, uint16_t index);
            //             comm_val_t *pv = get_comm_val(FILE_SIZE_INDEX); /*当前文件尺寸写回ini文件*/
            //             if (pv)
            //                 set_system_param(pv, FILE_SIZE_INDEX);
            // #undef FILE_SIZE_INDEX
        }
#if (USING_TEST_DEBUG)
        TEST_DEBUG_D("@note:str size: %d,timestamps: %ld,[%d]write done.\n", str_size, (long)tv.tv_sec, count);
#endif
    }
    else
    {
#if (USING_TEST_DEBUG)
        TEST_DEBUG_D("@error:timestamps: %ld,[%d]write failed,error code: %d.\n", tv.tv_sec,
                     count, rt_get_errno());
#endif
    }
    // count++;

#undef STR_FORMAT
}

/**
 * @brief	获取当前csv文件的行号
 * @details
 * @param   None
 * @retval  None
 */
void test_get_csv_file_cur_line(void)
{
    test_t *pt = &test_object;
    int fd, ret;
    struct stat file_info;
    char read_buf[8];

    if (test_check_csv(pt, &file_info) != RT_EOK) // 检查目标文件是否存在
        return;

    fd = open(CSV_FILE_NAME, O_RDONLY); // 读出文件尾的一行数据| O_APPEND

    if (fd >= 0)
    {
        ret = lseek(fd, -CSV_BUF_SIZE, SEEK_END); // 定位到当前文件指针位置
        if (ret >= 0)
        {
#if (USING_TEST_DEBUG)
            TEST_DEBUG_R("\r\nfile location succeeded.\n");
#endif
        }
        else
        {
#if (USING_TEST_DEBUG)
            TEST_DEBUG_R("failed to jump file location ^_^.\n");
#endif
        }
        ret = read(fd, read_buf, sizeof(read_buf));
        if (ret == 0)
        {
#if (USING_TEST_DEBUG)
            TEST_DEBUG_R("file end and no content to read ^_^.\n");
#endif
        }
        if (ret > 0)
        {
            sscanf(read_buf, "%d", &pt->file.csv_line_count);
#if (USING_TEST_DEBUG)
            TEST_DEBUG_R("cur line: %d,read succeede,buf: %s .\n\n",
                         pt->file.csv_line_count, read_buf);
#endif
        }
        close(fd);
    }
}

/**
 * @brief	测试系统数据存储
 * @details
 * @param	pt 测试对象句柄
 * @param   pdata 目标数据指针
 * @retval  None
 */
void test_data_save(test_t *pt, test_data_t *pdata)
{
    pModbusHandle pd = (pModbusHandle)pt->pmodbus;

    if (NULL == pt || NULL == pd ||
        NULL == pt->pre || NULL == pdata)
        return;

    if (pt->data.p && pt->pre->cur_exe_count < pt->data.size)
    {
        pt->data.p[pt->pre->cur_exe_count] = *pdata;
        /*数据写回输入寄存器*/
        pd->Mod_Operatex(pd, InputRegister, Write, pt->pre->cur_exe_count * sizeof(test_data_t),
                         (uint8_t *)pdata, sizeof(test_data_t));
        /*数据存到文件*/
        test_data_save_to_csv(pt, pdata);
    }
}

/**
 * @brief	测试系统数据处理
 * @details
 * @param	pt 测试对象句柄
 * @retval  None
 */
void test_data_handle(test_t *pt)
{
#define USER_DATA_NUMS 64U
    // float v0_sum = 0, v1_sum = 0, i0_sum = 0;
    // float ei0, ev0, ev1;
    float calc_result[] = {0, 0, 0}; // i0、v0、v1
    float sum_of_squares = 0;
    float voltage = 0;

    if (NULL == pt)
        return;

    for (uint8_t i = 0; i < sizeof(adc_buf) / sizeof(adc_buf[0]); ++i)
    {
#if (USING_TEST_DEBUG)
        TEST_DEBUG_R("\r\nadc_group[%d]:\r\n", i);
        TEST_DEBUG_R("----\t----\t----\t----\t----\t----\t----\t----\r\n");
#endif

        KFP hkfp = {
            .Last_Covariance = LASTP,
            .Kg = 0,
            .Now_Covariance = 0,
            .Output = 0,
            .Q = COVAR_Q,
            .R = COVAR_R,
        };                  // 确保每组特性不关联
        sum_of_squares = 0; // ac1和ac2结果不累加
        for (uint16_t j = 0; j < sizeof(adc_buf[0]) / sizeof(adc_buf[0][0]); ++j)
        {
            /*对adc0 、1数据进行分离*/
            if (i == 0)
            {
                adc_buf[1][j] = adc_buf[0][j] >> 16U;
                adc_buf[0][j] &= 0x0000FFFF;
            }

            if (!__GET_FLAG(pt->flag, test_developer_signal))
            {
                voltage = (float)adc_buf[i][j] * 3.3F / 4096.0F; // 替换为校准公式
            }
            else // 兼容开发者模式
                voltage = (float)adc_buf[i][j];
#if (USING_TEST_DEBUG)
            TEST_DEBUG_R("%.2f\t", voltage);
            if (((j + 1U) % 8) == 0)
                TEST_DEBUG_R("\r\n");
#endif

            /*ac模式：计算有效值*/
            // if (j < USER_DATA_NUMS)
            {
                switch (pt->pre->cur_power)
                {
                case dc_out:
                    // sum_of_squares += voltage; // 实际测试：是否需要卡尔曼
                    sum_of_squares = kalmanFilter(&hkfp, voltage);
                    break;
                case ac1_out:
                case ac2_out:
                    if (j > 95 && j <= 160) // 取256中间的64个点
                        sum_of_squares += powf(voltage, 2.0F);
                    break;
                default:
                    break;
                }
            }
        }
        if (i < sizeof(calc_result) / sizeof(calc_result[0]))
        {
            /*对dc、ac数据分类处理*/
            switch (pt->pre->cur_power)
            {
            case dc_out:
                // calc_result[i] = sum_of_squares / (float)USER_DATA_NUMS;
                calc_result[i] = sum_of_squares;
                break;
            case ac1_out:
            case ac2_out:
                calc_result[i] = sqrtf(sum_of_squares / (float)USER_DATA_NUMS);
                break;
            default:
                break;
            }
        }
    }

#if (USING_TEST_DEBUG)
    TEST_DEBUG_R("\r\nsite\ti0\tv0\tv1\r\n----\t----\t----\t----\t\r\n%d\t%.2f\t%.2f\t%.2f\r\n",
                 pt->pre->cur_exe_count, calc_result[0], calc_result[1], calc_result[2]);
#endif

    test_data_t data = {
        .voltage = calc_result[1],
        .current = calc_result[0],
        .voltage_offset = Get_Error((calc_result[1] + calc_result[2]) / 2.0F, calc_result[1]),
        .current_offset = Get_Error((calc_result[0] + calc_result[0]) / 2.0F, calc_result[0]),
    };

    if (pt->pre->cur_exe_count) // 从第二段开始计算真正电流偏差率
    {
        data.current_offset = Get_Error((pt->data.p[pt->pre->cur_exe_count - 1U].current +
                                         calc_result[0]) /
                                            2.0F,
                                        calc_result[0]);
    }

    /*site值回传；电压、电流数据写回modbus寄存器池；电压偏差、电流偏差计算*/
    test_data_save(pt, &data);
#undef USER_DATA_NUMS
}

/**
 * @brief 当前记录的文件位置写回
 * @note  仅在测试完成/失败时写回，主动结束的部分下一次将发生覆盖
 * @param None
 * @retval None
 */
static void test_write_back_cur_file_size(void)
{
#define FILE_SIZE_INDEX 0x0C
    extern comm_val_t *get_comm_val(uint16_t index);
    extern void set_system_param(comm_val_t * pv, uint16_t index);
    comm_val_t *pv = get_comm_val(FILE_SIZE_INDEX); /*当前文件尺寸写回ini文件*/
    if (pv)
        set_system_param(pv, FILE_SIZE_INDEX);
#undef FILE_SIZE_INDEX
}

/**
 * @brief  测试处于待机模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_standby(ptest_t pt)
{
    if (NULL == pt->pre || NULL == pt->pre->coil.p ||
        NULL == pt->pre->timer.p)
        return;

    test_timer_t *ptimer = (test_timer_t *)pt->pre->timer.p;
    ptimer = &ptimer[time_id_relay];

    // pt->ac.user_freq = low_freq;
    /*波形参数写入参数池*/
    test_set_run_wave(pt, 0);
    /*清除当前进度*/
    pt->pre->cur_exe_count = 0;
    /*意外错误或者测试完毕，关闭所有继电器（包括电源）*/
    memset(pt->pre->coil.p, 0x00, pt->pre->coil.size);
    /*复位开始信号和首次信号*/
    // __RESET_SOME_SYSTEM_FLAG(pt);

    // rt_thread_mdelay(1000);

    // extern rt_sem_t adc_semaphore;      // adc转换完成同步信号量
    // extern rt_sem_t trig_semaphore;     // 触发adc开始转换同步信号量
    // extern rt_sem_t continue_semaphore; // 继续同步信号量
    // /*清空所有信号量*/
    // rt_sem_take(adc_semaphore, 0);
    // rt_sem_take(trig_semaphore, 0);
    // rt_sem_take(continue_semaphore, 0);
    /*不确定直流期间是否会误触发过流信号：关闭过零检测*/
    __SET_FLAG(pt->flag, test_zero_crossing_signal);

    // extern rt_base_t pin_number;
    // /* 关闭中断 */
    // rt_pin_irq_enable(pin_number, PIN_IRQ_DISABLE);

    ptimer->set_flag = false; // 阶段定时器置为无效
    // memset(ptimer, 0x00, sizeof(test_timer_t));
}

/**
 * @brief  通过用户频率选择目标电源
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static relay_power_type test_select_power(ptest_t pt, enum wave_freq freq_type)
{
    relay_power_type target_power = null_out;

    if (NULL == pt)
        return null_out;

    switch (freq_type)
    {
    case low_freq:
        target_power = dc_out;
        break;
    case medium_freq:
        target_power = ac1_out;
        /*开启过流检测*/
        // __RESET_FLAG(pt->flag, test_zero_crossing_signal);
        break;
    case high_freq:
        target_power = ac2_out;
        // __RESET_FLAG(pt->flag, test_zero_crossing_signal);
        break;
    default:
        break;
    }

    return target_power;
}

/**
 * @brief  测试处于自动模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_auto(ptest_t pt)
{
    static uint8_t auto_count = 0;
#if (USING_TEST_DEBUG)
    // TEST_DEBUG_D("@note: exe 'test_at_auto'.\n");
#endif

    if (NULL == pt->pre || NULL == pt->pre->coil.p)
        return;

    // for (uint8_t i = low_freq; i < max_freq; ++i)
    // {
    //     pt->user_freq = (enum wave_freq)i;

    //     pt->ac.wave_param.frequency = pt->freq_table[i];
    //     /*波形参数写入参数池*/
    //     //        write_wave_param(pt);
    //     pt->pre->cur_power = test_select_power(pt); // 获取目标电源类型：自动获取
    //     relay_poll(pt->pre, pos_sque_action);       // 设置继电器策略：正向动作
    // }

    if (auto_count < max_freq)
    {
        /*波形参数写入参数池*/
        test_set_run_wave(pt, test_select_output_freq(pt, auto_count));
        pt->pre->cur_power = test_select_power(pt, (enum wave_freq)auto_count); // 获取目标电源类型：手动模式时需要
        relay_poll(pt->pre, pos_sque_action);                                   // 设置继电器策略：正向动作
        auto_count++;
    }
    else
    {
#if (USING_TEST_DEBUG)
        TEST_DEBUG_D("@note: end of automatic mode test.\n");
#endif
        auto_count = 0;
        __SET_FLAG(pt->flag, test_finsh_signal);
    }
}

/**
 * @brief  测试处于手动模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_manual(ptest_t pt)
{
#if (USING_TEST_DEBUG)
    // TEST_DEBUG_D("@note: exe 'test_at_manual'.\n");
#endif
    if (NULL == pt->pre || NULL == pt->pre->coil.p)
        return;

    /*波形参数写入参数池*/
    test_set_run_wave(pt, test_select_output_freq(pt, (uint8_t)pt->user_freq));
    pt->pre->cur_power = test_select_power(pt, pt->user_freq); // 获取目标电源类型：手动模式时需要
    relay_poll(pt->pre, pos_sque_action);                      // 设置继电器策略：正向动作
}

/**
 * @brief  测试处于开发者模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_developer(ptest_t pt)
{
    if (NULL == pt->pre)
        return;

    // 不复位线圈
    /*不关心频率:使用屏幕设定频率*/
    test_set_run_wave(pt, pt->ac.wave_param.frequency);
}

/**
 * @brief  测试处于失败模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_filed(ptest_t pt)
{
    if (NULL == pt->pre || NULL == pt->pre->coil.p)
        return;

    test_write_back_cur_file_size();
#if (USING_TEST_DEBUG)
    TEST_DEBUG_D("@note: exe 'test_at_filed'.\n");
#endif

    // pt->cur_stage = test_standby; // 错误提示后，转到待机模式

    // __RESET_FLAG(pt->flag, test_start_signal); // 复位开始信号
    /*测试失败后：应该置输出频率为0;实际显示频率为失败频率*/
    /*波形参数写入参数池*/
    // write_wave_param(pt);
    // rt_thread_mdelay(1000);
}

/**
 * @brief  测试处于完成模式
 * @note
 * @param pt 测试对象句柄
 * @retval None
 */
static void test_at_finish(ptest_t pt)
{
    if (NULL == pt->pre || NULL == pt->pre->coil.p)
        return;

    test_write_back_cur_file_size();
#if (USING_TEST_DEBUG)
    TEST_DEBUG_D("@note: exe 'test_at_finish'.\n");
#endif

    // pt->cur_stage = test_standby; // 成功提示后，转到待机模式

    // pt->ac.user_freq = low_freq;
    /*波形参数写入参数池*/
    // write_wave_param(pt);
    // rt_thread_mdelay(1000);
}

#ifdef RT_USING_FINSH
#include <finsh.h>

/**
 * @brief  创建一个csv格式文件
 * @note
 * @param None
 * @retval None
 */
void creat_csv_file(void)
{
    int fd;
    // char s[] = "sn,timestamp(s),voltage(v),currnet(mA),v_offset(%),c_offset(%),notes\
    // \n0,1668839363,18.0,120,0.05,0.05,sample\n";

    rt_kprintf("Write string %s to 'data.csv'.\n", CSV_TABLE_HEADER);

    /* 以创建和读写模式打开 /data.csv 文件，如果该文件不存在则创建该文件 */
    fd = open(CSV_FILE_NAME, O_WRONLY | O_CREAT);
    if (fd >= 0)
    {
        write(fd, CSV_TABLE_HEADER, sizeof(CSV_TABLE_HEADER));
        close(fd);
        rt_kprintf("Write done.\n");
    }
#undef CSV_TABLE_HEADER
}
MSH_CMD_EXPORT(creat_csv_file, Create a csv format file.);

/**
 * @brief   设置电源开关
 * @details
 * @param   none
 * @retval  none
 */
static void set_power(int argc, char **argv)
{
#ifdef COMMM_TITLE
#undef COMMM_TITLE
#endif
#define COMMM_TITLE "Please input'set_power<(0 ~2)[(eg. dc,ac1,ac2)]>'"

    ptest_t pt = &test_object;

    if (argc < 2)
    {
        TEST_DEBUG_R("@error: " COMMM_TITLE ".\n");
        return;
    }
    if (argc > 2)
    {
        TEST_DEBUG_R("@error: parameter is too long," COMMM_TITLE ".\n");
        return;
    }

    relay_power_type re_power = (relay_power_type)atoi(argv[1]);
    if (pt->pre)
        pt->pre->cur_power = re_power;
    TEST_DEBUG_R("@note: cur_power[%#x].\n", pt->pre->cur_power);
}
MSH_CMD_EXPORT(set_power, set_power<(0 ~2)>.);

/**
 * @brief   设置一些系统信号
 * @details
 * @param   none
 * @retval  none
 */
static void set_flags(int argc, char **argv)
{
#ifdef COMMM_TITLE
#undef COMMM_TITLE
#endif
#define COMMM_TITLE "Please input'set_flags <(0~31)|(0/1)>'"

    ptest_t pt = &test_object;

    if (argc < 3)
    {
        TEST_DEBUG_R("@error: " COMMM_TITLE ".\n");
        return;
    }
    if (argc > 3)
    {
        TEST_DEBUG_R("@error: parameter is too long," COMMM_TITLE ".\n");
        return;
    }
    uint8_t flag = atoi(argv[1]);
    if (flag > 31)
    {
        TEST_DEBUG_R("@error: unknown flag," COMMM_TITLE ".\n");
        return;
    }
    uint8_t val = atoi(argv[2]);
    if (val > 1)
    {
        TEST_DEBUG_R("@error: value error," COMMM_TITLE ".\n");
        return;
    }
    switch (val)
    {
    case 0:
        __RESET_FLAG(pt->flag, flag);
        break;
    case 1:
        __SET_FLAG(pt->flag, flag);
        break;
    default:
        break;
    }

    TEST_DEBUG_R("@note: cur_flags[%#x].\n", pt->flag);
}
MSH_CMD_EXPORT(set_flags, set_flags<(0 ~31) | (0 / 1)>.);
#endif
