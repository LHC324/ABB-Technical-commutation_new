/*
#include <packages/Small_Modbus/Inc/small_modbus.h>
 * ModbusSlave.c
 *
 *  Created on: 2022年04月08日
 *      Author: LHC
 */
#include "small_modbus.h"
#include "tool.h"

/*静态函数声明*/
static void Modbus_Poll(pModbusHandle pd);
#if defined(SMODBUS_USING_MASTER)
static void Modbus_46H(pModbusHandle pd, uint16_t regaddr, uint8_t *pdata, uint8_t datalen);
static void Modbus_Master_Request(pModbusHandle pd);
#endif
static bool Modbus_Operatex(pModbusHandle pd, Regsiter_Type reg_type, Regsiter_Operate operate,
                            uint16_t addr, uint8_t *pdata, uint8_t len);

/**
 * @brief  创建Modbus协议站对象(动态方式)
 * @param  pd 需要初始化对象指针
 * @param  ps 初始化数据指针
 * @retval None
 */
void Create_ModObject(pModbusHandle *pd, pModbusHandle ps)
{
    if (!ps)
        return;
#if (SMODBUS_USING_MALLOC)
    (*pd) = (pModbusHandle)smd_malloc(sizeof(MdbusHandle));
    if (!(*pd))
        smd_free(*pd);
    uint8_t *pTxbuf = (uint8_t *)smd_malloc(smd_tx_size(ps));
    if (!pTxbuf)
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:pTxbuf Creation failed!\r\n");
#endif
        smd_free(pTxbuf);
        return;
    }
    uint8_t *pRxbuf = (uint8_t *)smd_malloc(smd_rx_size(ps));
    if (!pRxbuf)
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:pRxbuf Creation failed!\r\n");
#endif
        smd_free(pRxbuf);
        return;
    }
#else
    static uint8_t pTxbuf[smd_tx_size(ps)];
    static uint8_t pRxbuf[smd_rx_size(ps)];
#endif

    memset(pTxbuf, 0x00, smd_tx_size(ps));
    memset(pRxbuf, 0x00, smd_rx_size(ps));
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("@note:Modbus[%d]_handler = 0x%p\r\n", ps->Slave.id, *pd);
#endif
    (*pd)->type = ps->type > Smd_Slave ? Smd_Slave : ps->type;
    (*pd)->Mod_CallBack = ps->Mod_CallBack;
    (*pd)->Mod_Error = ps->Mod_Error;
    (*pd)->Mod_Ota = ps->Mod_Ota;
#if (SMODBUS_USING_RTOS)
    (*pd)->Mod_Lock = ps->Mod_Lock;
    (*pd)->Mod_Unlock = ps->Mod_Unlock;
#endif
#if (SMODBUS_USING_DMA)
#if (TOOL_USING_STM32HAL)
    (*pd)->Mod_Recive = (void (*)(void *))uartx_recive_handle;
#else
    (*pd)->Mod_Recive = ps->Mod_Recive;
#endif
#endif
    (*pd)->Mod_Transmit = ps->Mod_Transmit;
    // (*pd)->Mod_ReportSeverId = ps->Mod_ReportSeverId;
    (*pd)->Mod_Poll = Modbus_Poll;
#if defined(SMODBUS_USING_MASTER)
    (*pd)->Mod_Code46H = Modbus_46H;
    (*pd)->Mod_Request = Modbus_Master_Request;
#endif
    (*pd)->Mod_Operatex = Modbus_Operatex;

    if (!ps->Uart.tx.pbuf)
    {
        ps->Uart.tx.pbuf = pTxbuf;
    }
    if (!ps->Uart.rx.pbuf)
    {
        ps->Uart.rx.pbuf = pRxbuf;
    }
    memcpy(&(*pd)->Uart, &ps->Uart, sizeof(UartHandle));
    memcpy(&(*pd)->Master, &ps->Master, sizeof(ps->Master));
    memcpy(&(*pd)->Slave, &ps->Slave, sizeof(ps->Slave));
    (*pd)->pPools = ps->pPools;
}

#if (SMODBUS_USING_MALLOC)
/**
 * @brief  销毁modbus对象
 * @param  pd 需要初始化对象指针
 * @retval None
 */
void Free_ModObject(pModbusHandle *pd)
{
    if (*pd)
    {
        smd_free((*pd)->Uart.tx.pbuf);
        smd_free((*pd)->Uart.rx.pbuf);
        smd_free((*pd));
    }
}
#endif

/*私有接口不对外开放*/
#if defined(SMODBUS_USING_COIL) || defined(SMODBUS_USING_INPUT_COIL)
static void Modbus_ReadXCoil(pModbusHandle pd);
static void Modbus_WriteCoil(pModbusHandle pd);
#endif
#if defined(SMODBUS_USING_INPUT_REGISTER) || defined(SMODBUS_USING_HOLD_REGISTER)
static void Modbus_ReadXRegister(pModbusHandle pd);
static void Modbus_WriteHoldRegister(pModbusHandle pd);
#endif
typedef void (*pSmallModbus_Operate)(pModbusHandle);
/**
 * @brief  small modbus根据协议栈注册类型获取操作指针
 * @param  pd small modbus对象句柄
 * @param  pg 函数操作指针句柄
 * @retval None
 */
static void Modbus_Get_Operate_Pointer(pModbusHandle pd,
                                       pSmallModbus_Operate *pg)
{
    Small_Modbus_Type smd_type = pd->type;
    pSmallModbus_Operate operate_group[] = {
        Modbus_ReadXCoil,
        Modbus_WriteCoil,
        Modbus_ReadXRegister,
        Modbus_WriteHoldRegister,
    };
    if (!pg)
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:Manipulate null pointer or illegal type operation.\r\n");
#endif
        return;
    }
    if (smd_type == Smd_Master)
    {
        uint16_t pg_num = sizeof(operate_group) / sizeof(pSmallModbus_Operate);
        if (pg_num % 2U)
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Non-2-aligned access.\r\n");
#endif
            return;
        }
        for (pSmallModbus_Operate *p = operate_group, *q;
             p < operate_group + pg_num; p += 2)
        {
            q = p + 1;
            SWAP(pSmallModbus_Operate, *p, *q);
        }
    }
    if (smd_type == Smd_Slave)
    {
        /*do something*/
    }
    memcpy(pg, operate_group, sizeof(operate_group));
}

/**
 * @brief  Modbus接收数据帧合法性检查
 * @param  pd modbus对象句柄
 * @retval Lhc_Modbus_State_Code 检验结果
 */
static Lhc_Modbus_State_Code Modbus_Recive_Check(pModbusHandle pd)
{
    /*检查协议栈类型*/
    if (pd->type > Smd_Slave)
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:Protocol stack type error,Must Master/Slave.\r\n");
#endif
        return lhc_mod_err_config;
    }
    /*首次调度时RXcount值被清零，导致计算crc时地址越界*/
    if ((smd_rx_count(pd) < 2U) || (smd_rx_count(pd) > smd_rx_size(pd)))
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:Protocol frame length error.\r\n");
#endif
        return lhc_mod_err_len;
    }
    uint8_t lhc_modbus_id = pd->Slave.id;
    if (pd->type == Smd_Master)
        lhc_modbus_id = pd->Master.id;
    /*检查是否是目标从站*/
    if (Get_Smodus_id() != lhc_modbus_id)
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:Protocol stack address error.\r\n");
#endif
        return lhc_mod_err_id;
    }
    uint16_t crc16 = get_crc16(smd_rx_buf, smd_rx_count(pd) - 2U, 0xffff);
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("md_rxcount = %d,crc16 = 0x%X.\r\n", smd_rx_count(pd),
                    (uint16_t)((crc16 >> 8U) | (crc16 << 8U)));
#endif

    if (Get_Smodbus_Data(smd_rx_buf, smd_rx_count(pd) - 2U, SMODBUS_WORD) !=
        ((uint16_t)((crc16 >> 8U) | (crc16 << 8U))))
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:crc check code error.\r\n");
#endif
        return lhc_mod_err_crc;
    }
    return lhc_mod_ok;
}

/**
 * @brief	Determine how the wifi module works
 * @details
 * @param	pd:modbus master/slave handle
 * @retval	true：MODBUS;fasle:shell
 */
static bool lhc_check_is_ota(pModbusHandle pd)
{
#define ENTER_OTA_MODE_CODE 0x0D
    return (((smd_rx_count(pd) == 1U) &&
             (smd_rx_buf[0] == ENTER_OTA_MODE_CODE)));
#undef ENTER_OTA_MODE_CODE
}

/**
 * @brief  Modbus接收数据解析[支持主机/从机]
 * @param  pd small modbus对象句柄
 * @retval None
 */
static void Modbus_Poll(pModbusHandle pd)
{
#if (!SMODBUS_USING_RTOS)
    if (!pd->Uart.recive_finish_flag)
        return;
    pd->Uart.recive_finish_flag = false;
#endif
    /*检查是否进入OTA升级*/
    if (lhc_check_is_ota(pd))
    {
        if (pd->Mod_Ota)
            pd->Mod_Ota(pd);
    }
    /*可以利用功能码和数据长度预测帧长度：可解析粘包数据*/
    Lhc_Modbus_State_Code lhc_state = Modbus_Recive_Check(pd);
    if (lhc_state != lhc_mod_ok)
    {
        if (pd->Mod_Error)
            pd->Mod_Error(pd, lhc_state);
        return;
    }
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("\r\nModbus_Buf[%d]:", smd_rx_count(pd));
    for (uint8_t i = 0; i < smd_rx_count(pd); i++)
    {
        SMODBUS_DEBUG_D("%02X ", smd_rx_buf[i]);
    }
    SMODBUS_DEBUG_D("\r\n\r\n");
#endif
    pSmallModbus_Operate pFunc_Group[] = {NULL, NULL, NULL, NULL};
    pSmallModbus_Operate *pOpt = pFunc_Group;
#define Using_Opt(__pd, __pOpt, __id)       \
    do                                      \
    {                                       \
        if (*((__pOpt) + (__id)))           \
            (*((__pOpt) + (__id)))((__pd)); \
    } while (false)
    /*确认协议栈是工作在主机还是从机*/
    Modbus_Get_Operate_Pointer(pd, pFunc_Group);
    switch (Get_SmodbusFunCode())
    {
#if defined(SMODBUS_USING_COIL) || defined(SMODBUS_USING_INPUT_COIL)
    case ReadCoil:
    case ReadInputCoil:
    {
        Using_Opt(pd, pOpt, 0U);
    }
    break;
    case WriteCoil:
    case WriteCoils:
    {
        Using_Opt(pd, pOpt, 1U);
    }
    break;
#endif
#if defined(SMODBUS_USING_INPUT_REGISTER) || defined(SMODBUS_USING_HOLD_REGISTER)
    case ReadHoldReg:
    case ReadInputReg:
    {
        Using_Opt(pd, pOpt, 2U);
    }
    break;
    case WriteHoldReg:
    case WriteHoldRegs:
    {
        Using_Opt(pd, pOpt, 3U);
    }
    break;
#endif
    case ReportSeverId:
    {
        //                    pd->Mod_ReportSeverId(pd);
        // pd->Mod_CallBack(pd, ReportSeverId);
    }
    break;
    default:
        break;
    }
    if (pd->Mod_CallBack)
        pd->Mod_CallBack(pd, (Function_Code)Get_SmodbusFunCode());
    memset(smd_rx_buf, 0x00, smd_rx_size(pd));
    smd_rx_count(pd) = 0U;
}

/*获取寄存器类型*/
#define Get_RegType(__obj, __type) \
    ((__type) < InputRegister ? (__obj)->pPools->Coils : (__obj)->pPools->InputRegister)

/*获取寄存器地址*/
#if defined(SMODBUS_USING_COIL) && defined(SMODBUS_USING_INPUT_COIL) && \
    defined(SMODBUS_USING_INPUT_REGISTER) && defined(SMODBUS_USING_HOLD_REGISTER)
#define Get_RegAddr(__obj, __type, __addr)                                                                 \
    ((__type) == Coil ? (uint8_t *)&(__obj)->pPools->Coils[__addr]                                         \
                      : ((__type) == InputCoil ? (uint8_t *)&(__obj)->pPools->InputCoils[__addr]           \
                                               : ((__type) == InputRegister                                \
                                                      ? (uint8_t *)&(__obj)->pPools->InputRegister[__addr] \
                                                      : (uint8_t *)&(__obj)->pPools->HoldRegister[__addr])))
#elif defined(SMODBUS_USING_COIL) || defined(SMODBUS_USING_INPUT_COIL)
#define Get_RegAddr(__obj, __type, __addr)                         \
    ((__type) == Coil ? (uint8_t *)&(__obj)->pPools->Coils[__addr] \
                      : (uint8_t *)&(__obj)->pPools->InputCoils[__addr])
#elif defined(SMODBUS_USING_INPUT_REGISTER) || defined(SMODBUS_USING_HOLD_REGISTER)
#define Get_RegAddr(__obj, __type, __addr)                                          \
    ((__type) == InputRegister ? (uint8_t *)&(__obj)->pPools->InputRegister[__addr] \
                               : (uint8_t *)&(__obj)->pPools->HoldRegister[__addr])
#else
#define Get_RegAddr(__obj, __type, __addr) (__obj, __type, __addr)
#endif

/**
 * @brief  Modbus协议读取/写入寄存器
 * @note   pd->Mod_Lock非空时（加锁）时，请不要在软件定时器或中断中调用该函数
 * @param  pd 需要初始化对象指针
 * @param  regaddr 寄存器地址[寄存器起始地址从0开始]
 * @param  pdat 数据指针
 * @param  len  读取数据长度
 * @retval None
 */
static bool Modbus_Operatex(pModbusHandle pd, Regsiter_Type reg_type, Regsiter_Operate operate,
                            uint16_t addr, uint8_t *pdata, uint8_t len)
{
    if (pd == NULL)
        return true;
    if (pd->Mod_Lock)
        pd->Mod_Lock();
    uint32_t max = reg_type < InputRegister
                       ? SMODBUS_REG_POOL_SIZE
                       : SMODBUS_REG_POOL_SIZE * 2U;
    uint8_t *pDest, *pSou;
    bool ret = false;

    switch (reg_type)
    {
#if defined(SMODBUS_USING_COIL)
    case Coil:
        max = sizeof(pd->pPools->Coils);
        break;
#endif
#if defined(SMODBUS_USING_INPUT_COIL)
    case InputCoil:
        max = sizeof(pd->pPools->InputCoils);
        break;
#endif
#if defined(SMODBUS_USING_INPUT_REGISTER)
    case InputRegister:
        max = sizeof(pd->pPools->InputRegister);
        break;
#endif
#if defined(SMODBUS_USING_HOLD_REGISTER)
    case HoldRegister:
        max = sizeof(pd->pPools->HoldRegister);
        break;
#endif
    default:
        max = 0;
        break;
    }
#if defined(SMODBUS_USING_RTOS)
    // taskENTER_CRITICAL();
#endif
    if ((addr < max) && (len <= max))
    {
#if defined(SMODBUS_USING_COIL) || defined(SMODBUS_USING_INPUT_COIL) || \
    defined(SMODBUS_USING_INPUT_REGISTER) || defined(SMODBUS_USING_HOLD_REGISTER)
        if (operate == Read)
        {
            pDest = pdata, pSou = Get_RegAddr(pd, reg_type, addr);
        }
        else
        {
            pDest = Get_RegAddr(pd, reg_type, addr), pSou = pdata;
        }
#endif
        if (memcpy(pDest, pSou, len))
            // for (uint8_t *p = pDest; p < pDest + len;)
            // {
            //     *p++ = *pSou++;
            // }
            ret = true;
        // __DSB();
        // __DMB();
    }
#if defined(SMODBUS_USING_RTOS)
    // taskEXIT_CRITICAL();
#endif
#if (SMODBUS_USING_DEBUG)
//    SMODBUS_DEBUG_D("pdest[%#x] = %#X, psou[%#x]= %#X, len= %d.\r\n", pDest, *pDest, pSou, *pSou, len);
#endif
    if (pd->Mod_Unlock)
        pd->Mod_Unlock();
    return ret;
}

#if defined(SMODBUS_USING_MASTER)
/**
 * @brief  Modbus协议主站有人云拓展46指令
 * @param  pd 需要初始化对象指针
 * @param  regaddr 寄存器地址
 * @param  pdata 数据指针
 * @param  datalen 数据长度
 * @retval None
 */
static void Modbus_46H(pModbusHandle pd, uint16_t regaddr, uint8_t *pdata, uint8_t datalen)
{
#define MASTER_FUNCTION_CODE 0x46
    uint8_t buf[] = {pd->Master.id, MASTER_FUNCTION_CODE, regaddr >> 8U, regaddr,
                     (datalen / 2U) >> 8U, (datalen / 2U), datalen};

    memset(smd_tx_buf, 0x00, smd_tx_size(pd));
    smd_tx_count(pd) = 0U;
    memcpy(smd_tx_buf, buf, sizeof(buf));
    smd_tx_count(pd) += sizeof(buf);
    memcpy(&smd_tx_buf[smd_tx_count(pd)], pdata, datalen);
    smd_tx_count(pd) += datalen;

    pd->Mod_Transmit(pd, UsedCrc);
}

/**
 * @brief  Modbus协议主站主动请求从机数据
 * @param  pd 需要初始化对象指针
 * @param  code 功能码
 * @param  regaddr 寄存器开始地址
 * @param  reglen 寄存器长度
 * @retval None
 */
static void Modbus_Master_Request(pModbusHandle pd)
{
    uint8_t buf[] = {
        (uint8_t)pd->Master.id,
        pd->Master.request_data.code,
        pd->Master.request_data.reg_start_addr >> 8U,
        pd->Master.request_data.reg_start_addr,
        (pd->Master.request_data.reg_len) >> 8U,
        (pd->Master.request_data.reg_len),
    };
    memset(smd_tx_buf, 0x00, smd_tx_size(pd));
    smd_tx_count(pd) = 0U;
    memcpy(smd_tx_buf, buf, sizeof(buf));
    smd_tx_count(pd) += sizeof(buf);

    pd->Mod_Transmit(pd, UsedCrc);
}
#endif

/**
 * @brief  Modbus协议读取线圈和输入线圈状态(0x01\0x02)
 * @note   暂未加入主机适配
 * @param  pd 需要初始化对象指针
 * @retval None
 */
#if defined(SMODBUS_USING_COIL) || defined(SMODBUS_USING_INPUT_COIL)
static void Modbus_ReadXCoil(pModbusHandle pd)
{
#define Byte_To_Bits 8U
    uint8_t len = Get_Smodbus_Data(smd_rx_buf, 4U, SMODBUS_WORD);
    // uint8_t bytes = len % Byte_To_Bits > 0 ? len / Byte_To_Bits + 1U : len / Byte_To_Bits;
    uint8_t bytes = len / Byte_To_Bits + !!(len % Byte_To_Bits);
    // uint8_t bytes = (len + Byte_To_Bits - 1U) / Byte_To_Bits;
    uint8_t *prbits = (uint8_t *)smd_malloc(len);
    Regsiter_Type reg_type = NullRegister;
    if (!prbits)
        goto __exit;

    memset(prbits, 0x00, len);
    memset(smd_tx_buf, 0x00, smd_tx_size(pd));
    smd_tx_count(pd) = 0U;
    memcpy(smd_tx_buf, smd_rx_buf, 2U);
    smd_tx_count(pd) += 2U;
    smd_tx_buf[smd_tx_count(pd)++] = bytes;
    /*通过功能码寻址寄存器*/
    reg_type = Get_SmodbusFunCode() == ReadCoil ? Coil
                                                : InputCoil;
    pd->Mod_Operatex(pd, reg_type, Read, Get_Smodbus_Data(smd_rx_buf, 2U, SMODBUS_WORD),
                     prbits, len);
#if (SMODBUS_USING_DEBUG)
    // for (uint8_t i = 0; i < len; i++)
    // SMODBUS_DEBUG_D("prbits[%d] = 0x%X, len= %d.\r\n", i, prbits[i], len);
#endif
    for (uint8_t i = 0; i < bytes; i++)
    {
        for (uint8_t j = 0; j < Byte_To_Bits && (i * Byte_To_Bits + j) < len; j++)
        {
            uint8_t bit = (prbits[i * Byte_To_Bits + j] & 0x01);
            if (bit)
                smd_tx_buf[smd_tx_count(pd)] |= (bit << j);
            else
                smd_tx_buf[smd_tx_count(pd)] &= ~(bit << j);
        }
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("pTbuf[%d] = 0x%X.\r\n", i, smd_tx_buf[smd_tx_count(pd)]);
#endif
        smd_tx_count(pd)++;
    }
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("smd_tx_count(pd) = %d.\r\n", smd_tx_count(pd));
#endif
    pd->Mod_Transmit(pd, UsedCrc);
__exit:
    smd_free(prbits);
}

/**
 * @brief  Modbus协议写线圈/线圈组(0x05\0x0F)
 * @param  pd 需要初始化对象指针
 * @retval None
 */
static void Modbus_WriteCoil(pModbusHandle pd)
{
    uint8_t *pdata = NULL, len = 0x00;
    enum Using_Crc crc;

    /*写单个线圈*/
    if (Get_SmodbusFunCode() == WriteCoil)
    {
        uint8_t wbit = !!(Get_Smodbus_Data(smd_rx_buf, 4U, SMODBUS_WORD) == 0xFF00);
        len = 1U;
        pdata = &wbit;
        smd_tx_count(pd) = smd_rx_count(pd);
        crc = NotUsedCrc;
    }
    /*写多个线圈*/
    else
    {
        len = Get_Smodbus_Data(smd_rx_buf, 4U, SMODBUS_WORD);
        // pdata = (uint8_t *)smd_malloc(len);
        /*利用发送缓冲区空间暂存数据*/
        pdata = smd_tx_buf;

        for (uint8_t i = 0; i < len; i++)
        {
            pdata[i] = (smd_rx_buf[7U + i / Byte_To_Bits] >> (i % Byte_To_Bits)) & 0x01;
        }
        smd_tx_count(pd) = 6U;
        crc = UsedCrc;
    }
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("pdata = %#X, len= %d.\r\n", *pdata, len);
#endif
    if (pdata)
        pd->Mod_Operatex(pd, Coil, Write, Get_Smodbus_Data(smd_rx_buf, 2U, SMODBUS_WORD),
                         pdata, len);
    memset(smd_tx_buf, 0x00, smd_tx_size(pd));
    /*请求数据原路返回*/
    memcpy(smd_tx_buf, smd_rx_buf, smd_tx_count(pd));
    /*从机模式才返回数据*/
    if (pd->type == Smd_Slave)
        pd->Mod_Transmit(pd, crc);
}
#endif

/**
 * @brief  Modbus协议读输入寄存器/保持寄存器(0x03\0x04)
 * @param  pd 需要初始化对象指针
 * @retval None
 */
#if defined(SMODBUS_USING_INPUT_REGISTER) || defined(SMODBUS_USING_HOLD_REGISTER)
static void Modbus_ReadXRegister(pModbusHandle pd)
{
    uint8_t len = Get_Smodbus_Data(smd_rx_buf, 4U, SMODBUS_WORD) * sizeof(uint16_t);
    uint8_t *prdata = (uint8_t *)smd_malloc(len);

    Regsiter_Type reg_type = NullRegister;
    if (!prdata)
        goto __exit;
    memset(prdata, 0x00, len);
    memset(smd_tx_buf, 0x00, smd_tx_size(pd));
    smd_tx_count(pd) = 0U;
    memcpy(smd_tx_buf, smd_rx_buf, 2U);
    smd_tx_count(pd) += 2U;
    smd_tx_buf[smd_tx_count(pd)] = len;
    smd_tx_count(pd) += sizeof(len);
    /*通过功能码寻址寄存器*/
    reg_type = Get_SmodbusFunCode() == ReadHoldReg ? HoldRegister
                                                   : InputRegister;
    pd->Mod_Operatex(pd, reg_type, Read, Get_Smodbus_Data(smd_rx_buf, 2U, SMODBUS_WORD),
                     prdata, len);
    memcpy(&smd_tx_buf[smd_tx_count(pd)], prdata, len);
    smd_tx_count(pd) += len;

    pd->Mod_Transmit(pd, UsedCrc);
__exit:
    smd_free(prdata);
}

/**
 * @brief  Modbus协议写保持寄存器/多个保持寄存器(0x06/0x10)
 * @note   支持主机和从机
 * @param  pd 需要初始化对象指针
 * @retval None
 */
static void Modbus_WriteHoldRegister(pModbusHandle pd)
{
    uint8_t *pdata = NULL, len = 0x00;
    uint16_t start_addr = Get_Smodbus_Data(smd_rx_buf, 0x02, SMODBUS_WORD);
    enum Using_Crc crc = UsedCrc;

    switch (Get_SmodbusFunCode())
    { /*写单个保持寄存器*/
    case WriteHoldReg:
    {
        len = sizeof(uint16_t);
        /*改变数据指针*/
        pdata = &smd_rx_buf[4U];
        smd_tx_count(pd) = smd_rx_count(pd);
        crc = NotUsedCrc;
    }
    break;
    case WriteHoldRegs:
    {
        len = smd_rx_buf[6U];
        /*改变数据指针*/
        pdata = &smd_rx_buf[7U];
        smd_tx_count(pd) = 6U;
    }
    break;
    case ReadHoldReg: /*主机模式*/
    {
        len = smd_rx_buf[2U];
        start_addr = pd->Master.request_data.reg_start_addr;
        /*改变数据指针*/
        pdata = &smd_rx_buf[3U];
    }
    break;
    default:
    {
        if (pd->Mod_Error)
            pd->Mod_Error(pd, lhc_mod_err_cmd);
    }
    break;
    }
    if (pdata)
        pd->Mod_Operatex(pd, HoldRegister, Write, start_addr, pdata, len);
    /*从机模式才返回数据*/
    if (pd->type == Smd_Slave)
    {
        memset(smd_tx_buf, 0x00, smd_tx_size(pd));
        /*请求数据原路返回*/
        memcpy(smd_tx_buf, smd_rx_buf, smd_tx_count(pd));
        pd->Mod_Transmit(pd, crc);
    }
}
#endif
