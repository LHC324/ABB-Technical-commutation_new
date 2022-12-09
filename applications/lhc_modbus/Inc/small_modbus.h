/*
 * ModbusSlave.h
 *
 *  Created on: 2022年04月08日
 *      Author: LHC
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include "small_modbus_cfg.h"
#include "tool.h"
#include "stdbool.h"

	/*错误码状态*/
	typedef enum
	{
		lhc_mod_ok = 0,		/*正常*/
		lhc_mod_err_config, /*配置错误*/
		lhc_mod_err_len,	/*数据帧长错误*/
		lhc_mod_err_id,		/*地址号错误*/
		lhc_mod_err_crc,	/*校验码错误*/
		lhc_mod_err_cmd,	/*不支持的功能码*/
		lhc_mod_err_addr,	/*寄存器地址错误*/
		lhc_mod_err_value,	/*数据值域错误*/
		lhc_mod_err_write,	/*写入失败*/
	} Lhc_Modbus_State_Code;

	typedef enum
	{
		Smd_Master = 0x00,
		Smd_Slave,
	} Small_Modbus_Type;
	typedef enum
	{
		Coil = 0x00,
		InputCoil,
		InputRegister,
		HoldRegister,
		NullRegister,
	} Regsiter_Type __attribute__((aligned(4)));

	typedef enum
	{
		ReadCoil = 0x01,
		ReadInputCoil = 0x02,
		ReadHoldReg = 0x03,
		ReadInputReg = 0x04,
		WriteCoil = 0x05,
		WriteHoldReg = 0x06,
		WriteCoils = 0x0F,
		WriteHoldRegs = 0x10,
		ReportSeverId = 0x11,

	} Function_Code __attribute__((aligned(4)));

	typedef enum
	{
		Read = 0x00,
		Write,
	} Regsiter_Operate __attribute__((aligned(4)));

	enum Using_Crc
	{
		UsedCrc,
		NotUsedCrc
	};

	typedef struct Modbus_HandleTypeDef *pModbusHandle;
	typedef struct Modbus_HandleTypeDef MdbusHandle;
#if defined(SMODBUS_USING_MASTER)
	typedef struct
	{
		Function_Code code;
		uint8_t reg_len;
		uint16_t reg_start_addr;
	} Request_HandleTypeDef;
#endif
	typedef struct
	{
#if defined(SMODBUS_USING_COIL)
		uint8_t Coils[SMODBUS_REG_POOL_SIZE];
#endif
#if defined(SMODBUS_USING_INPUT_COIL)
		uint8_t InputCoils[SMODBUS_REG_POOL_SIZE];
#endif
#if defined(SMODBUS_USING_INPUT_REGISTER)
		uint16_t InputRegister[SMODBUS_REG_POOL_SIZE * 2U];
#endif
#if defined(SMODBUS_USING_HOLD_REGISTER)
		uint16_t HoldRegister[SMODBUS_REG_POOL_SIZE * 2U];
#endif
	} ModbusPools __attribute__((aligned(4)));

	typedef struct
	{
		Regsiter_Type type;
		void *registers;
	} Pool __attribute__((aligned(4)));

	struct Modbus_HandleTypeDef
	{
#if (SMODBUS_USING_RTOS == 2U)
		/*对于rt thread版本，增加一个设备描述指针*/
		// char dev_name[32U];
		rt_device_t dev, old_console;
#endif
		Small_Modbus_Type type;
		void (*Mod_CallBack)(pModbusHandle, Function_Code);
		void (*Mod_Recive)(void *);
		void (*Mod_Poll)(pModbusHandle);
		void (*Mod_Transmit)(pModbusHandle, enum Using_Crc);
#if defined(SMODBUS_USING_MASTER)
		void (*Mod_Code46H)(pModbusHandle, uint16_t, uint8_t *, uint8_t);
		void (*Mod_Request)(pModbusHandle);
#endif
		bool (*Mod_Operatex)(pModbusHandle, Regsiter_Type, Regsiter_Operate, uint16_t, uint8_t *, uint8_t);
#if (SMODBUS_USING_RTOS)
		void (*Mod_Lock)(void);
		void (*Mod_Unlock)(void);
#endif
		// void (*Mod_ReportSeverId)(pModbusHandle);
		void (*Mod_Error)(pModbusHandle, Lhc_Modbus_State_Code);
		void (*Mod_Ota)(pModbusHandle);
#if defined(SMODBUS_USING_MASTER)
		struct
		{
			/*主机id：可变*/
			uint8_t id;
			Request_HandleTypeDef request_data;
			void *pHandle;
		} Master;
#endif
		struct
		{
			/*从机id*/
			uint8_t id;
			/*预留外部数据结构接口*/
			void *pHandle;
		} Slave;
		ModbusPools *pPools;
		UartHandle Uart;
	} __attribute__((aligned(4)));

#define ssss sizeof(ModbusPools)

#if defined(SMODBUS_USING_COIL)
/*读线圈组:寄存器个数1到SMODBUS_REG_POOL_SIZE*/
#define Modbus_ReadCoilS(__obj, __saddr, __pdata, __size)             \
	(((__saddr) + (__size)) > (SMODBUS_REG_POOL_SIZE) ? false : do {  \
		memcpy((__pdata), (__obj)->pPools->Coils[__saddr], (__size)); \
	} while (0))
/*写线圈组*/
#define Modbus_WriteCoils(__obj, __saddr, __size, __pdata)                          \
	(((__saddr) + (__size)) > (SMODBUS_REG_POOL_SIZE) ? false : do {                \
		memcpy((__obj)->pPools->Coils[(__saddr)-COIL_OFFSET], (__pdata), (__size)); \
	} while (0),                                                                    \
	 true)
#endif
	extern void Free_ModObject(pModbusHandle *pd);
/*带上(pd)解决多级指针解引用问题：(*pd)、(**pd)*/
#define smd_tx_size(pd) ((pd)->Uart.tx.size)
#define smd_rx_size(pd) ((pd)->Uart.rx.size)
#define smd_tx_count(pd) ((pd)->Uart.tx.count)
#define smd_rx_count(pd) ((pd)->Uart.rx.count)
#define smd_tx_buf ((pd)->Uart.tx.pbuf)
#define smd_rx_buf ((pd)->Uart.rx.pbuf)

#define SMODBUS_WORD 1U
#define SMODBUS_DWORD 2U
/*获取主机号*/
#define Get_Smodus_id() (smd_rx_buf[0U])
/*获取Modbus功能号*/
#define Get_SmodbusFunCode() (smd_rx_buf[1U])
/*获取Modbus协议数据*/
#define Get_Smodbus_Data(__pbuf, __s, __size)                                \
	((__size) < 2U ? ((__pbuf[__s] << 8U) |                                  \
					  (__pbuf[__s + 1U]))                                    \
				   : ((__pbuf[__s] << 24U) |                                 \
					  (__pbuf[__s + 1U] << 16U) | (__pbuf[__s + 2U] << 8U) | \
					  (__pbuf[__s + 3U])))
#define Modbus_ReciveHandle(__obj, __dma) ((__obj)->Mod_TI_Recive((__obj), (__dma)))

#if (SMODBUS_USING_MALLOC == 0)
#define Init_SmallModbus_Object(id) (static MdbusHandle Connect_Str(Modbus_Object, id))
#define Get_SmallModbus_Object(id) (&Connect_Str(Modbus_Object, id))
#else
#define small_modbus_handler(obj) (obj->Mod_Poll(obj))
extern void Create_ModObject(pModbusHandle *pd, pModbusHandle ps);
#endif

#ifdef __cplusplus
}
#endif
#endif /* INC_MODBUS_H_ */
