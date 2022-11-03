#ifndef __TOOL_H__
#define __TOOL_H__

#ifdef __cplusplus
extern "C"
{
#endif
    // #include "main.h"

#define __INFORMATION()                            \
    "Line: %d,  Date: %s, Time: %s, Name: %s\r\n", \
        __LINE__, __DATE__, __TIME__, __FILE__

// /*使用就绪列表*/
// #define USING_FREERTOS_LIST 0
/*使用RTOS[0:不使用RTOS;1:Freertos;2:rt_thread]*/
#define TOOL_USING_RTOS 2
/*使用STM32HAL库*/
#define TOOL_USING_STM32HAL 1
/*交换任意类型数据*/
#define TOOL_USING_SWAP_ANY 1
/*使用卡尔曼滤波*/
#define TOOL_USING_KALMAN 1
/*使用CRC16*/
#define TOOL_USING_CRC16 1
/*使用大小端字节交换*/
#define TOOOL_USING_ENDIAN 1

#if (TOOL_USING_RTOS == 1)
#include "cmsis_os.h"
#define release_semaphore osSemaphoreRelease
#elif (TOOL_USING_RTOS == 2)
#include <rtthread.h>
#define release_semaphore rt_sem_release
#endif
#if (TOOL_USING_STM32HAL)
// #define TOOL_STM32_HAL_H "main.h"
#include "main.h"
#endif

/*连接两个字符串S1、S2*/
#define Connect_Str(S1, S2) (S1##S2)
/*按位操作任意类型数据*/
#define __SET_FLAG(__OBJECT, __BIT) (((__OBJECT) |= 1U << (__BIT)))
#define __RESET_FLAG(__OBJECT, __BIT) (((__OBJECT) &= ~(1U << (__BIT))))
#define __GET_FLAG(__OBJECT, __BIT) (((__OBJECT) & (1U << (__BIT))))
#if (TOOL_USING_SWAP_ANY)
/**
 * @brief	任意元素交换
 * @details
 * @param	__type 数据类型
 * @param   __lhs  带交换左边数据r
 * @param   __rhs  带交换右边数据
 * @retval	None
 */
#define SWAP(__type, __lhs, __rhs) \
    do                             \
    {                              \
        __type temp = __lhs;       \
        __lhs = __rhs;             \
        __rhs = temp;              \
    } while (false)
#else
/*使用数组元素时，下标不能相同; 且限制定于整形数据交换*/
#define SWAP(__A, __B)  \
    do                  \
    {                   \
        (__A) ^= (__B); \
        (__B) ^= (__A); \
        (__A) ^= (__B); \
    } while (0)
#endif

    typedef struct
    {
        void *pGPIOx;
        unsigned short Gpio_Pin;
        // unsigned char PinState;
    } Gpiox_info;
    typedef struct
    {
        Gpiox_info *pGpio;
        uint8_t PinNumbers;
    } Gpio_InfoTypeDef;

#if (TOOL_USING_KALMAN)
/*以下为卡尔曼滤波参数*/
#define LASTP 0.500F   //上次估算协方差
#define COVAR_Q 0.005F //过程噪声协方差
#define COVAR_R 0.067F //测噪声协方差

    typedef struct
    {
        float Last_Covariance; //上次估算协方差 初始化值为0.02
        float Now_Covariance;  //当前估算协方差 初始化值为0
        float Output;          //卡尔曼滤波器输出 初始化值为0
        float Kg;              //卡尔曼增益 初始化值为0
        float Q;               //过程噪声协方差 初始化值为0.001
        float R;               //观测噪声协方差 初始化值为0.543
    } KFP;

    extern float kalmanFilter(KFP *kfp, float input);
#else
/*左移次数*/
#define FILTER_SHIFT 4U

typedef struct
{
    bool First_Flag;
    float SideBuff[1 << FILTER_SHIFT];
    float *Head;
    float Sum;
} SideParm;
extern float sidefilter(SideParm *side, float input);
#endif

    typedef void (*pfunc)(void *, unsigned char);
    typedef struct
    {
        uint32_t addr;
        float upper;
        float lower;
        pfunc event;
    } Event_Map;
    typedef struct
    {
        unsigned char *pbuf;
        unsigned int size, count;
    } Uart_Data_HandleTypeDef __attribute__((aligned(4)));

    typedef struct Uart_HandleTypeDef *pUartHandle;
    typedef struct Uart_HandleTypeDef UartHandle;
    struct Uart_HandleTypeDef
    {
        void *huart, *phdma;
        Uart_Data_HandleTypeDef rx, tx;
#if (!TOOL_USING_RTOS)
        bool recive_finish_flag;
#else
    void *semaphore;
#endif
    } __attribute__((aligned(4)));

#if (TOOL_USING_STM32HAL)
#define uartx_irq_recive(obj, name) (obj->name##_Recive(&(obj->Uart)))
    void uartx_recive_handle(pUartHandle pu);
#endif
#if (TOOL_USING_CRC16)
    unsigned short get_crc16(unsigned char *ptr, unsigned short length, unsigned short init_dat);
#endif
#if (TOOOL_USING_ENDIAN)
    void endian_swap(unsigned char *pdata, unsigned char start, unsigned char length);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __TOOL_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
