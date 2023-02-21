# 均压测试仪

## 简介

换流阀均压测试装置是一种智能型分段式晶闸管电气特性测试装置，本装置采用低压变频信号作为换流阀晶闸管及其外部附属元器件交直流特性测试的信号源，利用不同频率信号施加在待测晶闸管及其相关外部电路上，以检测其综合电路对不同频率的响应特性，从而检测出电路对工频及其不同谐波的响应特性，及时发现电路中性能异常的元器件，达到高效快速检修的目的。

<img src="G:\STM32_Workers\STM32F1\ABB Technical commutation_new\document\均压测试仪.png" style="zoom:30%;" />

## 软件

- 设备工作前请确保：

  - 若使用了arm dsp库，则keil编译屏蔽部分信息：

    ![](G:\STM32_Workers\STM32F1\ABB Technical commutation_new\document\keil警告信息控制.jpg)

    

  - printf函数用法：

    ![](G:\STM32_Workers\STM32F1\ABB Technical commutation_new\document\printf精度和宽度变量控制方法.jpg)

    

  - 文件系统已经初始化`config.ini`文件

    ```css
    [system info] #system information
    name = Voltage equalizing tester
    hardware vesion = 1.2.0
    software = 1.0.0
    os = rt-thread
    os vesion = 5.0.0
    
    ;[contrast] #System result comparison criteria
    
    
    ;[group] #Working Group
    
    
    [run] #System runtime parameters
    flag = 0 				;System flag
    work mode = 0			;0:auto,1:Manual
    freq_mode = 0			;0:low,1:medium,2:high
    voltage offset = 5.0
    current offset = 5.0
    start = 1
    end = 7
    record counts = 0		;Record times
    file_size = 155
    
    [ad9833] #dds working parameters
    frequency = 1000 			;0-12500000
    frequency register = 0 		;0/1
    phase = 0 					;0-360
    phase register = 0 			;0/1
    range = 155 			    ;0-255
    wave = 0 					;0:sine wave,1:square wave,2：Trigonometric wave
    
    [wifi]
    work state = 1					;0:close,1:open
    work mode = 2					;0:ap,1:sta,2:ap+sta
    module state = 0 				;0:error,1:success
    network state = 0				;0:off-line,1:on-line
    ap name = LHC
    ap password = 66666666
    baud rate = 115200
    data bit = 8
    stop bit = 1
    check bit = 0
    ```

    

  - 确保文件系统已经创建`data.csv`文件

    ```c
    msh > creat_csv_data
    ```

    

  - rt-thread 修改ymodem协议适配网络传输：

    <img src="G:\STM32_Workers\STM32F1\ABB Technical commutation_new\document\ymodem_issue1.jpg" style="zoom:70%;" />

    

  - ad9833模块的spi驱动时序：

    <img src="G:\STM32_Workers\STM32F1\ABB Technical commutation_new\document\操作时系说明.jpg" style="zoom:80%;" />

    

  - adc同步采样数据：

    ![](G:\STM32_Workers\STM32F1\ABB Technical commutation_new\document\交流电采样点.jpg)

    

  - adc同步采样波形：

    <img src="G:\STM32_Workers\STM32F1\ABB Technical commutation_new\document\ADC2_50Hz.jpg" style="zoom:50%;" />



### 存在的问题：

- 电路结构上每段间输出电压不均衡，最边上两段尤为突出。
- 三种不同类型信号共用一套电路结构导致采样精度大打折扣。
- 过流检测，硬件没有做出有效保护。
- DC信号和50HzAC信号硬件结构上存在谐波干扰[放大器到MCU段]。
- 部分WiFi参数的屏幕可配置暂时不完全。

### 设备使用说明

具体使用，请参看：

`ABB Technical commutation_new\document\换流阀均压测试装置使用说明书.docx`

