硬件清单：面包板、stm32f103c8t6最小系统板、stlink、串口转usb、mou050、0.96英寸oled 芯片CH116
引脚接线表：B10 -> mpu6050 scl \\ B11 -> mpu6050 sda\\B6 oled SCL\\ B7-> oled SDA\\A10 -> TXD\\ A9 -> RXD
关键配置参数
USART1	PA9(TX) / PA10(RX)	串口调试/回显	115200, 8N1, 无校验，DMA 收发
I2C1	PB6(SCL) / PB7(SDA)	OLED	400 kHz
I2C2	PB10(SCL) / PB11(SDA)	MPU6050	100 kHz
TIM4	内部	FreeRTOS/HAL 时基	替代 SysTick
SYS	PA13(SWDIO) / PA14(SWCLK)	SWD 调试/烧录	Serial Wire
DMA 配置：

DMA1_Channel5：USART1_RX（外设→内存，字节对齐，Normal 模式）
DMA1_Channel4：USART1_TX（内存→外设，字节对齐，Normal 模式）
中断配置（优先级分组 4bit）：USART1、DMA1_Channel4、DMA1_Channel5 优先级均 5；TIM4 优先级 15。

FreeRTOS 任务结构
任务名	入口函数	职责	周期
serial	serial()	串口 DMA 接收 + 回显	事件驱动（有数据才处理）
imu	mpu6050()	读 MPU6050 + 姿态解算 + 入队	10 ms
oled	oled()	从队列取数并刷新 OLED	阻塞在队列接收
任务间通信：一条队列 xQueue1（长度 10，元素类型 attitude_angel，即 3 个 float 共 12 字节）。

代码目录说明
high task/
├─ high task.ioc            # CubeMX 工程文件
├─ .mxproject               # CubeMX 工程辅助文件
├─ Core/
│  ├─ Inc/
│  │  ├─ main.h             # 主头文件
│  │  ├─ FreeRTOSConfig.h   # FreeRTOS 配置
│  │  ├─ usart.h / i2c.h / dma.h / gpio.h   # 外设头文件
│  │  ├─ oled2.h            # OLED 驱动接口
│  │  └─ font.h             # 字库/图片结构定义
│  └─ Src/
│     ├─ main.c             # ★ 主程序：3个任务 + MPU6050 读写 + 姿态解算 + 串口回调
│     ├─ freertos.c         # CubeMX 生成的 defaultTask（未使用）
│     ├─ usart.c / i2c.c / dma.c / gpio.c   # 外设初始化
│     ├─ stm32f1xx_it.c     # 中断服务程序
│     ├─ stm32f1xx_hal_msp.c# 外设 MSP(引脚/时钟/中断)配置
│     ├─ stm32f1xx_hal_timebase_tim.c # TIM4 时基
│     ├─ oled2.c            # ★ OLED 驱动（CH1116，双缓冲+绘图+文字）
│     └─ font.c             # 字库（16x16 等）
├─ Drivers/                 # ST HAL 库（STM32F1 HAL/LL）
├─ Middlewares/             # FreeRTOS 内核源码
└─ MDK-ARM/
   ├─ high task.uvprojx     # Keil MDK 工程文件
   ├─ startup_stm32f103xb.s # 启动文件
   └─ high task/
      ├─ high task.axf/.hex/.map  # 编译产物（已编译成功）
      └─ ...(*.o)          # 目标文件

 串口通信收发处理流程
配置：USART1，115200/8N1，DMA 收发 + 空闲中断。：

serial() 启动时调用 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, 50)，让 DMA 一直监听接收，直到"收到数据后总线空闲"或"收满 50 字节"。
收满一帧后，HAL 自动回调 HAL_UARTEx_RxEventCallback()（在 main.c），在这里只做两件事：记录实际长度 rxLen = Size，置标志 rx_data_ready = 1。回调里不做耗时处理。
serial 任务循环轮询 rx_data_ready：
把 receiveData 拷贝到 transmitData；
HAL_UART_Transmit_DMA() 用 DMA 把数据回显发出去；
重新调用 HAL_UARTEx_ReceiveToIdle_DMA() 继续监听下一帧；
vTaskDelay(100ms) 等待发送完成。
发送完成：DMA 发完触发 HAL_UART_TxCpltCallback()，置 tx_busy = 0。
printf 重定向：fputc() 里用阻塞式 HAL_UART_Transmit() 把字符发到 USART1，所以 printf 能直接打印到串口（配合 ShowHex() 做调试）。

 IMU：型号、接口、校准与姿态解算
型号：MPU6050（InvenSense/TDK 六轴传感器：3 轴加速度计 + 3 轴陀螺仪，无磁力计）。

接口：

挂在 I2C2 上：PB10 = SCL，PB11 = SDA，速率 100 kHz。
器件地址：写 0xD0 / 读 0xD1，即 7 位地址 0x68（AD0 引脚接地）。
读数据：从寄存器 0x3B 起连续读 14 字节（ACCEL_X/Y/Z → TEMP → GYRO_X/Y/Z），每轴 2 字节大端拼接。

无校准

姿态解算方法：互补滤波

OLED 显示说明
硬件：驱动芯片 CH1116（与 SSD1306 指令兼容），分辨率 128×64，挂在 I2C1（PB6 = SCL，PB7 = SDA，400 kHz），器件地址 0x78（7 位 0x3C）。

驱动架构（oled2.c，波特律动 OLED 库，双缓冲）：

OLED_NewFrame() — 清零显存 OLED_GRAM[8][128]（8 页 × 128 列）；
调用各种绘制函数往显存里画（点/线/矩形/圆/椭圆/三角形/图片/文字）；
OLED_ShowFrame() — 把整块显存通过 I2C 一次性刷到屏幕。

编译烧录步骤
编译（Keil MDK-ARM）：

；
菜单 Project → Build Target（或快捷键 F7）编译链接；

 SWD 烧录

用 ST-Link/J-Link 连接板子的 SWDIO(PA13)、SWCLK(PA14)、GND、3V3；
Keil 里 Flash → Download（快捷键 F8）直接下载
