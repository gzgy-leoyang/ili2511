#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>


#define DRV_NAME		"ili2511"
#define _TOUCH_SCREEN_WIDTH		(800)
#define _TOUCH_SCREEN_HEIGHT	(480)

// 删除如下注释，打开调试输出
// #define _DEBUG_PRINT	(１)

static int max_x = 0;
static int max_y = 0;

// 说明：
// [0x89]={0x00,0x02}, 按下--释放 两次中断,但如果手指存在抖动，则也会多次中断，但坐标会存在差别，也可以做滑动手势的检测
// [0x89]={0x00,0x00}, 按下--释放 连续多次中断

// 自定义设备
/**
 * ili2511_device_info
 * @brief 设备信息结构定义 \n
 * 以静态对象的方式记录本设备相关的资源
 */
struct ili2511_device_info {
	struct i2c_client		*client;		///>必须，操作系统提供的 client 对象指针
	struct workqueue_struct	*workq;			///> 工作队列，执行 IIC 中断的“底部分”的处理
	struct input_dev		*input_device;	///> 输入设备指针，用于向 input 层输出
	unsigned int			pin_int;		///> 标记中断输入引脚编号
	unsigned int			pin_reset;		///> 标记复位控制输出引脚编号
};
static struct ili2511_device_info ili2511_device;

static int dbg = 0;

// static int ili2511_read_fingerCoordination(unsigned int *x,unsigned int *y);
static int ili2511_read_block(unsigned int addr, unsigned int len, unsigned char  *data);
/////////////////////
// interrupt 有关操作
/**
 * @brief
 * 中断底部分处理，读取手指触摸的坐标，并向 input 层报告坐标和事件
 * @note
 * static inline void input_report_abs(struct input_dev *dev, 
									unsigned int code,
									int value){
	input_event(dev, EV_ABS, code, value);
}

// report new input event
void input_event(	struct input_dev *dev,
					unsigned int type, 
					unsigned int code, 
					int value){
	...
	input_handle_event(dev, type, code, value);
	...
}

报告事件以及对应的坐标
input_report_abs(xxx_device->input_dev, ABS_RX,x_coordinate_value);

input_event(dev, EV_KEY, BTN_TOUCH,  0/1 );
value为1，代表按下，
value为0代表抬起
code值为BTN_TOUCH ( #define BTN_TOUCH 0x14a)
*/

/**
 * @brief 读手指坐标数据
 * @param unsigned int * x	读取的手指 x 坐标的指针
 * @param unsigned int * y	读取的手指 y 坐标的指针
 * 
 * @note
 * 首先读取状态寄存器，判断发生的事件的类型
 * 目前仅使用两个手指进行判断，所以获取的内容仅仅检查 finger_0 和 finger_1,也就是 0x30
 * 如果非0,则有输入按下;如果为0,则判断为释放事件
*/
// static int ssd254xStatus = 0;
// static int ili2511_read_fingerCoordination(unsigned int * x,unsigned int * y){
// 	int ret;
// 	unsigned char data[8] = {0};
	
// 	unsigned int x1 = 0;
// 	unsigned int y1 = 0;
// 	// unsigned int temp = 0;

// 	// float fx = 0;
// 	// float fy = 0;
// 	ret = ili2511_read_block(0x10,6,data);

// #ifdef _DEBUG_PRINT
// 	printk("[ili2511] finger : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
// 					data[0],data[1],data[2],data[3],data[4],data[5]);
// #endif


// 	if ( (data[1] & 0x80) != 0 ){
// 		// 有输入
// 		x1 = ((data[1] & 0x3F) << 8 | data[2]);
// 		y1 = ((data[3] & 0x3F) << 8 | data[4]);
// #ifdef _DEBUG_PRINT
// 		printk("[ili2511] Origin X:%i Y:%i\n",x1,y1);
// #endif
// 		x1 = ((x1 * 800) / max_x)  ;
// 		y1 = ((y1* 480) / max_y)  ;
// 		*y = y1;
// 		*x = x1;
// 		return 1;
// 	} else {
// 		return 0;
// 	}
// }

int pressX = 0,pressY=0;
int tempX = 0,tempY=0;
int status = 0;

static void ili2511_interruptHandle(struct work_struct *work)
{
	struct input_dev *inputDev = ili2511_device.input_device;
	// int x = 0;
	// int y = 0;

	// static int dragCount = 0;

	int ret;
	unsigned char data[8] = {0};
	
	unsigned int x1 = 0;
	unsigned int y1 = 0;
	// unsigned int temp = 0;
	unsigned int touch = 0;

	ret = ili2511_read_block(0x10,6,data);

	// 有输入
	x1 = ((data[1] & 0x3F) << 8 | data[2]);
	y1 = ((data[3] & 0x3F) << 8 | data[4]);
#ifdef _DEBUG_PRINT
	printk("[ili2511] Origin X:%i Y:%i\n",x1,y1);
#endif
	x1 = ((x1 * 800) / max_x)  ;
	y1 = ((y1* 480) / max_y)  ;
	touch = (data[1] & 0x80); 

	input_report_abs(inputDev, ABS_X,x1);
	input_report_abs(inputDev, ABS_Y,y1);
	input_report_key(inputDev, BTN_TOUCH,touch);
	// 刷新输入事件
	input_sync(inputDev);
	// 重新使能中断
	enable_irq(ili2511_device.client->irq);

/*
	// 读取数据
	// 将触摸屏数据链接到 input 设备，完成与操作系统的对接
	if ( ili2511_read_fingerCoordination(&x,&y)) {
		// 按下
		if ( x < 16000 && y < 9600 ){
			if ( status == 0 ){
				// 按下边沿
				status = 1;
#ifdef _DEBUG_PRINT
				printk("[ili2511] Press (%d,%d)\n",x,y);
#endif
				input_report_abs(inputDev, ABS_X,x);
				input_report_abs(inputDev, ABS_Y,y);
				input_report_key(inputDev, BTN_TOUCH,1);
			} else if (status == 1){
				// 按住移动
					input_report_abs(inputDev, ABS_X,x);
					input_report_abs(inputDev, ABS_Y,y);
					input_report_key(inputDev, BTN_TOUCH,1);
#ifdef _DEBUG_PRINT
					printk("[ili2511] Drag (%d,%d)\n",x,y);
#endif
			}
		}
	} else {
		// 抬起
		if ( status == 1 ){
			// dragCount = 0;
			status = 0;
			input_report_abs(inputDev, ABS_X,x);
			input_report_abs(inputDev, ABS_Y,y);
#ifdef _DEBUG_PRINT
			printk("[ili2511] Relesae (%d,%d)\n",x,y);
#endif
			input_report_key(inputDev, BTN_TOUCH,0);
		}
	}
	// 刷新输入事件
	input_sync(inputDev);
	// 重新使能中断
	enable_irq(ili2511_device.client->irq);
	*/
}

/**
 * @brief 创建 静态工作队列项
 * 
 * @note
 * #define DECLARE_WORK(n, f)	struct work_struct n = __WORK_INITIALIZER(n, f)
 * 
 struct work_struct {
	atomic_long_t data;
	struct list_head entry;
	work_func_t func;
	#ifdef CONFIG_LOCKDEP
	struct lockdep_map lockdep_map;
	#endif
 };

 #define __WORK_INITIALIZER(n, f) {					\
	.data = WORK_DATA_STATIC_INIT(),				\
	.entry	= { &(n).entry, &(n).entry },				\
	.func = (f),							\
	__WORK_INIT_LOCKDEP_MAP(#n, &(n))				\
 }
*/
static DECLARE_WORK(ili2511_work_queue, ili2511_interruptHandle);

/**
 * @brief 中断响应函数
 * iic 中断触发后，立即执行该函数
 * @note
 * 中断处理函数,分为如下两个部分：\n
 * 顶部分：同步中断标记，在“中断上下文”中快速处理，返回\n
 * 底部分：导入“工作队列”，在“进程上下文”中处理耗时的工作\n
*/
static irqreturn_t ili2511_interrupt(int irq, void *dev_id){
#ifdef _DEBUG_PRINT
	printk("[ili2511] Touch IRQ...\n");
#endif
	disable_irq_nosync(ili2511_device.client->irq);
	queue_work(ili2511_device.workq, &ili2511_work_queue);
	return IRQ_HANDLED;
}
// 完成 interrupt 操作
/////////////////////

/////////////////////
// input 有关操作

/**
 * @brief 注册一个输入设备，通过该设备将触摸屏的事件/坐标等消息报告该 OS 及相关的 App
 * @note
 * 初始化过程如下：
 * 	1.申请一个input设备
	xxx_device->input_dev = input_allocate_device()；

	2.告知input子系统它支持哪些事件
	set_bit(EV_ABS, xxx_device->input_dev->evbit);

	3.设置参数访问范围
	input_set_abs_params(xxx_device->input_dev, ABS_RX, 0, 23040, 0, 0);

	4.在驱动模块函数中注册输入设备：
	input_register_device(xxx_device->input_dev);

附：常用事件
EV_SYN      同步事件，当使用input_event()函数后,就要使用这个上报个同步事件
EV_KEY       键盘事件
EV_REL       (relative)相对坐标事件，比如鼠标
EV_ABS       (absolute)绝对坐标事件，比如摇杆、触摸屏感应
EV_MSC      其他事件,功能
EV_LED       LED灯事件
EV_SND      (sound)声音事件
EV_REP       重复键盘按键事件 (内部会定义一个定时器,若有键盘按键事件一直按下/松开,就重复定时,时间一到就上报事件
EV_FF         受力事件
EV_PWR      电源事件
EV_FF_STATUS  受力状态事件

触摸屏涉及的数据
ABS_X(X坐标方向), ABS_Y(Y坐标方向), ABS_PRESSURE(压力方向,比如绘图,越用力线就越粗)* / 
int absmax[ABS_MAX + 1];      //绝对坐标的最大值
int absmin[ABS_MAX + 1];      //绝对坐标的最小值
int absfuzz[ABS_MAX + 1];     //绝对坐标的干扰值,默认为0,
int absflat[ABS_MAX + 1];     //绝对坐标的平焊位置,默认为0

涉及数据结构和函数接口：
struct input_dev *input_allocate_device(void);  	//向内存中分配input_dev结构体
input_free_device(struct input_dev *dev);   		//释放内存中的input_dev结构体
input_register_device(struct input_dev *dev);   	//注册一个input_dev,若有对应的驱动事件,则在/sys/class/input下创建这个类设备
input_unregister_device(struct input_dev *dev);   	//卸载/sys/class/input目录下的 input_dev 这个类设备 
set_bit(nr,p);										//设置某个结构体成员p里面的某位等于nr,支持这个功能
比如:
set_bit(EV_KEY,buttons_dev->evbit);   //设置input_dev结构体buttons_dev->evbit支持EV_KEY
set_bit(KEY_S,buttons_dev->keybit);  //设置input_dev结构体buttons_dev->keybit支持按键”S”

input_set_abs_params(struct input_dev *dev, int axis, int min, int max, int fuzz, int flat); //设置绝对位移的支持参数
//dev: 需要设置的input_dev结构体
//axis : 需要设置的数组号,常用的有: ABS_X(X坐标方向), ABS_Y(Y坐标方向), ABS_PRESSURE(压力方向)
//min: axis方向的最小值,
//max:axis方向的最大值
//fuzz: axis方向的干扰值
//flat:axis方向的平焊位置
 
input_report_abs(struct input_dev *dev, unsigned int code, int value);   //上报EV_ABS事件
// dev :要上报哪个input_dev驱动设备的事件
// code: EV_ABS事件里支持的哪个方向，比如X坐标方向则填入: ABS_X
//value:对应的方向的值,比如X坐标126

input_report_key(struct input_dev *dev, unsigned int code, int value);//上报EV_KEY事件 
 
input_sync(struct input_dev *dev); //同步事件通知，通知系统有事件上报
 
struct  clk *clk_get(struct device *dev, const char *id);    //获得*id模块的时钟,返回一个clk结构体
// dev:填0即可,     *id:模块名字, 比如"adc","i2c"等,名字定义在clock.c中

clk_enable(struct clk *clk);   //开启clk_get()到的模块时钟,就是使能CLKCON寄存器的某个模块的位
*/
static int ili2511_register_input(void)
{
	int ret;
	struct input_dev *inputDev;

	inputDev = ili2511_device.input_device = input_allocate_device();
	if (inputDev == NULL){
		printk("[ili2511] ERR: Input device allocate\n");
		return -ENOMEM;
	}

	inputDev->name = "ili2511_input_device";

	// 声明 input输入子系统 支持事件和按键
	set_bit(EV_KEY, inputDev->evbit);
	set_bit(EV_ABS, inputDev->evbit);
	set_bit(EV_SYN, inputDev->evbit);
	set_bit(BTN_TOUCH, inputDev->keybit);

	//input_set_abs_params(struct input_dev *dev, int axis, int min, int max, int fuzz, int flat)
	//表示支持绝对值x坐标，并设置它在坐标系中的最大值和最小值，以及干扰值和平焊位置等。
	input_set_abs_params(inputDev, ABS_X, 0,_TOUCH_SCREEN_WIDTH-1, 0, 0);
	input_set_abs_params(inputDev, ABS_Y, 0,_TOUCH_SCREEN_HEIGHT-1, 0, 0);

	ret = input_register_device(inputDev);
	if (ret < 0){
		printk("[ili2511] ERR: Input device register\n");
		goto errorHandle;
	}
#ifdef _DEBUG_PRINT
	printk("[ili2511] Input device register...OK\n");
#endif

	return 0;

errorHandle:
	input_free_device(inputDev);
	return ret;
}
// 完成 input 操作
/////////////////////


/////////////////////
// 对 ILI2511 的操作

/**
 * @brief 读 iic 总线数据
 * @param unsigned int addr 读内存的首地址
 * @param unsigned int len	读内存的长度
 * @param unsigned char  *data	数据缓冲区指针
*/
static int ili2511_read_block(unsigned int addr, unsigned int len, unsigned char  *data)
{
	unsigned char msgbuf0[1] = { addr };
	unsigned int slave = ili2511_device.client->addr;
	unsigned int flags = ili2511_device.client->flags;

	// 准备两条消息：
	// msg_1: start condition + slave addr(LSB=1,write) + cmd
	// msg_2: start condition + slave addr(LSB=0,read) + Receive Data0 + Receive Data1 + Stop condition
	struct i2c_msg msg[2] = {
		{ slave, flags				, 1		, msgbuf0 	},
		{ slave, flags | I2C_M_RD	, len	, data 		}
	};
	// 启动发送
	return i2c_transfer(ili2511_device.client->adapter, msg, ARRAY_SIZE(msg));
}

/**
 * @brief 写 iic 总线数据
 * @param unsigned int addr 写内存的首地址
 * @param unsigned int len	写内存的长度
 * @param unsigned char  *data	数据缓冲区指针
*/
// static int ili2511_write_block(unsigned int regAddr, unsigned int len, unsigned char *data)
// {
// 	unsigned char msgbuf[3] = {0};
// 	struct i2c_msg msg;

// 	msg.addr = ili2511_device.client->addr;
// 	msg.flags = ili2511_device.client->flags;
// 	msg.len = 3;
// 	msg.buf = msgbuf;

// 	msgbuf[0] = regAddr;
// 	msgbuf[1] = *(data+0);
// 	msgbuf[2] = *(data+1);

// 	// 启动发送
// 	return i2c_transfer(ili2511_device.client->adapter,&msg,1);
// }

/**
 * @brief 复位 ssd254x
 * 通过控制 gpio 端口产生 10ms 的低电平信号复位芯片
*/
static int ili2511_reset(void){
#ifdef _DEBUG_PRINT
	printk("[ili2511] Hardware Reset...\n");
#endif
	/* Generate a 0 => 1 edge explicitly, and wait for startup... */
	gpio_set_value(ili2511_device.pin_reset, 1);
	msleep(10);
	gpio_set_value(ili2511_device.pin_reset, 0);
	msleep(10);
	gpio_set_value(ili2511_device.pin_reset, 1);
	/* Wait for startup (up to 125ms according to datasheet) */
	msleep(125);

	return 0;
}


/**
 * @brief  获取 ILI2511 的 ID
 * @note
 * 目的有以下：
 * 1.验证 IIC 通信是正常的
 * 2.后期 ts 方案多了，可以通过验证 ID 和厂商来加载不同的驱动
*/
static int ili2511_get_ID(void){
	unsigned char dataBuff[8] = {0};
	unsigned int ret = 0;
	// 读 ID 寄存器（0x02,2 Bytes）
	ret = ili2511_read_block(0x40,8, dataBuff);
	if (ret < 0) {
		dev_err(&ili2511_device.client->dev,"Read device ID failed\n");
		return -1;
	} else {
#ifdef _DEBUG_PRINT
		printk("[ili2511] ILI2511 ID:0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n",
				dataBuff[0],dataBuff[1],dataBuff[2],dataBuff[3],
				dataBuff[4],dataBuff[5],dataBuff[6],dataBuff[7]);
#endif
	}
	return 0 ;
}

static int ili2511_get_range(void){
	unsigned char dataBuff[8] = {0};
	unsigned int ret = 0;
	// 读 ID 寄存器（0x02,2 Bytes）
	ret = ili2511_read_block(0x20,4, dataBuff);
	if (ret < 0) {
		dev_err(&ili2511_device.client->dev,"Read device ID failed\n");
		return -1;
	} else {
		max_x = ((dataBuff[1] << 8) | dataBuff[0]);
		max_y = ((dataBuff[3] << 8) | dataBuff[2]);
#ifdef _DEBUG_PRINT
		printk("[ili2511] Touch Max_X:%i Max_Y:%i\n",max_x,max_y);
#endif
	}
	return 0 ;
}
/////////////////////////////////////////////

/**
 * @brief 配置中断资源
 * 根据给定的 IO 的编号，向 OS 申请一个 IRQ，获取 IRQ 编号，并指定该 IRQ 的触发类型;
 * 
 * @param struct i2c_client *i2c	设备对象
 * @param unsigned int pin_int		设备所涉及的中断 IO 编号
 * @note
 * 中断触发类型
	#define IRQ_TYPE_NONE			0
	#define IRQ_TYPE_EDGE_RISING	1
	#define IRQ_TYPE_EDGE_FALLING	2
	#define IRQ_TYPE_EDGE_BOTH		(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
	#define IRQ_TYPE_LEVEL_HIGH		4
	#define IRQ_TYPE_LEVEL_LOW		8
*/

static int ili2511_rst_init(struct device_node *node,struct i2c_client *i2c)
{
	unsigned int pin_rst = 0;

#ifdef _DEBUG_PRINT
	printk("[ili2511] RESET IO init...\n");
#endif

	if (of_gpio_count(node) < 2){
		printk("[ili2511] Dev node error\n");
		return -ENODEV;
	}

	// dts 中描述的顺序决定以下函数的第二个参数，比如：在 dts 中，
	// int 首先描述，则应该采用 of_get_gpio(np, 0) 获取 int pin 的资源
	pin_rst = of_get_gpio(node, 1);
	ili2511_device.pin_reset = pin_rst;
#ifdef _DEBUG_PRINT
	printk("[ili2511] Pin_rst...%i\n",pin_rst);
#endif


	// 检查该 IO 是否可用，是否与其他模块存在冲突
	if (!gpio_is_valid(pin_rst)) {
		printk("[ili2511] Invalid RST pins\n");
		// pr_err("%s: invalid GPIO pins, int=%d/reset=%d\n",node->full_name,pin_int,pin_rst);
		return -ENODEV;
	}

	if ( gpio_request(pin_rst, "pin_reset") == 0 ){
		ili2511_device.pin_reset = pin_rst;
#ifdef _DEBUG_PRINT
		printk("[ili2511] RST IO...%i\n",pin_rst);
#endif
	} else {
		printk("[ili2511] ERR: Request RST pin\n");
		if ( gpio_request(pin_rst, "pin_reset") == 0 ){
#ifdef _DEBUG_PRINT
			printk("[ili2511] RST IO(2)...%i\n",pin_rst);
#endif
		} else {
			printk("[ili2511] ERR: Request RST pin again\n");
		}
	}

	if ( gpio_direction_output(pin_rst, 1) == 0 ){
#ifdef _DEBUG_PRINT
		printk("[ili2511] RST IO: %i\n",pin_rst);
#endif
	} else {
		printk("[ili2511] ERR: set RST IO direction\n");
		gpio_free(pin_rst);
		return -ENODEV;
	}
	return 0;
}

static int ili2511_irq_init(struct device_node *node,struct i2c_client *i2c)
{
	unsigned int pin_int = 0;
	int ret = 0;

#ifdef _DEBUG_PRINT
	printk("[ili2511] IRQ init...\n");
#endif

	if (of_gpio_count(node) < 2){
		printk("[ili2511] Dev node error\n");
		return -ENODEV;
	}

	// dts 中描述的顺序决定以下函数的第二个参数，比如：在 dts 中，
	// int 首先描述，则应该采用 of_get_gpio(np, 0) 获取 int pin 的资源
	pin_int = of_get_gpio(node, 0);
	ili2511_device.pin_int = pin_int;
#ifdef _DEBUG_PRINT
	printk("[ili2511] Pin_int = %i\n",pin_int);
#endif


	// 检查该 IO 是否可用，是否与其他模块存在冲突
	if (!gpio_is_valid(pin_int)) {
		printk("[ili2511] Invalid GPIO pins\n");
		// pr_err("%s: invalid GPIO pins, int=%d/reset=%d\n",node->full_name,pin_int,pin_rst);
		return -ENODEV;
	}

	// 请求 interrupt 端口资源,输入
	if (gpio_request( pin_int, "pin_interrupt") == 0){
#ifdef _DEBUG_PRINT
		printk("[ili2511] IRQ IO %i\n",pin_int);
#endif
	}else {
		printk("[ili2511] ERR: Request IRQ IO\n");
		// 貌似第一次执行会失败
		gpio_free(pin_int); // 释放 pin_int 的 IO 资源
		if (gpio_request( pin_int, "pin_interrupt") == 0){
#ifdef _DEBUG_PRINT
			printk("[ili2511] IRQ IO(2)...%i\n",pin_int);
#endif
		} else {
			printk("[ili2511] ERR: Request IRQ IO again\n");
		}
	}

	if ( gpio_direction_input(pin_int) == 0 ){
#ifdef _DEBUG_PRINT
		printk("[ili2511] Direction of IRQ..INPUT\n");
#endif
	}else {
		gpio_free(pin_int);
		printk("[ili2511] ERR: set direction of IRQ\n");
		return -ENODEV;
	}

	// 为该端口申请一个中断源，并返回中断号
	i2c->irq = gpio_to_irq( pin_int );
	if(!i2c->irq){
		printk("[ili2511] ERR: Request IRQ\n");
		return -ENOMEM;
	} else {
#ifdef _DEBUG_PRINT
		printk("[ili2511] IRQ Number %i\n",i2c->irq );
#endif
	}

	// 设置中断类型:下降边沿
	irq_set_irq_type(i2c->irq,IRQ_TYPE_EDGE_BOTH);

	// 绑定中断号和中断处理程序
	ret = request_irq(	i2c->irq, 
						ili2511_interrupt,
						IRQ_TYPE_EDGE_BOTH,
						i2c->name, 
						&ili2511_device ); // IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
	if (ret < 0) {
		dev_err(&i2c->dev, "can't get irq %d: %d\n", i2c->irq, ret);
	}
	return 0;
}

/**
 * @brief 配置中断资源
 * 根据给定的 IO 的编号，向 OS 申请一个 IRQ，获取 IRQ 编号，并指定该 IRQ 的触发类型;
 * 
 * 	@param struct device_node *node	设备树的对应节点
	@param unsigned int *int_pin,	设备结构体中的保存 int 引脚编号的指针
	@param unsigned int *reset_pin	设备结构体中的保存 rst 引脚编号的指针
 * @note
	dts 中对本模块的描述中，包含如下的 IO 资源
	gpios = <	&gpio2 3 0 //SODIMM 133,ts_interrupt
				&gpio2 6 0 //SODIMM 127,ts_reset
	>;
*/

/**
 * @brief
 * 驱动探测函数
 * @param struct i2c_client *i2c			iic 设备对象指针
 * @param const struct i2c_device_id *id	
 * @note
 * 启动过程中，如果正确匹配设备，则会调用该函数。
 * 传入的参数中，i2c 指向设备对象，通过该对象指针获取设备树中描述的节点，以此初始化相关设备
 * 
 * ************* 说明 *********************
struct i2c_client {G
	unsigned short flags;//div., see below
	unsigned short addr;//chip address - NOTE: 7bit,addresses are stored in the_LOWER_ 7 bits
	char name[I2C_NAME_SIZE];
	struct i2c_adapter *adapter;//the adapter we sit on
	struct device dev;//the device structure
	int irq;//irq issued by device
	struct list_head detected;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	i2c_slave_cb_t slave_cb;//callback for slave mode
#endif

其中，i2c_client 包含成员 dev (struct device),该成员如下：
struct device {
...
	struct bus_type	*bus;			//type of bus device is on
	struct device_driver *driver;	//which driver has allocated this device
	void		*platform_data;		//Platform specific data, device core doesn't touch it
	void		*driver_data;		//Driver data, set and get with dev_set/get_drvdata
...
	struct device_node	*of_node;	//associated device tree node
...
};

platform_data 实际是一个通用指针


 * @todo
 * struct i2c_client *i2c 是如何传入的 ？？？，什么时候由 OS 构造的 ？
*/
static int ili2511_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;

	struct device_node *dts_node = i2c->dev.of_node; // 获取设备树中描述的节点信息
	
#ifdef _DEBUG_PRINT
	printk("[ili2511] ili2511_probe...\n");
#endif

	// 检查确认节点：i2c->dev.of_node
	if (dts_node != NULL) {

		ret = ili2511_rst_init(i2c->dev.of_node,i2c);
		ret = ili2511_irq_init(i2c->dev.of_node,i2c);
		if (ret)
			return ret;
	} 

	// Attach the I2C client（双向）
	// 1.
	ili2511_device.client =  i2c;
	// 2.将自定义的设备结构 ili2511_device 赋给设备驱动 client 的私有指针，当我们需要使用到 ili2511_device 内数据时，可以这样
	// struct  ili2511_device* temp;
	// temp = (struct ili2511_device*)i2c_get_clientdata(client);
	// 由 client 即可获得 ili2511_device 对象实例
	// ( i2_client->device->driver_data = ili2511_device )
	i2c_set_clientdata(i2c, &ili2511_device); 					
#ifdef _DEBUG_PRINT
	printk("[ili2511] i2c set client data...\n");
#endif


	/* 
	返回 适配器的 nr 值，适配器的构造如下：
	struct i2c_adapter {
		...
		struct device dev; //the adapter device
		int nr;
		...
	}
	*/
	dev_info(	&i2c->dev,
				"Touchscreen registered with bus id (%d) with slave address 0x%x\n",
				i2c_adapter_id(ili2511_device.client->adapter),
				ili2511_device.client->addr);
	
	/// 以上，完成 I2C client 的初始化

	/// 以下，对 ILI2511 执行初始化
	ili2511_get_ID();

	ili2511_get_range();
	// 复位 ILI2511 
	ili2511_reset();

	// 注册“输入设备”
	ret = ili2511_register_input();
	if (ret < 0) {
		printk("[ili2511] Input device register ERROR\n");
		//goto bail1;
	}

	// 准备“工作队列”
	ili2511_device.workq = create_singlethread_workqueue(DRV_NAME);
	if (ili2511_device.workq == NULL) {
		dev_err(&i2c->dev, "can't create work queue\n");
		ret = -ENOMEM;
		//goto bail2;
	}
#ifdef _DEBUG_PRINT
	printk("[ili2511] Create workqueue for interrupt...\n");
#endif
	return 0;
}

/**
 * @brief
 * 卸载驱动模块，执行 rmmod 命令时，执行该函数，主要是释放端口资源，清除工作队列，释放中断资源等。
*/
static int ili2511_remove(struct i2c_client *i2c)
{
	// if ( dbg ){
	// 	printk("[ili2511] ili2511_remove\n");
	// }
	gpio_free( ili2511_device.pin_int);
	gpio_free( ili2511_device.pin_reset );
	destroy_workqueue(ili2511_device.workq);
	free_irq(i2c->irq, &ili2511_device);
	input_unregister_device(ili2511_device.input_device);
	i2c_set_clientdata(i2c, NULL);

#ifdef _DEBUG_PRINT
	printk("[ili2511] Device remove\n");
#endif
	return 0;
}

/*
************* 说明 *********************
#define I2C_NAME_SIZE	20

struct i2c_device_id {
	char name[I2C_NAME_SIZE];
	kernel_ulong_t driver_data;	// Data private to the driver
};
在i2c driver驱动中通过以下代码将 device 和 driver 进行匹配
*/
static struct i2c_device_id ili2511_id[] = {
	{"ili2511", 0},
	{},
};

/*
************* 说明 *********************
用于匹配设备树节点和当前驱动
struct of_device_id {
	char	name[32];
	char	type[32];
	char	compatible[128];
	const void *data;
};
设备树中，通过 .compatible = "yq,ssd254x" 匹配，如下：
&i2c3 {
	status = "okay";
	pcap@10 { 
		compatible = "yq,ssd254x";

注：如果 dts 中没有描述该结点（具体来说，没有描述 compatible 属性，就不会启动匹配过程，也就不会调用 probe 函数）		
*/
static const struct of_device_id ili2511_dts_ids[] = {
	{.compatible = "yq,ili2511",}, 
	{}
};

/*
************* 说明 *********************
MODULE_DEVICE_TABLE（设备类型，设备表）
将 xx_driver_ids结构输出到用户空间，加载模块时，匹配硬件设备。
（ 设备表的最后一项必须是空）
一般用在热插拔的设备驱动中。
*/
MODULE_DEVICE_TABLE(of, ili2511_dts_ids);

/*
************* 说明 *********************
The I2C addresses to probe (for detect)

//Internal numbers to terminate lists
#define I2C_CLIENT_END		0xfffeU
*/
static unsigned short ili2511_addrs[] = { 0x41, I2C_CLIENT_END };

// ************* 说明 ********************* 
//1. 准备 i2c_driver 结构，并提供相应的成员
/* i2c_driver 结构的主要成员：
   @class: What kind of i2c device we instantiate (for detect)
 * @attach_adapter: Callback for bus addition (deprecated)
 * @probe: Callback for device binding - soon to be deprecated
 * @probe_new: New callback for device binding
 * @remove: Callback for device unbinding
 * @shutdown: Callback for device shutdown
 * @alert: Alert callback, for example for the SMBus alert protocol
 * @command: Callback for bus-wide signaling (optional)
 * @driver: Device driver model driver
 * @id_table: List of I2C devices supported by this driver
 * @detect: Callback for device detection
 * @address_list: The I2C addresses to probe (for detect)
 * @clients: List of detected clients we created (for i2c-core use only)
 * @disable_i2c_core_irq_mapping: Tell the i2c-core to not do irq-mapping
*/
static struct i2c_driver ili2511_driver = {
	.driver = {
		.owner          = THIS_MODULE,
		.name           = DRV_NAME,
		.of_match_table = ili2511_dts_ids,
	},
	.probe          = ili2511_probe,
	.remove         = ili2511_remove,
	.id_table       = ili2511_id,
	.address_list   = ili2511_addrs,
};

// 2. 在插入过程中，通过 init 函数，启动注册设备驱动
static int __init ili2511_init( void )
{
	int ret;
    // 将 ili2511_device 全部初始化为 0
#ifdef _DEBUG_PRINT
	printk("[ili2511] ILI2511 Driver init..\n");
#endif
    
	memset(&ili2511_device, 0, sizeof(ili2511_device));

	// 调用 i2c_add_driver(driver)，实际调用的是 int i2c_register_driver(struct module *owner, struct i2c_driver *driver)
	// i2c_register_driver 中调用 res = driver_register(&driver->driver) 注册设备，如果发现有匹配的设备，则调用 probe 函数
	// 特别注意：如果 dts 中没有描述对应结点（具体来说，对应节点没有描述 compatible 属性，就不会启动匹配过程，也就不会调用 probe 函数
	// 将 driver 注册到了i2c_bus_type 的总线上
	ret = i2c_add_driver(&ili2511_driver);
	if (ret < 0) {
		printk(KERN_WARNING DRV_NAME " can't add i2c driver: %d\n", ret);
	}
#ifdef _DEBUG_PRINT
	printk("[ili2511] i2c_add_driver..ok\n");
#endif
	return ret;
}
/*
附：初始化过程
i2c_add_driver 函数调用了 i2c_register_driver 函数，在 i2c_register_driver 里调用了 driver_register(&i2c_driver->driver),
（ 注：driver_register 是 linux 系统设备模型里面的函数，每一类linux设备驱动的注册最终都会调用它，传递的参数也由原来的i2c_driver 变成了 device_driver ）
driver_register 调用了 bus_add_driver，然后调用 driver_attach，然后调用 bus_for_each_dev （ 搜索总线上所有的device，我们这里是i2c总线，也即搜索i2c总线上的i2c_client），
调用 __driver_attach 判断 i2c_driver i2c_client 是否配备，再调用 driver_match_device，最终调用了 bus 的 match 函数（ 也就是 i2c bus 的 match 函数）
再调用 i2c_match_id，这里到最后的 id 配备了，如果配备成功则正常返回到 __driver_attach 函数，调用 driver_probe_device，再调用 really_probe，
最后调用 bus 的 probe 函数，在 bus 的 probe 函数里利用 to_i2c_driver 将 device_driver 转换成了 i2c_driver，最后调用了 i2c_driver 的 probe 函数（自定义的 probe ），
*/


static void __exit ili2511_exit( void )
{
#ifdef _DEBUG_PRINT
	printk("[ili2511] ILI2511 Driver Exit..\n");
#endif

	i2c_del_driver(&ili2511_driver);
}
module_init(ili2511_init);
module_exit(ili2511_exit);

module_param(dbg,int,S_IRUGO);
/*
常用的 MODULE 定义如下：（linux/module.h）
MODULE_AUTHOR(name) 定义驱动的编程者，name为string
MODULE_LICENSE(license) 定义驱动的license，一般为GPL，或相关公司的license
MODULE_DESCRIPTION(desc) 对驱动程序的描述，string
MODULE_SUPPORTED_DEVICE(name) 驱动程序所支持的设备，string
MODULE_PARM(var,type)
*/
MODULE_DESCRIPTION("ili2511 Touchscreen Driver");
MODULE_LICENSE("GPL");