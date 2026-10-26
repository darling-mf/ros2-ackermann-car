/* @HEAD =====================================================================
 * 【研读标注版】代码未改。原始文件见 xtark_fw_original_gbk/Robot/ax_robot.c。
 *
 * 本文件是 B20 固件的"总调度台"：创建的不是数据通路，而是节拍。
 *   Robot_Task (50Hz 绝对周期) ：控制源解析 -> 运动学 -> 云台 -> IMU -> 发 0x10
 *   Trivia_Task (2Hz)          ：电池电压 + 蜂鸣器 + LED 心跳
 *   Key_Task / Disp_Task / Light_Task / Ps2_Task：HMI，与本项目无关
 *
 * 上位机看到的三个数据全部出自 ROBOT_SendDataToRos() 的 0x10 帧：
 *   acc xyz / gyro xyz（原始 int16）/ 速度 xyz（0.001 SI）/ 电池电压（0.1V）
 *
 * 阅读顺序建议：Robot_Task(92) -> ROBOT_IMUHandle(161) -> ROBOT_SendDataToRos(194)
 * -> 再看 Trivia_Task 的电池保护(240)。
 * ==========================================================================*/
/**			                                                    
		   ____                    _____ _______ _____       @塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： @塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *           
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  机器人控制主函数
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_robot.h"
#include "ax_light.h"
#include "ax_kinematics.h"
#include "ax_control.h"

//机器人速度数据
/* @L32 顺带一提：R_Vel 在 32 行和 44 行被定义了两次 -----------------
 * C 语言里两个"暂定定义"（tentative definition）会被合并成同一个对象，
 * 所以能编译通过，只是不严谨。属于厂商代码遗留，不影响运行。
 * ------------------------------------------------------------- */
ROBOT_Velocity  R_Vel;

//机器人轮子数据
ROBOT_Wheel  R_Wheel_A,R_Wheel_B;

//机器人IMU数据
ROBOT_Imu  R_Imu;

//机器人RGB数据
ROBOT_Light  R_Light;

//机器人速度数据
ROBOT_Velocity  R_Vel;

//机器人电池电压数据
uint16_t R_Bat_Vol;  

//IMU数据
int16_t ax_imu_acc_data[3];  
int16_t ax_imu_gyro_data[3]; 
int16_t ax_imu_gyro_offset[3]; 

//电机PID控制参数
/* @L55 轮速环仅两个增益：700 / 900 -----------------------------
 * 结合 ax_speed.c:45 的增量式写法 pwm += Kp·e + Kd·(e-e_last)，
 * 这里的 Kp 实际是积分作用、Kd 实际是比例作用（见 ax_speed.c 标注）。
 * 数值很大是因为 e 的量级很小（m/s 级误差，约 0.01~0.5）。
 * 建议不要凭直觉调这两个数：先理解增量式结构，再小步试。
 * ------------------------------------------------------------- */
int16_t ax_motor_kp=700;      
int16_t ax_motor_kd=900; 

//机器人运动使能开关,默认打开状态
uint8_t ax_robot_move_enable = 1;

//灯光开关失能，默认打开状态
uint8_t ax_light_enable = 1;

//蜂鸣器鸣叫一声
uint8_t ax_beep_ring = 0;

//PS2手柄键值结构体
JOYSTICK_TypeDef my_joystick;  

//控制方式选择
uint8_t ax_control_mode = CTL_ROS;


//阿克曼机器人前轮转向角度
int16_t ax_akm_angle = 0;

//舵机云台关节角度
int16_t ax_joint_angle[2] = {0};  


//函数定义
void JOINT_Control(void);       //舵机云台处理
void ROBOT_IMUHandle(void);     //IMU数据处理
void ROBOT_SendDataToRos(void);  //发送数据
	
	
/**
  * @简  述  机器人管理任务
  * @参  数  无
  * @返回值  无
  */
/* @L92 ★ 50Hz 主循环：全系统的心跳 ★ ---------------------------
 * 用 vTaskDelayUntil（绝对延时）而不是 vTaskDelay（相对延时）：
 *   前者保证"每 20ms 一拍"，即使某拍计算超时，下一拍会立即补上，周期不累积漂移；
 *   后者是"干完活再等 20ms"，计算耗时会叠加，实际频率被拉低。
 * 你在仿真里踩过的"墙钟 dt 与仿真时间不一致"是同类时序问题在另一侧的体现。
 * 顺序也有讲究：先解析控制源 -> 再算运动学 -> 最后发数据，
 * 保证发出的 RT 速度与本周期的 PWM 是同一拍的结果。
 * 注意：ROS 模式下 ax_control_mode==0，111 行的 if 被跳过 ——
 *       0x50 的目标速度是中断直接写全局变量的，不经过这里。
 * ------------------------------------------------------------- */
void Robot_Task(void* parameter)
{	

	//用于保存上次时间。调用后系统自动更新
	static portTickType PreviousWakeTime;

	//设置延时时间20ms，将时间转为节拍数 
	const portTickType TimeIncrement = pdMS_TO_TICKS(20);
	
	//获取当前系统时间 
	PreviousWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		
		//调用绝对延时函数20ms,执行频率50HZ
		vTaskDelayUntil(&PreviousWakeTime, TimeIncrement );
		
		//控制方式选择
/* @L111 控制源选择：ROS 模式在这里"什么都不做" ----------------
 * CTL_ROS=0 -> 跳过；目标速度由 UART4 中断写（ax_uart4.c:170~175）。
 * 其余模式（PS2/APP/SBUS）在这里轮询各自输入，写同一组 R_Vel.TG_I*。
 * 这就是"控制源互斥"的实现方式：一个全局变量 + 一个 if。
 * 风险：谁最后写谁生效，没有优先级仲裁；再叠加 L165 的强制切 ROS，
 *       实际上"发帧即可夺权"。
 * ------------------------------------------------------------- */
		if(ax_control_mode != 0)
		{
			if      (ax_control_mode == CTL_PS2)    AX_CTL_Ps2();    //PS2手柄控制
			else if (ax_control_mode == CTL_APP)    AX_CTL_App();    //APP控制
			else if (ax_control_mode == CTL_RMS)    AX_CTL_RemoteSbus();   //SBUS航模遥控器控制
		}
		
		//机器人运动学处理
/* @L119 运动学：本任务里唯一的重计算 ---------------------------
 * 内部顺序：读编码器 -> 限幅 -> IK -> 舵机 -> PID -> 写电机 -> FK 回算。
 * 全部同步执行，耗时远小于 20ms，所以这个任务不会饿死其它任务。
 * 若将来在里面加大量浮点运算（如在线辨识），要重新评估 20ms 预算。
 * ------------------------------------------------------------- */
		AX_ROBOT_Kinematics();	
		
		//舵机云台控制
		JOINT_Control();		
		
		//获取PMU6050加速度数据
        ROBOT_IMUHandle();
		
		//数据发送
		ROBOT_SendDataToRos();
			
	}
}

/**
  * @简  述  舵机云台控制函数
  * @参  数  无
  * @返回值  无
  */
void JOINT_Control(void)
{
	//幅度限制保护
	if(ax_joint_angle[0] > JOINTA_UP_LIMIT)	   ax_joint_angle[0] = JOINTA_UP_LIMIT;
	if(ax_joint_angle[0] < JOINTA_LOW_LIMIT)   ax_joint_angle[0] = JOINTA_LOW_LIMIT;
	
	if(ax_joint_angle[1] > JOINTB_UP_LIMIT)	   ax_joint_angle[1] = JOINTB_UP_LIMIT;
	if(ax_joint_angle[1] < JOINTB_LOW_LIMIT)   ax_joint_angle[1] = JOINTB_LOW_LIMIT;
	
	//查看舵机角度
	//printf("A%d  \r\n ", ax_joint_angle[0] );
	
	//设置舵机角度
	AX_SERVO_S1_SetAngle( ax_joint_angle[0] + JOINTA_ANGLE_OFFSET);
	AX_SERVO_S2_SetAngle( -ax_joint_angle[1] - JOINTB_ANGLE_OFFSET);	
		
}

/**
  * @简  述  机器人IMU数据处理
  * @参  数  无
  * @返回值  无
  */
/* @L161 IMU 处理：只做零偏和坐标变换，不做标度换算 -------------
 * 输出的 R_Imu.ACC_* / GYRO_* 是 **原始 LSB**，物理量换算在上位机做：
 *   acc  : (raw/32768)*2g   -> 1 LSB ≈ 0.0000611 g ≈ 0.000599 m/s²
 *   gyro : (raw/32768)*500  -> 1 LSB ≈ 0.01526 °/s ≈ 0.000266 rad/s
 * （量程 ±2g / ±500dps 是 MPU6050 的初始化配置，见 ax_mpu6050.c）
 * ★ 重要差异：这里的轴变换与更老的 robot.c:116~131 符号相反（差 180°）：
 *     ax_robot.c(B20): ACC_X=+imu[1], ACC_Y=-imu[0]
 *     robot.c(旧RM)  : ACC_X=-imu[1], ACC_Y=+imu[0]
 *   实车若出现"前进时 y 轴加速度指向反了"，先查这里，别先怀疑融合算法。
 * 零偏：ax_imu_gyro_offset 由上位机 0x51 帧触发 Imu_Task 校准（本文件未实现
 *   0x51 的接收，见 ax_uart4.c L180 标注 —— 所以这条校准路径在 B20 上不通，
 *   实际零偏只能靠上位机 sensor_fusion.py 的自校准）。
 * ------------------------------------------------------------- */
void ROBOT_IMUHandle(void)
{
		
	//获取PMU6050加速度数据
	AX_MPU6050_GetAccData(ax_imu_acc_data);
	
	//IMU坐标向机器人ROS坐标变换
	R_Imu.ACC_X =  ax_imu_acc_data[1];  //ROS坐标X轴对应IMU的Y轴
	R_Imu.ACC_Y = -ax_imu_acc_data[0];  //ROS坐标Y轴对应IMU的X轴反向
	R_Imu.ACC_Z =  ax_imu_acc_data[2];  //ROS坐标Z轴对应IMU的Z轴
	
	//获取PMU6050陀螺仪数据
	AX_MPU6050_GetGyroData(ax_imu_gyro_data);
	
	//陀螺仪加入零票校准数据
	ax_imu_gyro_data[0] += ax_imu_gyro_offset[0];
	ax_imu_gyro_data[1] += ax_imu_gyro_offset[1];
	ax_imu_gyro_data[2] += ax_imu_gyro_offset[2];
	
	//IMU坐标向机器人ROS坐标变换
	R_Imu.GYRO_X =  ax_imu_gyro_data[1];  //ROS坐标X轴对应IMU的Y轴
	R_Imu.GYRO_Y = -ax_imu_gyro_data[0];  //ROS坐标Y轴对应IMU的X轴反向
	R_Imu.GYRO_Z =  ax_imu_gyro_data[2];  //ROS坐标Z轴对应IMU的Z轴
	
	//观察数据
    //printf("@ %d  \r\n",R_Imu.ACC_X);
}

/**
  * @简  述  机器人发送数据到ROS
  * @参  数  无
  * @返回值  无
  */
/* @L194 ★ 0x10 帧组装：上位机三个话题的唯一来源 ★ --------------
 * 载荷 20 字节 = 10 个 int16 大端：
 *   [0:2]  ACC_X   [2:4]  ACC_Y   [4:6]  ACC_Z     原始 LSB
 *   [6:8]  GYRO_X  [8:10] GYRO_Y  [10:12] GYRO_Z   原始 LSB
 *   [12:14] RT_IX  0.001 m/s   [14:16] RT_IY 恒 0   [16:18] RT_IW 0.001 rad/s
 *   [18:20] 电池电压 0.1V
 * 大端写法 (u8)(x>>8) 先高字节后低字节 —— 与 protocol.py 的 struct '>h' 对应。
 * 无符号问题：int16 右移在 C 里是算术右移（保留符号），转 u8 截断取低 8 位，
 *   组合出的字节序列对负数也是正确的补码表示。
 * 频率 = 50Hz（跟着 Robot_Task），上位机若需更高频率 IMU，只能改架构。
 * ------------------------------------------------------------- */
void ROBOT_SendDataToRos(void)
{
    //串口发送数据
	static uint8_t comdata[20]; 	

	//加速度 = (ax_acc/32768) * 2G  
	comdata[0] = (u8)( R_Imu.ACC_X >> 8 );  
	comdata[1] = (u8)( R_Imu.ACC_X );
	comdata[2] = (u8)( R_Imu.ACC_Y >> 8 );
	comdata[3] = (u8)( R_Imu.ACC_Y );
	comdata[4] = (u8)( R_Imu.ACC_Z >> 8 );
	comdata[5] = (u8)( R_Imu.ACC_Z );
	
	//陀螺仪角速度 = (ax_gyro/32768) * 500
	comdata[6] = (u8)( R_Imu.GYRO_X >> 8 );
	comdata[7] = (u8)( R_Imu.GYRO_X );
	comdata[8] = (u8)( R_Imu.GYRO_Y >> 8 );
	comdata[9] = (u8)( R_Imu.GYRO_Y );
	comdata[10] = (u8)( R_Imu.GYRO_Z	>> 8 );
	comdata[11] = (u8)( R_Imu.GYRO_Z );
	
	//机器人速度值 单位为m/s，放大1000倍
	comdata[12] = (u8)( R_Vel.RT_IX >> 8 );
	comdata[13] = (u8)( R_Vel.RT_IX );
	comdata[14] = (u8)( R_Vel.RT_IY >> 8 );
	comdata[15] = (u8)( R_Vel.RT_IY );
	comdata[16] = (u8)( R_Vel.RT_IW >> 8 );
	comdata[17] = (u8)( R_Vel.RT_IW );
	
	//电池电压
	comdata[18] = (u8)( R_Bat_Vol >> 8 );
	comdata[19] = (u8)( R_Bat_Vol );

	//TTL串口发送数据
/* @L228 双串口同时发：UART4（TTL 接树莓派）+ UART1（USB 调试）---
 * 两路内容完全相同。UART1 与调试 printf 共用，所以厂商注释说
 * "需要调试时注释掉该句"——否则调试信息里会混入二进制帧。
 * 对我们的意义：烧录后用 USB 线接电脑也能抓 0x10 帧，便于 S7 前期验证，
 * 不必先接树莓派。
 * ------------------------------------------------------------- */
    AX_UART4_SendPacket(comdata, 20, ID_UTX_DATA);		
	
	//USB串口发送数据（与调试串口共用，需要调试时可注释掉该句）
    AX_UART1_SendPacket(comdata, 20, ID_UTX_DATA);	
}


/**
  * @简  述  琐事管理任务
  * @参  数  无
  * @返回值  无
  */
void Trivia_Task(void* parameter)
{	
	//计数变量
	static uint16_t ax_bat_vol_cnt = 0; 
	
	while (1)
	{	
		
		/*****电池管理***********************************/
		
		//采集电池电压
	    R_Bat_Vol = AX_VIN_GetVol_X100();
		
		//调试输出电池电压数据
        //printf("@ %d  \r\n",R_Bat_Vol);		
		
		//电量低于40%
		if(R_Bat_Vol < VBAT_40P)  
		{
			//红灯开始闪烁警示
			AX_LED_Red_Toggle();
			
			//电量低于20%
			if(R_Bat_Vol < VBAT_20P)
			{
				//红灯常亮
				AX_LED_Red_On();
				
				//电量低于10%，关闭系统进入保护状态
/* @L269 低电压保护：设计取舍值得注意 ---------------------------
 * 连续 11 次（约 5.5s）低于 10% -> 挂起 Robot_Task、电机置 0、
 * 然后进入 while(1) 蜂鸣报警 **永不退出**（必须断电重启）。
 * 这是"安全 > 可用性"的选择：宁可锁死也不能让电池过放。
 * 对调试的影响：若上电后车完全不动、蜂鸣器长响，先看电池电压，
 *   不要怀疑代码。VBAT_10P=984 表示 9.84V（3S 锂电 3.28V/cell）。
 * ------------------------------------------------------------- */
				if(R_Bat_Vol < VBAT_10P) 
				{
					//低压时间计数
					ax_bat_vol_cnt++;
					
					//超过10次，进入关闭状态
					if(ax_bat_vol_cnt > 10 )
					{
						//关闭绿灯，红灯常亮
						AX_LED_Green_Off();
						AX_LED_Red_On();
						
						//任务挂起
						vTaskSuspend(Robot_Task_Handle);
						vTaskSuspend(Disp_Task_Handle);
						
						//电机速度设置为0
						AX_MOTOR_A_SetSpeed(0);
						AX_MOTOR_B_SetSpeed(0);  
						
						//清除OLED启动画面显示
						AX_OLED_ClearScreen();  //
						
						//蜂鸣器鸣叫报警
						while(1)
						{	
							//显示机器人停止信息
							AX_OLED_DispStr(0, 3, "     Low power      ", 0);	
							AX_OLED_DispStr(0, 5, "  Robot has stopped ", 0);	
							AX_BEEP_On();
							vTaskDelay(30);
							AX_BEEP_Off();
							
							vTaskDelay(1000);	
							AX_OLED_ClearScreen(); 
							vTaskDelay(1000);						
						}								
					}
				}
				else
				{
					ax_bat_vol_cnt = 0;
				}				
			}
		}
		else
		{
			//红灯关闭
			AX_LED_Red_Off();
		}			
		
		/*****蜂鸣器鸣叫管理***********************************/
		if(ax_beep_ring != 0)
		{
			if(ax_beep_ring == BEEP_SHORT)
			{
				//短鸣叫一声
				AX_BEEP_On();
				vTaskDelay(200); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				ax_beep_ring = 0;
			}
			else //
			{
				//长鸣叫一声
				AX_BEEP_On();
				vTaskDelay(1000); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				ax_beep_ring = 0;				
			}
		}
		
		//LED系统心跳指示
		AX_LED_Green_Toggle();	
		
		
        //循环周期500ms
		vTaskDelay(500); 
	}			
}

/**
  * @简  述  按键处理任务
  * @参  数  无
  * @返回值  无
  */
void Key_Task(void* parameter)
{	
	uint8_t  i;
	//int16_t  temp;
	
	while (1)
	{		
		//按键扫描
		if(AX_KEY_Scan() != 0)
		{
			//软件延时
			vTaskDelay(50);  
			
			//确定按键按下
			if(AX_KEY_Scan() != 0)
			{
				//等待按键抬起
				for(i=0; i<200; i++)
				{

					vTaskDelay(50);
					
					if(AX_KEY_Scan() == 0)
					{
						break;
					}
					
					//按键时长3S时，给出提示音
					if(i == 60)
					{
						AX_BEEP_On();
						vTaskDelay(200);
						AX_BEEP_Off();
					}
				}

				//短按检测,小于1S
				if(i < 20)
				{
					//灯光效果切换
					if(R_Light.M < LEFFECT6)
						R_Light.M++;
					else
						R_Light.M = LEFFECT1;
				}
				
				//中按检测,大于3S，小于10S
				if(i>60 && i<200)
				{
					//暂无
				}				
				
					
				//长按检测，大于10S
				if(i == 200)
				{
					//暂无					
				}
			}
		}
		
		//循环周期
		vTaskDelay(50);    
	}			
}


/**
  * @简  述  灯光处理任务
  * @参  数  无
  * @返回值  无
  */
void Light_Task(void* parameter)
{	

	while (1)
	{	
        //延时30ms，30HZ显示频率		
		vTaskDelay(30); 
		
		//判断前灯是否打开
		if(ax_light_enable != 0)
		{
			//正常显示
			AX_LIGHT_Show();
		}
		else
		{
			//关闭状态
			AX_RGB_SetFullColor(0x00, 0x00, 0x00);
		}
	}			
}


/**
  * @简  述  按键处理任务
  * @参  数  无
  * @返回值  无
  */
void Disp_Task(void* parameter)
{	

	while (1)
	{	
		//延时	
		vTaskDelay(100); 
		
		//显示控制模式
		if(ax_control_mode)
		{
			if      (ax_control_mode == CTL_PS2)    AX_OLED_DispStr(90, 2, "PS2", 0);   //PS2手柄控制
			else if (ax_control_mode == CTL_APP)    AX_OLED_DispStr(90, 2, "APP", 0);   //APP控制
			else if (ax_control_mode == CTL_RMS)    AX_OLED_DispStr(90, 2, "RMS", 0);   //SBUS航模遥控器控制			
		}
		else
		{
			//ROS控制模式
			AX_OLED_DispStr(90, 2, "ROS", 0);
		}
		
		//显示电池电压，陀螺仪Z轴数据,舵机偏角,
		AX_OLED_DispValue(30, 3, (R_Bat_Vol*0.1), 2, 1, 0);
		AX_OLED_DispValue(90, 3, (R_Imu.GYRO_Z ), 6, 0, 0);	
		
		vTaskDelay(100); 
		//显示轮子实时速度
		AX_OLED_DispValue(30, 6, (R_Wheel_A.RT*100), 2, 2, 0);
		AX_OLED_DispValue(90, 6, (R_Wheel_B.RT*100), 2, 2, 0);	
	}			
}

/**
  * @简  述  PS2数据获取任务
  * @参  数  无
  * @返回值  无
  */
void Ps2_Task(void* parameter)
{	

	//用于保存上次时间。调用后系统自动更新
	static portTickType PreviousWakeTime1;

	//设置延时时间20ms，将时间转为节拍数 
	const portTickType TimeIncrement1 = pdMS_TO_TICKS(20);
	
	//获取当前系统时间 
	PreviousWakeTime1 = xTaskGetTickCount();
	
	while(1)
	{
		
		//调用绝对延时函数20ms,执行频率50HZ
		vTaskDelayUntil(&PreviousWakeTime1, TimeIncrement1 );
		
		//读取PS2手柄键值
		AX_PS2_ScanKey(&my_joystick);
		
		//不在PS2控制模式下
		if(ax_control_mode != CTL_PS2)
		{
			//判断是否开启PS2手柄控制
			//START按键被按下后，左边摇杆上推，进入PS2控制模式
			if((my_joystick.btn1 == PS2_BT1_START) && (my_joystick.LJoy_UD == 0x00))
			{
				//切换到PS2模式
				ax_control_mode = CTL_PS2;	

				//执行蜂鸣器鸣叫提示
				ax_beep_ring = BEEP_SHORT;
			}
		}
		
//		//打印手柄键值
//		printf("MODE:%2x BTN1:%2x BTN2:%2x RJOY_LR:%2x RJOY_UD:%2x LJOY_LR:%2x LJOY_UD:%2x\r\n",
//		my_joystick.mode, my_joystick.btn1, my_joystick.btn2, 
//		my_joystick.RJoy_LR, my_joystick.RJoy_UD, my_joystick.LJoy_LR, my_joystick.LJoy_UD);
	}
}

/******************* (C) 版权 2023 XTARK **************************************/

