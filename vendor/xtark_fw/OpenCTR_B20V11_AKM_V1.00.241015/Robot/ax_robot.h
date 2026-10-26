/* @HEAD =====================================================================
 * 【研读标注版】代码未改。原始文件见 xtark_fw_original_gbk/Robot/ax_robot.h。
 *
 * 本文件是"全系统的常量与契约"：几何参数、速度限幅、帧号、控制模式、告警阈值。
 * 读它比读任何 .c 都重要，因为**所有单位约定的源头在这里**。
 *
 * ★ 几何参数三方对照（S8 必须实测裁决）★
 *                      固件(本文件)   厂商手册     仿真 URDF(当前)
 *   轴距 (前后轮)        0.146 m       0.112 m      0.112 m
 *   轮距 (左右轮)        0.160 m       0.164 m      0.164 m
 *   轮子直径             0.065 m       0.065 m      0.065 m
 *   最小转弯半径         0.35 m        0.55 m       未显式设置
 *   编码器分辨率         1560          1560          —
 * 固件注释自称 R_min = L·cot30° − W/2 ≈ 0.173 m，与宏值 0.35 也不符 ——
 * 说明这些常量被改过但注释没跟上。实测方法：卷尺量 + 满舵画圆。
 *
 * ★ 单位总表 ★
 *   R_VX_LIMIT 1500 -> 1.5 m/s      R_VW_LIMIT 6280 -> 6.28 rad/s
 *   AKM_WHEEL_SCALE = π·D·50/1560 ≈ 0.006545 m/s per count/20ms
 *   ax_akm_angle 0.001 rad   TB_IX 0.001 m/s   TB_IW 0.001 rad/s
 *   IMU acc/gyro 原始 LSB（量程 ±2g / ±500dps）
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_ROBOT_H
#define __AX_ROBOT_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
 
//外设相关头文件
#include "ax_sys.h"      //系统设置
#include "ax_delay.h"    //软件延时
#include "ax_led.h"      //LED灯控制
#include "ax_beep.h"     //蜂鸣器控制
#include "ax_vin.h"      //输入电压检测
#include "ax_key.h"      //按键检测 

#include "ax_uart1.h"    //调试串口
#include "ax_uart2.h"    //蓝牙串口
#include "ax_uart4.h"    //TTL串口
#include "ax_uart5.h"    //预留串口

#include "ax_servo.h"    //舵机控制
#include "ax_motor.h"    //直流电机调速控制
#include "ax_encoder.h"  //编码器控制

#include "ax_mpu6050.h"  //IMU加速度陀螺仪测量

#include "ax_rgb.h"      //RGB彩灯控制
#include "ax_sbus.h"     //SBUS航模遥控器控制
#include "ax_oled.h"     //OLED显示
#include "ax_ps2.h"      //PS2手柄

#include "ax_akm.h"      //阿克曼机器人处理函数


//机器人轮子速度数据结构体
typedef struct  
{
	double  RT;       //车轮实时速度，单位m/s
	float  TG;       //车轮目标速度，单位m/s
	short  PWM;      //车轮PWM控制速度
	
}ROBOT_Wheel;

//机器人速度结构体
typedef struct  
{
	short  RT_IX;     //实时X轴速度（16位整数）
	short  RT_IY;     //实时Y轴速度（16位整数）
	short  RT_IW;     //实时Yaw旋转轴速度（16位整数）
	
	short  TG_IX;     //目标X轴速度（16位整数）
	short  TG_IY;     //目标Y轴速度（16位整数）
	short  TG_IW;     //目标Yaw旋转轴速度（16位整数）
	
	float  RT_FX;     //实时X轴速度（浮点）
	float  RT_FY;     //实时Y轴速度（浮点）
	float  RT_FW;     //实时Yaw旋转轴速度（浮点）
	
	float  TG_FX;     //目标X轴速度（浮点）
	float  TG_FY;     //目标Y轴速度（浮点）
	float  TG_FW;     //目标Yaw旋转轴速度（浮点）
	
}ROBOT_Velocity;


//机器人IMU数据
typedef struct  
{
	short  ACC_X;     //X轴
	short  ACC_Y;     //Y轴
	short  ACC_Z;     //Z轴
	
	short  GYRO_X;     //X轴
	short  GYRO_Y;     //Y轴
	short  GYRO_Z;     //Z轴
	
}ROBOT_Imu;


//RGB灯效数据
typedef struct  
{
	uint8_t  M;    //灯效主模式
	uint8_t  S;    //灯效从模式
	uint8_t  T;    //灯效时间参数
	
	uint8_t  R;     //灯效颜色 R
	uint8_t  G;     //灯效颜色 G
	uint8_t  B;     //灯效颜色 B
	
}ROBOT_Light;

//杂类
#define  PI           3.1416     //圆周率PI
#define  SQRT3        1.732      //3开平方
#define  PID_RATE     50         //PID频率

//机器人软件版本
#define  ROBOT_FW_VER   "V1.00"

/******机器人型号***********************************/
//包含R5/R10
#define ROBOT_R5

//3S锂电池电量电压关系
#define  VBAT_40P    1065     //电池40%电压
#define  VBAT_20P    1012     //电池20%电压
#define  VBAT_10P    984      //电池10%电压

/******机器人参数*************************************/

/* @L150 R5 参数块：每个数都要能溯源 -----------------------------
 * 轴距 0.146 = 前后轮中心距（固件实际使用值，参与所有 IK 计算）
 * 轮距 0.160 = 左右轮中心距（只用于差速分配和 FK 的 yaw 计算）
 * 直径 0.065 = 轮子直径（注意：不是半径！URDF 里要用 0.0325）
 * 1560 = 13线编码器 × 减速比30 × 4倍频
 * 0.35 = 最小转弯半径（钳位用，防止舵机打死）
 * SCALE = π·D·50/1560：把"20ms 脉冲数"直接换成 m/s，推导见 ax_kinematics.c L73
 * 提醒：R10 块(159~166)是另一款车，改参数时不要改错块。
 * ------------------------------------------------------------- */
#ifdef  ROBOT_R5
#define  AKM_WHEEL_BASE           0.160	     //轮距，左右轮的距离
#define  AKM_ACLE_BASE            0.146f     //轴距，前后轮的距离
#define  AKM_WHEEL_DIAMETER	      0.065		 //轮子直径
#define  AKM_WHEEL_RESOLUTION     1560.0     //编码器分辨率(13线),减速比30,13x30x4=1560
#define  AKM_TURN_R_MINI          0.35f      //最小转弯半径( L*cot30-W/2)
#define  AKM_WHEEL_SCALE          (PI*AKM_WHEEL_DIAMETER*PID_RATE/AKM_WHEEL_RESOLUTION)  //轮子速度m/s与编码器转换系数
#endif

#ifdef  ROBOT_R10
#define  AKM_WHEEL_BASE           0.165	     //轮距，左右轮的距离
#define  AKM_ACLE_BASE            0.156f     //轴距，前后轮的距离
#define  AKM_WHEEL_DIAMETER	      0.075		 //轮子直径
#define  AKM_WHEEL_RESOLUTION     1560.0     //编码器分辨率(13线),减速比30,13x30x4=1560
#define  AKM_TURN_R_MINI          0.35f      //最小转弯半径( L*cot30-W/2)
#define  AKM_WHEEL_SCALE          (PI*AKM_WHEEL_DIAMETER*PID_RATE/AKM_WHEEL_RESOLUTION)  //轮子速度m/s与编码器转换系数
#endif

/******机器人通信协议*************************************/

/* @L168 帧号定义：B20 实际只认前两项 ---------------------------
 *   0x10 STM32->ROS 综合数据（20 字节）
 *   0x50 ROS->STM32 速度（6 字节 vx,vy,vw）
 *   0x60 ROS->STM32 云台角度（4 字节，本项目不用）
 * 更老的 robot.h 里还有 0x51(IMU校准)/0x5f(舵机零偏)，
 * 但 B20 的 ax_uart4.c 没实现 —— 别照抄老上位机代码。
 * ------------------------------------------------------------- */
//串口通信帧头定义
#define  ID_UTX_DATA    0x10    //发送的综合数据
#define  ID_URX_VEL     0x50    //接收的速度数据
#define  ID_URX_ANGLE   0x60    //接收的机械臂角度数据

//蓝牙APP通信帧头定义
#define  ID_BLERX_CM      0x30    //APP蓝牙发送 蓝牙连接指令
#define  ID_BLERX_YG      0x31    //APP蓝牙发送 摇杆模式 控制指令
#define  ID_BLERX_SB      0x32    //APP蓝牙发送 手柄模式 控制指令
#define  ID_BLERX_ZL      0x33    //APP蓝牙发送 重力模式 控制指令
#define  ID_BLERX_TK      0x34    //APP蓝牙发送 坦克模式 控制指令
#define  ID_BLERX_AM      0x3A    //APP蓝牙发送 机械臂模式 控制指令
#define  ID_BLERX_LG      0x41    //APP蓝牙发送 灯光控制指令
#define  ID_BLERX_LS      0x42    //APP蓝牙发送 保存灯光效果指令

//机器人速度限制
/* @L186 速度限幅 -------------------------------------------------
 * 1500 = 1.5 m/s，6280 = 6.28 rad/s（约 1 圈/秒）。
 * 这两个值决定了"上位机发再大也没用"的上限。你们当前项目限速 1.0 m/s，
 * 所以实车不会碰到这个钳位；它是最后一道保险，不是工作点。
 * 注意：转向角没有对应的 R_ANGLE_LIMIT（ax_kinematics.c 里被注释了），
 * 只在 ax_akm.c 通过最小半径间接限制。
 * ------------------------------------------------------------- */
#define R_VX_LIMIT  1500   //X轴速度限值 m/s*1000
#define R_VY_LIMIT  1200   //Y轴速度限值 m/s*1000
#define R_VW_LIMIT  6280   //W旋转角速度限值 rad/s*1000

//灯光模式
#define  LEFFECT1    0x01    //单色模式  
#define  LEFFECT2    0x02    //呼吸模式  
#define  LEFFECT3    0x03    //彩色模式  
#define  LEFFECT4    0x04    //警灯模式 
#define  LEFFECT5    0x05    //定义模式  
#define  LEFFECT6    0x06    //定义模式

//机器人控制方式
#define  CTL_ROS    0x00    //ROS控制，包含串口和CAN
#define  CTL_PS2    0x01    //PS2手柄控制
#define  CTL_APP    0x02    //APP控制
#define  CTL_RMS    0x03    //SBUS航模遥控器控制	

//蜂鸣器鸣长短
#define  BEEP_SHORT   0x01    //蜂鸣器短鸣叫一声(200ms)
#define  BEEP_LONG    0x02    //蜂鸣器长鸣叫一声(1000ms)

//云台关节角度限制，单位角度放大10倍
#define JOINTA_UP_LIMIT    900
#define JOINTA_LOW_LIMIT  -900
#define JOINTB_UP_LIMIT    900
#define JOINTB_LOW_LIMIT  -900

//阿克曼转向舵机零偏修正角 
#define AKM_ANGLE_OFFSET      0     //S3舵机

//舵机云台零偏修正角 
#define JOINTA_ANGLE_OFFSET   0     //S1舵机，水平关节
#define JOINTB_ANGLE_OFFSET   0     //S2舵机，垂直关节

//机器人关键全局变量
extern  ROBOT_Velocity  R_Vel; //机器人速度数据
extern  ROBOT_Wheel  R_Wheel_A,R_Wheel_B,R_Wheel_C,R_Wheel_D; //机器人轮子数据
extern  ROBOT_Imu  R_Imu; //机器人IMU数据
extern  ROBOT_Light  R_Light; //机器人RGB数据
extern  uint16_t R_Bat_Vol;  //机器人电池电压数据

//全局变量
extern uint8_t ax_robot_move_enable;

extern int16_t ax_imu_acc_data[3];  
extern int16_t ax_imu_gyro_data[3];
extern int16_t ax_imu_gyro_offset[3];   

extern int16_t ax_motor_kp;  
extern int16_t ax_motor_kd; 

extern uint8_t ax_light_enable;
extern uint8_t ax_beep_ring;   
extern uint8_t ax_control_mode;

//PS2手柄键值结构体
extern JOYSTICK_TypeDef my_joystick;  

//阿克曼机器人
extern int16_t ax_akm_angle;

//舵机云台关节
extern int16_t ax_joint_angle[2];

//任务句柄
extern TaskHandle_t Robot_Task_Handle;
extern TaskHandle_t Key_Task_Handle;
extern TaskHandle_t Disp_Task_Handle;
extern TaskHandle_t Trivia_Task_Handle;


#endif

/******************* (C) 版权 2023 XTARK **************************************/
