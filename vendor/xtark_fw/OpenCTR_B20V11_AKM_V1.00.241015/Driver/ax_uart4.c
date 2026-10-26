/* @HEAD =====================================================================
 * 【研读标注版】代码未改。原始文件见 xtark_fw_original_gbk/Driver/ax_uart4.c。
 *
 * ★ 这是 B20 固件里真正收发上位机数据的地方 ★
 *   main.c:118  AX_UART4_Init(230400);   // 230400 8N1，PC10=TX / PC11=RX
 *   收：0x50 速度帧 -> R_Vel.TG_I* + AX_AKM_WToAngle()
 *       0x60 云台角度帧 -> ax_joint_angle[2]
 *   发：ax_robot.c:228  AX_UART4_SendPacket(comdata, 20, 0x10)  50Hz
 * 对照实现：Driver/ax_uart_pi.c（更老的树莓派版，本工程未调用，仅作镜像参考）
 *
 * X-Protocol 帧格式（与 protocol.py 必须逐字节互为镜像）：
 *   AA 55 | LEN | ID | DATA... | CK
 *   LEN = 数据长度 + 5（含帧头2字节、LEN自身、ID、CK）
 *   CK  = AA+55+LEN+ID+DATA 全部累加和的低 8 位（注意：LEN 也计入）
 *   多字节数据高位在前（大端）
 *   例：AA 55 0B 01 03 E8 FC 18 00 0A 14 -> 1000, -1000, 10
 *
 * 安全相关（S6 要改的地方都在这里）：
 *   S1 LEN 无上限校验 -> 缓冲区溢出风险（见 L149 标注）
 *   S2 无通信看门狗 -> 0x50 停发后车继续按最后一次目标速度跑（见 L165 标注）
 *   S3 每个合法帧强制切到 CTL_ROS，会抢走 PS2/SBUS 控制权
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
  * @内  容  TTL串口通信
  *
  ******************************************************************************
  * @说  明
  *
  * 1.可以使用具有X-Protocol协议进行数据发送
  * 2.开启UART的串口数据接收功能，使用中断方式，X-Protocol协议通信
  *
  * X-Protocol协议介绍（变帧长）
  * 帧定义：AA 55 | 0B  | 01  | 03 E8 FC 18 00 0A | 14
  *        帧头   帧长   帧码  数据                校验和
  * 帧  头：双帧头，抗干扰强
  * 帧  长：根据数据长度设定
  * 帧  码：用户根据功能设定，标识帧的唯一性
  * 数  据：高位在前，长度可变，内容自由组合8位，16位，32位数据
  * 校验和：前面数据累加和的低8位
  * 帧示例：( AA 55 0B 01 03 E8 FC 18 00 0A 14 ) 内容：1000，-1000,10,
  * 
  ******************************************************************************
  */

#include "ax_uart4.h"
#include <stdio.h>
#include "ax_robot.h"

/* @L45 接收状态全局变量 -----------------------------------------
 * 中断里写、任务里读，没有 volatile 也没有临界区。
 * 在 Cortex-M3 上单字节读写是原子的，所以目前不会炸；
 * 但如果之后改成多字节结构体共享，必须加 volatile 或关中断保护。
 * uart4_rx_ok 在本文件里从未被置 1 -> AX_UART4_GetData 是死代码（见 L200）。
 * ------------------------------------------------------------- */
static uint8_t uart4_rx_ok = 0; //接收成功标志
static uint8_t uart4_rx_con=0;       //接收计数器
static uint8_t uart4_rx_checksum;    //帧头部分校验和
static uint8_t uart4_rx_buf[40];     //接收缓冲，数据内容小于等于32Byte
static uint8_t uart4_tx_buf[40];     //发送缓冲

/**
  * @简  述  串口初始化
  * @参  数  baud： 波特率设置
  * @返回值	 无
  */
/* @L56 串口初始化：UART4, PC10/PC11 ----------------------------
 * 实际波特率由 main.c:118 传入 230400（必须与上位机 230400 一致，
 * serial_port.py 默认值已对齐；改波特率要两边同时改）。
 * NVIC 抢占优先级 2、子优先级 1 —— 比 Robot_Task 的 50Hz 控制环"硬"，
 * 所以串口接收不会因为控制计算而丢字节。这是正确的中断优先级设计。
 * ------------------------------------------------------------- */
void AX_UART4_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//**USART配置******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //打开串口GPIO的时钟

	//将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);  //打开串口外设的时钟

	//配置USART参数
	USART_InitStructure.USART_BaudRate = baud;		//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	//配置USART为中断源
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢断优先级	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能中断
	NVIC_Init(&NVIC_InitStructure);//初始化配置NVIC

	//使能串口接收中断
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	//使能 USART， 配置完毕
	USART_Cmd(UART4, ENABLE);
}

/**
  * @简  述  串口中断服务程序
  * @参  数  无
  * @返回值  无
  */
/* @L106 中断服务程序：变长帧状态机 -----------------------------
 * uart4_rx_con 是状态机的"当前索引"，同时兼职状态：
 *   0      等 AA
 *   1      已收 AA，等 55
 *   2      已收 55，等 LEN
 *   3..    正在收 ID/DATA，收到第 LEN-1 个字节后，下一个就是 CK
 * 设计要点：用"已收字节数"驱动状态，不需要显式状态枚举；
 *   一旦校验失败不做任何处理，直接回到 0 等下一个 AA —— 靠双帧头自同步。
 * 局限：LEN 恰好是最后一个到达的，无法在收数据前做合法性检查（见 L149）。
 * ------------------------------------------------------------- */
void UART4_IRQHandler(void)                	
{
	uint8_t Res;
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断
	{
	    //printf("Get Data!\r\n");
		Res =USART_ReceiveData(UART4);	
	
/* @L115 帧头 + 长度阶段 ----------------------------------------
 * con==1 时收到非 55：直接 con=0（注意：没有回退检查这个字节是不是新的 AA，
 * 若数据流里出现 "AA AA 55"，会丢掉第一个 AA，第二帧才同步上 —— 可接受）。
 * ------------------------------------------------------------- */
		if(uart4_rx_con < 3)    //==接收帧头 + 长度
		{
			if(uart4_rx_con == 0)  //接收帧头1 0xAA
			{
				if(Res == 0xAA)
				{
					uart4_rx_buf[0] = Res;
					uart4_rx_con = 1;					
				}
				else
				{
					
				}
			}else if(uart4_rx_con == 1) //接收帧头2 0x55
			{
				if(Res == 0x55)
				{
					uart4_rx_buf[1] = Res;
					uart4_rx_con = 2;
				}
				else
				{
					uart4_rx_con = 0;						
				}				
			}
			else  //接收数据长度
			{
				uart4_rx_buf[2] = Res;
				uart4_rx_con = 3;
/* @L144 校验和从这里开始累计 -----------------------------------
 * 起始值 = AA+55+LEN，之后每收一个字节累加，直到 CK 前。
 * 也就是说校验覆盖 AA,55,LEN,ID,DATA（含长度字节），
 * 这与 protocol.py 的 pack_frame 必须完全一致，否则每帧都被丢弃。
 * ------------------------------------------------------------- */
				uart4_rx_checksum = (0xAA+0x55) + Res;	//计算校验和
			}
		}
		else    //==接收数据
		{
/* @L149 ★ 缺陷：LEN 没有上限检查 -> 可越界写（S1）★ -----------
 * uart4_rx_buf 只有 40 字节，而 LEN 是 uint8（最大 255）。
 * 若线路噪声造成一个 "AA 55 F0 ..." 的假帧头，LEN=240，
 * 本分支会把后续 240 个字节依次写入 buf[3..242] -> 越界改写相邻全局变量。
 * 校验和只能在收完之后才否决，越界已经发生。
 * 另外 LEN=0 时 buf[2]-1 = 255（uint8 回绕），状态机会一直收到下一个 CK，
 * 期间所有字节被吞掉，直到恰好校验对上。
 * 修复（S6 一并做）：在 L144 处加 if(Res < 5 || Res > 36){ con=0; return; }
 * 顺带把 buf 扩到 64 并留出余量。
 * ------------------------------------------------------------- */
			if(uart4_rx_con < (uart4_rx_buf[2]-1) )
			{
				uart4_rx_buf[uart4_rx_con] = Res;
				uart4_rx_con++;
				uart4_rx_checksum = uart4_rx_checksum + Res;					
			}
			else    //判断最后1位
			{
				
				//接收完成，恢复初始状态
				uart4_rx_con = 0;	
				
				//数据校验
				if( Res == uart4_rx_checksum )  //校验正确
				{	
					//设置为ROS控制模式
/* @L165 ★★ 安全核心：每帧都强制切 ROS 模式，且没有失效保护（S2）★★
 * 这意味着两件事：
 *   1) 上位机只要发一帧合法 0x50，PS2/SBUS 的控制权立即被夺走
 *      （实车调试时"手柄突然失灵"就是这个原因，反过来也成立：断电重启后
 *       默认 ax_control_mode=CTL_ROS，不发帧车也不会动，因为目标速度初值为 0）；
 *   2) 没有任何超时判断 —— 0x50 停止到达后，R_Vel.TG_IX/IW 保留最后一次的值，
 *      Robot_Task 继续按它算 PWM，车会一直跑下去。
 *      树莓派死机、USB 掉线、航模遥控接管失败，都会导致"失控直行"。
 * 兜底现状：上位机 serial_bridge.py 有 0.5s 看门狗（发 0 速度）。但这只是
 *   软件兜底 —— 进程被杀或系统卡死就失效。真正可靠的方案是下位机自己做：
 *   记录最后一帧的 tick，超过 300ms 未更新就强制目标速度清零（S6 第一件事）。
 * ------------------------------------------------------------- */
					ax_control_mode = CTL_ROS;
					
					//速度控制帧
/* @L168 0x50 速度帧落点（与 protocol.py 的对照点）--------------
 *   buf[4..5] -> TG_IX  (0.001 m/s，大端)
 *   buf[6..7] -> TG_IY  (0.001 m/s，阿克曼恒为 0，仅占位)
 *   buf[8..9] -> TG_IW  (0.001 rad/s)
 * 最后一行把 (vx, vw) 立刻折算成前轮角 —— 注意：转角只在收到帧的那一刻更新，
 * 不参与 50Hz 控制环的重算。所以上位机发的 vw 若不变而 vx 变了
 * （例如 tracker 让车减速），前轮角会按"新的 vx 与旧的 vw"重算，
 * 曲率半径随之变化 —— 这是"减速时车头突然偏一下"的机制来源。
 * ------------------------------------------------------------- */
					if(uart4_rx_buf[3] == ID_URX_VEL)
					{
						R_Vel.TG_IX = (int16_t)((uart4_rx_buf[4]<<8) | uart4_rx_buf[5]);
						R_Vel.TG_IY = (int16_t)((uart4_rx_buf[6]<<8) | uart4_rx_buf[7]);
						R_Vel.TG_IW = (int16_t)((uart4_rx_buf[8]<<8) | uart4_rx_buf[9]);
						
						//阿克曼机器人，根据矢量速度计算前轮转向角
						ax_akm_angle = AX_AKM_WToAngle(R_Vel.TG_IX, R_Vel.TG_IW);
					}
					else
					{
						//舵机角度控制帧
/* @L180 0x60 云台角度帧 ----------------------------------------
 * 只控制 S1/S2 云台，与底盘无关。本项目的上位机不使用，可忽略。
 * 注意：B20 的这份固件只认 0x50 和 0x60；
 * ax_uart_pi.c 里的 0x51（陀螺校准）和 0x5f（舵机零偏）在这条路径上
 * **不存在**，别照抄老代码去发这两个 ID。
 * ------------------------------------------------------------- */
						if(uart4_rx_buf[3] == ID_URX_ANGLE)
						{
							ax_joint_angle[0] = (int16_t)((uart4_rx_buf[4]<<8) | uart4_rx_buf[5]);
							ax_joint_angle[1] = (int16_t)((uart4_rx_buf[6]<<8) | uart4_rx_buf[7]);
						}				
					}		
				}	
				
			}
		}
			
      USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	} 
}

/**
  * @简  述  获取接收的数据
  * @参  数  *pbuf：接收数据指针,第1个字节为帧编码，后面为数据
  * @返回值	 0-无数据接收，other-需要读取的数据字节个数
  */
/* @L200 死代码：uart4_rx_ok 从未被置 1 ------------------------
 * 保守推断：作者原本想"中断只拼帧、任务里解析"，后来直接在中断里解析了，
 * 这个接口没删。留着无害，但读代码时别把它当成数据通路。
 * ------------------------------------------------------------- */
uint8_t AX_UART4_GetData(uint8_t *pbuf)
{
	uint8_t cnt,i;
	
	if(uart4_rx_ok != 0)
	{
		cnt = uart4_rx_buf[2]-4;
		
		for(i=0; i<cnt; i++)
		{
			*(pbuf+i) = uart4_rx_buf[3+i];
		}
		
		uart4_rx_ok = 0;
		return cnt;
	}
	else
	{
		return 0;
	}	
}

/**
  * @简  述  发送数据（X-Protocol协议）
  * @参  数  *pbuf：发送数据指针
  *          len：发送数据长度个数，≤27 (32-5)
  *          num：帧号，帧编码
  * @返回值	 无
  */
/* @L229 发送：查询方式逐字节阻塞发送 ---------------------------
 * 参数 len 是"数据区"长度，帧总长 = len+5。
 * 缺陷：判断条件是 len<=50，而 uart4_tx_buf 只有 40 字节 ——>
 *       当 len 在 36~50 之间时越界写 1~11 字节。
 *       目前唯一的调用者（ax_robot.c:228）传 len=20，安全；改协议时要先修这里。
 * 时序：20 数据字节 -> 25 字节帧 @230400 8N1 = 25×10/230400 ≈ 1.09ms，
 *       以 50Hz 调用占比约 5%，可以接受；若加长帧需重新评估。
 * 发送采用阻塞查询（等 TC 标志），且与中断共用 UART4 —— 中断优先级更高，
 * 接收不会被阻塞发送卡住。
 * ------------------------------------------------------------- */
void AX_UART4_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
    uint8_t tx_checksum = 0;//发送校验和
	
	if(len <= 50)
	{
		/******获取数据******/
		uart4_tx_buf[0] = 0xAA;    //帧头
		uart4_tx_buf[1] = 0x55;    //
		uart4_tx_buf[2] = len+5;  //根据输出长度计算帧长度
		uart4_tx_buf[3] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			uart4_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******计算校验和******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart4_tx_buf[i];
		}
		uart4_tx_buf[i] = tx_checksum;
		
		
		/******发送数据******/	
		cnt = 5+len;
		
		//查询传输方式
		for(i=0; i<cnt; i++)
		{
			USART_SendData(UART4, uart4_tx_buf[i]);
			while(USART_GetFlagStatus(UART4,USART_FLAG_TC) != SET);
		}	
	}
}



/******************* (C) 版权 2023 XTARK **************************************/
