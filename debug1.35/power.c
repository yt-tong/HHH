/*******************************************************************************
 * 预警电源控制器下位机软件
 *
 * debug1.00  20210122  一号厂房
 * 软件构架
 * 
 * debug1.01  20210123  一号厂房
 * CAN总线接口调试
 * 
 * debug1.02  20210125  一号厂房
 * 数据组帧处理
 * 
 * debug1.03  20210126  一号厂房
 * GF6实验板CAN总线协议调试 
 * 
 * debug1.04-1.05  20210207  一号厂房
 * 总线指令、上注数据块、RAM地址下传
 * a. 修订切区标志为独立对应关系
 * b. CAN总线协议调试
 * 
 * debug1.06  20210216  一号厂房
 * A/B总线一致性修盖 
 * 
 * debug1.07  20210220  一号厂房
 * 修改500K  
 * 
 * debug1.08  20210222  一号厂房
 * 修改重要数据帧格式  
 * 
 * debug1.09  20210223  一号厂房
 * 修改指令格式修改 
 *
 * debug1.10  20210223  一号厂房
 * 修改状态量为0/1，增加状态解析
 * 更新服务请求时间间隔
 * 
 * debug1.11 20210224  一号厂房
 * 增加AB总线遥测信息
 * 上注程序测试
 * 
 * debug1.12  20210225  一号厂房
 * 修订遥测信息  太阳阵遥测5-9、8-12
 * 修订系数
 * 增加拟合电流等的负值保护设计
 * 
 * debug1.13	20210312  一号厂房
 * 协议更改后联试
 * 
 * debug1.14-debug1.15	20210312/15  home 
 * 协议更改后联试
 * 重要数据、过滤包、RAM设置，服务请求
 * 
 * debug1.16	20210316 塔楼十楼调试间
 * 修正分流状态，快包长度等问题
 * 
 * debug1.17~debug1.18	20210316 塔楼十楼调试间
 * 增加422均衡器数据接收设置
 * 均衡指令发送
 *  
 * debug1.19	20210317 塔楼十楼调试间
 * 增加单体物理值排序
 * 增加电量计功能模块
 * 
 * debug1.20-debug1.21	20210317 home
 * 增加上电初始化指令设置
 * 参数上注接收处理
 * 重要数据发送、接收处理
 * 
 * debug1.22	20210318 塔楼十楼调试间  
 * 健康字检测
 * 蓄电池组过充保护、单体过充保护
 * 
 * debug1.23	20210318 home
 * 多余代码删除
 * 
 * debug1.24	20210319  塔楼十楼调试间  
 * 健康字监测状态、自检等
 * 
 * debug1.25	20210319  塔楼十楼调试间  测试版本V1
 * 修正均衡器状态判读顺序bug
 * 
 * debug1.26	20210320  塔楼十楼调试间  测试版本V1
 * 修正重要数据应答TITLE
 * 增加总线AB标识
 * 修正过滤包基准电压、修正取值错误
 * 
 * debug1.27	20210322  塔楼十楼调试间  测试版本V1
 * 修正指令均衡指令设置、均衡状态<< bug 修正
 * 增加间接指令均衡控制输出（通断）
 * 修正内存地址设置应答判读bug  CANRXD-B  / A000 -> C000   
 * 增加均衡禁止时发送全部均衡断开指令
 * 
 * debug1.28	20210322  HOME
 * 修改单体充电保护默认为禁止状态 -- singleProtectChargeEn=FALSE
 * 修改蓄电池充电保护默认为禁止状态 -- overChargeProtectEn=FALSE  
 * 增加系统时间维护
 * 增加自主指令记录功能
 * 上注参数范围判断
 * 修正充电电流1、2计算bug
 * 
 * debug1.29	20210322    塔楼十楼调试间 
 * 修改中断函数调用using bug 
 * 修正healthStateWord2组状态字 清零 bug
 * 修正equVRefPhy基准电压判断逻辑bug
 * 增加均衡器采集等待时间约 140ms
 * 
 * debug1.30-deubg1.31	20210324    塔楼十楼调试间 
 * 修改单体过充保护、蓄电池组过充保护模块 
 * 修正电量上注范围判断bug
 * 修正屏蔽字设置
 * 修正均衡采集数量 40字节
 
 
 * debug1.32	20210324    塔楼十楼调试间 
 * 代码优化
 * 
 
 
 * 注意： 
 * 1. 健康字设置标志， 16为4秒后清除标志 
 
*******************************************************************************/

#include <reg52.h>
#include <absacc.h>
#include <intrins.h>
#include <math.h>

#define TRUE 0xAA  // 定义真值
#define FALSE 0x55 // 定义假值

#define SILENT_TIME 4 // 最大总线沉寂时间(s) -- 根据通讯协议或数据约定确认

#define VERSION 131 // 内部版本

#define BUSA_ADDR 0xA000 // 总线A地址
#define BUSB_ADDR 0xC000 // 总线B地址

#define CAN_ACR 0x9B // 接收码寄存器
#define CAN_AMR 0xD4 // 接收屏蔽寄存器

#define ONOFF_ADDR XBYTE[0xF800] // ONOFF指令地址
#define AN_ADDR XBYTE[0xF400]	 // 模拟量地址

#define AD574S XBYTE[0xFC00] // AD启动转换
#define AD574H XBYTE[0xFC02] // AD转换高地址
#define AD574L XBYTE[0xFC03] // AD转换低地址

#define DA_A_CUR XBYTE[0xE400] // A组电流
#define DA_A_VOL XBYTE[0xE800] // A组电压
#define DA_B_CUR XBYTE[0xEC00] // B组电流
#define DA_B_VOL XBYTE[0xF000] // B组电流

#define TXD422BUFF XBYTE[0xE000] // RS422发送
#define RXD422BUFF XBYTE[0xE080] // RS422的A接收

#define K1 0.025 // 充分模块1太阳电池阵电流系数   	 811所 联试标定V1  20210225
#define b1 0
#define K2 0.025 // 充分模块2太阳电池阵电流系数   	 811所 联试标定V1  20210225
#define b2 0
#define K3 0.025 // 充分模块3太阳电池阵电流系数   	 811所 联试标定V1  20210225
#define b3 0
#define K4 0.025 // 充分模块4太阳电池阵电流系数   	 811所 联试标定V1  20210225
#define b4 0

#define K5 0.025 // 母线电流1系数   				811所 联试标定V1  20210225
#define b5 0
#define K6 0.025 // 母线电流2系数   				811所 联试标定V1  20210225
#define b6 0
#define K7 0.025 // 母线电流3系数   				811所 联试标定V1  20210225
#define b7 0
#define K8 0.025 // 母线电流4系数   				811所 联试标定V1  20210225
#define b8 0

#define K9 0.025 // 充电电流1系数   				811所 联试标定V1  20210225
#define b9 0.0
#define K10 0.025 // 充电电流2系数  				811所 联试标定V1  20210225
#define b10 0.0

#define K11 0.025 // 放电电流1系数   				811所 联试标定V1  20210225
#define b11 0.0
#define K12 0.025 // 放电电流2系数   				811所 联试标定V1  20210225
#define b12 0.0

#define K13 0.0025 // 均衡器基准5V电压系数  -- 均衡器  V = A*1000/4096
#define b13 0.00

#define K14 0.025 // 蓄电池主份电压（下位机采集）
#define b14 0.0

#define K16 0.025 // 蓄电池主份电压（均衡器采集）  V = A*1000/4096
#define b16 0.0
#define K17 0.025 // 蓄电池备份电压（均衡器采集）  V = A*1000/4096
#define b17 0.0

#define K18 0.025 // 充电阵A1电流系数
#define b18 0.0
#define K19 0.025 // 充电阵A2电流系数
#define b19 0.0
#define K20 0.025 // 充电阵A3电流系数
#define b20 0.0
#define K21 0.025 // 充电阵A4电流系数
#define b21 0.0
#define K22 0.025 // 充电阵A5电流系数
#define b22 0.0

#define K25 0.025 // 充电阵B1电流系数
#define b25 0.0
#define K26 0.025 // 充电阵B2电流系数
#define b26 0.0
#define K27 0.025 // 充电阵B3电流系数
#define b27 0.0
#define K28 0.025 // 充电阵B4电流系数
#define b28 0.0
#define K29 0.025 // 充电阵B5电流系数
#define b29 0.0

#define K23 0.025 // MEA系数
#define b23 0.0
#define K24 0.025 // 母线电压系数
#define b24 0.0

#define Ka1 0.0025 // 单体1电压系数  -- 均衡器  V = A*1000/4096
#define ba1 0
#define Ka2 0.0025 // 单体2电压系数  -- 均衡器  V = A*1000/4096
#define ba2 0
#define Ka3 0.0025 // 单体3电压系数  -- 均衡器  V = A*1000/4096
#define ba3 0
#define Ka4 0.0025 // 单体4电压系数  -- 均衡器  V = A*1000/4096
#define ba4 0
#define Ka5 0.0025 // 单体5电压系数  -- 均衡器  V = A*1000/4096
#define ba5 0
#define Ka6 0.0025 // 单体6电压系数  -- 均衡器  V = A*1000/4096
#define ba6 0
#define Ka7 0.0025 // 单体7电压系数  -- 均衡器  V = A*1000/4096
#define ba7 0
#define Ka8 0.0025 // 单体8电压系数  -- 均衡器  V = A*1000/4096
#define ba8 0
#define Ka9 0.0025 // 单体9电压系数  -- 均衡器  V = A*1000/4096
#define ba9 0

#define Ks1 0.025 // 太阳阵1电压 系数
#define bs1 0
#define Ks2 0.025 // 太阳阵2电压 系数
#define bs2 0
#define Ks3 0.025 // 太阳阵3电压 系数
#define bs3 0
#define Ks4 0.025 // 太阳阵4电压 系数
#define bs4 0
#define Ks5 0.025 // 太阳阵5电压 系数
#define bs5 0
#define Ks6 0.025 // 太阳阵6电压 系数
#define bs6 0
#define Ks7 0.025 // 太阳阵7电压 系数
#define bs7 0
#define Ks8 0.025 // 太阳阵8电压 系数
#define bs8 0
#define Ks9 0.025 // 太阳阵9电压 系数
#define bs9 0
#define Ks10 0.025 // 太阳阵10电压 系数
#define bs10 0
#define Ks11 0.025 // 太阳阵11电压 系数
#define bs11 0
#define Ks12 0.025 // 太阳阵12电压 系数
#define bs12 0
#define Ks13 0.025 // 太阳阵13电压 系数
#define bs13 0
#define Ks14 0.025 // 太阳阵14电压 系数
#define bs14 0
#define Ks15 0.025 // 太阳阵15电压 系数
#define bs15 0
#define Ks16 0.025 // 太阳阵16电压 系数
#define bs16 0

sbit fgRstType = P1 ^ (unsigned int)(0x02); // 热复位标识，0=冷复位 1=热复位
sbit onoffEn = P1 ^ (unsigned int)(0x04);	// 指令输出使能
sbit dog = P1 ^ (unsigned int)(0x05);		// 看门狗输出
sbit RXD422AEn = P1 ^ (unsigned int)0x00;	// 422A总线接收使能，0为使能
sbit TXD422En = P1 ^ (unsigned int)0x06;	// 422A总线发送使能，0为使能
sbit TXDCONTROL = P3 ^ (unsigned int)0x01;	// 发送门控信号
sbit RXDCONTROL = P3 ^ (unsigned int)0x00;	// 接收门控信号

const unsigned char code rsTab[100] = {
	// 模拟量采集码表
	0x10, // 00 母线电流1
	0x11, // 01 母线电流3
	0x12, // 02 母线电流2
	0x13, // 03 母线电流4
	0x14, // 04 放电电流1（SC）
	0x15, // 05 蓄电池电压1（SC）
	0x16, // 06 放电电流2（SC）
	0x17, // 07 蓄电池电压2（SC）
	0x18, // 08 BDRA1工作状态
	0x19, // 09 放电输出电流A1
	0x1A, // 10 BDRB1工作状态
	0x1B, // 11 放电输出电流B1
	0x1C, // 12 BDRA2工作状态
	0x1D, // 13 放电输出电流A2
	0x1E, // 14 BDRB2工作状态
	0x1F, // 15 放电输出电流B2
	0x20, // 16 BDRA3工作状态
	0x21, // 17 放电输出电流A3
	0x22, // 18 BDRB3工作状态
	0x23, // 19 放电输出电流B3
	0x24, // 20 BDRA4工作状态
	0x25, // 21 放电输出电流A4
	0x26, // 22 BDRB4工作状态
	0x27, // 23 放电输出电流B4
	0x28, // 24 放电模块温度1
	0x29, // 25 充电电流1（SC）
	0x2A, // 26 放电模块温度2
	0x2B, // 27 充电电流2（SC）
	0x2C, // 28 预接通开关1状态
	0x2D, // 29 放电开关2-1状态
	0x2E, // 30 放电开关1-1状态
	0x2F, // 31 放电开关2-2状态
	0x30, // 32 放电开关1-2状态
	0x31, // 33 预接通开关2状态
	0x32, // 34 母线电压
	0x33, // 35 充分模块1太阳阵总电流
	0x34, // 36 充分模块2太阳阵总电流
	0x35, // 37 充分模块3太阳阵总电流
	0x36, // 38 充分模块4太阳阵总电流
	0x37, // 39 充电阵A1状态
	0x38, // 40 BEA电压1
	0x39, // 41 充电阵A2状态
	0x3A, // 42 BEA电压2
	0x3B, // 43 充电阵A3状态
	0x3C, // 44 充电阵A4状态
	0x3D, // 45 充电阵A5状态
	0x3E, // 46 充电阵B1状态
	0x3F, // 47 充电阵B2状态
	0x40, // 48 MEA电压
	0x41, // 49 充电阵B3状态
	0x42, // 50 充电阵B4状态
	0x43, // 51 充电阵B5状态
	0x44, // 52 太阳电池阵3电压
	0x45, // 53 太阳电池阵7电压
	0x46, // 54 太阳电池阵11电压
	0x47, // 55 太阳电池阵15电压
	0x48, // 56 太阳电池阵2电压
	0x49, // 57 太阳电池阵6电压
	0x4A, // 58 太阳电池阵10电压
	0x4B, // 59 太阳电池阵14电压
	0x4C, // 60 太阳电池阵1电压
	0x4D, // 61 太阳电池阵5电压
	0x4E, // 62 太阳电池阵9电压
	0x4F, // 63 太阳电池阵13电压
	0x50, // 64 太阳电池阵4电压
	0x51, // 65 太阳电池阵8电压
	0x52, // 66 太阳电池阵12电压
	0x53, // 67 太阳电池阵16电压
	0x54, // 68 充电阵A1充电电流
	0x55, // 69 充电阵A3充电电流
	0x56, // 70 充电阵A2充电电流
	0x57, // 71 充电阵A5充电电流
	0x58, // 72 充电阵A4充电电流
	0x59, // 73 充电阵B1充电电流
	0x5A, // 74 充电阵B3充电电流
	0x5B, // 75 充电阵B5充电电流
	0x5C, // 76 充电阵B2充电电流
	0x5D, // 77 充分模块3分流保护管通断指示
	0x5E, // 78 充电阵B4充电电流
	0x5F, // 79 分流模块温度1
	0x60, // 80 分流模块温度2
	0x61, // 81 充分模块2分流保护管通断指示
	0x62, // 82 充分模块1分流保护管通断指示
	0x63, // 83 充分模块4分流保护管通断指示
	0x64, // 84 下位机主机+5V电源
	0x65, // 85 下位机备机+5V电源
};

const unsigned char code onoffTab[50] = {
	// 间接指令通码表
	0xE1, // 01 分流保护管1断开
	0xE2, // 02 分流保护管5断开
	0xE3, // 03 分流保护管9断开
	0xE4, // 04 分流保护管13断开
	0xE5, // 05 充电阵A1断开
	0xE6, // 06 充电阵A3断开
	0xE7, // 07 充电阵A5断开
	0xE8, // 08 A组充电阵接通
	0xE9, // 09 充分模块1分流保护管接通
	0xEA, // 10 分流保护管3断开
	0xEB, // 11 分流保护管7断开
	0xEC, // 12 分流保护管11断开
	0xED, // 13 分流保护管15断开
	0xEE, // 14 充电阵A2断开
	0xD1, // 15 充电阵A4断开
	0xD2, // 16 充分模块2分流保护管接通
	0xD3, // 17 分流保护管2断开
	0xD4, // 18 分流保护管6断开
	0xD5, // 19 分流保护管10断开
	0xD6, // 20 分流保护管14断开
	0xD7, // 21 充电阵B1断开
	0xD8, // 22 充电阵B3断开
	0xD9, // 23 充电阵B5断开
	0xDA, // 24 B组充电阵接通
	0xDB, // 25 充分模块3分流保护管接通
	0xDC, // 26 分流保护管4断开
	0xDD, // 27 分流保护管8断开
	0xDE, // 28 分流保护管12断开
	0xB1, // 29 分流保护管16断开
	0xB2, // 30 充电阵B2断开
	0xB3, // 31 充电阵B4断开
	0xB4, // 32 充分模块4分流保护管接通
	0xB5, // 33 BDRA1使能
	0xB6, // 34 BDRA1禁止
	0xB7, // 35 BDRB1使能
	0xB8, // 36 BDRB1禁止
	0xB9, // 37 BDRA2使能
	0xBA, // 38 BDRA2禁止
	0xBB, // 39 BDRB2使能
	0xBC, // 40 BDRB2禁止
	0xBD, // 41 BDRA3使能
	0xBE, // 42 BDRA3禁止
	0x71, // 43 BDRB3使能
	0x72, // 44 BDRB3禁止
	0x73, // 45 BDRA4使能
	0x74, // 46 BDRA4禁止
	0x75, // 47 BDRB4使能
	0x76  // 48 BDRB4禁止
};

const unsigned int code equAqOnTab[9] = {
	0x8000, // 01 单体1接通
	0x8005, // 02 单体2接通
	0x8009, // 03 单体3接通
	0x800C, // 04 单体4接通
	0x8011, // 05 单体5接通
	0x8014, // 06 单体6接通
	0x8018, // 07 单体7接通
	0x801D, // 08 单体8接通
	0x8021	// 09 单体9接通
};

const unsigned int code equAqOffTab[9] = {
	0x8003, // 01 单体1断开
	0x8006, // 02 单体2断开
	0x800A, // 03 单体3断开
	0x800F, // 04 单体4断开
	0x8012, // 05 单体5断开
	0x8017, // 06 单体6断开
	0x801B, // 07 单体7断开
	0x801E, // 08 单体8断开
	0x8022	// 09 单体9断开
};

const float code volTab[8] = {(float)32.45, (float)32.85, (float)28.35, (float)32.05, (float)35.65, (float)36.1, (float)36.55, (float)37.00}; // 蓄电池限压值
const float code singlechargeVolt[8] = {(float)28.35, (float)32.05, (float)32.45, (float)32.85, (float)35.65, (float)36.1, (float)36.55, (float)37.00};
const unsigned char code volKickTab[8] = {0x03, 0x04, 0x01, 0x02, 0x05, 0x06, 0x07, 0x08}; // 对应电压档位

const unsigned char code curCsTab[8] = {0x01, 0x02, 0x03, 0x04, 0x00, 0x05, 0x06, 0x07}; // 蓄电池限流值
const unsigned char code volCsTab[8] = {0x01, 0x02, 0x03, 0x04, 0x00, 0x06, 0x07, 0x05}; // 蓄电池限压值

const unsigned char code volSwitchOnoffTabA[8] = {0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24}; // 蓄电池A切换指令表
const unsigned char code volSwitchOnoffTabB[8] = {0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34}; // 蓄电池B切换指令表

float xdata limbCurrent;  // 太阳翼总电流 PASS INIT
float xdata limbCurrentA; // 太阳翼A电流 PASS INIT
float xdata limbCurrentB; // 太阳翼B电流 PASS INIT

float xdata chargeCurrent;	// 充电电流  PASS INIT
float xdata chargeCurrent1; // 充电电流1 PASS INIT
float xdata chargeCurrent2; // 充电电流2 PASS INIT

float xdata dischargeCurrent;  // 放电电流  PASS INIT
float xdata dischargeCurrent1; // 放电电流1 PASS INIT
float xdata dischargeCurrent2; // 放电电流2 PASS INIT

float xdata genCurrent;				// 母线电流  PASS INIT
float xdata meaV;					// meaV  PASS INIT
float xdata generatrixVol;			// 母线电压  PASS INIT
float xdata chargeArrayCurrent[10]; // 充电阵A-B电流1-5  PASS INIT

unsigned int xdata accVAHard; // 电池组电压主采集值(均衡器原码)  PASS INIT

float xdata accVAHardPyh; // 电池组电压主采集值 物理值  -- 下位机采集  PASS INIT
float xdata accVAEquPyh;  // 电池组电压主采集值 物理值  -- 均衡器采集  PASS INIT
float xdata accVBEquPyh;  // 电池组电压备采集值 物理值  -- 均衡器采集  PASS INIT
float xdata equVRefPhy;	  // 均衡基准电压(物理值)  		-- 均衡器采集  PASS INIT

unsigned char xdata chargeVolKickA;	  // 充电限压档数A A  PASS INIT	2/3
unsigned char xdata chargeVolKickA_B; // 充电限压档数A B  PASS INIT
unsigned char xdata chargeVolKickA_C; // 充电限压档数A C  PASS INIT

unsigned char xdata chargeVolKickB;	  // 充电限压档数B A  PASS INIT	2/3
unsigned char xdata chargeVolKickB_B; // 充电限压档数B B  PASS INIT
unsigned char xdata chargeVolKickB_C; // 充电限压档数B C  PASS INIT

unsigned char xdata chargeVolKickRsA; // 充电限压切换档数A  PASS INIT
unsigned char xdata chargeVolKickRsB; // 充电限压切换档数B  PASS INIT

unsigned char xdata chargeCurKickA;	  // 充电限流档数A A  PASS INIT	2/3
unsigned char xdata chargeCurKickA_B; // 充电限流档数A B  PASS INIT
unsigned char xdata chargeCurKickA_C; // 充电限流档数A C  PASS INIT

unsigned char xdata chargeCurKickB;	  // 充电限流档数B A  PASS INIT	2/3
unsigned char xdata chargeCurKickB_B; // 充电限流档数B B  PASS INIT
unsigned char xdata chargeCurKickB_C; // 充电限流档数B C  PASS INIT

unsigned char xdata fg16S; // 16s 时间定时标志 -- 重要数据发送请求周期 PASS INIT
unsigned char xdata fg30S; // 30s 时间定时计数标志  均衡周期计数
unsigned char xdata fg60S; // 60s 时间定时计数标志(过滤包) PASS INIT

unsigned char clockCount;		 // 1s程序运行定时计数器  PASS INIT
unsigned char errorCount;		 // 总线通讯不符合协议计数器  内部变量  PASS INIT
unsigned char xdata errorCountA; // 总线通讯不符合协议计数器A  PASS INIT
unsigned char xdata errorCountB; // 总线通讯不符合协议计数器B  PASS INIT

unsigned char xdata resetCount;	 // 热复位计数  PASS INIT	2/3
unsigned char xdata resetCountB; // 热复位计数B  PASS INIT
unsigned char xdata resetCountC; // 热复位计数C  PASS INIT

unsigned char xdata onoffExeCount;	// 遥控指令执行计数 A  PASS INIT	2/3
unsigned char xdata onoffExeCountB; // 遥控指令执行计数 B  PASS INIT
unsigned char xdata onoffExeCountC; // 遥控指令执行计数 C  PASS INIT

unsigned char xdata onoffErrorCount;  // 遥控指令错误计数 A  PASS INIT	2/3
unsigned char xdata onoffErrorCountB; // 遥控指令错误计数 B  PASS INIT
unsigned char xdata onoffErrorCountC; // 遥控指令错误计数 C  PASS INIT

unsigned char xdata onoffReceCount;	 // 总线指令接收计数器 A PASS INIT	2/3
unsigned char xdata onoffReceCountB; // 总线指令接收计数器 B PASS INIT
unsigned char xdata onoffReceCountC; // 总线指令接收计数器 C PASS INIT

unsigned char xdata fgSwitch1; // 数据采集保存切区标志1 PASS INIT
unsigned char xdata fgSwitch2; // 数据采集保存切区标志2 PASS INIT

unsigned char xdata wrOnoffIndex;		// 写索引 PASS INIT
unsigned char xdata rdOnoffIndex;		// 读索引 PASS INIT
unsigned char xdata onoffBuffer[24][2]; // 指令接收缓冲区 PASS INIT

unsigned char xdata wrOnoffRecIndex;				  // 自主指令记录区写索引 PASS INIT
unsigned int xdata onoffBufferRec[24][6] _at_ 0x9800; // 指令记录缓存区 		PASS INIT

unsigned int xdata anData[100];				   // 原始模拟量数据采集区 16bit
unsigned char xdata rsFrame1A[70] _at_ 0x9000; // 快包遥测参数数组 初始化组帧更新
unsigned char xdata rsFrame1B[70];
unsigned char xdata rsFrame2A[170] _at_ 0x9200; // 慢包遥测参数数组 初始化组帧更新
unsigned char xdata rsFrame2B[170];
unsigned char xdata rsFrame3[70] _at_ 0x9400; // 过滤包遥测参数数组 初始化组帧更新

unsigned char xdata filterDataRec[41]; // 过滤包数据 对比包 PASS INIT
unsigned char xdata filterFrame[41];   // 过滤包数据缓存区 PASS INIT

unsigned char xdata fgRsRam;					  // RAM包下传请求标志 PASS INIT
unsigned char ramUploadData[8];					  // RAM地址上注数据信息
unsigned char xdata rsRamFrame[190];			  // RAM包 -- 128字节 组帧更新
unsigned char xdata ramTestData[128] _at_ 0x9600; // 特定地址 8000起始地址 PASS INIT

unsigned char xdata fgImportantDataRx;	   // 重要数据恢复请求标志 PASS INIT
unsigned char xdata rsImportantData[220];  // 重要数据保存包-缓存区 PASS INIT
unsigned char xdata rsImportantFrame[320]; // 重要数据保存包-组帧发送区 初始化填充处理

unsigned char xdata reqData[12]; // 服务请求码 PASS INIT
unsigned char wrReqIndex;		 // 写索引 PASS INIT
unsigned char rdReqIndex;		 // 读索引 PASS INIT

unsigned char xdata fgUploadPara;	   // 上注参数请求标志 PASS INIT
unsigned char xdata uploadParaData[4]; // 上注参数数据 PASS INIT

unsigned char xdata fgCanBus; // 当前总线使用状态标志 PASS INIT

unsigned char xdata *pSendA;			 // A总线发送指针 --  使用前更新
unsigned char sendIndexA;				 // A总线发送帧索引 --  CANInfoA初始化
unsigned char sendFrameA;				 // A总线发送数据包总帧数 --  CANInfoA初始化
unsigned char receiveFrameA;			 // A总线接收数据帧计数 --  CANInfoA初始化
unsigned int receiveSumA;				 // A总线接收数据包累加和 --  CANInfoA初始化
unsigned char receiveCountA;			 // A总线接收数据计数 --  CANInfoA初始化
unsigned char xdata receiveBufferA[256]; // A总线数据包接收缓存区(wLength+Title+Data+Sum) --  CANInfoA初始化

unsigned char xdata *pSendB;			 // B总线发送指针 --  使用前更新
unsigned char sendIndexB;				 // B总线发送帧索引 --  CANInfoA初始化
unsigned char sendFrameB;				 // B总线发送数据包总帧数 --  CANInfoA初始化
unsigned char receiveFrameB;			 // B总线接收数据帧计数 --  CANInfoA初始化
unsigned int receiveSumB;				 // B总线接收数据包累加和 --  CANInfoA初始化
unsigned char receiveCountB;			 // B总线接收数据计数 --  CANInfoA初始化
unsigned char xdata receiveBufferB[256]; // B总线数据包接收缓存区(wLength+Title+Data+Sum) --  CANInfoA初始化

unsigned char xdata pcuState1; // PCU状态字1 PASS INIT
unsigned char xdata pcuState2; // PCU状态字2 PASS INIT
unsigned char xdata pcuState3; // PCU状态字3 PASS INIT
unsigned char xdata pcuState4; // PCU状态字4 PASS INIT

unsigned int xdata accEquVA; // 电池组电压主(原码) PASS INIT
unsigned int xdata accEquVB; // 电池组电压备(原码) PASS INIT
unsigned int xdata equ12VP;	 // 均衡+12V电压(原码) PASS INIT
unsigned int xdata equ12VN;	 // 均衡-12V电压(原码) PASS INIT
unsigned int xdata equ5VP;	 // 均衡+5V电压(原码) PASS INIT
unsigned int xdata equVRef;	 // 均衡基准电压(原码) PASS INIT

unsigned int xdata aq[11];			 // 单体电压 PASS INIT
float xdata aqPhy[11];				 // 单体电压 物理值 PASS INIT
float xdata aqPhyPx[11];			 // 单体电压 物理值排序 PASS INIT
unsigned char xdata aqControlNumber; // 单体控制数量 PASS INIT

unsigned char xdata fgEquRx;		   // 均衡422接收标志 PASS INIT
unsigned char xdata fgEquTx;		   // 均衡422发送标志 PASS INIT
unsigned char xdata equDataBuffer[40]; // 均衡数据接收缓存 PASS INIT
unsigned char xdata equRxCount;		   // 均衡422接收计数 PASS INIT
unsigned char xdata equExeCount[9];	   // 均衡执行计数 PASS INIT

unsigned char xdata wrEquOnoffIndex;   // 均衡指令写索引 PASS INIT
unsigned char xdata rdEquOnoffIndex;   // 均衡指令读索引 PASS INIT
unsigned int xdata equOnoffBuffer[48]; // 均衡指令指令接收缓冲区 PASS INIT

float xdata equOpenValue;  // 均衡开门限值A PASS INIT  !!!!
float xdata equOpenValueB; // 均衡开门限值B PASS INIT
float xdata equOpenValueC; // 均衡开门限值C PASS INIT

float xdata equCloseValue;	// 均衡关门限值A PASS INIT !!!!
float xdata equCloseValueB; // 均衡关门限值B PASS INIT
float xdata equCloseValueC; // 均衡关门限值C PASS INIT

unsigned char xdata equState1; // 均衡422状态1 PASS INIT
unsigned char xdata equState2; // 均衡422状态2 PASS INIT

unsigned char xdata sepCurrentState1; // 分流1保护通断状态 PASS INIT
unsigned char xdata sepCurrentState2; // 分流2保护通断状态 PASS INIT
unsigned char xdata sepCurrentState3; // 分流3保护通断状态 PASS INIT
unsigned char xdata sepCurrentState4; // 分流4保护通断状态 PASS INIT

unsigned char xdata fgAh;		  // 安时计启动标志 PASS INIT
unsigned int xdata AhCount16S;	  // 安时计计数时间 （16S 判读）PASS INIT
unsigned char xdata powerSave[6]; // 电量值存储 PASS INIT

unsigned char xdata proportion;	 // 充放比A PASS INIT  !!!
unsigned char xdata proportionB; // 充放比B PASS INIT
unsigned char xdata proportionC; // 充放比C PASS INIT

float xdata chargePower;	// 蓄电池组充电电量值 PASS INIT
float xdata dischargePower; // 蓄电池组放电电量值 PASS INIT
float xdata currentPower;	// 蓄电池组当前电量值 PASS INIT

float xdata ntrPower;  // 蓄电池组满电量A PASS INIT
float xdata ntrPowerB; // 蓄电池组满电量B PASS INIT
float xdata ntrPowerC; // 蓄电池组满电量C PASS INIT

unsigned char xdata importDataRxState; // 重要数据恢复状态 PASS INIT
unsigned char xdata selfControlCount;  // 自主控制计数 PASS INIT

unsigned char xdata selfCheckStateWord; // 自检状态字 PASS INIT
unsigned int xdata healthStateWord1;	// 健康状态字1 PASS INIT
unsigned int xdata healthStateWord2;	// 健康状态字2 PASS INIT

unsigned char xdata selfCheckStateContrl;  // 自检状态控制字A  PASS INIT  !!!
unsigned char xdata selfCheckStateContrlB; // 自检状态控制字B  PASS INIT
unsigned char xdata selfCheckStateContrlC; // 自检状态控制字C  PASS INIT

unsigned int xdata aqStateContrl;  // 单体控制字A  PASS INIT  !!!
unsigned int xdata aqStateContrlB; // 单体控制字B  PASS INIT
unsigned int xdata aqStateContrlC; // 单体控制字C  PASS INIT

unsigned int xdata healthStateContr1;  // 健康状态字1控制字A   PASS INIT  !!!
unsigned int xdata healthStateContr1B; // 健康状态字1控制字B   PASS INIT
unsigned int xdata healthStateContr1C; // 健康状态字1控制字C   PASS INIT

unsigned int xdata healthStateContr2;  // 健康状态字2控制字A   PASS INIT  !!!
unsigned int xdata healthStateContr2B; // 健康状态字2控制字B   PASS INIT
unsigned int xdata healthStateContr2C; // 健康状态字2控制字C   PASS INIT

unsigned char xdata equControlEn;  // 单体保护使能状态A   PASS INIT  !!!
unsigned char xdata equControlEnB; // 单体保护使能状态B   PASS INIT
unsigned char xdata equControlEnC; // 单体保护使能状态C   PASS INIT

unsigned char xdata singleProtectChargeEn;	//单体保护使能状态A   PASS INIT  !!!
unsigned char xdata singleProtectChargeEnB; //单体保护使能状态B   PASS INIT
unsigned char xdata singleProtectChargeEnC; //单体保护使能状态C   PASS INIT

unsigned char xdata overChargeProtectEn;  // 过充保护允许标志A   PASS INIT  !!!
unsigned char xdata overChargeProtectEnB; // 过充保护允许标志B   PASS INIT
unsigned char xdata overChargeProtectEnC; // 过充保护允许标志C   PASS INIT

float xdata singleVProtectValueUP;	// 单体上限保护值A   PASS INIT  !!!
float xdata singleVProtectValueUPB; // 单体上限保护值B   PASS INIT
float xdata singleVProtectValueUPC; // 单体上限保护值C   PASS INIT

float xdata singleVWardValueUP;	 // 单体上限报警值A   PASS INIT  !!!
float xdata singleVWardValueUPB; // 单体上限报警值B   PASS INIT
float xdata singleVWardValueUPC; // 单体上限报警值C   PASS INIT

float xdata singleVWardValueDOWN;  // 单体下限报警值A   PASS INIT  !!!
float xdata singleVWardValueDOWNB; // 单体下限报警值B   PASS INIT
float xdata singleVWardValueDOWNC; // 单体下限报警值C   PASS INIT

float xdata batteryVAWardValueUP;  // 蓄电池组电压上限保护值A   PASS INIT  !!!
float xdata batteryVAWardValueUPB; // 蓄电池组电压上限保护值B   PASS INIT
float xdata batteryVAWardValueUPC; // 蓄电池组电压上限保护值C   PASS INIT

float xdata batteryVAWardValueDOWN;	 // 蓄电池组电压下限保护值A   PASS INIT  !!!
float xdata batteryVAWardValueDOWNB; // 蓄电池组电压下限保护值B   PASS INIT
float xdata batteryVAWardValueDOWNC; // 蓄电池组电压下限保护值C   PASS INIT

unsigned char xdata fgOverChargeprotect;  // 蓄电池组过充标志   PASS INIT
unsigned char xdata chargeArrayCount[10]; // 充电阵过充计数AB-共计10组   PASS INIT
unsigned char xdata fgOverChargePro;	  // 充电阵单路故障标志   PASS INIT
unsigned char xdata fgOverChargeShort;	  // 充电阵短路故障标志   PASS INIT

unsigned char xdata singlesaveflag;	   // 单体记忆状态标志   PASS INIT
unsigned char xdata singlesaveVLevelA; // 档位过程记录保存变量   PASS INIT
unsigned char xdata singlesaveVLevelB; // 档位过程记录保存变量   PASS INIT

unsigned char xdata chargeFlag;			  // 充电状态标志   PASS INIT
unsigned char xdata currentModeCountA[3]; // 充电模式判读计数1    PASS INIT
unsigned char xdata currentModeCountB[3]; // 充电模式判读计数2    PASS INIT

unsigned char xdata checkCount1;	  // 母线过压时间计数 -- 健康监测计数1
unsigned char xdata checkCount2;	  // 母线欠压时间计数 -- 健康监测计数2
unsigned char xdata checkCount3;	  // 蓄电池组过压时间计数 -- 健康监测计数3
unsigned char xdata checkCount4;	  // 蓄电池组欠压时间计数 -- 健康监测计数4
unsigned char xdata checkCount5;	  // 均衡基准电压时间计数 -- 健康监测计数5
unsigned char xdata checkCount6;	  // 均衡工作监测时间计数 -- 健康监测计数6
unsigned char xdata checkCount7;	  // 分流管短路时间计数 -- 健康监测计数7
unsigned char xdata checkCount8;	  // 供电自检时间计数 -- 健康监测计数8
unsigned char xdata checkCountAqA[9]; // 单体欠压时间计数
unsigned char xdata checkCountAqB[9]; // 单体过压时间计数

unsigned char xdata fgCheck1;	   // 母线过压标志 -- 健康标志1
unsigned char xdata fgCheck2;	   // 母线欠压标志 -- 健康标志2
unsigned char xdata fgCheck3;	   // 蓄电池组电压过压标志 -- 健康标志3
unsigned char xdata fgCheck4;	   // 蓄电池组电压欠压标志 -- 健康标志4
unsigned char xdata fgCheck5;	   // 均衡基准电压检测标志 -- 健康标志5
unsigned char xdata fgCheck6;	   // 均衡工作检测标志 -- 健康标志6
unsigned char xdata fgCheck7;	   // 分流管短路检测标志 -- 健康标志7
unsigned char xdata fgAqCheckA[9]; // 单体欠压标志
unsigned char xdata fgAqCheckB[9]; // 单体过压标志
unsigned char xdata fgCheck8;	   // 供电自检标志 -- 自检标志

unsigned int xdata overChargeProCount;		 // 过充保护执行计数
unsigned int xdata singleProtectChargeCount; // 过充保护执行计数

unsigned char xdata selfCheckState; //自检状态
unsigned char xdata selfCheckWord;	//自检控制字
unsigned char xdata selfCheckWordB; //自检控制字
unsigned char xdata selfCheckWordC; //自检控制字

unsigned long xdata sysTime; // 系统时间	 广播时间修正

unsigned char code selectAB _at_ 0x7FFF; // AB机工作判读位  32K

void SystemInit(void);						   // 系统初始化函数
void DataInit(void);						   // 数据初始化函数
void CANInit(unsigned int address);			   // SJA1000初始化函数
void CPUInit(void);							   // 80C32初始化函数
void ANCollect(void);						   // 模拟量采集函数
unsigned int ADConvert(unsigned char channel); // AD转换函数
void StateWordBuild(void);					   // 组状态字函数

void SaveData(void);			// 遥测包组帧函数
void SaveDataRs1(void);			// 组帧函数--包1
void SaveDataRs2(void);			// 组帧函数--包2
void SaveDataRs3(void);			// 组帧函数--包3
void SaveRamDataRs(void);		// 组帧函数--RAM
void SaveImportantDataRs(void); // 组帧函数--重要数据

void ImportantDataTx(void);						  // 重要数据发送处理函数
void ImportantDataRx(void);						  // 重要数据恢复处理函数
void ImportantSaveData(unsigned char xdata *pRs); //重要数据组帧函数
void ImportDataFlush(void);
void DefaultInit(void); // 重要数据恢复失败初始化

void FilterManage(void);
void FilterSaveData(unsigned char xdata *pRs);
unsigned char FilterDataComp(void);

void EquAncollect(void); // EQU接收起始处理

void CANInfoA(void);  // CAN初始化-A
void CANInfoB(void);  // CAN初始化-B
void CanResume(void); // CAN恢复函数

void ONOFFOutput(unsigned char index);							// 指令输出函数
void ONOFFHook(void);											// 指令输出链接函数
void OnoffSelfExeRec(unsigned char type, unsigned char number); // 指令输出记录

void UploadParaManage(void); // 上注参数处理函数
void ChargeCheck(void);

unsigned char Get2_3_U(unsigned char *a, unsigned char *b, unsigned char *c); // 数据3取2判决函数
unsigned char Get2_3_F(float *a, float *b, float *c);						  /* 三取二浮点型 		*/
unsigned char Get2_3_I(unsigned int *a, unsigned int *b, unsigned int *c);	  /* 三取二无符号整型   	*/

void Delay(unsigned int time); // 固定延时函数
void DataProtect(void);		   // 数据参数保护函数

void CANRXDA(void);			 // A总线CAN中断接收处理函数
void CANTXDA(void);			 // A总线CAN中断发送处理函数
void RsManageA(void);		 // A总线CAN中断轮询处理函数
void ONOFFManageA(void);	 // A总线CAN中断指令处理函数
void DataManageA(void);		 // A总线CAN中断数据块（程序代码）处理函数
void RamManageA(void);		 // A总线RAM中断处理函数
void ImportantManageA(void); // A总线重要数据中断处理函数
void TimeBcA(void);			 // A总线星时广播中断处理函数

void CANRXDB(void);			 // B总线CAN中断接收处理函数
void CANTXDB(void);			 // B总线CAN中断发送处理函数
void RsManageB(void);		 // B总线CAN中断轮询处理函数
void ONOFFManageB(void);	 // B总线CAN中断指令处理函数
void DataManageB(void);		 // B总线CAN中断数据块（程序代码）处理函数
void RamManageB(void);		 // B总线RAM中断处理函数
void ImportantManageB(void); // B总线重要数据中断处理函数
void TimeBcB(void);			 // B总线星时广播中断处理函数

unsigned char SepCurrentStateCal(unsigned int val); // 分流状态
void PowerControl(void);							// 安时计计算
void SingleProtectChargeControl(void);				// 单体保护
void OverChargeProtect(void);						// 过充保护

void EquOnoffHook(void);	  // 均衡器指令422总线发送
unsigned char RamCheck(void); // SRAM自检
void EquControl(void);		  // 均衡控制模块
void ChargeCheck(void);		  // 充电检测
void HealthStaClear(void);

void SelfHealthCheck(void);		 // 自检监测
void HealthCheck(void);			 // 健康监测函数
void HealthStateWordBuild(void); // 健康状态字
void GenVolOverCheck(void);		 // 电源母线过压监测
void GenVolUnderCheck(void);	 // 电源母线欠压监测
void AhVolOverCheck(void);		 // 蓄电池组电压过压监测
void AhVolUnderCheck(void);		 // 蓄电池组电压欠压监测
void EquVRefCheck(void);		 // 均衡基准监测
void EquWorkCheck(void);		 // 均衡工作监测
void SepCurrentShortCheck(void); // 分流管短路监测
void AqVolCheck(void);			 // 单体电压健康监测
void PowerSupplyCheck(void);	 // 供电自检监测

// *****************************************************************
// 函数名称 : main
// 功能描述 :
//*****************************************************************
void main(void)
{
	unsigned char fg4S;					   // 4S控制周期
	volatile unsigned char clockCountTemp; // 满足规则检查增加临时变量

	SystemInit(); // 系统初始化

	while (1) // 执行固定周期任务  1s
	{
		dog = !dog; // 1s 牵狗一次

		fg4S++; // 周期计时

		SaveData();	   // 组帧函数
		DataProtect(); // 数据保护函数

		ANCollect();	  // 模拟量采集
		StateWordBuild(); // 构成状态字  必须先更新后续有使用
		Delay(2000);	  // 延时操作
		ChargeCheck();

		EquControl();	// 均衡控制功能 -- 30s 控制周期
		EquOnoffHook(); // 均衡器指令发送 -- 每秒发送1条均衡指令

		FilterManage(); // 过滤包管理

		if (fg4S > 3)
		{
			fg4S = 0;
			PowerControl();
			OverChargeProtect();		  // 蓄电池组过充保护
			SingleProtectChargeControl(); // 单体过充保护控制
		}

		CanResume();		// CAN总线异常处理函数
		UploadParaManage(); // 上注参数更新处理

		HealthCheck();	   // 健康监测
		SelfHealthCheck(); // 自检监测

		ONOFFHook(); // 间接指令处理钩子
		SaveData();	 // 组帧函数
		dog = !dog;	 // 1s 牵狗2次

		ImportantDataTx(); // 发送重要数据

		clockCountTemp = clockCount; // 增加临时变量 静态规则修改
		while (clockCountTemp < 100) // 延时等待1s
		{
			clockCountTemp = clockCount;
		}
		EA = 0;
		clockCount = (unsigned char)(clockCount - 100); // 时间补偿
		sysTime++;										// 维护系统时间
		EA = 1;
	}
}

//*****************************************************************
// 函数名称 : Delay
// 功能描述 :
//*****************************************************************
void Delay(unsigned int time)
{
	unsigned int k;

	for (k = 0; k < time; k++) // 循环延时
	{
		_nop_(); // 执行空语句
	}
}

//*****************************************************************
// 函数名称 : DataProtect
// 功能描述 :
//*****************************************************************
void DataProtect(void)
{
	unsigned char result1;

	result1 = Get2_3_U(&resetCount, &resetCountB, &resetCountC); // 热复位计数3取2判决
	if (result1 == FALSE)
	{
		resetCount = 0; // 三取二恢复设置
		resetCountB = 0;
		resetCountC = 0;
	}

	result1 = Get2_3_U(&onoffExeCount, &onoffExeCountB, &onoffExeCountC); // 遥控指令执行计数(包括指令、上注数据、上注代码) 3取2判决
	if (result1 == FALSE)
	{
		onoffExeCount = 0; // 三取二恢复设置
		onoffExeCountB = 0;
		onoffExeCountC = 0;
	}

	result1 = Get2_3_U(&onoffReceCount, &onoffReceCountB, &onoffReceCountC); // 遥控指令执行计数(包括指令、上注数据、上注代码) 3取2判决
	if (result1 == FALSE)
	{
		onoffReceCount = 0; // 三取二恢复设置
		onoffReceCountB = 0;
		onoffReceCountC = 0;
	}

	result1 = Get2_3_U(&onoffErrorCount, &onoffErrorCountB, &onoffErrorCountC); // 遥控指令执行计数(包括指令、上注数据、上注代码) 3取2判决
	if (result1 == FALSE)
	{
		onoffErrorCount = 0; // 三取二恢复设置
		onoffErrorCountB = 0;
		onoffErrorCountC = 0;
	}

	result1 = Get2_3_U(&chargeVolKickA, &chargeVolKickA_B, &chargeVolKickA_C); //
	if (result1 == FALSE)
	{
		chargeVolKickA = 5; //  三取二恢复设置
		chargeVolKickA_B = 5;
		chargeVolKickA_C = 5;
	}

	result1 = Get2_3_U(&chargeVolKickB, &chargeVolKickB_B, &chargeVolKickB_C); //
	if (result1 == FALSE)
	{
		chargeVolKickB = 5; //  三取二恢复设置
		chargeVolKickB_B = 5;
		chargeVolKickB_C = 5;
	}

	result1 = Get2_3_U(&chargeCurKickA, &chargeCurKickA_B, &chargeCurKickA_C); //
	if (result1 == FALSE)
	{
		chargeCurKickA = 5; //  三取二恢复设置
		chargeCurKickA_B = 5;
		chargeCurKickA_C = 5;
	}

	result1 = Get2_3_U(&chargeCurKickB, &chargeCurKickB_B, &chargeCurKickB_C); //
	if (result1 == FALSE)
	{
		chargeCurKickB = 5; //  三取二恢复设置
		chargeCurKickB_B = 5;
		chargeCurKickB_C = 5;
	}

	result1 = Get2_3_F(&equOpenValue, &equOpenValueB, &equOpenValueC); //
	if (result1 == FALSE)
	{
		equOpenValue = 0.06;  // 均衡开门限值  A
		equOpenValueB = 0.06; // 均衡开门限值  B
		equOpenValueC = 0.06; // 均衡开门限值  C
	}

	result1 = Get2_3_F(&equCloseValue, &equCloseValueB, &equCloseValueC); //
	if (result1 == FALSE)
	{
		equCloseValue = 0.01;  // 均衡关门限值  A
		equCloseValueB = 0.01; // 均衡关门限值  B
		equCloseValueC = 0.01; // 均衡关门限值  C
	}

	result1 = Get2_3_U(&proportion, &proportionB, &proportionC); //
	if (result1 == FALSE)
	{
		proportion = 102;  // 充放比
		proportionB = 102; // 充放比
		proportionC = 102; // 充放比
	}

	result1 = Get2_3_U(&selfCheckStateContrl, &selfCheckStateContrlB, &selfCheckStateContrlC); //
	if (result1 == FALSE)
	{
		selfCheckStateContrl = 0xFF;  // 默认自检控制字 全有效  A
		selfCheckStateContrlB = 0xFF; // 默认自检控制字 全有效  B
		selfCheckStateContrlC = 0xFF; // 默认自检控制字 全有效  C
	}

	result1 = Get2_3_I(&aqStateContrl, &aqStateContrlB, &aqStateContrlC); //
	if (result1 == FALSE)
	{
		aqStateContrl = 0xFFFF;	 // 均衡控制字初始化A
		aqStateContrlB = 0xFFFF; // 均衡控制字初始化B
		aqStateContrlC = 0xFFFF; // 均衡控制字初始化C
	}

	result1 = Get2_3_I(&healthStateContr1, &healthStateContr1B, &healthStateContr1C); //
	if (result1 == FALSE)
	{
		healthStateContr1 = 0xFFFF;	 // 健康状态字1控制字   默认 A
		healthStateContr1B = 0xFFFF; // 健康状态字1控制字   默认 B
		healthStateContr1C = 0xFFFF; // 健康状态字1控制字   默认 C
	}

	result1 = Get2_3_I(&healthStateContr2, &healthStateContr2B, &healthStateContr2C); //
	if (result1 == FALSE)
	{
		healthStateContr2 = 0xFFFF;	 // 健康状态字4控制字   默认 A
		healthStateContr2B = 0xFFFF; // 健康状态字4控制字   默认 B
		healthStateContr2C = 0xFFFF; // 健康状态字4控制字   默认 C
	}

	result1 = Get2_3_U(&equControlEn, &equControlEnB, &equControlEnC); //
	if (result1 == FALSE)
	{
		equControlEn = TRUE;  // 均衡使能状态A
		equControlEnB = TRUE; // 均衡使能状态B
		equControlEnC = TRUE; // 均衡使能状态C
	}

	result1 = Get2_3_U(&singleProtectChargeEn, &singleProtectChargeEnB, &singleProtectChargeEnC); //
	if (result1 == FALSE)
	{
		singleProtectChargeEn = FALSE;	//单体保护使能状态A
		singleProtectChargeEnB = FALSE; //单体保护使能状态B
		singleProtectChargeEnC = FALSE; //单体保护使能状态C
	}

	result1 = Get2_3_U(&overChargeProtectEn, &overChargeProtectEnB, &overChargeProtectEnC); //
	if (result1 == FALSE)
	{
		overChargeProtectEn = FALSE;  // 过充保护允许标志A
		overChargeProtectEnB = FALSE; // 过充保护允许标志B
		overChargeProtectEnC = FALSE; // 过充保护允许标志C
	}

	result1 = Get2_3_F(&singleVProtectValueUP, &singleVProtectValueUPB, &singleVProtectValueUPC); //
	if (result1 == FALSE)
	{
		singleVProtectValueUP = (float)4.20;  // 单体电压上限保护报警值A
		singleVProtectValueUPB = (float)4.20; // 单体电压上限保护报警值B
		singleVProtectValueUPC = (float)4.20; // 单体电压上限保护报警值C
	}

	result1 = Get2_3_F(&singleVWardValueUP, &singleVWardValueUPB, &singleVWardValueUPC); //
	if (result1 == FALSE)
	{
		singleVWardValueDOWN = (float)3.50;	 // 单体下限报警值A
		singleVWardValueDOWNB = (float)3.50; // 单体下限报警值B
		singleVWardValueDOWNC = (float)3.50; // 单体下限报警值C
	}

	result1 = Get2_3_F(&singleVWardValueDOWN, &singleVWardValueDOWNB, &singleVWardValueDOWNC); //
	if (result1 == FALSE)
	{
		singleVWardValueDOWN = 0;  // 单体下限报警值A
		singleVWardValueDOWNB = 0; // 单体下限报警值B
		singleVWardValueDOWNC = 0; // 单体下限报警值C
	}

	result1 = Get2_3_F(&batteryVAWardValueUP, &batteryVAWardValueUPB, &batteryVAWardValueUPC); //
	if (result1 == FALSE)
	{
		batteryVAWardValueUP = (float)37.8;	 // 蓄电池组电压上限保护值A
		batteryVAWardValueUPB = (float)37.8; // 蓄电池组电压上限保护值B
		batteryVAWardValueUPC = (float)37.8; // 蓄电池组电压上限保护值C
	}

	result1 = Get2_3_F(&batteryVAWardValueDOWN, &batteryVAWardValueDOWNB, &batteryVAWardValueDOWNC); //
	if (result1 == FALSE)
	{
		batteryVAWardValueDOWN = (float)27.0;  // 蓄电池组电压下限保护值A
		batteryVAWardValueDOWNB = (float)27.0; // 蓄电池组电压下限保护值B
		batteryVAWardValueDOWNC = (float)27.0; // 蓄电池组电压下限保护值C
	}
}

//*****************************************************************
// 函数名称 : SystemInit
//    功能描述 :
//*****************************************************************
void SystemInit(void)
{
	EA = 0; // 关中断
	dog = !dog;

	ONOFF_ADDR = 0xFF; // 输出指令码FFH
	onoffEn = 0;	   // 指令输出使能
	onoffEn = 1;	   // 指令输出禁能

	RXDCONTROL = 1; // 门控禁止
	RXD422AEn = 1;	// 接收禁止
	TXDCONTROL = 1; // 门控禁止
	TXD422En = 1;	// 发送禁止

	DA_A_CUR = 0; // 默认 0 - 5档
	DA_A_VOL = 0; // 默认 0 - 5档
	DA_B_CUR = 0; // 默认 0 - 5档
	DA_B_VOL = 0; // 默认 0 - 5档

	fgRstType = 1;		// 设置P1口输入状态
	if (fgRstType == 1) // 判断是冷复位或热复位
	{
		resetCount++;									 // 热启动计数自加		init
		resetCount = (unsigned char)(resetCount & 0x0F); // 计数清零操作		init

		resetCountB = resetCount; // B,C赋值		init
		resetCountC = resetCount; //				init
	}
	else
	{
		resetCount = 0;	 // 冷启动复位计数清零
		resetCountB = 0; // B,C赋值
		resetCountC = 0; // 初始化
	}

	DataInit(); // 默认数据初始化

	dog = !dog;	 // 增加喂狗操作
	ANCollect(); // 模拟量采集
	dog = !dog;	 // 增加喂狗操作

	StateWordBuild(); // 构成状态字
	SaveData();		  // 组帧函数 数据填充A/B区
	SaveData();		  // 组帧函数
	SaveRamDataRs();  // 填充RAM发送缓存区  -- 填充处理

	FilterSaveData(&filterFrame[0]); // 过滤参数遥测组帧  -- 填充处理
	FilterDataComp();				 // 过滤包数据对比  -- 填充处理

	ImportantSaveData(&rsImportantData[0]); // 重要数据缓存更新  -- 填充处理
	SaveImportantDataRs();					// 重要数据组帧更新  -- 填充处理

	CPUInit(); // 80C32初始化

	CANInit(BUSA_ADDR); // CAN总线初始化
	CANInit(BUSB_ADDR);
	CANInfoA();
	CANInfoB();

	EA = 1; // 开中断

	ImportantDataRx(); // 重要数据恢复处理
}

//*****************************************************************
// 函数名称 : DataInit
// 功能描述 :
// *****************************************************************
void DataInit(void)
{
	unsigned int i;
	unsigned int j;

	clockCount = 0; // 1s程序运行定时计数器		   	 init
	errorCount = 0; // 总线通讯不符合协议计数器	   	 init

	fg16S = 0; // 重要数据服务请求包 init
	fg30S = 0; // 均衡时间计数 init
	fg60S = 0; // 过滤报60s计时清零 init

	errorCountA = 0; // 总线通讯不符合协议计数器A  	   init
	errorCountB = 0; // 总线通讯不符合协议计数器B  	   init

	onoffExeCount = 0;	// 遥控指令执行计数A  	 init
	onoffExeCountB = 0; // 遥控指令执行计数B   	 init
	onoffExeCountC = 0; // 遥控指令执行计数C 	 init

	onoffErrorCount = 0;  // 指令错误计数A 	 init
	onoffErrorCountB = 0; // 指令错误计数B 	 init
	onoffErrorCountC = 0; // 指令错误计数C 	 init

	onoffReceCount = 0;	 // 指令接收计数A 	 init
	onoffReceCountB = 0; // 指令接收计数B 	 init
	onoffReceCountC = 0; // 指令接收计数C 	 init

	fgCanBus = 0xAA; // 默认A总线				   		 init

	fgSwitch1 = FALSE; // 数据采集保存切区标志1		   	 init
	fgSwitch2 = FALSE; // 数据采集保存切区标志2		   	 init

	equState1 = 0; // 均衡状态字1清零	init
	equState2 = 0; // 均衡状态字2清零	init

	genCurrent = (float)0.0; // 母线电流  init

	limbCurrent = (float)0.0;  // 太阳翼总电流 init
	limbCurrentA = (float)0.0; // 太阳翼A电流  init
	limbCurrentB = (float)0.0; // 太阳翼B电流  init

	chargeCurrent = (float)0.0;	 // 充电电流   init
	chargeCurrent1 = (float)0.0; // 充电电流1  init
	chargeCurrent2 = (float)0.0; // 充电电流2  init

	dischargeCurrent = (float)0.0;	// 放电电流     init
	dischargeCurrent1 = (float)0.0; // 放电电流1    init
	dischargeCurrent2 = (float)0.0; // 放电电流2    init
	meaV = 0.0;						// MEA清零		init
	generatrixVol = 0.0;			// 母线电压		init

	fgOverChargeShort = FALSE;	 // 充电阵短路故障 	 init
	fgOverChargePro = FALSE;	 // 充电阵单路故障 	 init
	fgOverChargeprotect = FALSE; // 蓄电池组过充标志 init

	singlesaveflag = FALSE; // init
	singlesaveVLevelA = 5;	// 默认5档
	singlesaveVLevelB = 5;	// 默认5档

	chargeFlag = FALSE; // 默认不充电状态

	for (i = 0; i < 10; i++)
	{
		chargeArrayCurrent[i] = 0.0; // 充电阵电流清零 init
		chargeArrayCount[i] = 0;	 // 充电阵计数清零 init
	}

	chargeVolKickA = 5;	  // 充电限压档数A A 	 init
	chargeVolKickA_B = 5; // 充电限压档数A B 	 init
	chargeVolKickA_C = 5; // 充电限压档数A C 	 init

	chargeVolKickB = 5;	  // 充电限压档数B A	 init
	chargeVolKickB_B = 5; // 充电限压档数B B 	 init
	chargeVolKickB_C = 5; // 充电限压档数B C 	 init

	chargeCurKickA = 5;	  // 充电限流档数A A	 init
	chargeCurKickA_B = 5; // 充电限流档数A B 	 init
	chargeCurKickA_C = 5; // 充电限流档数A C 	 init

	chargeCurKickB = 5;	  // 充电限流档数B A	 init
	chargeCurKickB_B = 5; // 充电限流档数B B 	 init
	chargeCurKickB_C = 5; // 充电限流档数B C 	 init

	chargeVolKickRsA = 5; // 充电切换档位A  遥测使用
	chargeVolKickRsB = 5; // 充电切换档位B  遥测使用

	for (i = 0; i < 41; i++)
	{
		filterFrame[i] = 0;	  // init
		filterDataRec[i] = 0; // init	 过滤包缓存区清零
	}

	for (i = 0; i < 128; i++) // 测试RAM区数据设置
	{
		ramTestData[i] = 0x20 + i; // init
	}

	fgImportantDataRx = FALSE; // 默认无重要数据恢复处理请求	 init
	importDataRxState = 0x00;  // 重要数据恢复状态  默认初始	 init

	for (i = 0; i < 220; i++) // 重要数据初始化设置
	{
		rsImportantData[i] = 0x00; // init
	}

	pcuState1 = 0; // PCU状态字1初始化  init
	pcuState2 = 0; // PCU状态字2初始化  init
	pcuState3 = 0; // PCU状态字3初始化  init
	pcuState4 = 0; // PCU状态字4初始化  init

	uploadParaData[0] = 0; // 参数上注清零 init
	uploadParaData[1] = 0;
	uploadParaData[2] = 0;
	uploadParaData[3] = 0;

	fgUploadPara = FALSE; // 默认无上注数据处理请求 init

	wrOnoffIndex = 0; // 写索引 init
	rdOnoffIndex = 0; // 读索引 init
	for (i = 0; i < 24; i++)
	{
		onoffBuffer[i][0] = 0; // 指令接收缓冲区	清零 init
		onoffBuffer[i][1] = 0; // 指令接收缓冲区	清零 init
	}

	wrOnoffRecIndex = 0;	 // 自主指令记录写索引清零 init
	for (i = 0; i < 24; i++) // 自主指令记录区清零
	{
		for (j = 0; j < 6; j++)
		{
			onoffBufferRec[i][j] = 0x00; //  init
		}
	}

	fgRsRam = FALSE; // 默认下传无效 init
	for (i = 0; i < 8; i++)
	{
		ramUploadData[i] = 0; // RAM上注数据清零 	init
	}

	for (i = 0; i < 12; i++) // 服务请求清零
	{
		reqData[i] = 0; // init
	}
	wrReqIndex = 0; // 写索引 init
	rdReqIndex = 0; // 读索引 init

	accEquVA = 0;  // 电池组电压主 init
	accEquVB = 0;  // 电池组电压备 init
	accVAHard = 0; // 电池组电压主采集值 init

	accVAHardPyh = (float)0.0; // 电池组电压主采集值 物理值  -- 下位机采集 init
	accVAEquPyh = (float)0.0;  // 电池组电压主采集值 物理值  -- 均衡器采集 init
	accVBEquPyh = (float)0.0;  // 电池组电压备采集值 物理值  -- 均衡器采集 init
	equVRefPhy = (float)0.0;   // 均衡器基准电压 物理值  	  -- 均衡器采集 init

	equ5VP = 0;	 // 均衡+5V电压  init
	equ12VP = 0; // 均衡+12V电压 init
	equ12VN = 0; // 均衡-12V电压 init
	equVRef = 0; // 均衡基准电压 init

	equOpenValue = 0.06;  // 均衡开门限值 init A
	equOpenValueB = 0.06; // 均衡开门限值 init B
	equOpenValueC = 0.06; // 均衡开门限值 init C

	equCloseValue = 0.01;  // 均衡关门限值 init A
	equCloseValueB = 0.01; // 均衡关门限值 init B
	equCloseValueC = 0.01; // 均衡关门限值 init C

	singleVProtectValueUP = (float)4.20;  // 单体电压上限保护报警值A  init
	singleVProtectValueUPB = (float)4.20; // 单体电压上限保护报警值B  init
	singleVProtectValueUPC = (float)4.20; // 单体电压上限保护报警值C  init

	singleVWardValueUP = (float)4.30;  // 单体电压上限报警值A  init
	singleVWardValueUPB = (float)4.30; // 单体电压上限报警值B  init
	singleVWardValueUPC = (float)4.30; // 单体电压上限报警值C  init

	singleVWardValueDOWN = (float)3.50;	 // 单体下限报警值A  init
	singleVWardValueDOWNB = (float)3.50; // 单体下限报警值B  init
	singleVWardValueDOWNC = (float)3.50; // 单体下限报警值C  init

	batteryVAWardValueUP = (float)37.8;	 // 蓄电池组电压上限保护值A  init
	batteryVAWardValueUPB = (float)37.8; // 蓄电池组电压上限保护值B  init
	batteryVAWardValueUPC = (float)37.8; // 蓄电池组电压上限保护值C  init

	batteryVAWardValueDOWN = (float)27.0;  // 蓄电池组电压下限保护值A  init
	batteryVAWardValueDOWNB = (float)27.0; // 蓄电池组电压下限保护值B  init
	batteryVAWardValueDOWNC = (float)27.0; // 蓄电池组电压下限保护值C  init

	wrEquOnoffIndex = 0; // 均衡指令写索引 	init
	rdEquOnoffIndex = 0; // 均衡指令读索引	init
	for (i = 0; i < 48; i++)
	{
		equOnoffBuffer[i] = 0; // init
	}

	fgEquRx = FALSE;		 // 均衡422接收标志 默认禁止  init
	fgEquTx = FALSE;		 // 均衡422发送标志 默认禁止  init
	for (i = 0; i < 11; i++) // 单体电压初始化
	{
		aq[i] = 0; // init
	}

	equRxCount = 0;			 // 均衡422接收计数  init
	for (i = 0; i < 40; i++) // 均衡器数据接收缓存
	{
		equDataBuffer[i] = 0; // init
	}

	for (i = 0; i < 9; i++)
	{
		equExeCount[i] = 0; // 均衡执行计数 init
	}

	aqStateContrl = 0xFFFF;	 // 均衡控制字初始化A init
	aqStateContrlB = 0xFFFF; // 均衡控制字初始化B init
	aqStateContrlC = 0xFFFF; // 均衡控制字初始化C init

	sepCurrentState1 = 0x00; // 分流保护通断状态 初始清零 init
	sepCurrentState2 = 0x00; // 分流保护通断状态 初始清零 init
	sepCurrentState3 = 0x00; // 分流保护通断状态 初始清零 init
	sepCurrentState4 = 0x00; // 分流保护通断状态 初始清零 init

	ntrPower = (float)150.0;  // 设置满电量 默认值为150 A init
	ntrPowerB = (float)150.0; // 设置满电量 默认值为150	B init
	ntrPowerC = (float)150.0; // 设置满电量 默认值为150	C init

	chargePower = (float)0.0;	 // 蓄电池组充电电量值	 init
	dischargePower = (float)0.0; // 蓄电池组放电电量值	 init
	currentPower = (float)150.0; // 蓄电池组当前电量值   init

	powerSave[0] = (unsigned char)((unsigned int)((currentPower * 1000) / 3.0) >> 8);	// 电量初始化   init
	powerSave[1] = (unsigned char)((unsigned int)((currentPower * 1000) / 3.0) & 0xFF); // init
	powerSave[2] = 0x00;
	powerSave[3] = 0x00;
	powerSave[4] = 0x00;
	powerSave[5] = 0x00;

	proportion = 102;  // 充放比	 init  A
	proportionB = 102; // 充放比	 init  B
	proportionC = 102; // 充放比	 init  C

	selfControlCount = 0x00; // 自主控制计数清零 init

	selfCheckStateWord = 0; // 自检状态字 init

	selfCheckStateContrl = 0xFF;  // 默认自检控制字 全有效  A init
	selfCheckStateContrlB = 0xFF; // 默认自检控制字 全有效  B init
	selfCheckStateContrlC = 0xFF; // 默认自检控制字 全有效  C init

	healthStateWord1 = 0x0000; // 健康状态字1	 init  		默认
	healthStateWord2 = 0x0000; // 健康状态字4	  init 		默认

	healthStateContr1 = 0xFFFF;	 // 健康状态字1控制字 init  默认 A
	healthStateContr1B = 0xFFFF; // 健康状态字1控制字 init  默认 B
	healthStateContr1C = 0xFFFF; // 健康状态字1控制字 init  默认 C

	healthStateContr2 = 0xFFFF;	 // 健康状态字4控制字 init   默认 A
	healthStateContr2B = 0xFFFF; // 健康状态字4控制字 init   默认 B
	healthStateContr2C = 0xFFFF; // 健康状态字4控制字 init   默认 C

	equControlEn = TRUE;  // 单体保护使能状态A	init
	equControlEnB = TRUE; // 单体保护使能状态B	init
	equControlEnC = TRUE; // 单体保护使能状态C	init

	singleProtectChargeEn = FALSE;	// 单体保护使能状态A	init
	singleProtectChargeEnB = FALSE; // 单体保护使能状态B	init
	singleProtectChargeEnC = FALSE; // 单体保护使能状态C	init

	overChargeProtectEn = FALSE;  // 过充保护允许标志A	init
	overChargeProtectEnB = FALSE; // 过充保护允许标志B	init
	overChargeProtectEnC = FALSE; // 过充保护允许标志C	init

	fgAh = FALSE;		 // 默认安时计未启动 init
	aqControlNumber = 9; // 默认控制9节参与 init

	for (i = 0; i < 3; i++)
	{
		currentModeCountA[i] = 0; // 电流模式计数A清零 init
		currentModeCountB[i] = 0; // 电流模式计数B清零 init
	}

	checkCount1 = 0; // 母线过压时间计数 		-- 健康监测计数1 init
	checkCount2 = 0; // 母线欠压时间计数 		-- 健康监测计数2 init
	checkCount3 = 0; // 蓄电池组过压时间计数 	-- 健康监测计数3 init
	checkCount4 = 0; // 蓄电池组欠压时间计数 	-- 健康监测计数4 init
	checkCount5 = 0; // 均衡基准电压时间计数 	-- 健康监测计数5 init
	checkCount6 = 0; // 均衡工作监测时间计数 	-- 健康监测计数6 init
	checkCount7 = 0; // 分流管短路时间计数 		-- 健康监测计数7 init
	checkCount8 = 0; // 供电自检时间计数 		-- 自检监测计数8 init

	fgCheck1 = FALSE; // 母线过压标志 			-- 健康标志1 init
	fgCheck2 = FALSE; // 母线欠压标志 			-- 健康标志2 init
	fgCheck3 = FALSE; // 蓄电池组电压过压标志 	-- 健康标志3 init
	fgCheck4 = FALSE; // 蓄电池组电压欠压标志 	-- 健康标志4 init
	fgCheck5 = FALSE; // 均衡基准电压检测标志 	-- 健康标志5 init
	fgCheck6 = FALSE; // 均衡工作检测标志 		-- 健康标志6 init
	fgCheck7 = FALSE; // 分流管短路检测标志 	-- 健康标志7 init
	fgCheck8 = FALSE; // 供电自检标志 			-- 标志 init

	for (i = 0; i < 9; i++) // 单体健康相关计数清零
	{
		checkCountAqA[i] = 0; // 单体欠压计数清零 init
		checkCountAqB[i] = 0; // 单体过压计数清零 init

		fgAqCheckA[i] = FALSE; // 单体欠压标志清零 init
		fgAqCheckB[i] = FALSE; // 单体过压标志清零 init
	}

	overChargeProCount = 0;		  // 过充保护执行计数 init
	singleProtectChargeCount = 0; // 过充保护执行计数 init

	sysTime = 0; // 时间初始化 init
}

/***************************************************************************************/
/* 函数介绍: AutoOnoff                                                                 */
/* 功能描述: 自动指令输出                                                              */
/* 修改记录:                                                                           */
/*                                                                                     */
/***************************************************************************************/
void AutoOnoff(void)
{
	DA_A_CUR = curCsTab[4]; // A组充电阵限流5
	Delay(3000);			// 延时	约20ms
	dog = !dog;

	DA_B_CUR = curCsTab[4]; // B组充电阵限流5
	Delay(3000);			// 延时	约20ms
	dog = !dog;

	DA_A_VOL = volCsTab[4]; // A组充电阵限压5
	Delay(3000);			// 延时	约20ms
	dog = !dog;

	DA_B_VOL = volCsTab[4]; // B组充电阵限压5
	Delay(3000);			// 延时	约20ms
	dog = !dog;

	ONOFFOutput(33); // BDRA1使能
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(37); // BDRA2使能
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(41); // BDRA3使能
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(45); // BDRA4使能
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(35); // BDRB1使能
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(39); // BDRB2使能
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(43); // BDRB3使能
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(47); // BDRB4使能
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(8); // A组充电阵接通
	Delay(3000);	// 延时	约20ms
	dog = !dog;

	ONOFFOutput(24); // B组充电阵接通
	Delay(3000);	 // 延时	约20ms
	dog = !dog;

	ONOFFOutput(9); // 充分模块1分流保护管接通
	Delay(3000);	// 延时	约20ms
	dog = !dog;		// 牵狗

	ONOFFOutput(16); // 充分模块2分流保护管接通
	Delay(3000);	 // 延时	约20ms
	dog = !dog;		 // 牵狗

	ONOFFOutput(25); // 充分模块3分流保护管接通
	Delay(3000);	 // 延时	约20ms
	dog = !dog;		 // 牵狗

	ONOFFOutput(32); // 充分模块4分流保护管接通
	Delay(3000);	 // 延时	约20ms
	dog = !dog;		 // 牵狗
}

//*****************************************************************
// 函数名称 : CANInit
// 功能描述 :
// *****************************************************************
void CANInit(unsigned int address)
{
	unsigned char temp;

	XBYTE[address] = 0x41; // 进入复位模式
	Delay(5);
	XBYTE[address + 1] = 0xEC;	  // 命令寄存器设置
	XBYTE[address + 4] = CAN_ACR; // 接受代码寄存器设置
	XBYTE[address + 5] = CAN_AMR; // 接受屏蔽寄存器设置
	XBYTE[address + 6] = 0x00;	  // 总线时序寄存器0设置  16M - 500K
	XBYTE[address + 7] = 0x1C;	  // 总线时序寄存器1设置
	XBYTE[address + 8] = 0x4A;	  // 输出控制寄存器设置
	XBYTE[address + 31] = 0x04;	  // 时钟分频寄存器设置

	XBYTE[address] = 0x46; // 中断的开启处理
	Delay(1);			   // 延时约31us
	XBYTE[address] = 0x46; // 重复进行工作模式（应对总线挂起处理）

	temp = XBYTE[address + 2]; // 清状态寄存器
	temp = XBYTE[address + 3]; // 清中断寄存器
	Delay(10);				   // 增加延时 回到工作模式
}

//*****************************************************************
// 函数名称 : CPUInit
// 功能描述 :
// *****************************************************************
void CPUInit(void)
{
	IP = 0x06; // 设置优先级: T0为高优先级、INT0为低优先级、INT1为高优先级
	IT0 = 0;   // 电平中断触发
	IT1 = 0;   // 电平中断触发

	TMOD = 0x01; // 定时器0设定工作方式1
	TH0 = 0xB8;
	TL0 = 0x00; // 定时器10ms定时
	TR0 = 1;	// 启动T0计数

	ET0 = 1; // 允许定时器0中断
	EX0 = 1; // 允许外部中断0中断
	EX1 = 1; // 允许外部中断1中断
}

//*****************************************************************
// 函数名称 : ANCollect
// 功能描述 :
// *****************************************************************
void ANCollect(void)
{
	unsigned char i;

	EquAncollect(); // 进行均衡器422接收		先进行均衡性数据采集处理!!!

	for (i = 0; i < 86; i++) // 采集全部遥测参数
	{
		anData[i] = ADConvert(rsTab[i]);
	}

	limbCurrentA = (float)(((float)anData[35] * K1) + b1) + (float)(((float)anData[37] * K3) + b3); // A翼太阳阵电流 = 充分模块1电流 + 充分模块3电流;
	if (limbCurrentA < 0)																			// 负数保护设置
	{
		limbCurrentA = 0;
	}

	limbCurrentB = (float)(((float)anData[36] * K2) + b2) + (float)(((float)anData[38] * K4) + b4); // B翼太阳阵电流 = 充分模块2电流 + 充分模块4电流;
	if (limbCurrentB < 0)																			// 负数保护设置
	{
		limbCurrentB = 0;
	}

	limbCurrent = limbCurrentA + limbCurrentB; // 太阳翼总电流

	genCurrent = (float)(((float)anData[0] * K5) + b5) + (float)(((float)anData[2] * K6) + b6) + (float)(((float)anData[1] * K7) + b7) + (float)(((float)anData[3] * K8) + b8); // 母线电流 = 母线电流1 + 2 + 3 + 4;
	if (genCurrent < 0)																																							// 负数保护设置
	{
		genCurrent = 0;
	}

	chargeCurrent1 = (float)(((float)anData[25] * K9) + b9);
	if (chargeCurrent1 < 0) // 负数保护设置
	{
		chargeCurrent1 = 0;
	}
	chargeCurrent2 = (float)(((float)anData[27] * K10) + b10);
	if (chargeCurrent2 < 0) // 负数保护设置
	{
		chargeCurrent2 = 0;
	}
	chargeCurrent = chargeCurrent1 + chargeCurrent2; // 充电电流

	dischargeCurrent1 = (float)(((float)anData[4] * K11) + b11); // 放电电流
	if (dischargeCurrent1 < 0)									 // 负数保护设置
	{
		dischargeCurrent1 = 0;
	}
	dischargeCurrent2 = (float)(((float)anData[6] * K12) + b12); // 放电电流
	if (dischargeCurrent2 < 0)									 // 负数保护设置
	{
		dischargeCurrent2 = 0;
	}
	dischargeCurrent = dischargeCurrent1 + dischargeCurrent2; // 放电电流

	meaV = (float)(((float)anData[48] * K23) + b23); // MEA
	if (meaV < 0)									 // 负数保护设置
	{
		meaV = 0;
	}

	generatrixVol = (float)(((float)anData[34] * K24) + b24); // 母线电压
	if (generatrixVol < 0)									  // 负数保护设置
	{
		generatrixVol = 0;
	}

	accVAHard = anData[5]; // 电池组电压主采集值更新

	accVAHardPyh = (float)(((float)accVAHard * K14) + b14); // 蓄电池组主份 物理值更新
	if (accVAHardPyh < 0)									// 负数保护设置
	{
		accVAHardPyh = 0;
	}

	chargeArrayCurrent[0] = (float)(((float)anData[68] * K18) + b18); // 充电阵A1工作电流
	if (chargeArrayCurrent[0] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[0] = 0;
	}
	chargeArrayCurrent[1] = (float)(((float)anData[70] * K19) + b19); // 充电阵A2工作电流
	if (chargeArrayCurrent[1] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[1] = 0;
	}
	chargeArrayCurrent[2] = (float)(((float)anData[69] * K20) + b20); // 充电阵A3工作电流
	if (chargeArrayCurrent[2] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[2] = 0;
	}
	chargeArrayCurrent[3] = (float)(((float)anData[72] * K21) + b21); // 充电阵A4工作电流
	if (chargeArrayCurrent[3] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[3] = 0;
	}
	chargeArrayCurrent[4] = (float)(((float)anData[71] * K22) + b22); // 充电阵A5工作电流
	if (chargeArrayCurrent[4] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[4] = 0;
	}

	chargeArrayCurrent[5] = (float)(((float)anData[73] * K18) + b18); // 充电阵B1工作电流
	if (chargeArrayCurrent[5] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[5] = 0;
	}
	chargeArrayCurrent[6] = (float)(((float)anData[76] * K19) + b19); // 充电阵B2工作电流
	if (chargeArrayCurrent[6] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[6] = 0;
	}
	chargeArrayCurrent[7] = (float)(((float)anData[74] * K20) + b20); // 充电阵B3工作电流
	if (chargeArrayCurrent[7] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[7] = 0;
	}
	chargeArrayCurrent[8] = (float)(((float)anData[78] * K21) + b21); // 充电阵B4工作电流
	if (chargeArrayCurrent[8] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[8] = 0;
	}
	chargeArrayCurrent[9] = (float)(((float)anData[75] * K22) + b22); // 充电阵B5工作电流
	if (chargeArrayCurrent[9] < 0)									  // 负数保护设置
	{
		chargeArrayCurrent[9] = 0;
	}

	sepCurrentState1 = SepCurrentStateCal(anData[82]); // 分流通断状态1   更新分流通断状态	 分流通断状态无系数
	sepCurrentState2 = SepCurrentStateCal(anData[81]); // 分流通断状态2
	sepCurrentState3 = SepCurrentStateCal(anData[77]); // 分流通断状态3
	sepCurrentState4 = SepCurrentStateCal(anData[83]); // 分流通断状态4
}

//*****************************************************************
// 函数名称 : EquAncollect
// 功能描述 :
// *****************************************************************
void EquAncollect(void)
{
	unsigned char i;
	unsigned char j;
	unsigned char tempU;
	float tempF;

	for (i = 0; i < 40; i++) // 均衡器接收缓存区清零
	{
		equDataBuffer[i] = 0;
	}

	RXDCONTROL = 1; // 门控禁止
	equRxCount = 0; // 接收计数清零

	fgEquRx = TRUE;		// 设置接收标志
	tempU = TXD422BUFF; // 清除接收缓存
	RXD422AEn = 0;		// 接收使能
	RXDCONTROL = 0;		// 门控有效

	dog = !dog;	  // 1s 牵狗一次
	Delay(20000); // 延时等待
	dog = !dog;	  // 1s 牵狗一次

	for (i = 0; i < 9; i++) // 读取单体电压及
	{
		aq[i] = (equDataBuffer[2 + 2 * i] * 256 + equDataBuffer[2 + 2 * i + 1]) & 0x0FFF;
	}

	accEquVA = (equDataBuffer[20] * 256 + equDataBuffer[21]) & 0x0FFF; // 电池组电压主
	accEquVB = (equDataBuffer[22] * 256 + equDataBuffer[23]) & 0x0FFF; // 电池组电压备
	equ12VP = (equDataBuffer[24] * 256 + equDataBuffer[25]) & 0x0FFF;  // 均衡+12V电压
	equ12VN = (equDataBuffer[26] * 256 + equDataBuffer[27]) & 0x0FFF;  // 均衡-12V电压
	equ5VP = (equDataBuffer[28] * 256 + equDataBuffer[29]) & 0x0FFF;   // 均衡+5V电压
	equVRef = (equDataBuffer[30] * 256 + equDataBuffer[31]) & 0x0FFF;  // 均衡基准电压

	equVRefPhy = (float)(((float)equVRef * K13) + b13);	  // 均衡器基准电压修正
	accVAEquPyh = (float)(((float)accEquVA * K16) + b16); // 电池组电压主采集值 物理值  -- 均衡器采集
	accVBEquPyh = (float)(((float)accEquVB * K17) + b17); // 电池组电压备采集值 物理值  -- 均衡器采集

	aqPhy[0] = (float)(((float)aq[0] * Ka1) + ba1); // 单体1电压物理值更新
	aqPhy[1] = (float)(((float)aq[1] * Ka2) + ba2); // 单体2电压物理值更新
	aqPhy[2] = (float)(((float)aq[2] * Ka3) + ba3); // 单体3电压物理值更新
	aqPhy[3] = (float)(((float)aq[3] * Ka4) + ba4); // 单体4电压物理值更新
	aqPhy[4] = (float)(((float)aq[4] * Ka5) + ba5); // 单体5电压物理值更新
	aqPhy[5] = (float)(((float)aq[5] * Ka6) + ba6); // 单体6电压物理值更新
	aqPhy[6] = (float)(((float)aq[6] * Ka7) + ba7); // 单体7电压物理值更新
	aqPhy[7] = (float)(((float)aq[7] * Ka8) + ba8); // 单体8电压物理值更新
	aqPhy[8] = (float)(((float)aq[8] * Ka9) + ba9); // 单体9电压物理值更新

	for (i = 0; i < 9; i++) // 更新排序后单体电压
	{
		aqPhyPx[i] = aqPhy[i];
	}

	for (i = 0; i < 8; i++) // 冒泡法
	{
		for (j = 1; j < (unsigned char)(9 - i); j++)
		{
			if (aqPhyPx[i] > aqPhyPx[i + j]) // 升序
			{
				tempF = aqPhyPx[i + j];
				aqPhyPx[i + j] = aqPhyPx[i];
				aqPhyPx[i] = tempF;
			}
		}
	}
}

//*****************************************************************
// 函数名称 : ADConvert
// 功能描述 :
// *****************************************************************
unsigned int ADConvert(unsigned char channel)
{
	unsigned char i; // 临时变量
	unsigned char j;
	unsigned int value;
	unsigned int xdata ad[10];

	AN_ADDR = channel; // 打开模拟量通道
	Delay(10);		   // 延时等待通道打开
	for (i = 0; i < 10; i++)
	{
		value = AD574S;														 // A/D启动转换
		Delay(6);															 // A/D转换延时
		ad[i] = (unsigned int)(((unsigned int)AD574H << 4) + (AD574L >> 4)); // D11-D0
	}
	for (i = 0; i < 9; i++)
	{												  // 从大到小排序 TData[9]--TData[0]
		for (j = 1; j < (unsigned char)(10 - i); j++) // 冒泡法排序
		{
			if (ad[i] > ad[i + j])
			{
				value = ad[i + j];
				ad[i + j] = ad[i];
				ad[i] = value;
			}
		}
	}
	value = 0;
	for (i = 1; i < 9; i++) // 剔除最大、最小值，取平均值
	{
		value = value + ad[i];
	}
	value = value + 4;	  // 四舍五入
	value = (value >> 4); // 取平均值
	return (value);
}

//********************************************************************
// 函数名称 : StateWordBuild
// 功能描述 :
// ********************************************************************
void StateWordBuild(void)
{
	pcuState1 = 0;
	if ((anData[39] >> 3) > 125) // 充电阵A1状态
	{
		pcuState1 = pcuState1 | 0x80;
	}
	if ((anData[41] >> 3) > 125) // 充电阵A2状态
	{
		pcuState1 = pcuState1 | 0x40;
	}
	if ((anData[43] >> 3) > 125) // 充电阵A3状态
	{
		pcuState1 = pcuState1 | 0x20;
	}
	if ((anData[44] >> 3) > 125) // 充电阵A4状态
	{
		pcuState1 = pcuState1 | 0x10;
	}
	if ((anData[45] >> 3) > 125) // 充电阵A5状态
	{
		pcuState1 = pcuState1 | 0x08;
	}
	if ((anData[46] >> 3) > 125) // 充电阵B1状态
	{
		pcuState1 = pcuState1 | 0x04;
	}
	if ((anData[47] >> 3) > 125) // 充电阵B2状态
	{
		pcuState1 = pcuState1 | 0x02;
	}
	if ((anData[49] >> 3) > 125) // 充电阵B3状态
	{
		pcuState1 = pcuState1 | 0x01;
	}

	pcuState2 = 0;
	if ((anData[50] >> 3) > 125) // 充电阵B4状态
	{
		pcuState2 = pcuState2 | 0x80;
	}
	if ((anData[51] >> 3) > 125) // 充电阵B5状态
	{
		pcuState2 = pcuState2 | 0x40;
	}
	if ((anData[28] >> 3) > 125) // 预接通开关1状态
	{
		pcuState2 = pcuState2 | 0x20;
	}
	if ((anData[30] >> 3) > 125) // 放电开关1-1状态
	{
		pcuState2 = pcuState2 | 0x10;
	}
	if ((anData[32] >> 3) > 125) // 放电开关1-2状态
	{
		pcuState2 = pcuState2 | 0x08;
	}
	if ((anData[33] >> 3) > 125) // 预接通开关2状态
	{
		pcuState2 = pcuState2 | 0x04;
	}
	if ((anData[29] >> 3) > 125) // 放电开关2-1状态
	{
		pcuState2 = pcuState2 | 0x02;
	}
	if ((anData[31] >> 3) > 125) // 放电开关2-2状态
	{
		pcuState2 = pcuState2 | 0x01;
	}

	pcuState3 = 0;
	if ((anData[8] >> 3) > 125) // BDRA1工作状态
	{
		pcuState3 = pcuState3 | 0x80;
	}
	if ((anData[12] >> 3) > 125) // BDRA2工作状态
	{
		pcuState3 = pcuState3 | 0x40;
	}
	if ((anData[16] >> 3) > 125) // BDRA3工作状态
	{
		pcuState3 = pcuState3 | 0x20;
	}
	if ((anData[20] >> 3) > 125) // BDRA4工作状态
	{
		pcuState3 = pcuState3 | 0x10;
	}
	if ((anData[10] >> 3) > 125) // BDRB1工作状态
	{
		pcuState3 = pcuState3 | 0x08;
	}
	if ((anData[14] >> 3) > 125) // BDRB2工作状态
	{
		pcuState3 = pcuState3 | 0x04;
	}
	if ((anData[18] >> 3) > 125) // BDRB3工作状态
	{
		pcuState3 = pcuState3 | 0x02;
	}
	if ((anData[22] >> 3) > 125) // BDRB4工作状态
	{
		pcuState3 = pcuState3 | 0x01;
	}

	pcuState4 = 0;
	if (fgOverChargeprotect == TRUE) // 蓄电池组过充状态	bit7
	{
		pcuState4 = pcuState4 | 0x80;
	}
	if (fgCheck4 == TRUE) // 蓄电池组欠压状态	bit6  -- 借用健康字标志
	{
		pcuState4 = pcuState4 | 0x40;
	}
	if (overChargeProtectEn == TRUE) // 过充保护使能状态	bit5
	{
		pcuState4 = pcuState4 | 0x20;
	}
	if (equControlEn == TRUE) // 均衡控制使能状态	bit4
	{
		pcuState4 = pcuState4 | 0x10;
	}
	if (singleProtectChargeEn == TRUE) // 单体保护充电控制使能状态	bit3
	{
		pcuState4 = pcuState4 | 0x08;
	}
	if (fgAh == TRUE) // 电量计启动标志	bit2
	{
		pcuState4 = pcuState4 | 0x04;
	}
	if (fgCanBus == 0xAA) // 总线使用标志	bit1
	{
		pcuState4 = pcuState4 | 0x02;
	}

	equState1 = 0;
	if ((equDataBuffer[0] & 0x01) == 0x01) // 均衡1状态
	{
		equState1 = equState1 | 0x80; // 均衡422状态1
	}
	if ((equDataBuffer[1] & 0x80) == 0x80) // 均衡2状态
	{
		equState1 = equState1 | 0x40; // 均衡422状态
	}
	if ((equDataBuffer[1] & 0x40) == 0x40) // 均衡3状态
	{
		equState1 = equState1 | 0x20; // 均衡422状态
	}
	if ((equDataBuffer[1] & 0x20) == 0x20) // 均衡4状态
	{
		equState1 = equState1 | 0x10; // 均衡422状态
	}
	if ((equDataBuffer[1] & 0x10) == 0x10) // 均衡5状态
	{
		equState1 = equState1 | 0x08; // 均衡422状态
	}
	if ((equDataBuffer[1] & 0x08) == 0x08) // 均衡6状态
	{
		equState1 = equState1 | 0x04; // 均衡422状态
	}
	if ((equDataBuffer[1] & 0x04) == 0x04) // 均衡7状态
	{
		equState1 = equState1 | 0x02; // 均衡422状态
	}
	if ((equDataBuffer[1] & 0x02) == 0x02) // 均衡8状态
	{
		equState1 = equState1 | 0x01; // 均衡422状态
	}

	equState2 = 0;						   // 均衡422状态2
	if ((equDataBuffer[1] & 0x01) == 0x01) // 均衡9状态
	{
		equState2 = equState2 | 0x80; // 均衡422状态
	}
}

//********************************************************************
// 函数名称 : SaveDataRs1
// 功能描述 :
// ********************************************************************
void SaveDataRs1(void) // 组帧函数--包1
{
	unsigned char *p1; // RS1
	unsigned char sum; // 累加和

	//----------------------CAN遥测组帧-------------------------------//
	if (fgSwitch1 == TRUE) // 组幀切换标志
	{
		p1 = &rsFrame1B[0]; // 指向RS1缓存区B
	}
	else
	{
		p1 = &rsFrame1A[0]; // 指向RS1缓存区A
	}

	sum = 0;
	//===============================================================//
	//----------------------RS包组帧--1 -----------------------------//
	*p1 = 0x9B; // 第1帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x00; // 帧计数
	p1++;
	*p1 = 0x2C; // 长度
	p1++;
	*p1 = 0x30; // title
	sum = sum + *p1;
	p1++;
	*p1 = healthStateWord1 >> 8; // 健康字1 - H W0
	sum = sum + *p1;
	p1++;
	*p1 = healthStateWord1; // 健康字1 - L  W1
	sum = sum + *p1;
	p1++;
	*p1 = healthStateWord2 >> 8; // 健康字2 - H  W2
	sum = sum + *p1;
	p1++;
	*p1 = healthStateWord2; // 健康字2 - L  W3
	sum = sum + *p1;
	p1++;
	*p1 = anData[34] >> 8; // 母线电压 - H  W4
	sum = sum + *p1;
	p1++;
	//----------------------RS包组帧--2 --------------------------//
	*p1 = 0x9B; // 第2帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x01; // 帧计数
	p1++;
	*p1 = anData[34]; // 母线电压 - L  W5
	sum = sum + *p1;
	p1++;
	*p1 = anData[0] >> 8; // 母线电流1 - H  W6
	sum = sum + *p1;
	p1++;
	*p1 = anData[0]; // 母线电流1 - L  W7
	sum = sum + *p1;
	p1++;
	*p1 = anData[2] >> 8; // 母线电流2 - H W8
	sum = sum + *p1;
	p1++;

	*p1 = anData[2]; // 母线电流2 - L  W9
	sum = sum + *p1;
	p1++;
	*p1 = anData[1] >> 8; // 母线电流3 - H  W10
	sum = sum + *p1;
	p1++;
	*p1 = anData[1]; //  母线电流3 - L  W11
	sum = sum + *p1;
	p1++;
	//----------------------RS包组帧--3 ---------------------------//
	*p1 = 0x9B; // 第3帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x02; // 帧计数
	p1++;
	*p1 = anData[3] >> 8; // 母线电流4 - H  W12
	sum = sum + *p1;
	p1++;
	*p1 = anData[3]; // 母线电流4 - L   W13
	sum = sum + *p1;
	p1++;
	*p1 = accEquVA >> 8; //  蓄电池电压主 - H  W14
	sum = sum + *p1;
	p1++;
	*p1 = accEquVA; //  蓄电池电压主 - L  W15
	sum = sum + *p1;
	p1++;
	*p1 = accEquVB >> 8; //  蓄电池组备 - H  W16
	sum = sum + *p1;
	p1++;
	*p1 = accEquVB; //  蓄电池组备 - L  W17
	sum = sum + *p1;
	p1++;
	*p1 = anData[4] >> 8; // 放电电流1 - H  W18
	sum = sum + *p1;
	p1++;
	//----------------------RS包组帧--4----------------------------//
	*p1 = 0x9B; // 第4帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x03; // 帧计数
	p1++;
	*p1 = anData[4]; // 放电电流1- L  W19
	sum = sum + *p1;
	p1++;
	*p1 = anData[25] >> 8; // 充电电流1 - H  W20
	sum = sum + *p1;
	p1++;
	*p1 = anData[25]; // 充电电流1 - L  W21
	sum = sum + *p1;
	p1++;
	*p1 = anData[6] >> 8; // 放电电流2 - H  W22
	sum = sum + *p1;
	p1++;
	*p1 = anData[6]; // 放电电流2 - L W23
	sum = sum + *p1;
	p1++;
	*p1 = anData[27] >> 8; // 充电电流2 - H  W24
	sum = sum + *p1;
	p1++;
	*p1 = anData[27]; // 充电电流2 - L  W25
	sum = sum + *p1;
	p1++;
	//-----------------------RS包组帧--5 -------------------------//
	*p1 = 0x9B; // 第5帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x04; // 帧计数
	p1++;
	*p1 = (unsigned int)((limbCurrentA * 100) + 0.5) >> 8; // A翼太阳阵电流 - H  W22    A * 100
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((limbCurrentA * 100) + 0.5); // A翼太阳阵电流 - L  W23
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((limbCurrentB * 100) + 0.5) >> 8; // B翼太阳阵电流 - H  W24
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((limbCurrentB * 100) + 0.5); // B翼太阳阵电流 - L  W25
	sum = sum + *p1;
	p1++;
	*p1 = anData[48] >> 8; // MEA电压 - H  W30
	sum = sum + *p1;
	p1++;
	*p1 = anData[48]; // MEA电压 - L  W31
	sum = sum + *p1;
	p1++;
	*p1 = anData[40] >> 8; // BEA电压1 - H  W32
	sum = sum + *p1;
	p1++;
	//----------------------RS包组帧--6 --------------------------//
	*p1 = 0x9B; // 第6帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x05; // 帧计数
	p1++;
	*p1 = anData[40]; // BEA电压1 - L  W33
	sum = sum + *p1;
	p1++;
	*p1 = anData[42] >> 8; //  BEA电压2 - H  W34
	sum = sum + *p1;
	p1++;
	*p1 = anData[42]; // BEA电压2 - L  W35
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((genCurrent * 100) + 0.5) >> 8; // 母线总电流 - H  W36
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((genCurrent * 100) + 0.5); // 母线总电流 - L  W37
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((chargeCurrent * 100) + 0.5) >> 8; // 充电总电流 - H  W38
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((chargeCurrent * 100) + 0.5); // 充电总电流 - L  W39
	sum = sum + *p1;
	p1++;
	//---------------------- RS包组帧--7  ----------------------------//
	*p1 = 0x9B; // 第7帧
	p1++;
	*p1 = 0x66;
	p1++;
	*p1 = 0x06; // 帧计数
	p1++;
	*p1 = (unsigned int)((dischargeCurrent * 100) + 0.5) >> 8; // 放电总电流 - H  W40
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((dischargeCurrent * 100) + 0.5); // 放电总电流 - L  W41
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((limbCurrent * 100) + 0.5) >> 8; // ̫太阳翼总电流 - H  W42
	sum = sum + *p1;
	p1++;
	*p1 = (unsigned int)((limbCurrent * 100) + 0.5); // ̫太阳翼总电流 - L  W43
	sum = sum + *p1;
	p1++;
	*p1 = sum; // SUM

	if (fgSwitch1 == TRUE) // 组幀切换标志
	{
		fgSwitch1 = FALSE; // 切换标志
	}
	else
	{
		fgSwitch1 = TRUE; // 切换标志
	}
}

//*****************************************************************
// 函数名称 : SaveDataRs2
// 功能描述 :
// *****************************************************************
void SaveDataRs2(void) // 组帧函数--包2
{
	unsigned char *p1; // RS1
	unsigned char sum; // 累加和

	//----------------------CAN遥测组帧-------------------------//
	if (fgSwitch2 == TRUE) // 组幀切换标志
	{
		p1 = &rsFrame2B[0]; // 指向缓存区B
	}
	else
	{
		p1 = &rsFrame2A[0]; // 指向缓存区A
	}

	sum = 0;

	//----------------------RS包组帧--1 ---------------------//
	*p1 = 0x9B; // 第1帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x00; // 帧计数
	p1++;
	*p1 = 0x6E; // 长度
	p1++;
	*p1 = 0x50; // title
	sum = sum + *p1;
	p1++;
	*p1 = anData[9] >> 8; // 放电输出电流A1 - H  W0
	sum = sum + *p1;
	p1++;
	*p1 = anData[9]; // 放电输出电流A1 - L W1
	sum = sum + *p1;
	p1++;
	*p1 = anData[13] >> 8; // 放电输出电流A2 - H  W2
	sum = sum + *p1;
	p1++;
	*p1 = anData[13]; // 放电输出电流A2 - L  W3
	sum = sum + *p1;
	p1++;
	*p1 = anData[17] >> 8; // 放电输出电流A3 - H  W4
	sum = sum + *p1;
	p1++;
	//----------------------RS包组帧--2 -----------------------------------------------------------//
	*p1 = 0x9B; // 第2帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x01; // 帧计数
	p1++;

	*p1 = anData[17]; // 放电输出电流A3 - L  W5
	sum = sum + *p1;
	p1++;
	*p1 = anData[21] >> 8; // 放电输出电流A4 - H  W6
	sum = sum + *p1;
	p1++;
	*p1 = anData[21]; // 放电输出电流A4 - L  W7
	sum = sum + *p1;
	p1++;
	*p1 = anData[11] >> 8; // 放电输出电流B1 - H  W8
	sum = sum + *p1;
	p1++;
	*p1 = anData[11]; // 放电输出电流B1 - L  W9
	sum = sum + *p1;
	p1++;
	*p1 = anData[15] >> 8; // 放电输出电流B2 - H  W10
	sum = sum + *p1;
	p1++;
	*p1 = anData[15]; // 放电输出电流B2 - L  W11
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--3 -----------------------------------------------------------//
	*p1 = 0x9B; // 第3帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x02; // 帧计数
	p1++;

	*p1 = anData[19] >> 8; // 放电输出电流B3 - H  W12
	sum = sum + *p1;
	p1++;
	*p1 = anData[19]; // 放电输出电流B3 - L  W13
	sum = sum + *p1;
	p1++;
	*p1 = anData[23] >> 8; // 放电输出电流B4 - H  W14
	sum = sum + *p1;
	p1++;
	*p1 = anData[23]; // 放电输出电流B4 - L  W15
	sum = sum + *p1;
	p1++;
	*p1 = anData[35] >> 8; // 充分模块1电流 - H  W16
	sum = sum + *p1;
	p1++;
	*p1 = anData[35]; // 充分模块1电流 - L  W17
	sum = sum + *p1;
	p1++;
	*p1 = anData[36] >> 8; // 充分模块2电流 - H  W18
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--4 ----------------------//
	*p1 = 0x9B; // 第4帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x03; // 帧计数
	p1++;

	*p1 = anData[36]; // 充分模块2电流 - L  W19
	sum = sum + *p1;
	p1++;
	*p1 = anData[37] >> 8; // 充分模块3电流 - H  W20
	sum = sum + *p1;
	p1++;
	*p1 = anData[37]; // 充分模块3电流 - L  W21
	sum = sum + *p1;
	p1++;
	*p1 = anData[38] >> 8; // 充分模块4电流 - H  W22
	sum = sum + *p1;
	p1++;
	*p1 = anData[38]; // 充分模块4电流 - L  W23
	sum = sum + *p1;
	p1++;
	*p1 = anData[68] >> 8; // 充电阵A1充电电流 - H  W24
	sum = sum + *p1;
	p1++;
	*p1 = anData[68]; // 充电阵A1充电电流 - L  W25
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--5 -----------------------------------------------------------//
	*p1 = 0x9B; // 第5帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x04; // 帧计数
	p1++;

	*p1 = anData[70] >> 8; // 充电阵A2充电电流 - H  W26
	sum = sum + *p1;
	p1++;
	*p1 = anData[70]; // 充电阵A2充电电流 - L  W27
	sum = sum + *p1;
	p1++;
	*p1 = anData[69] >> 8; // 充电阵A3充电电流 - H  W28
	sum = sum + *p1;
	p1++;
	*p1 = anData[69]; // 充电阵A3充电电流 - L  W29
	sum = sum + *p1;
	p1++;
	*p1 = anData[72] >> 8; // 充电阵A4充电电流 - H  W30
	sum = sum + *p1;
	p1++;
	*p1 = anData[72]; // 充电阵A4充电电流 - L  W31
	sum = sum + *p1;
	p1++;
	*p1 = anData[71] >> 8; // 充电阵A5充电电流 - H  W32
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--6 -----------------------------------------------------------//
	*p1 = 0x9B; // 第6帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x05; // 帧计数
	p1++;

	*p1 = anData[71]; // 充电阵A5充电电流 - L  W33
	sum = sum + *p1;
	p1++;
	*p1 = anData[73] >> 8; // 充电阵B1充电电流 - H  W34
	sum = sum + *p1;
	p1++;
	*p1 = anData[73]; // 充电阵B1充电电流 - L  W35
	sum = sum + *p1;
	p1++;
	*p1 = anData[76] >> 8; // 充电阵B2充电电流 - H  W36
	sum = sum + *p1;
	p1++;
	*p1 = anData[76]; // 充电阵B2充电电流 - L  W37
	sum = sum + *p1;
	p1++;
	*p1 = anData[74] >> 8; // 充电阵B3充电电流 - H  W38
	sum = sum + *p1;
	p1++;
	*p1 = anData[74]; // 充电阵B3充电电流 - L  W39
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--7 -----------------------------------------------------------//
	*p1 = 0x9B; // 第7帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x06; // 帧计数
	p1++;

	*p1 = anData[78] >> 8; // 充电阵B4充电电流 - H  W40
	sum = sum + *p1;
	p1++;
	*p1 = anData[78]; // 充电阵B4充电电流 - L  W41
	sum = sum + *p1;
	p1++;
	*p1 = anData[75] >> 8; // 充电阵B5充电电流 - H  W42
	sum = sum + *p1;
	p1++;
	*p1 = anData[75]; // 充电阵B5充电电流 - L  W43
	sum = sum + *p1;
	p1++;
	*p1 = anData[60] >> 8; // 太阳电池阵1电压 - H  W44
	sum = sum + *p1;
	p1++;
	*p1 = anData[60]; // 太阳电池阵1电压 - L  W45
	sum = sum + *p1;
	p1++;
	*p1 = anData[56] >> 8; // 太阳电池阵2电压 - H  W46
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--8 -----------------------------------------------------------//
	*p1 = 0x9B; // 第7帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x07; // 帧计数
	p1++;

	*p1 = anData[56]; // 太阳电池阵2电压 - L  W47
	sum = sum + *p1;
	p1++;
	*p1 = anData[52] >> 8; // 太阳电池阵3电压 - H  W48
	sum = sum + *p1;
	p1++;
	*p1 = anData[52]; // 太阳电池阵3电压 - L  W49
	sum = sum + *p1;
	p1++;
	*p1 = anData[64] >> 8; // 太阳电池阵4电压 - H  W50
	sum = sum + *p1;
	p1++;
	*p1 = anData[64]; // 太阳电池阵4电压 - L  W51
	sum = sum + *p1;
	p1++;
	*p1 = anData[61] >> 8; // 太阳电池阵5电压 - H  W52
	sum = sum + *p1;
	p1++;
	*p1 = anData[61]; // 太阳电池阵5电压 - L  W53
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--9 -----------------------------------------------------------//
	*p1 = 0x9B; // 第9帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x08; // 帧计数
	p1++;

	*p1 = anData[57] >> 8; // 太阳电池阵6电压 - H  W54
	sum = sum + *p1;
	p1++;
	*p1 = anData[57]; // 太阳电池阵6电压 - L  W55
	sum = sum + *p1;
	p1++;
	*p1 = anData[53] >> 8; // 太阳电池阵7电压 - H  W56
	sum = sum + *p1;
	p1++;
	*p1 = anData[53]; // 太阳电池阵7电压 - L  W57
	sum = sum + *p1;
	p1++;
	*p1 = anData[65] >> 8; // 太阳电池阵8电压 - H  W58
	sum = sum + *p1;
	p1++;
	*p1 = anData[65]; // 太阳电池阵8电压 - L  W59
	sum = sum + *p1;
	p1++;
	*p1 = anData[62] >> 8; // 太阳电池阵9电压 - H  W60
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--10 -----------------------------------------------------------//
	*p1 = 0x9B; // 第10帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x09; // 帧计数
	p1++;

	*p1 = anData[62]; // 太阳电池阵9电压 - L  W61
	sum = sum + *p1;
	p1++;
	*p1 = anData[58] >> 8; // 太阳电池阵10电压 - H  W62
	sum = sum + *p1;
	p1++;
	*p1 = anData[58]; // 太阳电池阵10电压 - L  W63
	sum = sum + *p1;
	p1++;
	*p1 = anData[54] >> 8; // 太阳电池阵11电压 - H  W64
	sum = sum + *p1;
	p1++;
	*p1 = anData[54]; // 太阳电池阵11电压 - L  W65
	sum = sum + *p1;
	p1++;
	*p1 = anData[66] >> 8; // 太阳电池阵12电压 - H  W66
	sum = sum + *p1;
	p1++;
	*p1 = anData[66]; // 太阳电池阵12电压 - L  W67
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--11 -----------------------------------------------------------//
	*p1 = 0x9B; // 第11帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x0A; // 帧计数
	p1++;

	*p1 = anData[63] >> 8; // 太阳电池阵13电压 - H  W68
	sum = sum + *p1;
	p1++;
	*p1 = anData[63]; // 太阳电池阵13电压 - L  W69
	sum = sum + *p1;
	p1++;
	*p1 = anData[59] >> 8; // 太阳电池阵14电压 - H  W70
	sum = sum + *p1;
	p1++;
	*p1 = anData[59]; // 太阳电池阵14电压 - L  W71
	sum = sum + *p1;
	p1++;
	*p1 = anData[55] >> 8; // 太阳电池阵15电压 - H  W72
	sum = sum + *p1;
	p1++;
	*p1 = anData[55]; // 太阳电池阵15电压 - L  W73
	sum = sum + *p1;
	p1++;
	*p1 = anData[67] >> 8; // 太阳电池阵16电压 - H  W74
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--12 -----------------------------------------------------------//
	*p1 = 0x9B; // 第12帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x0B; // 帧计数
	p1++;

	*p1 = anData[67]; // 太阳电池阵16电压 - L  W75 - W81
	sum = sum + *p1;
	p1++;
	*p1 = anData[79] >> 8; // 分流模块温度1 - H
	sum = sum + *p1;
	p1++;
	*p1 = anData[79]; // 分流模块温度1 - L
	sum = sum + *p1;
	p1++;
	*p1 = anData[80] >> 8; // 分流模块温度2 - H
	sum = sum + *p1;
	p1++;
	*p1 = anData[80]; // 分流模块温度2 - L
	sum = sum + *p1;
	p1++;
	*p1 = anData[24] >> 8; // 放电模块温度1 - H
	sum = sum + *p1;
	p1++;
	*p1 = anData[24]; // 放电模块温度1 - L
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--13 ----------------------//
	*p1 = 0x9B; // 第13帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x0C; // 帧计数
	p1++;

	*p1 = anData[26] >> 8; // 放电模块温度2 - H  W82 - W88
	sum = sum + *p1;
	p1++;
	*p1 = anData[26]; // 放电模块温度2 - L
	sum = sum + *p1;
	p1++;

	*p1 = powerSave[0]; // 当前电量-H  W84
	sum = sum + *p1;
	p1++;
	*p1 = powerSave[1]; // 当前电量-L  W85
	sum = sum + *p1;
	p1++;
	*p1 = powerSave[2]; // 充电电量-H  W86
	sum = sum + *p1;
	p1++;
	*p1 = powerSave[3]; // 充电电量-L  W87
	sum = sum + *p1;
	p1++;
	*p1 = powerSave[4]; // 放电电量-H  W88
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--14 -----------------------//
	*p1 = 0x9B; // 第14帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x0D; // 帧计数
	p1++;

	*p1 = powerSave[5]; // 放电电量-L  W89
	sum = sum + *p1;
	p1++;
	*p1 = aq[0] >> 8; //  单体电压1 - H W90
	sum = sum + *p1;
	p1++;
	*p1 = aq[0]; //  单体电压1 - L W91
	sum = sum + *p1;
	p1++;
	*p1 = aq[1] >> 8; //  单体电压2 - H W92
	sum = sum + *p1;
	p1++;
	*p1 = aq[1]; //  单体电压2 - L W93
	sum = sum + *p1;
	p1++;
	*p1 = aq[2] >> 8; // 单体电压3 - H W94
	sum = sum + *p1;
	p1++;
	*p1 = aq[2]; // 单体电压3 - L W95
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--15  ---------------------//
	*p1 = 0x9B; // 第15帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x0E; // ֡帧计数
	p1++;

	*p1 = aq[3] >> 8; //  单体电压4 - H   W96
	sum = sum + *p1;
	p1++;
	*p1 = aq[3]; //  单体电压4 - L   W97
	sum = sum + *p1;
	p1++;
	*p1 = aq[4] >> 8; //  单体电压5 - H   W98
	sum = sum + *p1;
	p1++;
	*p1 = aq[4]; //  单体电压5 - L   W99
	sum = sum + *p1;
	p1++;
	*p1 = aq[5] >> 8; //  单体电压6 - H    W100
	sum = sum + *p1;
	p1++;
	*p1 = aq[5]; //  单体电压6 - L    W101
	sum = sum + *p1;
	p1++;
	*p1 = aq[6] >> 8; //   单体电压7 - H   W102
	sum = sum + *p1;
	p1++;

	//----------------------RS包组帧--16 ----------------//
	*p1 = 0x9B; // 第16帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x0F; // 帧计数
	p1++;

	*p1 = aq[6]; //   单体电压7 - L   W103
	sum = sum + *p1;
	p1++;
	*p1 = aq[7] >> 8; //  单体电压8 - H  W104
	sum = sum + *p1;
	p1++;
	*p1 = aq[7]; //  单体电压8 - L   W105
	sum = sum + *p1;
	p1++;
	*p1 = aq[8] >> 8; //  单体电压9 - H   W106
	sum = sum + *p1;
	p1++;
	*p1 = aq[8]; //  单体电压9 - L    W107
	sum = sum + *p1;
	p1++;
	*p1 = equState1; // 均衡状态1    W108
	sum = sum + *p1;
	p1++;
	*p1 = equState2; // 均衡状态2     W109
	sum = sum + *p1;
	p1++;
	//----------------------RS包组帧--17 ----------------//
	*p1 = 0x9B; // 第17帧
	p1++;
	*p1 = 0x62;
	p1++;
	*p1 = 0x10; // 帧计数
	p1++;

	*p1 = sum; // SUM

	if (fgSwitch2 == TRUE) // 组幀切换标志
	{
		fgSwitch2 = FALSE; // 切换标志
	}
	else
	{
		fgSwitch2 = TRUE; // 切换标志
	}
}

//*****************************************************************
// 函数名称 : SaveDataRs3
// 功能描述 :
// *****************************************************************
void SaveDataRs3(void) // 组帧函数--包3
{
	unsigned char i; // 临时变量
	unsigned char j;

	unsigned char *p1; // 过滤包发送缓冲区
	unsigned char *p2; // 过滤包数据缓冲区
	unsigned char sum; // 累加和

	//----------------------CAN遥测组帧---------------------------------------------------------------//
	p1 = &rsFrame3[0];	  // 指向缓存区
	p2 = &filterFrame[0]; // 指向缓存区

	sum = 0;

	//----------------------RS包组帧--1 -----------------------------------------------------------//
	*p1 = 0x9B; // 第1帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x00; // 帧计数
	p1++;
	*p1 = 0x29; // 长度
	p1++;
	*p1 = 0x70; // title
	sum = sum + *p1;
	p1++;

	for (i = 0; i < 5; i++)
	{
		*p1 = *p2; // title
		sum = sum + *p1;
		p1++;
		p2++;
	}

	//----------------------RS包组帧--中间帧 -----------------------------------------------------------//
	for (j = 0; j < 5; j++) // W5 - W39
	{
		*p1 = 0x9B; // 第N帧
		p1++;
		*p1 = 0x68;
		p1++;
		*p1 = j + 1; // 帧计数
		p1++;

		for (i = 0; i < 7; i++)
		{
			*p1 = *p2;
			sum = sum + *p1;
			p1++;
			p2++;
		}
	}

	//----------------------RS包组帧--结束帧 -----------------------------------------------------------//
	*p1 = 0x9B; // 第N帧
	p1++;
	*p1 = 0x63;
	p1++;
	*p1 = 0x06; // 帧计数
	p1++;

	*p1 = *p2; // W40
	sum = sum + *p1;
	p1++;
	p2++;

	*p1 = sum;
}

//*****************************************************************
// 函数名称 : SaveRamDataRs
// 功能描述 :
// *****************************************************************
void SaveRamDataRs(void) // 组帧函数--RAM
{
	unsigned char i; // 变量
	unsigned char j; // 变量

	unsigned char *p1;		 // RS
	unsigned char sum;		 // 累加和
	unsigned int ramAddress; // 内存地址

	if (fgRsRam == TRUE) // 下传地址设置有效
	{
		fgRsRam = FALSE;

		ramAddress = ramUploadData[4] * 256 + ramUploadData[5]; // 设置内存地址

		//----------------------CAN遥测组帧-----------------------------------------------//
		p1 = &rsRamFrame[0]; // 指向RS缓存区RAM

		sum = 0;
		//================================================================================//

		//----------------------RS包组帧--1 -----------------------------------------------//
		*p1 = 0x9B; // 第1帧
		p1++;
		*p1 = 0x68;
		p1++;
		*p1 = 0x00; // 帧计数
		p1++;
		*p1 = 0x80; // 长度 128字节
		p1++;
		*p1 = 0x90; // title
		sum = sum + *p1;
		p1++;

		for (i = 0; i < 5; i++) // 内存地址1-5
		{
			*p1 = ramUploadData[i];
			sum = sum + *p1;
			p1++;
		}

		//----------------------RS包组帧--第1帧 -----------------------------//
		*p1 = 0x9B; // 第N帧
		p1++;
		*p1 = 0x68;
		p1++;
		*p1 = 1; // 帧计数
		p1++;

		for (i = 0; i < 3; i++) // 内存地址5-8
		{
			*p1 = ramUploadData[5 + i];
			sum = sum + *p1;
			p1++;
		}

		for (i = 0; i < 4; i++)
		{
			*p1 = XBYTE[ramAddress];
			sum = sum + *p1;
			p1++;
			ramAddress++;
		}

		//----------------------RS包组帧--中间帧 -----------------------------//
		for (j = 0; j < 16; j++)
		{
			*p1 = 0x9B; // 第N帧
			p1++;
			*p1 = 0x68;
			p1++;
			*p1 = j + 2; // 帧计数
			p1++;

			for (i = 0; i < 7; i++)
			{
				*p1 = XBYTE[ramAddress];
				sum = sum + *p1;
				p1++;
				ramAddress++;
			}
		}

		//----------------------RS包组帧--结束帧 -----------------------------------------------------------//
		*p1 = 0x9B; // 第N帧
		p1++;
		*p1 = 0x66;
		p1++;
		*p1 = 0x12; // 帧计数
		p1++;

		for (i = 0; i < 4; i++)
		{
			*p1 = XBYTE[ramAddress];
			sum = sum + *p1;
			p1++;
			ramAddress++;
		}

		*p1 = sum;
	}
}

//*****************************************************************
// 函数名称 : SaveImportantDataRs
// 功能描述 :
// *****************************************************************
void SaveImportantDataRs(void) // 组帧函数--重要数据
{
	unsigned char i; // 变量
	unsigned char j; // 变量

	unsigned char *p1; // RS
	unsigned char *p2; // data
	unsigned char sum; // 累加和

	//----------------------CAN遥测组帧---------------------------------------------------------------//
	p1 = &rsImportantFrame[0]; // 指向RS缓存区RAM
	p2 = &rsImportantData[0];  // 指向重要数据缓存

	sum = 0;
	//=============================================================================================//

	//----------------------RS包组帧--1 -----------------------------------------------------------//
	*p1 = 0x9B; // 第1帧
	p1++;
	*p1 = 0x68;
	p1++;
	*p1 = 0x00; // 帧计数
	p1++;
	*p1 = 0xDC; // 长度 220字节
	p1++;
	*p1 = 0xD0; // title
	sum = sum + *p1;
	p1++;

	for (i = 0; i < 5; i++)
	{
		*p1 = *p2;
		sum = sum + *p1;
		p1++;
		p2++;
	}

	//----------------------RS包组帧--中间帧 -----------------------------------------------------------//
	for (j = 0; j < 30; j++)
	{
		*p1 = 0x9B; // 第N帧
		p1++;
		*p1 = 0x68;
		p1++;
		*p1 = j + 1; // 帧计数
		p1++;

		for (i = 0; i < 7; i++)
		{
			*p1 = *p2;
			sum = sum + *p1;
			p1++;
			p2++;
		}
	}

	//----------------------RS包组帧--结束帧 -----------------------------------------------------------//
	*p1 = 0x9B; // 第N帧
	p1++;
	*p1 = 0x67;
	p1++;
	*p1 = 0x1F; // 帧计数
	p1++;

	for (i = 0; i < 5; i++)
	{
		*p1 = *p2;
		sum = sum + *p1;
		p1++;
		p2++;
	}

	*p1 = sum;
}

/***************************************************************************************/
/*  函数名称:  HealthCheck                                                             */
/*  功能描述:  健康监测                                                                 */
/*  修改记录:                                                                           */
/***************************************************************************************/
void SelfHealthCheck(void)
{
	unsigned char result;

	selfCheckStateWord = 0; // 自检状态字清零

	result = RamCheck();  // RAM自检
	if (RamCheck == TRUE) // 异常时
	{
		selfCheckStateWord = (selfCheckStateWord | 0x01); // 设置bit0 - 1
	}
	else
	{
		selfCheckStateWord = (selfCheckStateWord & 0xFE); // 设置bit0 - 0
	}

	PowerSupplyCheck();	  // 供电自检
	if (fgCheck8 == TRUE) // 异常时
	{
		selfCheckStateWord = (selfCheckStateWord | 0x02); // 设置bit1 - 1
	}
	else
	{
		selfCheckStateWord = (selfCheckStateWord & 0xFD); // 设置bit1 - 0
	}

	selfCheckStateWord = selfCheckStateContrl & selfCheckStateWord; // 控制字1相关
}

/***************************************************************************************/
/*  函数名称:  HealthCheck                                                             */
/*  功能描述:  健康监测                                                                 */
/*  修改记录:                                                                           */
/***************************************************************************************/
void HealthCheck(void)
{
	GenVolOverCheck();		// 母线过压判断
	GenVolUnderCheck();		// 母线欠压判断
	AhVolOverCheck();		// 蓄电池组电压过压
	AhVolUnderCheck();		// 蓄电池组电压欠压
	EquVRefCheck();			// 均衡基准判断
	EquWorkCheck();			// 均衡工作自检
	SepCurrentShortCheck(); // 分流管短路
	AqVolCheck();			// 单体电压自检

	AhVolOverCheck();

	HealthStateWordBuild(); // 健康字状态字组帧
}

/***************************************************************************************/
/*  函数名称:  GenVolOverCheck                                                         */
/*  功能描述:  健康字组帧                                                           */
/*  修改记录:                                                                          */
/***************************************************************************************/
void HealthStateWordBuild(void)
{
	healthStateWord1 = 0; // 健康字1 清零
	if (fgCheck1 == TRUE) // bit15 电源母线过压
	{
		healthStateWord1 = healthStateWord1 | 0x8000;
	}
	if (fgCheck2 == TRUE) // bit14 电源母线欠压
	{
		healthStateWord1 = healthStateWord1 | 0x4000;
	}
	if (fgCheck3 == TRUE) // bit13 蓄电池组电压过压
	{
		healthStateWord1 = healthStateWord1 | 0x2000;
	}
	if (fgCheck4 == TRUE) // bit12 蓄电池组电压欠压
	{
		healthStateWord1 = healthStateWord1 | 0x1000;
	}
	if (fgCheck5 == TRUE) // bit11 均衡器基准故障
	{
		healthStateWord1 = healthStateWord1 | 0x0800;
	}
	if (fgCheck6 == TRUE) // bit10 均衡器工作故障
	{
		healthStateWord1 = healthStateWord1 | 0x0400;
	}
	if (fgCheck7 == TRUE) // bit9 分流管短路故障
	{
		healthStateWord1 = healthStateWord1 | 0x0200;
	}
	if (fgOverChargeShort == TRUE) // bit8 充电管短路故障
	{
		healthStateWord1 = healthStateWord1 | 0x0100;
	}
	if (fgOverChargeprotect == TRUE) // bit7 充电故障
	{
		healthStateWord1 = healthStateWord1 | 0x0080;
	}

	healthStateWord1 = healthStateWord1 | (unsigned int)((overChargeProCount & 0x03) << 5);		  // bit6-5 蓄电池组过充保护记录
	healthStateWord1 = healthStateWord1 | (unsigned int)((singleProtectChargeCount & 0x03) << 3); // bit4-3 单体过充保护记录

	if (fgAqCheckA[0] == TRUE) // bit1 单体1欠压
	{
		healthStateWord1 = healthStateWord1 | 0x0002;
	}
	if (fgAqCheckA[1] == TRUE) // bit0 单体2欠压
	{
		healthStateWord1 = healthStateWord1 | 0x0001;
	}

	healthStateWord2 = 0;	   // 健康字2 清零
	if (fgAqCheckA[2] == TRUE) // bit15 单体3欠压
	{
		healthStateWord2 = healthStateWord2 | 0x8000;
	}
	if (fgAqCheckA[3] == TRUE) // bit14 单体4欠压
	{
		healthStateWord2 = healthStateWord2 | 0x4000;
	}
	if (fgAqCheckA[4] == TRUE) // bit13 单体5欠压
	{
		healthStateWord2 = healthStateWord2 | 0x2000;
	}
	if (fgAqCheckA[5] == TRUE) // bit12 单体6欠压
	{
		healthStateWord2 = healthStateWord2 | 0x1000;
	}
	if (fgAqCheckA[6] == TRUE) // bit11 单体7欠压
	{
		healthStateWord2 = healthStateWord2 | 0x0800;
	}
	if (fgAqCheckA[7] == TRUE) // bit10 单体8欠压
	{
		healthStateWord2 = healthStateWord2 | 0x0400;
	}
	if (fgAqCheckA[8] == TRUE) // bit9 单体9欠压
	{
		healthStateWord2 = healthStateWord2 | 0x0200;
	}
	if (fgAqCheckB[0] == TRUE) // bit8 单体1过压
	{
		healthStateWord2 = healthStateWord2 | 0x0100;
	}
	if (fgAqCheckB[1] == TRUE) // bit7 单体2过压
	{
		healthStateWord2 = healthStateWord2 | 0x0080;
	}
	if (fgAqCheckB[2] == TRUE) // bit6 单体3过压
	{
		healthStateWord2 = healthStateWord2 | 0x0040;
	}
	if (fgAqCheckB[3] == TRUE) // bit5 单体4过压
	{
		healthStateWord2 = healthStateWord2 | 0x0020;
	}
	if (fgAqCheckB[4] == TRUE) // bit4 单体5过压
	{
		healthStateWord2 = healthStateWord2 | 0x0010;
	}
	if (fgAqCheckB[5] == TRUE) // bit3 单体6过压
	{
		healthStateWord2 = healthStateWord2 | 0x0008;
	}
	if (fgAqCheckB[6] == TRUE) // bit2 单体7过压
	{
		healthStateWord2 = healthStateWord2 | 0x0004;
	}
	if (fgAqCheckB[7] == TRUE) // bit1 单体8过压
	{
		healthStateWord2 = healthStateWord2 | 0x0002;
	}
	if (fgAqCheckB[8] == TRUE) // bit0 单体9过压
	{
		healthStateWord2 = healthStateWord2 | 0x0001;
	}

	healthStateWord1 = healthStateWord1 & healthStateContr1; // 控制字1相关
	healthStateWord2 = healthStateWord2 & healthStateContr2; // 控制字2相关
}

/***************************************************************************************/
/*  函数名称:  GenVolOverCheck                                                         */
/*  功能描述:  电源母线过压                                                            */
/*  修改记录:                                                                          */
/***************************************************************************************/
void GenVolOverCheck(void)
{
	if (generatrixVol > 42.0) // 母线电压大于42V
	{
		checkCount1++;		 // 计数增加
		if (checkCount1 > 4) // 持续时间超4S
		{
			fgCheck1 = TRUE; // 自检异常 返回 TRUE - 异常范围
		}
	}
	else
	{
		fgCheck1 = FALSE; // 自检持续异常 返回 TRUE  禁止检测 恢复状态
		checkCount1 = 0;  // 清除计数
	}
}

/***************************************************************************************/
/*  函数名称:  GenVolUnderCheck                                                        */
/*  功能描述:  电源母线欠压                                                            */
/*  修改记录:                                                                          */
/***************************************************************************************/
void GenVolUnderCheck(void)
{
	if (generatrixVol < 38.0) // 母线电压小于38V
	{
		checkCount2++;		 // 计数增加
		if (checkCount2 > 4) // 持续时间超4S
		{
			fgCheck2 = TRUE; // 自检异常 返回 TRUE - 异常范围
		}
	}
	else
	{
		fgCheck2 = FALSE; // 自检持续异常 返回 TRUE  禁止检测 恢复状态
		checkCount2 = 0;  // 清除计数
	}
}

/***************************************************************************************/
/*  函数名称:  AhVolOverCheck                                                         */
/*  功能描述:  蓄电池组过压                                                           */
/*  修改记录:                                                                          */
/***************************************************************************************/
void AhVolOverCheck(void)
{
	if (accVAHardPyh > batteryVAWardValueUP) //  蓄电池组电压 大于 上限保护值
	{
		checkCount3++;		  // 计数增加
		if (checkCount3 > 16) // 持续时间超16S
		{
			fgCheck3 = TRUE; // 自检异常
		}
	}
	else
	{
		fgCheck3 = FALSE; // 自检持续异常
		checkCount3 = 0;  // 清除计数
	}
}

/***************************************************************************************/
/*  函数名称:  AhVolUnderCheck                                                         */
/*  功能描述:  蓄电池组欠压                                                           */
/*  修改记录:                                                                          */
/***************************************************************************************/
void AhVolUnderCheck(void)
{
	if (accVAHardPyh < batteryVAWardValueDOWN) //  蓄电池组电压 大于 上限保护值
	{
		checkCount4++;		  // 计数增加
		if (checkCount4 > 16) // 持续时间超16S
		{
			fgCheck4 = TRUE; // 自检异常
		}
	}
	else
	{
		fgCheck4 = FALSE; // 自检持续异常
		checkCount4 = 0;  // 清除计数
	}
}

/***************************************************************************************/
/*  函数名称:  EquVRefCheck                                                         */
/*  功能描述:  均衡器基准自检                                                           */
/*  修改记录:                                                                          */
/***************************************************************************************/
void EquVRefCheck(void)
{
	if ((equVRefPhy < 4.9) || (equVRefPhy > 5.1)) // 均衡器基准能否正常工作
	{
		checkCount5++;		 // 计数增加
		if (checkCount5 > 4) // 持续时间超4S
		{
			fgCheck5 = TRUE; // 自检异常
		}
	}
	else
	{
		fgCheck5 = FALSE; // 自检持续异常
		checkCount5 = 0;  // 清除计数
	}
}

/***************************************************************************************/
/*  函数名称:  EquWorkCheck                                                         */
/*  功能描述:  均衡器工作自检                                                           */
/*  修改记录:                                                                          */
/***************************************************************************************/
void EquWorkCheck(void)
{
	if ((aqPhyPx[4] > 4.5) || (aqPhyPx[4] < 3)) // 均衡器电压监测4节以上
	{
		checkCount6++;		 // 计数增加
		if (checkCount6 > 4) // 持续时间超4S
		{
			fgCheck6 = TRUE; // 自检异常
		}
	}
	else
	{
		fgCheck6 = FALSE; // 自检持续异常
		checkCount6 = 0;  // 清除计数
	}
}

/***************************************************************************************/
/*  函数名称:  SepCurrentShortCheck                                                    */
/*  功能描述:  分流管短路                                                              */
/*  修改记录:                                                                          */
/***************************************************************************************/
void SepCurrentShortCheck(void)
{
	unsigned char i;
	float xdata min;
	float xdata sunWingVol[16]; /* 太阳翼电压 				*/

	sunWingVol[0] = (float)(((float)anData[60] * Ks1) + bs1);	 /* 太阳翼电池组电压01	  	*/
	sunWingVol[1] = (float)(((float)anData[56] * Ks2) + bs2);	 /* 太阳翼电池组电压02	  	*/
	sunWingVol[2] = (float)(((float)anData[52] * Ks3) + bs3);	 /* 太阳翼电池组电压03	  	*/
	sunWingVol[3] = (float)(((float)anData[64] * Ks4) + bs4);	 /* 太阳翼电池组电压04	 	*/
	sunWingVol[4] = (float)(((float)anData[61] * Ks5) + bs5);	 /* 太阳翼电池组电压05	 	*/
	sunWingVol[5] = (float)(((float)anData[57] * Ks6) + bs6);	 /* 太阳翼电池组电压06	  	*/
	sunWingVol[6] = (float)(((float)anData[53] * Ks7) + bs7);	 /* 太阳翼电池组电压07	  	*/
	sunWingVol[7] = (float)(((float)anData[65] * Ks8) + bs8);	 /* 太阳翼电池组电压08	  	*/
	sunWingVol[8] = (float)(((float)anData[62] * Ks9) + bs9);	 /* 太阳翼电池组电压09	  	*/
	sunWingVol[9] = (float)(((float)anData[58] * Ks10) + bs10);	 /* 太阳翼电池组电压10	  	*/
	sunWingVol[10] = (float)(((float)anData[54] * Ks11) + bs11); /* 太阳翼电池组电压11	  	*/
	sunWingVol[11] = (float)(((float)anData[66] * Ks12) + bs12); /* 太阳翼电池组电压12	  	*/
	sunWingVol[12] = (float)(((float)anData[63] * Ks13) + bs13); /* 太阳翼电池组电压13	 	*/
	sunWingVol[13] = (float)(((float)anData[59] * Ks14) + bs14); /* 太阳翼电池组电压14	 	*/
	sunWingVol[14] = (float)(((float)anData[55] * Ks15) + bs15); /* 太阳翼电池组电压15	 	*/
	sunWingVol[15] = (float)(((float)anData[67] * Ks16) + bs16); /* 太阳翼电池组电压16	  	*/

	min = sunWingVol[0];
	for (i = 1; i < 16; i++) /* 排序  					*/
	{
		if (min > sunWingVol[i]) /* 参数传递 				*/
		{
			min = sunWingVol[i];
		}
	}

	if ((limbCurrent > 60.0) && (meaV < 7.5) && (min < 5.0)) /* 三个条件同时成立四分钟  	*/
	{
		checkCount7++;		   /* 计数增加  				*/
		if (checkCount7 > 240) /* 持续时间超4min  			*/
		{
			fgCheck7 = TRUE; /* 自检异常 返回 TRUE - 异常范围  */
		}
	}
	else
	{
		fgCheck7 = FALSE; /* 自检持续异常 返回 TRUE  禁止检测 恢复状态  	*/
		checkCount7 = 0;  /* 清除计数   									*/
	}
}

/********************************************************************/
/*  函数名称:  AqVolCheck                                        */
/*  功能描述:                                                        */
/*  修改记录:                                                        */
/*********************************************************************/
void AqVolCheck(void)
{
	unsigned char i;

	for (i = 0; i < 9; i++) // 单体执行判读  欠压
	{
		if (aqPhy[i] < singleVWardValueDOWN) // 单体过压
		{
			checkCountAqA[i]++;		   // 单体过压计数
			if (checkCountAqA[i] > 16) // 持续16s
			{
				fgAqCheckA[i] = TRUE; // 设置异常
			}
		}
		else
		{
			checkCountAqA[i] = 0;  // 单体欠压计数清零
			fgAqCheckA[i] = FALSE; // 设置正常
		}
	}

	for (i = 0; i < 9; i++) // 单体执行判读 过压
	{
		if (aqPhy[i] > singleVWardValueUP) // 单体过压
		{
			checkCountAqB[i]++;		   // 单体过压计数
			if (checkCountAqB[i] > 16) // 持续16s
			{
				fgAqCheckB[i] = TRUE; // 设置异常
			}
		}
		else
		{
			checkCountAqB[i] = 0;  // 单体过压计数清零
			fgAqCheckB[i] = FALSE; // 设置正常
		}
	}
}

//*****************************************************************
// 函数名称 : SaveData
// 功能描述 :
// *****************************************************************
void SaveData(void) // 组帧函数
{
	SaveDataRs1();	 // 组帧1 - 快包
	SaveDataRs2();	 // 组帧2 - 慢包
	SaveRamDataRs(); // RAM数据包组帧  （当设置内存地址后更新）
}

//*****************************************************************
// 函数名称 : CanResume
// 功能描述 :
// *****************************************************************
void CanResume(void)
{
	unsigned char aSR;
	unsigned char bSR;
	unsigned char aCR;
	unsigned char bCR;

	aCR = XBYTE[BUSA_ADDR]; // 读A、B总线SJA1000控制字
	bCR = XBYTE[BUSB_ADDR];

	aSR = XBYTE[BUSA_ADDR + 2]; // 读A、B总线SJA1000状态字
	bSR = XBYTE[BUSB_ADDR + 2];

	errorCount++; // 每秒执行计数，由中断进行清零

	if ((aSR & 0x02) != 0) // 判断A总线控制器数据溢出
	{
		EX0 = 0;			// 关闭中断
		CANInit(BUSA_ADDR); // 对A总线控制器重新初始化
		CANInfoA();
		EX0 = 1; // 开中断
	}

	if ((bSR & 0x02) != 0) // 判断B总线控制器数据溢出
	{
		EX0 = 0;			// 关闭中断
		CANInit(BUSB_ADDR); // 对B总线控制器重新初始化
		CANInfoB();
		EX0 = 1; // 开中断
	}

	if (((aSR & 0x80) == 0x80) || ((aCR & 0x02) == 0x00)) // 判断A总线挂起 或 A控制器接收无效
	{
		errorCountA++; // A总线计数+1

		if (errorCountA > 15) // 持续16秒
		{
			EX0 = 0;			// 关闭中断
			CANInit(BUSA_ADDR); // 对A总线控制器重新初始化
			CANInfoA();
			errorCountA = 0;
			EX0 = 1;
			EA = 1;
		}
	}
	else // 条件不满足时
	{
		errorCountA = 0;
	}

	if (((bSR & 0x80) == 0x80) || ((bCR & 0x02) == 0x00)) // 判断B总线挂起 或 B控制器接收无效
	{
		errorCountB++; // B总线计数+1

		if (errorCountB > 15) // 持续16秒
		{
			EX0 = 0;			// 关闭中断
			CANInit(BUSB_ADDR); // 对B总线控制器重新初始化
			CANInfoB();
			errorCountB = 0;
			EX0 = 1;
			EA = 1;
		}
	}
	else // 条件不满足时
	{
		errorCountB = 0;
	}

	if (errorCount >= SILENT_TIME) // 不符合协议时间超限
	{
		Delay(50);
		EX0 = 0;			// 关闭中断
		CANInit(BUSA_ADDR); // 对A总线控制器重新初始化
		CANInit(BUSB_ADDR); // 对B总线控制器重新初始化
		errorCount = 0;
		CANInfoA();
		CANInfoB();
		EX0 = 1; // 增加开中断操作
		EA = 1;
	}
}

//*****************************************************************
// 函数名称 : ONOFFOutput
// 功能描述 :
// *****************************************************************
void ONOFFOutput(unsigned char index)
{
	ONOFF_ADDR = onoffTab[index - 1]; // 指令路序
	onoffEn = 0;					  // 使能允许
	Delay(5600);					  // 延时约40ms
	Delay(5600);					  // 延时约40ms
	ONOFF_ADDR = 0xFF;				  // 地址无效
	onoffEn = 1;					  // 使能禁止
}

//*****************************************************************
// 函数名称 : ONOFFHook
// 功能描述 :
// *****************************************************************
void ONOFFHook(void)
{
	unsigned int i;

	if (rdOnoffIndex != wrOnoffIndex)
	{
		onoffReceCount++;  // 接收指令计数++
		onoffReceCountB++; // 接收指令计数B++
		onoffReceCountC++; // 接收指令计数C++

		switch (onoffBuffer[rdOnoffIndex][0])
		{
		case 0x01: // BDRA1使能
			ONOFFOutput(33);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x02: // BDRA2使能
			ONOFFOutput(37);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x03: // BDRA3使能
			ONOFFOutput(41);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x04: // BDRA4使能
			ONOFFOutput(45);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x05: // BDRB1使能
			ONOFFOutput(35);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x06: // BDRB2使能
			ONOFFOutput(39);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x07: // BDRB3使能
			ONOFFOutput(43);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x08: // BDRB4使能
			ONOFFOutput(47);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x09: // BDRA1禁止
			ONOFFOutput(34);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x0A: // BDRA2禁止
			ONOFFOutput(38);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x0B: // BDRA3禁止
			ONOFFOutput(42);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x0C: // BDRA4禁止
			ONOFFOutput(46);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x0D: // BDRB1禁止
			ONOFFOutput(36);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x0E: // BDRB2禁止
			ONOFFOutput(40);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x0F: // BDRB3禁止
			ONOFFOutput(44);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x10: // BDRB4禁止
			ONOFFOutput(48);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x11: // 充电阵A1断开
			ONOFFOutput(5);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x12: // 充电阵A2断开
			ONOFFOutput(14);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x13: // 充电阵A3断开
			ONOFFOutput(6);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x14: // 充电阵A4断开
			ONOFFOutput(15);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x15: // 充电阵A5断开
			ONOFFOutput(7);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x16: // A组充电阵接通
			ONOFFOutput(8);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x17: // 充电阵B1断开
			ONOFFOutput(21);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x18: // 充电阵B2断开
			ONOFFOutput(30);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x19: // 充电阵B3断开
			ONOFFOutput(22);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x1A: // 充电阵B4断开
			ONOFFOutput(31);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x1B: // 充电阵B5断开
			ONOFFOutput(23);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x1C: // B组充电阵接通
			ONOFFOutput(24);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x1D:					// A组充电限压1
			DA_A_VOL = volCsTab[0]; // A组电压
			chargeVolKickA = 1;
			chargeVolKickA_B = 1;
			chargeVolKickA_C = 1;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x1E:					// A组充电限压2
			DA_A_VOL = volCsTab[1]; // A组电压
			chargeVolKickA = 2;
			chargeVolKickA_B = 2;
			chargeVolKickA_C = 2;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x1F:					// A组充电限压3
			DA_A_VOL = volCsTab[2]; // A组电压
			chargeVolKickA = 3;
			chargeVolKickA_B = 3;
			chargeVolKickA_C = 3;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x20:					// A组充电限压4
			DA_A_VOL = volCsTab[3]; // A组电压
			chargeVolKickA = 4;
			chargeVolKickA_B = 4;
			chargeVolKickA_C = 4;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x21:					// A组充电限压5
			DA_A_VOL = volCsTab[4]; // A组电压
			chargeVolKickA = 5;
			chargeVolKickA_B = 5;
			chargeVolKickA_C = 5;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x22:					// A组充电限压6
			DA_A_VOL = volCsTab[5]; // A组电压
			chargeVolKickA = 6;
			chargeVolKickA_B = 6;
			chargeVolKickA_C = 6;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x23:					// A组充电限压7
			DA_A_VOL = volCsTab[6]; // A组电压
			chargeVolKickA = 7;
			chargeVolKickA_B = 7;
			chargeVolKickA_C = 7;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x24:					// A组充电限压8
			DA_A_VOL = volCsTab[7]; // A组电压
			chargeVolKickA = 8;
			chargeVolKickA_B = 8;
			chargeVolKickA_C = 8;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x25:					// A组充电限流1
			DA_A_CUR = curCsTab[0]; // 设置限流档位
			chargeCurKickA = 1;
			chargeCurKickA_B = 1;
			chargeCurKickA_C = 1;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x26:					// A组充电限流2
			DA_A_CUR = curCsTab[1]; // 设置限流档位
			chargeCurKickA = 2;
			chargeCurKickA_B = 2;
			chargeCurKickA_C = 2;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x27:					// A组充电限流3
			DA_A_CUR = curCsTab[2]; // 设置限流档位
			chargeCurKickA = 3;
			chargeCurKickA_B = 3;
			chargeCurKickA_C = 3;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x28:					// A组充电限流4
			DA_A_CUR = curCsTab[3]; // 设置限流档位
			chargeCurKickA = 4;
			chargeCurKickA_B = 4;
			chargeCurKickA_C = 4;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x29:					// A组充电限流5
			DA_A_CUR = curCsTab[4]; // 设置限流档位
			chargeCurKickA = 5;
			chargeCurKickA_B = 5;
			chargeCurKickA_C = 5;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x2A:					// A组充电限流6
			DA_A_CUR = curCsTab[5]; // 设置限流档位
			chargeCurKickA = 6;
			chargeCurKickA_B = 6;
			chargeCurKickA_C = 6;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x2B:					// A组充电限流7
			DA_A_CUR = curCsTab[6]; // 设置限流档位
			chargeCurKickA = 7;
			chargeCurKickA_B = 7;
			chargeCurKickA_C = 7;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x2C:					// A组充电限流8
			DA_A_CUR = curCsTab[7]; // 设置限流档位
			chargeCurKickA = 8;
			chargeCurKickA_B = 8;
			chargeCurKickA_C = 8;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x2D:					// B组充电限压1
			DA_B_VOL = volCsTab[0]; // B组电压
			chargeVolKickB = 1;
			chargeVolKickB_B = 1;
			chargeVolKickB_C = 1;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x2E:					// B组充电限压2
			DA_B_VOL = volCsTab[1]; // B组电压
			chargeVolKickB = 2;
			chargeVolKickB_B = 2;
			chargeVolKickB_C = 2;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x2F:					// B组充电限压3
			DA_B_VOL = volCsTab[2]; // B组电压
			chargeVolKickB = 3;
			chargeVolKickB_B = 3;
			chargeVolKickB_C = 3;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x30:					// B组充电限压4
			DA_B_VOL = volCsTab[3]; // B组电压
			chargeVolKickB = 4;
			chargeVolKickB_B = 4;
			chargeVolKickB_C = 4;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x31:					// B组充电限压5
			DA_B_VOL = volCsTab[4]; // B组电压
			chargeVolKickB = 5;
			chargeVolKickB_B = 5;
			chargeVolKickB_C = 5;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x32:					// B组充电限压6
			DA_B_VOL = volCsTab[5]; // B组电压
			chargeVolKickB = 6;
			chargeVolKickB_B = 6;
			chargeVolKickB_C = 6;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x33:					// B组充电限压7
			DA_B_VOL = volCsTab[6]; // B组电压
			chargeVolKickB = 7;
			chargeVolKickB_B = 7;
			chargeVolKickB_C = 7;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x34:					// B组充电限压8
			DA_B_VOL = volCsTab[7]; // B组电压
			chargeVolKickB = 8;
			chargeVolKickB_B = 8;
			chargeVolKickB_C = 8;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x35:					// B组充电限流1
			DA_B_CUR = curCsTab[0]; // 设置限流档位
			chargeCurKickB = 1;
			chargeCurKickB_B = 1;
			chargeCurKickB_C = 1;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x36:					// B组充电限流2
			DA_B_CUR = curCsTab[1]; // 设置限流档位
			chargeCurKickB = 2;
			chargeCurKickB_B = 2;
			chargeCurKickB_C = 2;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x37:					// B组充电限流3
			DA_B_CUR = curCsTab[2]; // 设置限流档位
			chargeCurKickB = 3;
			chargeCurKickB_B = 3;
			chargeCurKickB_C = 3;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x38:					// B组充电限流4
			DA_B_CUR = curCsTab[3]; // 设置限流档位
			chargeCurKickB = 4;
			chargeCurKickB_B = 4;
			chargeCurKickB_C = 4;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x39:					// B组充电限流5
			DA_B_CUR = curCsTab[4]; // 设置限流档位
			chargeCurKickB = 5;
			chargeCurKickB_B = 5;
			chargeCurKickB_C = 5;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x3A:					// B组充电限流6
			DA_B_CUR = curCsTab[5]; // 设置限流档位
			chargeCurKickB = 6;
			chargeCurKickB_B = 6;
			chargeCurKickB_C = 6;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x3B:					// B组充电限流7
			DA_B_CUR = curCsTab[6]; // 设置限流档位
			chargeCurKickB = 7;
			chargeCurKickB_B = 7;
			chargeCurKickB_C = 7;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x3C:					// B组充电限流8
			DA_B_CUR = curCsTab[7]; // 设置限流档位
			chargeCurKickB = 8;
			chargeCurKickB_B = 8;
			chargeCurKickB_C = 8;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x3D: // 分流保护管1断开
			ONOFFOutput(1);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x3E: // 分流保护管2断开
			ONOFFOutput(17);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x3F: // 分流保护管3断开
			ONOFFOutput(10);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x40: // 分流保护管4断开
			ONOFFOutput(26);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x41: // 分流保护管5断开
			ONOFFOutput(2);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x42: // 分流保护管6断开
			ONOFFOutput(18);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x43: // 分流保护管7断开
			ONOFFOutput(11);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x44: // 分流保护管8断开
			ONOFFOutput(27);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x45: // 分流保护管9断开
			ONOFFOutput(3);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x46: // 分流保护管10断开
			ONOFFOutput(19);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x47: // 分流保护管11断开
			ONOFFOutput(12);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x48: // 分流保护管12断开
			ONOFFOutput(28);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x49: // 分流保护管13断开
			ONOFFOutput(4);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x4A: // 分流保护管14断开
			ONOFFOutput(20);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x4B: // 分流保护管15断开
			ONOFFOutput(13);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x4C: // 分流保护管16断开
			ONOFFOutput(29);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x4D: // 充分模块1分流保护管接通
			ONOFFOutput(9);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x4E: // 充分模块2分流保护管接通
			ONOFFOutput(16);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x4F: // 充分模块3分流保护管接通
			ONOFFOutput(25);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x50: // 充分模块4分流保护管接通
			ONOFFOutput(32);

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x51: // 单体1均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[0];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x52: // 单体1均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[0];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x53: // 单体2均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[1];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x54: // 单体2均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[1];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x55: // 单体3均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[2];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x56: // 单体3均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[2];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x57: // 单体4均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[3];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x58: // 单体4均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[3];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x59: // 单体5均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[4];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x5A: // 单体5均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[4];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x5B: // 单体6均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[5];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x5C: // 单体6均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[5];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x5D: // 单体7均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[6];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x5E: // 单体7均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[6];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x5F: // 单体8均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[7];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x60: // 单体8均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[7];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x61: // 单体9均衡通
			equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[8];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x62: // 单体9均衡断
			equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[8];
			wrEquOnoffIndex++;
			if (wrEquOnoffIndex > 47) // 循环指回
			{
				wrEquOnoffIndex = 0;
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x63: // 预留1

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x64: // 预留2

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x65: // 蓄电池过充保护使能
			overChargeProtectEn = TRUE;
			overChargeProtectEnB = TRUE;
			overChargeProtectEnC = TRUE;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x66: // 蓄电池过充保护禁止
			overChargeProtectEn = FALSE;
			overChargeProtectEnB = FALSE;
			overChargeProtectEnC = FALSE;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x67: // 均衡控制使能
			equControlEn = TRUE;
			equControlEnB = TRUE;
			equControlEnC = TRUE;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;
		case 0x68: // 均衡控制禁止
			equControlEn = FALSE;
			equControlEnB = FALSE;
			equControlEnC = FALSE;

			for (i = 0; i < 9; i++) // 同时发送全部均衡断开指令
			{
				equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[i];
				wrEquOnoffIndex++;
				if (wrEquOnoffIndex > 47) // 循环指回
				{
					wrEquOnoffIndex = 0;
				}
			}

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x69: // 蓄电池单体保护充电使能
			singleProtectChargeEn = TRUE;
			singleProtectChargeEnB = TRUE;
			singleProtectChargeEnC = TRUE;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x6A: // 蓄电池单体保护充电禁止
			singleProtectChargeEn = FALSE;
			singleProtectChargeEnB = FALSE;
			singleProtectChargeEnC = FALSE;

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x6B:				// CAN-A总线复位
			EX0 = 0;			// 关闭中断
			CANInit(BUSA_ADDR); // 对A总线控制器重新初始化
			CANInfoA();
			EX0 = 1; // 开中断

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		case 0x6C:				// CAN-B总线复位
			EX0 = 0;			// 关闭中断
			CANInit(BUSB_ADDR); // 对B总线控制器重新初始化
			CANInfoB();
			EX0 = 1; // 开中断

			onoffExeCount++; // 指令执行计数加1
			onoffExeCountB++;
			onoffExeCountC++;
			break;

		default:
			onoffErrorCount++;	// 错误指令计数++
			onoffErrorCountB++; // 错误指令计数B++
			onoffErrorCountC++; // 错误指令计数C++
			break;
		}

		onoffBuffer[rdOnoffIndex][0] = 0x00; // 指令码清零1
		onoffBuffer[rdOnoffIndex][1] = 0x00; // 指令码清零2

		rdOnoffIndex++;
		if (rdOnoffIndex >= 24)
		{
			rdOnoffIndex = 0;
		}
	}
}

//*****************************************************************
// 函数名称 : Get2_3_U
// 功能描述 :
// *****************************************************************
unsigned char Get2_3_U(unsigned char *a, unsigned char *b, unsigned char *c)
{
	if ((*a == *b) && (*b == *c)) // 三者一致 返回
	{
		return (TRUE); // 正确返回
	}
	else
	{
		if (*a == *b) // 任意两者一致 返回
		{
			EA = 0; // 关闭中断进行数据纠错
			*c = *a;
			EA = 1;
			return (TRUE); // 正确返回
		}
		else
		{
			if (*a == *c)
			{
				EA = 0; // 关闭中断进行数据纠错
				*b = *a;
				EA = 1;
				return (TRUE);
			}
			else
			{
				if (*b == *c)
				{
					EA = 0; // 关闭中断进行数据纠错
					*a = *b;
					EA = 1;
					return (TRUE);
				}
				else // 任意两者不一致 返回
				{
					return (FALSE); // 错误返回
				}
			}
		}
	}
}

/***************************************************************************************/
/* 函数介绍: 2/3判决函数(float使用)                                                    */
/* 功能描述:                                                                           */
/* 修改记录:                                                                           */
/*                                                                                     */
/***************************************************************************************/
unsigned char Get2_3_F(float *a, float *b, float *c)
{
	float xdata temp1;
	float xdata temp2;
	float xdata temp3;

	temp1 = fabs(*a - *b);
	temp2 = fabs(*a - *c);
	temp3 = fabs(*b - *c);

	if ((temp1 < 0.0001) && (temp2 < 0.0001) && (temp3 < 0.0001)) /* 三者一致 返回 */
	{
		return (TRUE); /* 正确返回TRUE     */
	}
	else
	{
		if (temp1 < 0.0001) /* 两者一致 返回 */
		{
			*c = *a;
			return (TRUE); /* 正确返回TRUE */
		}
		else
		{
			if (temp2 < 0.0001)
			{
				*b = *a;
				return (TRUE);
			}
			else
			{
				if (temp3 < 0.0001)
				{
					*a = *b;
					return (TRUE);
				}
				else /* 三者不一致 返回 */
				{
					return (FALSE); /* 错误返回FALSE */
				}
			}
		}
	}
}

/***************************************************************************************/
/* 函数介绍: Get2_3_I 判决函数(uint使用)                                               */
/* 功能描述:                                                                           */
/* 修改记录:                                                                           */
/*                                                                                     */
/***************************************************************************************/
unsigned char Get2_3_I(unsigned int *a, unsigned int *b, unsigned int *c)
{
	if ((*a == *b) && (*b == *c)) /* 三区对比一致 */
	{
		return (TRUE); /* 3取2判读正确 */
	}
	else
	{
		if (*a == *b) /* 前两个数相等	 */
		{
			*c = *a;	   /* 赋值操作 */
			return (TRUE); /* 3取2判读正确 */
		}
		else
		{
			if (*a == *c) /* 另两个数相等 */
			{
				*b = *a;	   /* 赋值操作 */
				return (TRUE); /* 3取2判读正确 */
			}
			else
			{
				if (*b == *c) /* 剩余两个数相等 */
				{
					*a = *b;	   /* 赋值操作 */
					return (TRUE); /* 3取2判读正确 */
				}
				else
				{
					return (FALSE); /* 3取2判读异常 */
				}
			}
		}
	}
}

//*****************************************************************
// 函数名称 : EQU_ISR
// 功能描述 :
// *****************************************************************
void EQU_ISR(void) interrupt 2 using 3
{
	unsigned char temp;

	temp = TXD422BUFF;	 // 清中断
	if (fgEquRx == TRUE) // 接收标志
	{
		equDataBuffer[equRxCount] = RXD422BUFF; // 读出均衡器状态
		equRxCount++;							// 接收计数

		if (equRxCount >= 39) // 接收48后停止门控
		{
			RXDCONTROL = 1;
			RXD422AEn = 1;

			equRxCount = 0;
			fgEquRx = FALSE;
		}
	}
	else if (fgEquTx == TRUE) // 发送标志
	{
		TXD422BUFF = (equOnoffBuffer[rdEquOnoffIndex] & 0xFF);
		fgEquTx = FALSE;
	}
}

//*****************************************************************
// 函数名称 : CAN_ISR
// 功能描述 :
// *****************************************************************
void CAN_ISR(void) interrupt 0 using 1
{
	unsigned char IR;

	IR = XBYTE[BUSA_ADDR + 3]; // 读A总线中断寄存器
	if ((IR & 0x01) == 0x01)   // 中断寄存器接收状态字判读
	{
		fgCanBus = 0xAA;
		CANRXDA(); // A总线接收
	}

	if ((IR & 0x02) == 0x02) // 中断寄存器发送状态字判读
	{
		CANTXDA(); // A总线发送
	}

	IR = XBYTE[BUSB_ADDR + 3]; // 读B总线中断寄存器
	if ((IR & 0x01) == 0x01)   // 中断寄存器接收状态判读
	{
		fgCanBus = 0xBB;
		CANRXDB(); // B总线接收
	}

	if ((IR & 0x02) == 0x02) // 中断寄存器发送状态判读
	{
		CANTXDB(); // B总线发送
	}
}

//*****************************************************************
// 函数名称 : CANRXDA
// 功能描述 :
// *****************************************************************
void CANRXDA(void) using 1
{
	unsigned char i; // 定义临时变量
	unsigned char dlc;
	unsigned char length;
	unsigned char fgEndFrame;
	unsigned char xdata *p;

	if ((XBYTE[0xA014] == 0x9B) || (XBYTE[0xA014] == 0x5B) || (XBYTE[0xA014] == 0x4F)) // 判断为电源下位机ID 或 广播
	{
		fgEndFrame = FALSE;							 // 置数据包接收结束标志
		dlc = (unsigned char)(XBYTE[0xA015] & 0x0F); // 帧有效数据长度

		if ((XBYTE[0xA015] & 0x20) == 0x00) // 判断单帧/多帧结束帧/多帧中断帧			??????  首帧判读 ???????????????????
		{
			length = dlc;									 // 置接收数据长度
			p = 0xA016;										 // 置接收数据指针
			receiveSumA = 0;								 // 接收数据累加和清零
			receiveBufferA[0] = (unsigned char)(length - 2); // 存储长度字
			receiveCountA = 1;
		}
		else
		{
			if (dlc < 8) // 数据帧为多帧结束帧
			{
				length = (unsigned char)(dlc - 1);			   // 置接收数据长度
				receiveSumA = receiveSumA - receiveBufferA[0]; // 接收数据累加和减去多帧数据的length字节
			}
			else // 数据帧为多帧中继帧
			{
				length = 7;		   // 置接收数据长度
				fgEndFrame = TRUE; // 置数据包正在接收标志
			}

			p = 0xA017;		 // 置接收数据指针(不包括帧index)
							 //			if ((XBYTE[0xA014] != 0x5B) || (XBYTE[0xA016] == 0x00) || (XBYTE[0xA016] != receiveFrameA))
							 //			{
							 //				receiveSumA = 0; // 累加和、计数清零
							 //				receiveCountA = 0;
							 //				receiveFrameA = 0;
							 //				length = 0;
							 //				// 清除数据长度
							 //				if ((XBYTE[0xA014] == 0x5B) && (XBYTE[0xA016] == 0x00))
							 //				{ // 首帧则置接收数据长度
							 //					length = 7;
							 //				}
							 //			}
			receiveFrameA++; // 多帧计数增加
		}

		for (i = 0; i < length; i++) // 接收数据(TITLE+DATA+SUM)并计算累加和
		{
			receiveBufferA[receiveCountA] = *p;
			receiveSumA = receiveSumA + *p; // 累加和计算(Title + Data + Sum)
			p++;
			receiveCountA++; // 存储DATA计数
		}

		if ((fgEndFrame == FALSE) && (receiveCountA > 0)) // 判断结束处理
		{
			receiveSumA = ((receiveSumA - receiveBufferA[receiveCountA - 1]) & 0x00FF); // 计算累加和

			if ((receiveSumA == receiveBufferA[receiveCountA - 1]) && (receiveCountA == (unsigned char)(receiveBufferA[0] + 3))) // 判断数据包的累加和与长度都正确
			{
				errorCount = 0; // 通讯错误计数清零
				switch (receiveBufferA[1])
				{
				case 0x00: // 轮询控制处理
					if ((XBYTE[0xA014] == 0x9B) && (receiveBufferA[2] == 0x10))
					{
						RsManageA();
					}
					break;

				case 0x40: // 间接指令处理
					if (XBYTE[0xA014] == 0x5B)
					{
						ONOFFManageA();
					}

					if (XBYTE[0xA014] == 0x4F)
					{
						TimeBcA();
					}

					break;

				case 0x60: // 上行参数块
					if (XBYTE[0xA014] == 0x5B)
					{
						DataManageA();
					}
					break;

				case 0xD0: // 重要数据返回（接收）
					if (XBYTE[0xA014] == 0x9B)
					{
						ImportantManageA();
					}
					break;

				case 0xE0: // 内存地址设置
					if (XBYTE[0xA014] == 0x5B)
					{
						RamManageA();
					}
					break;

				default:
					break;
				}
			}

			receiveSumA = 0; // 累加和清零
			receiveCountA = 0;
			receiveFrameA = 0;
		}
	}

	XBYTE[0xA001] = 0x0C; // 释放接收缓存器
}

//*****************************************************************
// 函数名称 : CANRXDB
// 功能描述 :
// *****************************************************************
void CANRXDB(void) using 1
{
	unsigned char i; // 定义临时变量
	unsigned char dlc;
	unsigned char length;
	unsigned char fgEndFrame;
	unsigned char xdata *p;

	if ((XBYTE[0xC014] == 0x9B) || (XBYTE[0xC014] == 0x5B) || (XBYTE[0xC014] == 0x4F)) // 判断为电源下位机ID 或 广播
	{
		fgEndFrame = FALSE;							 // 置数据包接收结束标志
		dlc = (unsigned char)(XBYTE[0xC015] & 0x0F); // 帧有效数据长度

		if ((XBYTE[0xC015] & 0x20) == 0x00) // 判断单帧/多帧结束帧/多帧中断帧			??????  首帧判读 ???????????????????
		{
			length = dlc;									 // 置接收数据长度
			p = 0xC016;										 // 置接收数据指针
			receiveSumB = 0;								 // 接收数据累加和清零
			receiveBufferB[0] = (unsigned char)(length - 2); // 存储长度字
			receiveCountB = 1;
		}
		else
		{
			if (dlc < 8) // 数据帧为多帧结束帧
			{
				length = (unsigned char)(dlc - 1);			   // 置接收数据长度
				receiveSumB = receiveSumB - receiveBufferB[0]; // 接收数据累加和减去多帧数据的length字节
			}
			else // 数据帧为多帧中继帧
			{
				length = 7;		   // 置接收数据长度
				fgEndFrame = TRUE; // 置数据包正在接收标志
			}

			p = 0xC017;		 // 置接收数据指针(不包括帧index)
							 //			if ((XBYTE[0xC014] != 0x5B) || (XBYTE[0xC016] == 0x00) || (XBYTE[0xC016] != receiveFrameB))
							 //			{
							 //				receiveSumB = 0; // 累加和、计数清零
							 //				receiveCountB = 0;
							 //				receiveFrameB = 0;
							 //				length = 0;
							 //				// 清除数据长度
							 //				if ((XBYTE[0xC014] == 0x5B) && (XBYTE[0xC016] == 0x00))
							 //				{ // 首帧则置接收数据长度
							 //					length = 7;
							 //				}
							 //			}
			receiveFrameB++; // 多帧计数增加
		}

		for (i = 0; i < length; i++) // 接收数据(TITLE+DATA+SUM)并计算累加和
		{
			receiveBufferB[receiveCountB] = *p;
			receiveSumB = receiveSumB + *p; // 累加和计算(Title + Data + Sum)
			p++;
			receiveCountB++; // 存储DATA计数
		}

		if ((fgEndFrame == FALSE) && (receiveCountB > 0)) // 判断结束处理
		{
			receiveSumB = ((receiveSumB - receiveBufferB[receiveCountB - 1]) & 0x00FF); // 计算累加和

			if ((receiveSumB == receiveBufferB[receiveCountB - 1]) && (receiveCountB == (unsigned char)(receiveBufferB[0] + 3))) // 判断数据包的累加和与长度都正确
			{
				errorCount = 0; // 通讯错误计数清零
				switch (receiveBufferB[1])
				{
				case 0x00: // 轮询控制处理
					if ((XBYTE[0xC014] == 0x9B) && (receiveBufferB[2] == 0x10))
					{
						RsManageB();
					}
					break;

				case 0x40: // 间接指令处理
					if (XBYTE[0xC014] == 0x5B)
					{
						ONOFFManageB();
					}

					if (XBYTE[0xC014] == 0x4F)
					{
						TimeBcB();
					}

					break;

				case 0x60: // 上行参数块
					if (XBYTE[0xC014] == 0x5B)
					{
						DataManageB();
					}
					break;

				case 0xD0: // 重要数据返回（接收）
					if (XBYTE[0xC014] == 0x9B)
					{
						ImportantManageB();
					}
					break;

				case 0xE0: // 内存地址设置
					if (XBYTE[0xC014] == 0x5B)
					{
						RamManageB();
					}
					break;

				default:
					break;
				}
			}

			receiveSumB = 0; // 累加和清零
			receiveCountB = 0;
			receiveFrameB = 0;
		}
	}

	XBYTE[0xC001] = 0x0C; // 释放接收缓存器
}

//*****************************************************************
// 函数名称 : CANTXDA
// 功能描述 :
// *****************************************************************
void CANTXDA(void) using 1
{
	unsigned char i;

	if (sendIndexA != sendFrameA) // 判断应答帧是否发完
	{
		for (i = 0; i < 10; i++) // 构成发送帧
		{
			XBYTE[0xA00A + i] = *pSendA;
			pSendA++;
		}
		XBYTE[0xA001] = 0x01;
		sendIndexA++; // 发送帧索引号+1
	}
	else
	{
		sendIndexA = 1; // 设置发送结束
		sendFrameA = 1;
	}
}

//*****************************************************************
// 函数名称 : CANTXDB
// 功能描述 :
// *****************************************************************
void CANTXDB(void) using 1
{
	unsigned char i;

	if (sendIndexB != sendFrameB) // 判断应答帧是否发完
	{
		for (i = 0; i < 10; i++) // 构成发送帧
		{
			XBYTE[0xC00A + i] = *pSendB;
			pSendB++;
		}
		XBYTE[0xC001] = 0x01;
		sendIndexB++; // 发送帧索引号+1
	}
	else
	{
		sendIndexB = 1; // 设置发送结束
		sendFrameB = 1;
	}
}

//*****************************************************************
// 函数名称 : RsManageA
// 功能描述 :
// *****************************************************************
void RsManageA(void) using 1
{
	unsigned char i;
	unsigned char sum;
	unsigned char xdata *p;

	fgCanBus = 0xAA; // 设置A总线标志

	p = 0xA00A; // 指向CAN总线发送缓冲区

	switch (receiveBufferA[3]) // 判断控制轮询数据类型
	{
	case 0x01:					// 电源状态遥测参数
		if (fgSwitch1 == FALSE) // 切换B缓冲区发送
		{
			pSendA = &rsFrame1B[0];
		}
		else
		{
			pSendA = &rsFrame1A[0]; // 切换A缓冲区发送
		}

		for (i = 0; i < 10; i++) // 写入发送缓冲区
		{
			*p = *pSendA;
			p++;
			pSendA++;
		}
		XBYTE[0xA001] = 0x01; // 启动发送帧
		sendIndexA = 1;
		sendFrameA = 7; // 6帧速变参数

		break;

	case 0x02:					// 电源缓变遥测参数
		if (fgSwitch2 == FALSE) // 切换B区发送
		{
			pSendA = &rsFrame2B[0];
		}
		else
		{
			pSendA = &rsFrame2A[0]; // 切换A区发送
		}

		for (i = 0; i < 10; i++) // 写入发送缓冲区
		{
			*p = *pSendA;
			p++;
			pSendA++;
		}
		XBYTE[0xA001] = 0x01; // 启动发送帧
		sendIndexA = 1;
		sendFrameA = 17; // 22帧缓变参数

		break;

	case 0x03: //
		pSendA = &rsFrame3[0];

		for (i = 0; i < 10; i++)
		{
			*p = *pSendA;
			p++;
			pSendA++;
		}
		XBYTE[0xA001] = 0x01; // 启动发送帧
		sendIndexA = 1;		  // 构成发送信息
		sendFrameA = 7;
		break;

	case 0x61: // RAM下传包
		pSendA = &rsRamFrame[0];

		for (i = 0; i < 10; i++)
		{
			*p = *pSendA;
			p++;
			pSendA++;
		}
		XBYTE[0xA001] = 0x01; // 启动发送帧
		sendIndexA = 1;		  // 构成发送信息
		sendFrameA = 19;
		break;

	case 0x80: // 重要数据发送
		pSendA = &rsImportantFrame[0];

		for (i = 0; i < 10; i++)
		{
			*p = *pSendA;
			p++;
			pSendA++;
		}
		XBYTE[0xA001] = 0x01; // 启动发送帧
		sendIndexA = 1;		  // 构成发送信息
		sendFrameA = 32;
		break;

	case 0xC0: // 服务请求
		sum = 0;

		*p = 0x9B; //
		p++;
		*p = 0x44;
		p++;
		*p = 0x10;
		sum = sum + *p;
		p++;

		if (wrReqIndex != rdReqIndex) // 有服务请求时
		{
			*p = reqData[rdReqIndex];
			sum = sum + *p;
			p++;
			*p = reqData[rdReqIndex];
			sum = sum + *p;
			p++;

			reqData[rdReqIndex] = 0; // 清除服务请求码

			rdReqIndex++; // 读索引
			if (rdReqIndex > 11)
			{
				rdReqIndex = 0;
			}
		}
		else // 设置服务请求
		{
			*p = 0x55;
			sum = sum + *p;
			p++;
			*p = 0x55;
			sum = sum + *p;
			p++;
		}

		*p = sum;

		XBYTE[0xA001] = 0x01; // 启动发送帧
		sendIndexA = 1;		  // 构成发送信息
		sendFrameA = 1;
		break;

	default:
		break;
	}
}

//*****************************************************************
// 函数名称 : RsManageB
// 功能描述 :
// *****************************************************************
void RsManageB(void) using 1
{
	unsigned char i;
	unsigned char sum;
	unsigned char xdata *p;

	fgCanBus = 0xBB; // 设置B总线标志

	p = 0xC00A; // 指向CAN总线发送缓冲区

	switch (receiveBufferB[3]) // 判断控制轮询数据类型
	{
	case 0x01:					// 电源状态遥测参数
		if (fgSwitch1 == FALSE) // 切换B缓冲区发送
		{
			pSendB = &rsFrame1B[0];
		}
		else
		{
			pSendB = &rsFrame1A[0]; // 切换A缓冲区发送
		}

		for (i = 0; i < 10; i++) // 写入发送缓冲区
		{
			*p = *pSendB;
			p++;
			pSendB++;
		}
		XBYTE[0xC001] = 0x01; // 启动发送帧
		sendIndexB = 1;
		sendFrameB = 7; // 6帧速变参数

		break;

	case 0x02:					// 电源缓变遥测参数
		if (fgSwitch2 == FALSE) // 切换B区发送
		{
			pSendB = &rsFrame2B[0];
		}
		else
		{
			pSendB = &rsFrame2A[0]; // 切换A区发送
		}

		for (i = 0; i < 10; i++) // 写入发送缓冲区
		{
			*p = *pSendB;
			p++;
			pSendB++;
		}
		XBYTE[0xC001] = 0x01; // 启动发送帧
		sendIndexB = 1;
		sendFrameB = 17; // 22帧缓变参数

		break;

	case 0x03: //
		pSendB = &rsFrame3[0];

		for (i = 0; i < 10; i++)
		{
			*p = *pSendB;
			p++;
			pSendB++;
		}
		XBYTE[0xC001] = 0x01; // 启动发送帧
		sendIndexB = 1;		  // 构成发送信息
		sendFrameB = 7;
		break;

	case 0x61: // RAM下传包
		pSendB = &rsRamFrame[0];

		for (i = 0; i < 10; i++)
		{
			*p = *pSendB;
			p++;
			pSendB++;
		}
		XBYTE[0xC001] = 0x01; // 启动发送帧
		sendIndexB = 1;		  // 构成发送信息
		sendFrameB = 19;
		break;

	case 0x80: // 重要数据发送
		pSendB = &rsImportantFrame[0];

		for (i = 0; i < 10; i++)
		{
			*p = *pSendB;
			p++;
			pSendB++;
		}
		XBYTE[0xC001] = 0x01; // 启动发送帧
		sendIndexB = 1;		  // 构成发送信息
		sendFrameB = 32;
		break;

	case 0xC0: // 服务请求
		sum = 0;

		*p = 0x9B; //
		p++;
		*p = 0x44;
		p++;
		*p = 0x10;
		sum = sum + *p;
		p++;

		if (wrReqIndex != rdReqIndex) // 有服务请求时
		{
			*p = reqData[rdReqIndex];
			sum = sum + *p;
			p++;
			*p = reqData[rdReqIndex];
			sum = sum + *p;
			p++;

			reqData[rdReqIndex] = 0; // 清除服务请求码

			rdReqIndex++; // 读索引
			if (rdReqIndex > 11)
			{
				rdReqIndex = 0;
			}
		}
		else // 设置服务请求
		{
			*p = 0x55;
			sum = sum + *p;
			p++;
			*p = 0x55;
			sum = sum + *p;
			p++;
		}

		*p = sum;

		XBYTE[0xC001] = 0x01; // 启动发送帧
		sendIndexB = 1;		  // 构成发送信息
		sendFrameB = 1;
		break;

	default:
		break;
	}
}

//*****************************************************************
// 函数名称 : ONOFFManageA
// 功能描述 :
// *****************************************************************
void ONOFFManageA(void) using 1
{
	unsigned char xdata *p;

	if ((receiveBufferA[2] == 0x01) && (receiveBufferA[3] == 0x1B) && (receiveBufferA[4] == 0x2A) && (receiveBufferA[5] == 0x02)) // 指令标识判断
	{
		p = 0xA00A; // 指向CAN总线发送缓冲区

		*p = 0x5B; // 指令应答码
		p++;
		*p = 0x44;
		p++;
		*p = 0x10;
		p++;
		*p = 0xFF;
		p++;
		*p = 0x40;
		p++;
		*p = 0x4F;

		XBYTE[0xA001] = 0x01; // 启动发送帧
		sendIndexA = 1;		  // 构成发送信息
		sendFrameA = 1;

		onoffBuffer[wrOnoffIndex][0] = receiveBufferA[6]; // 读取有效指令码
		onoffBuffer[wrOnoffIndex][1] = receiveBufferA[7]; // 读取有效指令码

		wrOnoffIndex++;
		if (wrOnoffIndex >= 24) // 指令缓存 24条
		{
			wrOnoffIndex = 0;
		}
	}
}

//*****************************************************************
// 函数名称 : ONOFFManageB
// 功能描述 :
// *****************************************************************
void ONOFFManageB(void) using 1
{
	unsigned char xdata *p;

	if ((receiveBufferB[2] == 0x01) && (receiveBufferB[3] == 0x1B) && (receiveBufferB[4] == 0x2A) && (receiveBufferB[5] == 0x02)) // 指令标识判断
	{
		p = 0xC00A; // 指向CAN总线发送缓冲区

		*p = 0x5B; // 指令应答码
		p++;
		*p = 0x44;
		p++;
		*p = 0x10;
		p++;
		*p = 0xFF;
		p++;
		*p = 0x40;
		p++;
		*p = 0x4F;

		XBYTE[0xC001] = 0x01; // 启动发送帧
		sendIndexB = 1;		  // 构成发送信息
		sendFrameB = 1;

		onoffBuffer[wrOnoffIndex][0] = receiveBufferB[6]; // 读取有效指令码
		onoffBuffer[wrOnoffIndex][1] = receiveBufferB[7]; // 读取有效指令码

		wrOnoffIndex++;
		if (wrOnoffIndex >= 24) // 指令缓存 24条
		{
			wrOnoffIndex = 0;
		}
	}
}

//*****************************************************************
// 函数名称 : DataManageA
// 功能描述 :
// *****************************************************************
void DataManageA(void) using 1
{
	//	unsigned char i;
	unsigned char xdata *p;

	if ((receiveBufferA[2] == 0x4A) && (receiveBufferA[3] == 0x01)) // 指令标识判断
	{

		p = 0xA00A; // 指向CAN总线发送缓冲区

		*p = 0x5B; // 指令应答码
		p++;
		*p = 0x44;
		p++;
		*p = 0x10;
		p++;
		*p = 0xFF;
		p++;
		*p = 0x60;
		p++;
		*p = 0x6F;

		XBYTE[0xA001] = 0x01; // 启动发送帧
		sendIndexA = 1;		  // 构成发送信息
		sendFrameA = 1;

		uploadParaData[0] = receiveBufferA[4]; // 更新参数
		uploadParaData[1] = receiveBufferA[5];
		uploadParaData[2] = receiveBufferA[6];
		uploadParaData[3] = receiveBufferA[7];

		fgUploadPara = TRUE; // 设置参数上注请求
	}
}

//*****************************************************************
// 函数名称 : DataManageB
// 功能描述 :
// *****************************************************************
void DataManageB(void) using 1
{
	//	unsigned char i;
	unsigned char xdata *p;

	if ((receiveBufferB[2] == 0x4A) && (receiveBufferB[3] == 0x01)) // 指令标识判断
	{

		p = 0xC00A; // 指向CAN总线发送缓冲区

		*p = 0x5B; // 指令应答码
		p++;
		*p = 0x44;
		p++;
		*p = 0x10;
		p++;
		*p = 0xFF;
		p++;
		*p = 0x60;
		p++;
		*p = 0x6F;

		XBYTE[0xC001] = 0x01; // 启动发送帧
		sendIndexB = 1;		  // 构成发送信息
		sendFrameB = 1;

		uploadParaData[0] = receiveBufferB[4]; // 更新参数
		uploadParaData[1] = receiveBufferB[5];
		uploadParaData[2] = receiveBufferB[6];
		uploadParaData[3] = receiveBufferB[7];

		fgUploadPara = TRUE; // 设置参数上注请求
	}
}

//*****************************************************************
// 函数名称 : RamManageA
// 功能描述 :
// *****************************************************************
void RamManageA(void) using 1
{
	unsigned char i;
	unsigned char xdata *p;

	p = 0xA00A; // 指向CAN总线发送缓冲区

	*p = 0x5B; // 指令应答码
	p++;
	*p = 0x44;
	p++;
	*p = 0x10;
	p++;
	*p = 0xFF;
	p++;
	*p = 0xE0;
	p++;
	*p = 0xEF;

	XBYTE[0xA001] = 0x01; // 启动发送帧
	sendIndexA = 1;		  // 构成发送信息
	sendFrameA = 1;

	for (i = 0; i < 8; i++) // 地址设置信息数据(8byte)
	{
		ramUploadData[i] = receiveBufferA[2 + i];
	}

	fgRsRam = TRUE; // 设置地址处理请求有效
}

//*****************************************************************
// 函数名称 : RamManageB
// 功能描述 :
// *****************************************************************
void RamManageB(void) using 1
{
	unsigned char i;
	unsigned char xdata *p;

	p = 0xC00A; // 指向CAN总线发送缓冲区

	*p = 0x5B; // 指令应答码
	p++;
	*p = 0x44;
	p++;
	*p = 0x10;
	p++;
	*p = 0xFF;
	p++;
	*p = 0xE0;
	p++;
	*p = 0xEF;

	XBYTE[0xC001] = 0x01; // 启动发送帧
	sendIndexB = 1;		  // 构成发送信息
	sendFrameB = 1;

	for (i = 0; i < 8; i++) // 地址设置信息数据(8byte)
	{
		ramUploadData[i] = receiveBufferB[2 + i];
	}

	fgRsRam = TRUE; // 设置地址处理请求有效
}

//*****************************************************************
// 函数名称 : ImportantManageA
// 功能描述 :
// *****************************************************************
void ImportantManageA(void) using 1
{
	unsigned char i;
	unsigned char xdata *p;

	p = 0xA00A; // 指向CAN总线发送缓冲区

	*p = 0x5B; // 指令应答码
	p++;
	*p = 0x44;
	p++;
	*p = 0x10;
	p++;
	*p = 0xFF;
	p++;
	*p = 0xC0;
	p++;
	*p = 0xCF;

	XBYTE[0xA001] = 0x01; // 启动发送帧
	sendIndexA = 1;		  // 构成发送信息
	sendFrameA = 1;

	for (i = 0; i < 220; i++) // 读取重要数据
	{
		rsImportantData[i] = receiveBufferA[2 + i];
	}
	fgImportantDataRx = TRUE; // 重要数据恢复有效标志
}

//*****************************************************************
// 函数名称 : ImportantManageB
// 功能描述 :
// *****************************************************************
void ImportantManageB(void) using 1
{
	unsigned char i;
	unsigned char xdata *p;

	p = 0xC00A; // 指向CAN总线发送缓冲区

	*p = 0x5B; // 指令应答码
	p++;
	*p = 0x44;
	p++;
	*p = 0x10;
	p++;
	*p = 0xFF;
	p++;
	*p = 0xC0;
	p++;
	*p = 0xCF;

	XBYTE[0xC001] = 0x01; // 启动发送帧
	sendIndexB = 1;		  // 构成发送信息
	sendFrameB = 1;

	for (i = 0; i < 220; i++) // 读取重要数据
	{
		rsImportantData[i] = receiveBufferB[2 + i];
	}
	fgImportantDataRx = TRUE; // 重要数据恢复有效标志
}

//*****************************************************************
// 函数名称 : TimeBcA
// 功能描述 : 仅读取秒
// *****************************************************************
void TimeBcA(void) using 1
{
	sysTime = (unsigned long)(receiveBufferA[3] << 24) | (unsigned long)(receiveBufferA[4] << 16) | (unsigned long)(receiveBufferA[5] << 8) | (unsigned long)(receiveBufferA[6]);
}

//*****************************************************************
// 函数名称 : TimeBcB
// 功能描述 : 仅读取秒
// *****************************************************************
void TimeBcB(void) using 1
{
	sysTime = (unsigned long)(receiveBufferB[3] << 24) | (unsigned long)(receiveBufferB[4] << 16) | (unsigned long)(receiveBufferB[5] << 8) | (unsigned long)(receiveBufferB[6]);
}

//*****************************************************************
// 函数名称 : TIMER_ISR
// 功能描述 :
// *****************************************************************
void TIMER_ISR(void) interrupt 1 using 2
{
	TR0 = 0;	// 定时器重新计数
	TH0 = 0xB8; // 装载新值
	TL0 = 0x00;
	TR0 = 1;

	clockCount++; // 时钟计数
}

//*****************************************************************
// 函数名称 : CANInfoA
// 功能描述 :
// *****************************************************************
void CANInfoA(void)
{
	unsigned int i;

	sendIndexA = 0; // 变量清零
	receiveFrameA = 0;
	sendFrameA = 0;
	receiveSumA = 0;
	receiveCountA = 0;

	for (i = 0; i < 256; i++)
	{
		receiveBufferA[i] = 0x00; //   清零
	}
}

//*****************************************************************
// 函数名称 : CANInfoB
// 功能描述 :
// *****************************************************************
void CANInfoB(void)
{
	unsigned int i;

	sendIndexB = 0; // 变量清零
	receiveFrameB = 0;
	sendFrameB = 0;
	receiveSumB = 0;
	receiveCountB = 0;

	for (i = 0; i < 256; i++)
	{
		receiveBufferB[i] = 0x00; //   清零
	}
}

//*****************************************************************
// 函数名称 : SepCurrentStateCal
// 功能描述 :
// *****************************************************************
unsigned char SepCurrentStateCal(unsigned int val)
{
	float xdata valTemp;
	unsigned char stateVal;

	valTemp = (float)((float)val * 0.0025); // 计算电压值
	stateVal = 0xFF;						// 默认填充 0xFF

	if (valTemp < 0.5) // 0-0.5  04H
	{
		stateVal = 0x04;
	}

	if ((valTemp < 1.5) && (valTemp >= 0.7)) // 0.7-1.7  03H
	{
		stateVal = 0x03;
	}

	if ((valTemp < 2.5) && (valTemp >= 1.7)) // 1.7-2.5  02H
	{
		stateVal = 0x02;
	}

	if ((valTemp < 3.5) && (valTemp >= 2.7)) // 2.7-3.5  01H
	{
		stateVal = 0x01;
	}

	if ((valTemp < 4.5) && (valTemp >= 3.7)) // 2.7-3.5  01H
	{
		stateVal = 0x00;
	}

	return (stateVal);
}

//*****************************************************************
// 函数名称 : ImportantDataRx
// 功能描述 : 重要数据周期接收处理
// *****************************************************************
void ImportantDataRx(void)
{
	unsigned int i;
	unsigned int j;

	for (j = 0; j < 3; j++) // 最多连续三次服务请求
	{
		reqData[wrReqIndex] = 0xA3; // 设置重要数据恢复请求
		wrReqIndex++;				// 读索引
		if (wrReqIndex > 11)
		{
			wrReqIndex = 0;
		}

		for (i = 0; i < 150; i++) // 延时保证重要数据恢复  待测试时间间隔
		{
			dog = !dog; // 牵狗
			Delay(2000);
		}

		if (fgImportantDataRx == TRUE) // 重要数据有效恢复
		{
			fgImportantDataRx = FALSE; // 清除重要数据恢复请求
			ImportDataFlush();		   // 重要数据刷新
		}

		if (importDataRxState == 0x0F) // 重要数据恢复成功
		{
			break;
		}
	}

	if (importDataRxState == 0x00) // 重要数据恢复失败
	{
		AutoOnoff(); // 进行默认状态下的指令初始设置
	}

	clockCount = 0; // 增加时间变量清零操作
}

//*************************************************************************************
// 函数介绍: ImportDataFlush
// 功能描述: 重要数据恢复更新
// 修改记录:
//*************************************************************************************
void ImportDataFlush(void)
{
	unsigned char i;
	unsigned char sum;	  // 校验和
	unsigned char fgZero; // 非全零标志

	fgZero = FALSE; // 默认为非全零

	sum = 0;
	for (i = 0; i < 43; i++) // WORD -> BYTE
	{
		sum = sum + rsImportantData[i];
		if (rsImportantData[i] != 0) // 存在非零数据
		{
			fgZero = TRUE; // 设置非全零有效
		}
	}

	if ((sum == rsImportantData[43]) && (fgZero == TRUE)) // 校验和一致 且 非全零
	{
		importDataRxState = 0x0F; // 重要数据恢复状态  成功

		if ((rsImportantData[0] & 0x80) == 0x80) // W0 - bit7  均衡控制使能状态
		{
			equControlEn = TRUE; // 使能有效
			equControlEnB = TRUE;
			equControlEnC = TRUE;
		}
		else
		{
			equControlEn = FALSE; // 使能无效
			equControlEnB = FALSE;
			equControlEnC = FALSE;
		}

		if ((rsImportantData[0] & 0x40) == 0x40) // W0 - bit6  蓄电池组过充保护使能状态
		{
			overChargeProtectEn = TRUE;	 // 过充保护允许标志A
			overChargeProtectEnB = TRUE; // 过充保护允许标志B
			overChargeProtectEnC = TRUE; // 过充保护允许标志C
		}
		else
		{
			overChargeProtectEn = FALSE;  // 过充保护允许标志A
			overChargeProtectEnB = FALSE; // 过充保护允许标志B
			overChargeProtectEnC = FALSE; // 过充保护允许标志C
		}

		if ((rsImportantData[0] & 0x20) == 0x20) // W0 - bit5  蓄电池单体保护充电使能状态
		{
			singleProtectChargeEn = TRUE;  //单体保护使能状态
			singleProtectChargeEnB = TRUE; //单体保护使能状态B
			singleProtectChargeEnC = TRUE; //单体保护使能状态C
		}
		else
		{
			singleProtectChargeEn = FALSE;	//单体保护使能状态
			singleProtectChargeEnB = FALSE; //单体保护使能状态B
			singleProtectChargeEnC = FALSE; //单体保护使能状态C
		}

		chargeVolKickA = rsImportantData[1]; // A组充电限压档数
		chargeVolKickA_B = rsImportantData[1];
		chargeVolKickA_C = rsImportantData[1];

		chargeVolKickB = rsImportantData[2]; // B组充电限压档数
		chargeVolKickB_B = rsImportantData[2];
		chargeVolKickB_C = rsImportantData[2];

		chargeVolKickRsA = chargeVolKickA; // 充电切换档位A  遥测使用
		chargeVolKickRsB = chargeVolKickB; // 充电切换档位B  遥测使用

		chargeCurKickA = rsImportantData[3]; // A组充电电流档数
		chargeCurKickA_B = rsImportantData[3];
		chargeCurKickA_C = rsImportantData[3];

		chargeCurKickB = rsImportantData[4]; // B组充电电流档数
		chargeCurKickB_B = rsImportantData[4];
		chargeCurKickB_C = rsImportantData[4];

		DA_A_VOL = volCsTab[chargeVolKickA - 1]; // A组电压 恢复执行
		DA_A_CUR = curCsTab[chargeCurKickA - 1]; // A组电流
		DA_B_VOL = volCsTab[chargeVolKickB - 1]; // B组电压 恢复执行
		DA_B_CUR = curCsTab[chargeCurKickB - 1]; // B组电流

		ntrPower = (float)((float)((unsigned int)(rsImportantData[5] * 256) + rsImportantData[6]) * 3.0 / 1000.0); // NTR 满电量值恢复
		ntrPowerB = (float)((float)((unsigned int)(rsImportantData[5] * 256) + rsImportantData[6]) * 3.0 / 1000.0);
		ntrPowerC = (float)((float)((unsigned int)(rsImportantData[5] * 256) + rsImportantData[6]) * 3.0 / 1000.0);

		currentPower = ntrPower; // NTR更新当前电量

		powerSave[0] = (unsigned char)((unsigned int)((ntrPower * 1000) / 3.0) >> 8);	// 取当前电量的高字节 -A  更新满电量值
		powerSave[1] = (unsigned char)((unsigned int)((ntrPower * 1000) / 3.0) & 0xFF); // 取当前电量的低字节
		powerSave[2] = 0;
		powerSave[3] = 0;
		powerSave[4] = 0;
		powerSave[5] = 0;

		proportion = rsImportantData[8]; // 蓄电池组 充放比 使用双字节 低位
		proportionB = rsImportantData[8];
		proportionC = rsImportantData[8];

		aqStateContrl = (unsigned int)((unsigned int)(rsImportantData[9] * 256) + rsImportantData[10]); // 单体控制字

		healthStateContr1 = (unsigned int)((unsigned int)(rsImportantData[11] * 256) + rsImportantData[12]); // 健康控制字1
		healthStateContr1B = (unsigned int)((unsigned int)(rsImportantData[11] * 256) + rsImportantData[12]);
		healthStateContr1C = (unsigned int)((unsigned int)(rsImportantData[11] * 256) + rsImportantData[12]);

		healthStateContr2 = (unsigned int)((unsigned int)(rsImportantData[13] * 256) + rsImportantData[14]); // 健康控制字4
		healthStateContr2B = (unsigned int)((unsigned int)(rsImportantData[13] * 256) + rsImportantData[14]);
		healthStateContr2C = (unsigned int)((unsigned int)(rsImportantData[13] * 256) + rsImportantData[14]);

		selfCheckStateContrl = rsImportantData[15]; // 自检状态控制字
		selfCheckStateContrlB = rsImportantData[15];
		selfCheckStateContrlC = rsImportantData[15];

		equOpenValue = (float)((float)((unsigned int)(rsImportantData[16] * 256) + rsImportantData[17]) / 1000.0);	// 均衡开启电压门限值A
		equOpenValueB = (float)((float)((unsigned int)(rsImportantData[16] * 256) + rsImportantData[17]) / 1000.0); // 均衡开启电压门限值B
		equOpenValueC = (float)((float)((unsigned int)(rsImportantData[16] * 256) + rsImportantData[17]) / 1000.0); // 均衡开启电压门限值C

		equCloseValue = (float)((float)((unsigned int)(rsImportantData[18] * 256) + rsImportantData[19]) / 1000.0);	 // 均衡关闭电压门限值A
		equCloseValueB = (float)((float)((unsigned int)(rsImportantData[18] * 256) + rsImportantData[19]) / 1000.0); // 均衡关闭电压门限值B
		equCloseValueC = (float)((float)((unsigned int)(rsImportantData[18] * 256) + rsImportantData[19]) / 1000.0); // 均衡关闭电压门限值C

		singleVWardValueUP = (float)((float)((unsigned int)(rsImportantData[20] * 256) + rsImportantData[21]) / 100.0);	 // 单体电压上限报警值A
		singleVWardValueUPB = (float)((float)((unsigned int)(rsImportantData[20] * 256) + rsImportantData[21]) / 100.0); // 单体电压上限报警值B
		singleVWardValueUPC = (float)((float)((unsigned int)(rsImportantData[20] * 256) + rsImportantData[21]) / 100.0); // 单体电压上限报警值C

		singleVWardValueDOWN = (float)((float)((unsigned int)(rsImportantData[22] * 256) + rsImportantData[23]) / 100.0);  // 单体电压下限报警值A
		singleVWardValueDOWNB = (float)((float)((unsigned int)(rsImportantData[22] * 256) + rsImportantData[23]) / 100.0); // 单体电压下限报警值B
		singleVWardValueDOWNC = (float)((float)((unsigned int)(rsImportantData[22] * 256) + rsImportantData[23]) / 100.0); // 单体电压下限报警值C

		singleVProtectValueUP = (float)((float)((unsigned int)(rsImportantData[24] * 256) + rsImportantData[25]) / 100.0);	// 单体电压上限报警值A
		singleVProtectValueUPB = (float)((float)((unsigned int)(rsImportantData[24] * 256) + rsImportantData[25]) / 100.0); // 单体电压上限报警值B
		singleVProtectValueUPC = (float)((float)((unsigned int)(rsImportantData[24] * 256) + rsImportantData[25]) / 100.0); // 单体电压上限报警值C

		batteryVAWardValueUP = (float)((float)((unsigned int)(rsImportantData[26] * 256) + rsImportantData[27]) / 100.0);  // 蓄电池组电压上限保护值A
		batteryVAWardValueUPB = (float)((float)((unsigned int)(rsImportantData[26] * 256) + rsImportantData[27]) / 100.0); // 蓄电池组电压上限保护值B
		batteryVAWardValueUPC = (float)((float)((unsigned int)(rsImportantData[26] * 256) + rsImportantData[27]) / 100.0); // 蓄电池组电压上限保护值C

		batteryVAWardValueDOWN = (float)((float)((unsigned int)(rsImportantData[28] * 256) + rsImportantData[29]) / 100.0);	 // 蓄电池组电压下限保护值A
		batteryVAWardValueDOWNB = (float)((float)((unsigned int)(rsImportantData[28] * 256) + rsImportantData[29]) / 100.0); // 蓄电池组电压下限保护值B
		batteryVAWardValueDOWNC = (float)((float)((unsigned int)(rsImportantData[28] * 256) + rsImportantData[29]) / 100.0); // 蓄电池组电压下限保护值C

		for (i = 0; i < 9; i++) // 单体均衡次数
		{
			equExeCount[i] = rsImportantData[34 + i];
		}
	}
	else
	{
		importDataRxState = 0x00; // 重要数据恢复状态  失败
	}
}

//*****************************************************************
// 函数名称 : ImportantDataTx
// 功能描述 : 重要数据周期发送处理-- 16s间隔 *
// ****************************************************************
void ImportantDataTx(void)
{
	fg16S++;
	if (fg16S > 15) // 16秒周期
	{
		fg16S = 0;

		ImportantSaveData(&rsImportantData[0]); // 重要数据缓存更新
		SaveImportantDataRs();					// 重要数据组帧更新

		reqData[wrReqIndex] = 0xA0; // 设置过滤包索取
		wrReqIndex++;				// 读索引
		if (wrReqIndex > 11)
		{
			wrReqIndex = 0;
		}
	}
}

//*****************************************************************
// 函数名称 : FilterSaveData
// 功能描述 :
// *****************************************************************
void ImportantSaveData(unsigned char xdata *pRs)
{
	unsigned char i;
	unsigned char sum;
	unsigned char state;

	state = 0;
	if (equControlEn == TRUE) // 均衡控制使能  bit7
	{
		state = state | 0x80;
	}
	if (overChargeProtectEn == TRUE) // 过充保护使能  bit6
	{
		state = state | 0x40;
	}
	if (singleProtectChargeEn == TRUE) // 单体保护充电状态  bit5
	{
		state = state | 0x20;
	}

	sum = 0; // 累加和清零

	*pRs = state; // 状态  W0
	sum = sum + *pRs;
	pRs++;

	*pRs = chargeVolKickRsA; // A组充电限压档数  W1
	sum = sum + *pRs;
	pRs++;
	*pRs = chargeVolKickRsB; // B组充电限压档数  W2
	sum = sum + *pRs;
	pRs++;
	*pRs = chargeCurKickA; // A组充电电流档数  W3
	sum = sum + *pRs;
	pRs++;
	*pRs = chargeCurKickB; // B组充电电流档数  W4
	sum = sum + *pRs;
	pRs++;

	*pRs = (unsigned char)(((unsigned int)((ntrPower * 1000) / 3.0)) >> 8); // 蓄电池组电量 NTRA H  W5
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)(((unsigned int)(ntrPower * 1000) / 3.0)); // 蓄电池组电量 NTRA L W6
	sum = sum + *pRs;
	pRs++;

	*pRs = 0x00; // 充放比 - H - 00H
	sum = sum + *pRs;
	pRs++;
	*pRs = proportion; // 充放比 - L
	sum = sum + *pRs;
	pRs++;

	*pRs = aqStateContrl >> 8; // 单体控制字 - H	W9
	sum = sum + *pRs;
	pRs++;
	*pRs = aqStateContrl; // 单体控制字 - L	W10
	sum = sum + *pRs;
	pRs++;

	*pRs = healthStateContr1 >> 8; // 健康控制字1 - H	W11
	sum = sum + *pRs;
	pRs++;
	*pRs = healthStateContr1; // 健康控制字1 - L	W12
	sum = sum + *pRs;
	pRs++;

	*pRs = healthStateContr2 >> 8; // 健康控制字2 - H	W13
	sum = sum + *pRs;
	pRs++;
	*pRs = healthStateContr2; // 健康控制字2 - L	W14
	sum = sum + *pRs;
	pRs++;

	*pRs = selfCheckStateContrl; // 自检控制字	W15
	sum = sum + *pRs;
	pRs++;

	*pRs = (unsigned char)(((unsigned int)(equOpenValue * 1000 + 0.001)) >> 8); // 均衡开启电压设定值 - H	W16
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)((unsigned int)(equOpenValue * 1000 + 0.001)); // 均衡开启电压设定值 - L	W17
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)(((unsigned int)(equCloseValue * 1000 + 0.001)) >> 8); // 均衡关闭电压设定值 - H	W18
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)((unsigned int)(equCloseValue * 1000 + 0.001)); // 均衡关闭电压设定值 - L	W19
	sum = sum + *pRs;
	pRs++;

	*pRs = (unsigned char)(((unsigned int)(singleVWardValueUP * 100 + 0.001)) >> 8); // 单体电压上限报警值 - H	W20
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)((unsigned int)(singleVWardValueUP * 100 + 0.001)); // 单体电压上限报警值 - L	W21
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)(((unsigned int)(singleVWardValueDOWN * 100 + 0.001)) >> 8); // 单体电压下限报警值 - H	W22
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)((unsigned int)(singleVWardValueDOWN * 100 + 0.001)); // 单体电压下限报警值 - L	W23
	sum = sum + *pRs;
	pRs++;

	*pRs = (unsigned char)(((unsigned int)((float)(singleVProtectValueUP * 100) + 0.001)) >> 8); // 单体电压上限保护值 - H	W24
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)((unsigned int)((float)(singleVProtectValueUP * 100) + 0.001)); // 单体电压上限保护值 - L	W25
	sum = sum + *pRs;
	pRs++;

	*pRs = (unsigned char)(((unsigned int)(batteryVAWardValueUP * 100 + 0.001)) >> 8); // 蓄电池组电压上限保护值 - H	W26
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)((unsigned int)(batteryVAWardValueUP * 100 + 0.001)); // 蓄电池组电压上限保护值 - L	W27
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)(((unsigned int)(batteryVAWardValueDOWN * 100 + 0.001)) >> 8); // 蓄电池组电压下限保护值 - H	W28
	sum = sum + *pRs;
	pRs++;
	*pRs = (unsigned char)((unsigned int)(batteryVAWardValueDOWN * 100 + 0.001)); // 蓄电池组电压下限保护值 - L	W29
	sum = sum + *pRs;
	pRs++;

	*pRs = 0xAA; // 预留	W30
	sum = sum + *pRs;
	pRs++;
	*pRs = 0xAA; // 预留	W31
	sum = sum + *pRs;
	pRs++;
	*pRs = 0xAA; // 预留	W32
	sum = sum + *pRs;
	pRs++;
	*pRs = 0xAA; // 预留	W33
	sum = sum + *pRs;
	pRs++;

	for (i = 0; i < 9; i++)
	{
		*pRs = equExeCount[i]; // 单体均衡计数 W34 ~ W42
		sum = sum + *pRs;
		pRs++;
	}

	*pRs = sum; // SUM
}

//*****************************************************************
// 函数名称 : FilterManage
// 功能描述 :
// *****************************************************************
void UploadParaManage(void)
{
	unsigned int dataTemp;

	if (fgUploadPara == TRUE) // 上注参数处理请求
	{
		fgUploadPara = FALSE;													// 清除请求标志
		dataTemp = (unsigned int)(uploadParaData[1] * 256) + uploadParaData[2]; // 上注原码范围判断

		switch (uploadParaData[0])
		{
		case 0x21:					  // 单体控制字
			aqStateContrl = dataTemp; // 单体控制字
			break;

		case 0x22:						   // 健康控制字1
			healthStateContr1 = dataTemp;  // 健康控制字1A
			healthStateContr1B = dataTemp; // 健康控制字1B
			healthStateContr1C = dataTemp; // 健康控制字1C
			break;

		case 0x23:						   // 健康控制字2
			healthStateContr2 = dataTemp;  // 健康控制字2A
			healthStateContr2B = dataTemp; // 健康控制字2B
			healthStateContr2C = dataTemp; // 健康控制字2C
			break;

		case 0x24:										// 单体电压上限报警值
			if ((dataTemp >= 350) && (dataTemp <= 450)) // 3.5-4.5V
			{
				singleVWardValueUP = (float)((float)dataTemp / 100.0);	// 单体电压上限报警值A
				singleVWardValueUPB = (float)((float)dataTemp / 100.0); // 单体电压上限报警值B
				singleVWardValueUPC = (float)((float)dataTemp / 100.0); // 单体电压上限报警值C
			}
			break;

		case 0x25:										// 单体电压下限报警值
			if ((dataTemp >= 250) && (dataTemp <= 350)) // 2.5-3.5V
			{
				singleVWardValueDOWN = (float)((float)dataTemp / 100.0);  // 单体电压上限报警值A
				singleVWardValueDOWNB = (float)((float)dataTemp / 100.0); // 单体电压上限报警值B
				singleVWardValueDOWNC = (float)((float)dataTemp / 100.0); // 单体电压上限报警值C
			}
			break;

		case 0x26:										// 单体电压上限保护值
			if ((dataTemp >= 350) && (dataTemp <= 450)) // 3.5-4.5V
			{
				singleVProtectValueUP = (float)((float)dataTemp / 100.0);  // 单体电压上限报警值A
				singleVProtectValueUPB = (float)((float)dataTemp / 100.0); // 单体电压上限报警值B
				singleVProtectValueUPC = (float)((float)dataTemp / 100.0); // 单体电压上限报警值C
			}
			break;

		case 0x27:										  // 蓄电池组电压上限保护值
			if ((dataTemp >= 2400) && (dataTemp <= 3900)) // 24~39V
			{
				batteryVAWardValueUP = (float)((float)dataTemp / 100.0);  // 蓄电池组电压上限保护值A
				batteryVAWardValueUPB = (float)((float)dataTemp / 100.0); // 蓄电池组电压上限保护值B
				batteryVAWardValueUPC = (float)((float)dataTemp / 100.0); // 蓄电池组电压上限保护值C
			}
			break;

		case 0x28:											// 当前电量NTR
			if ((dataTemp >= 40000) && (dataTemp <= 60000)) // 120~180Ah
			{
				ntrPower = (float)((float)dataTemp * 3.0 / 1000.0);	 // NTR 满电量值恢复A
				ntrPowerB = (float)((float)dataTemp * 3.0 / 1000.0); // NTR 满电量值恢复B
				ntrPowerC = (float)((float)dataTemp * 3.0 / 1000.0); // NTR 满电量值恢复C

				currentPower = ntrPower;  /* A组蓄电池组电量  */
				chargePower = (float)0.0; /* 当前电量恢复 ， 充电电量设置0  */
				dischargePower = (float)0.0;
				fgAh = TRUE;

				powerSave[0] = (unsigned char)((unsigned int)((ntrPower * 1000) / 3.0) >> 8);	/* 取当前电量的高字节 -A  更新满电量值  */
				powerSave[1] = (unsigned char)((unsigned int)((ntrPower * 1000) / 3.0) & 0xFF); /* 取当前电量的低字节  */
				powerSave[2] = 0x00;
				powerSave[3] = 0x00;
				powerSave[4] = 0x00;
				powerSave[5] = 0x00;
			}
			break;

		case 0x29:									   // 充放比
			dataTemp = (dataTemp & 0x00FF);			   // 上注原码范围判断
			if ((dataTemp >= 80) && (dataTemp <= 150)) // 0.8 ~ 1.5
			{
				proportion = dataTemp; // 蓄电池组 充放比 使用双字节 低位
				proportionB = dataTemp;
				proportionC = dataTemp;
			}
			break;

		case 0x2A:									   // 均衡开启电压值
			if ((dataTemp >= 30) && (dataTemp <= 200)) // 0.03~0.2V
			{
				equOpenValue = (float)((float)dataTemp / 1000.0);  // 均衡开启电压门限值A
				equOpenValueB = (float)((float)dataTemp / 1000.0); // 均衡开启电压门限值B
				equOpenValueC = (float)((float)dataTemp / 1000.0); // 均衡开启电压门限值C
			}
			break;

		case 0x2B:									   // 均衡关闭电压值
			if ((dataTemp >= 10) && (dataTemp <= 100)) // 3.5-4.5V
			{
				equCloseValue = (float)((float)dataTemp / 1000.0);	// 均衡开启电压门限值A
				equCloseValueB = (float)((float)dataTemp / 1000.0); // 均衡开启电压门限值B
				equCloseValueC = (float)((float)dataTemp / 1000.0); // 均衡开启电压门限值C
			}
			break;

		case 0x2C:										  // 蓄电池组电压下限报警值
			if ((dataTemp >= 1800) && (dataTemp <= 3000)) // 18~30V
			{
				batteryVAWardValueDOWN = (float)((float)dataTemp / 100.0);	// 蓄电池组电压上限保护值A
				batteryVAWardValueDOWNB = (float)((float)dataTemp / 100.0); // 蓄电池组电压上限保护值B
				batteryVAWardValueDOWNC = (float)((float)dataTemp / 100.0); // 蓄电池组电压上限保护值C
			}
			break;

		case 0x2D:							// 自检状态控制字
			dataTemp = (dataTemp & 0x00FF); // 上注原码范围判断

			selfCheckStateContrl = dataTemp; // 自检状态控制字
			selfCheckStateContrlB = dataTemp;
			selfCheckStateContrlC = dataTemp;
			break;

		default:
			break;
		}
	}
}

//*****************************************************************
// 函数名称 : FilterManage
// 功能描述 :
// *****************************************************************
void FilterManage(void)
{
	unsigned char result;

	fg60S++; // 周期计数增加  无变化时 60s 更新

	FilterSaveData(&filterFrame[0]); // 过滤参数遥测组帧
	result = FilterDataComp();		 // 过滤包数据对比

	if ((result == TRUE) || (fg60S > 59)) // 对比结果出现变化时 或是 60s 周期
	{
		fg60S = 0x00; // 重新计数

		SaveDataRs3(); // 组帧过滤包

		reqData[wrReqIndex] = 0xA6; // 设置过滤包索取
		wrReqIndex++;				// 读索引
		if (wrReqIndex > 11)
		{
			wrReqIndex = 0;
		}
	}
}

//*****************************************************************
// 函数名称 : FilterDataComp
// 功能描述 : 过滤包数据对比处理处理
// *****************************************************************
unsigned char FilterDataComp(void)
{
	unsigned int i;
	unsigned char fgComp; // 对比结果标志
	int result;			  // 比对结果 临时变量

	fgComp = FALSE; // 默认无变化

	for (i = 0; i < 4; i++) // 状态值比较  bit 变化
	{
		if (filterFrame[i] != filterDataRec[i])
		{
			fgComp = TRUE;
		}
	}

	result = abs((int)(filterFrame[4] * 256 + filterFrame[5]) - (int)(filterDataRec[4] * 256 + filterDataRec[5]));
	if (result > 100) // 5V 主 对比结果大于100分层  0.25V
	{
		fgComp = TRUE;
	}
	result = abs((int)(filterFrame[6] * 256 + filterFrame[7]) - (int)(filterDataRec[6] * 256 + filterDataRec[7]));
	if (result > 100) // 5V 备 对比结果大于100分层  0.25V
	{
		fgComp = TRUE;
	}
	result = abs((int)(filterFrame[8] * 256 + filterFrame[9]) - (int)(filterDataRec[8] * 256 + filterDataRec[9]));
	if (result > 100) // 5V 均衡 对比结果大于100分层  0.25V
	{
		fgComp = TRUE;
	}
	result = abs((int)(filterFrame[10] * 256 + filterFrame[11]) - (int)(filterDataRec[10] * 256 + filterDataRec[11]));
	if (result > 480) // +12V 均衡 对比结果大于480分层  1.2V
	{
		fgComp = TRUE;
	}
	result = abs((int)(filterFrame[12] * 256 + filterFrame[13]) - (int)(filterDataRec[12] * 256 + filterDataRec[13]));
	if (result > 480) // -12V 均衡 对比结果大于480分层  1.2V
	{
		fgComp = TRUE;
	}
	result = abs((int)(filterFrame[14] * 256 + filterFrame[15]) - (int)(filterDataRec[14] * 256 + filterDataRec[15]));
	if (result > 100) // +12V 均衡 对比结果大于480分层  1.2V
	{
		fgComp = TRUE;
	}

	for (i = 16; i < 41; i++) // 状态值比较  bit 变化
	{
		if (filterFrame[i] != filterDataRec[i])
		{
			fgComp = TRUE;
		}
	}

	for (i = 0; i < 41; i++) // 数据更新
	{
		filterDataRec[i] = filterFrame[i];
	}

	return (fgComp);
}

//*****************************************************************
// 函数名称 : FilterSaveData
// 功能描述 :
// *****************************************************************
void FilterSaveData(unsigned char xdata *pRs)
{
	unsigned char i;

	*pRs = pcuState1; // PCU状态字1  W0
	pRs++;
	*pRs = pcuState2; // PCU状态字2  W1
	pRs++;
	*pRs = pcuState3; // PCU状态字3  W2
	pRs++;
	*pRs = pcuState4; // PCU状态字4  W3
	pRs++;

	*pRs = anData[84] >> 8; // 主机5V - H  W4
	pRs++;
	*pRs = anData[84]; // 主机5V - L  W5
	pRs++;
	*pRs = anData[85] >> 8; // 备机5V - H  W6
	pRs++;
	*pRs = anData[86]; // 备机5V - L  W7
	pRs++;

	*pRs = equ5VP >> 8; // 均衡+5V电压 - H  W8
	pRs++;
	*pRs = equ5VP; // 均衡+5V电压 - L  W9
	pRs++;
	*pRs = equ12VP >> 8; // 均衡+12V电压 - H  W10
	pRs++;
	*pRs = equ12VP; // 均衡+12V电压 - L  W11
	pRs++;
	*pRs = equ12VN >> 8; // 均衡-12V电压 - H  W12
	pRs++;
	*pRs = equ12VN; // 均衡-12V电压 - L  W13
	pRs++;
	*pRs = equVRef >> 8; // 均衡基准电压 - H  W14
	pRs++;
	*pRs = equVRef; // 均衡基准电压 - L  W15
	pRs++;

	*pRs = sepCurrentState1; // 充分模块1分流指示  W16
	pRs++;
	*pRs = sepCurrentState2; // 充分模块2分流指示  W17
	pRs++;
	*pRs = sepCurrentState3; // 充分模块3分流指示  W18
	pRs++;
	*pRs = sepCurrentState4; // 充分模块4分流指示  W19
	pRs++;

	*pRs = chargeVolKickRsA; // A组充电限压档数  W20
	pRs++;
	*pRs = chargeVolKickRsB; // B组充电限压档数  W21
	pRs++;
	*pRs = chargeCurKickA; // A组充电限流档数  W22
	pRs++;
	*pRs = chargeCurKickB; // B组充电限流档数  W23
	pRs++;

	*pRs = proportion; // 蓄电池组充放比  W24
	pRs++;
	*pRs = importDataRxState; // 重要数据恢复状态字 W25
	pRs++;
	*pRs = selfControlCount; // 自主控制器计数 W26
	pRs++;
	*pRs = selfCheckStateWord; // 下位机自检状态字 W27
	pRs++;
	*pRs = onoffReceCount; // 指令接收计数器 W28
	pRs++;
	*pRs = resetCount; // 热复位计数 W29
	pRs++;
	*pRs = onoffErrorCount; // 错误指令计数 W30
	pRs++;
	*pRs = onoffExeCount; // 指令执行计数 W31
	pRs++;

	for (i = 0; i < 9; i++)
	{
		*pRs = equExeCount[i]; // 单体均衡计数 W33 ~ W41
		pRs++;
	}
}

/***************************************************************************************/
/* 函数介绍: PowerControl 安时计电量计算函数                                          */
/* 功能描述:                                                                           */
/* 修改记录:                                                                           */
/*                                                                                     */
/***************************************************************************************/
void PowerControl(void)
{
	unsigned char fgVa;

	// 蓄电池电压 三取二
	if ((accVAHardPyh > volTab[chargeCurKickA - 1]) && (accVAEquPyh > volTab[chargeCurKickA - 1]))
	{
		fgVa = TRUE;
	}
	else if ((accVAHardPyh > volTab[chargeCurKickA - 1]) && (accVBEquPyh > volTab[chargeCurKickA - 1]))
	{
		fgVa = TRUE;
	}
	else if ((accVAEquPyh > volTab[chargeCurKickA - 1]) && (accVBEquPyh > volTab[chargeCurKickA - 1]))
	{
		fgVa = TRUE;
	}
	else // 无满足条件
	{
		fgVa = FALSE;
	}

	if (fgVa == TRUE) // 满电量蓄电池组电压判读
	{
		AhCount16S++;		 // 满电量时间判读
		if (AhCount16S >= 4) // 满电量时间持续16s以上
		{
			if ((limbCurrentA + limbCurrentB) > (genCurrent + chargeCurrent + 2)) // 电流判断
			{
				if ((chargeCurrent < 5.0) && (chargeCurrent > 0.5)) // 有效区间
				{
					fgAh = TRUE; // 设置 安时计 启动标志

					currentPower = ntrPower;  // 设置当前电量为满电量
					chargePower = (float)0.0; // 充电、放电电量是0Ah
					dischargePower = (float)0.0;

					AhCount16S = 0; // 计数清零
				}
			}
		}
	}
	else
	{
		AhCount16S = 0; // 计数清零
	}

	if (fgAh == TRUE) // 安时计标志为1
	{
		if (chargeCurrent > 0.5) // 充电电流 有效范围
		{
			chargePower = chargePower + (chargeCurrent / (float)(9 * proportion)); // 充电电量计算
		}

		if (dischargeCurrent > 2.0)
		{
			dischargePower = dischargePower + (dischargeCurrent / 900); // 放电电流计算
		}

		currentPower = ntrPower + chargePower - dischargePower; // 当前电量计算

		if (chargePower >= dischargePower) // 充电电量大于等于放电电量蓄电池充满
		{
			chargePower = (float)0.0;
			dischargePower = (float)0.0;
			currentPower = ntrPower;
		}
	}

	if (currentPower <= 0) // 如果当前电量值出现负值
	{
		currentPower = (float)0.0; // 当前电量维持0Ah
	}

	if (dischargePower >= ntrPower) // 如果放电电量超过满电量
	{
		dischargePower = dischargePower - chargePower; // 放电电量取放电电量与充电电量差值
		chargePower = (float)0.0;					   // 充电电量清零
	}

	powerSave[0] = (unsigned char)((unsigned int)((currentPower * 1000) / 3.0) >> 8);	  /* 取当前电量的高字节  */
	powerSave[1] = (unsigned char)((unsigned int)((currentPower * 1000) / 3.0) & 0xFF);	  /* 取当前电量的低字节  */
	powerSave[2] = (unsigned char)((unsigned int)((chargePower * 1000) / 3.0) >> 8);	  /* 取充电电量的高字节  */
	powerSave[3] = (unsigned char)((unsigned int)((chargePower * 1000) / 3.0) & 0xFF);	  /* 取充电电量的低字节  */
	powerSave[4] = (unsigned char)((unsigned int)((dischargePower * 1000) / 3.0) >> 8);	  /* 取放电电量的高字节  */
	powerSave[5] = (unsigned char)((unsigned int)((dischargePower * 1000) / 3.0) & 0xFF); /* 取放电电量的低字节  */
}

//******************************************************
// 函数名称 : EquControl
// 功能描述 : 均衡控制
// 修改记录 :
// *******************************************************
void EquControl(void)
{
	unsigned char i;
	unsigned char j;
	unsigned char index;
	unsigned char maxindex; // 标示3~4.5V的最大序号
	unsigned int TlevelSet;

	if (equControlEn == TRUE) // 均衡控制使能
	{
		fg30S++; // 均衡时间计数

		if (fg30S > 29) // 30秒控制周期
		{
			fg30S = 0; // 控制周期清零

			index = 0;
			maxindex = 9;

			for (i = 0; i < 9; i++)
			{
				if (aqPhyPx[i] > (float)4.5) // 单体电压大于3V小于4.5V 3/0.0025=1200
				{
					maxindex = i; // 电压大于3V的单体计数
					break;
				}
			}

			for (i = 0; i < maxindex; i++)
			{
				if (aqPhyPx[i] >= (float)3.0) // 单体电压大于3V小于4.5V 3/0.0025=1200
				{
					index++; // 电压大于等于3V的单体计数
				}
			}

			for (j = 0; j < index; j++)
			{
				for (i = 0; i < 9; i++)
				{
					if (fabs(aqPhy[i] - aqPhyPx[(maxindex - index) + j]) < 0.0001) // 判断是否相等
					{
						if (((aqStateContrl << i) & 0x8000) == 0x8000) // 最小单体参与控制
						{
							break;
						}
					}
				}
				if (i < 9)
				{
					break;
				}
			}

			index = (unsigned char)(index - j);
			if (index > 1) //参与均衡控制的单体大于等于2
			{
				for (i = 0; i < 9; i++)
				{
					if (((aqPhy[i] >= (float)3.0) && (aqPhy[i] <= (float)4.5)) && (fabs(aqPhy[i] - aqPhyPx[maxindex - index]) >= 0.0001) && (((aqStateContrl << i) & 0x8000) == 0x8000)) //单体电压不是最小值且参与控制
					{
						if ((aqPhy[i] - aqPhyPx[maxindex - index]) > equOpenValue) //差值大于60mv
						{
							equOnoffBuffer[wrEquOnoffIndex] = equAqOnTab[i]; // 发通指令
							wrEquOnoffIndex++;
							if (wrEquOnoffIndex > 47) // 循环指回
							{
								wrEquOnoffIndex = 0;
							}

							TlevelSet = equState1 * 256 + equState2;   // 均衡器状态字
							if (((TlevelSet << i) & 0x8000) != 0x8000) // 断
							{
								equExeCount[i]++; // 均衡计数++  （断到通）
							}
						}
						else //状态为开启
						{
							if ((aqPhy[i] - aqPhyPx[maxindex - index]) < equCloseValue) //差值小于10mv
							{
								equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[i]; // 发断指令
								wrEquOnoffIndex++;
								if (wrEquOnoffIndex > 47) // 循环指回
								{
									wrEquOnoffIndex = 0;
								}
							}
						}
					}
					else
					{
						equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[i]; // 发断指令
						wrEquOnoffIndex++;
						if (wrEquOnoffIndex > 47) // 循环指回
						{
							wrEquOnoffIndex = 0;
						}
					}
				}
			}
			else
			{
				for (i = 0; i < 9; i++) //否则全部断开
				{
					equOnoffBuffer[wrEquOnoffIndex] = equAqOffTab[i];
					wrEquOnoffIndex++;
					if (wrEquOnoffIndex > 47) // 循环指回
					{
						wrEquOnoffIndex = 0;
					}
				}
			}
		}
	}
	else // 均衡禁止时，控制时间计数清零
	{
		fg30S = 0;
	}
}

//******************************************************
// 函数名称 : EquOnoffHook
// 功能描述 : 均衡控制
// 修改记录 :
// *******************************************************
void EquOnoffHook(void)
{
	if (wrEquOnoffIndex != rdEquOnoffIndex) // 是否有转发指令请求
	{
		fgEquTx = TRUE; //设置标志

		TXD422En = 0; //发送使能

		TXD422BUFF = (equOnoffBuffer[rdEquOnoffIndex] >> 8) & 0xFF;
		TXDCONTROL = 0; // 门控使能

		Delay(1000);	// 延时4ms
		TXDCONTROL = 1; // 门控禁止
		TXD422En = 1;	// 发送禁止

		rdEquOnoffIndex++;
		if (rdEquOnoffIndex > 47) // 循环指回
		{
			rdEquOnoffIndex = 0;
		}
	}
}

//****************************************************
// 函数名称 : SingleProtectChargeAControl
// 功能描述 : 单体保护充电控制模块
// *****************************************************
void SingleProtectChargeControl(void)
{
	unsigned char i;
	unsigned char j;
	unsigned char tflag;
	unsigned char singleV;
	float xdata aqCalVA;
	float xdata aqCalVB;
	float xdata result1;
	float xdata result2;

	if ((singleProtectChargeEn == TRUE) && (chargeFlag == TRUE)) //单体保护充电控制允许&& 充电
	{
		if ((equVRefPhy >= 4.9) && (equVRefPhy <= 5.1)) //均衡器基准能否正常工作//均衡器基准能否正常工作
		{
			if (accVAEquPyh > (float)(3.6 * (float)aqControlNumber)) //来自均衡器的蓄电池组电压 -- 主电压
			{
				for (i = 0; i < 9; i++)
				{
					tflag = FALSE;
					if ((((aqStateContrl << i) & 0x8000) != 0x8000) || (aqPhy[i] <= 3.50) || (aqPhy[i] >= 4.50)) //最小单体参与控制
					{
						tflag = FALSE;
					}
					else // 正常单体控制时
					{
						for (j = 0; j < 9; j++) // 计算除此节单体外电压值
						{
							if (i = !j) // 除此节单体电压外
							{
								aqCalVA = accVAEquPyh - aqPhy[i]; // 蓄电池组A 计算单体值
								aqCalVB = accVBEquPyh - aqPhy[i]; // 蓄电池组B 计算单体值
							}
						}

						if ((aqPhy[i] >= singleVProtectValueUP) && (aqCalVA >= singleVProtectValueUP)) // 两者同时超过上限保护值
						{
							tflag = TRUE;
						}
						else if (((aqPhy[i] >= singleVProtectValueUP)) || (aqCalVA >= singleVProtectValueUP)) // 有一个超过上限保护值
						{
							result1 = fabs(aqPhy[i] - aqCalVB);		// 差值1
							result2 = fabs(aqCalVA  - aqCalVB);		// 差值2
							if (result1 > result2) // A组值大于B组
							{
								if (aqCalVA >= singleVProtectValueUP)
								{
									tflag = TRUE;
								}
							}
							else // A组织小于B组
							{
								if (aqPhy[i] >= singleVProtectValueUP)
								{
									tflag = TRUE;
								}
							}
						}
					}

					if (tflag == TRUE) // 需要进行单体保护
					{
						for (singleV = 1; singleV < 8; singleV++) // 电压档位
						{
							if ((accVAEquPyh <= singlechargeVolt[singleV]) && (accVAEquPyh > singlechargeVolt[singleV - 1])) // 找到需要降档的档数
							{
								if (singlesaveflag == FALSE)
								{
									singlesaveVLevelA = chargeVolKickA; // 保存单体保护前的状态
									singlesaveVLevelB = chargeVolKickB; // 保存单体保护前的状态
									singlesaveflag = TRUE;
								}

								chargeVolKickRsA = volKickTab[singleV - 1]; // 记录档位
								chargeVolKickRsB = volKickTab[singleV - 1]; // 记录档位

								DA_A_VOL = volCsTab[chargeVolKickRsA - 1]; // A组电压 切换执行
								DA_B_VOL = volCsTab[chargeVolKickRsA - 1]; // B组电压 切换执行

								OnoffSelfExeRec(0xAA, chargeVolKickRsA); // 自主指令记录 - A组 切换过压切换记录
								OnoffSelfExeRec(0xAB, chargeVolKickRsB); // 自主指令记录 - B组 切换过压切换记录

								selfControlCount++; // 自主保护计数加1
								singleProtectChargeCount++;
								if (singleProtectChargeCount > 3) // 有效范围
								{
									singleProtectChargeCount = 0;
								}

								break;
							}
						}
					}
				}
			}
		}
	}
}

// ****************************************************
// 函数名称 : OnoffSelfExeRec
// 功能描述 :
// *****************************************************
void OnoffSelfExeRec(unsigned char type, unsigned char number)
{
	if (type == 0xAA) // A组 过压切换记录
	{
		onoffBufferRec[wrOnoffRecIndex][0] = sysTime >> 24; // 记录自主指令时间
		onoffBufferRec[wrOnoffRecIndex][1] = sysTime >> 16;
		onoffBufferRec[wrOnoffRecIndex][2] = sysTime >> 8;
		onoffBufferRec[wrOnoffRecIndex][3] = sysTime;
		onoffBufferRec[wrOnoffRecIndex][4] = volSwitchOnoffTabA[number - 1]; // 记录指令码
		onoffBufferRec[wrOnoffRecIndex][5] = volSwitchOnoffTabA[number - 1];

		wrOnoffRecIndex++;
		if (wrOnoffRecIndex > 23) // 指令记录范围
		{
			wrOnoffRecIndex = 0;
		}
	}
	else if (type == 0xAB) // B组 过压切换记录
	{
		onoffBufferRec[wrOnoffRecIndex][0] = sysTime >> 24; // 记录自主指令时间
		onoffBufferRec[wrOnoffRecIndex][1] = sysTime >> 16;
		onoffBufferRec[wrOnoffRecIndex][2] = sysTime >> 8;
		onoffBufferRec[wrOnoffRecIndex][3] = sysTime;
		onoffBufferRec[wrOnoffRecIndex][4] = volSwitchOnoffTabB[number - 1]; // 记录指令码
		onoffBufferRec[wrOnoffRecIndex][5] = volSwitchOnoffTabB[number - 1];

		wrOnoffRecIndex++;
		if (wrOnoffRecIndex > 23) // 指令记录范围
		{
			wrOnoffRecIndex = 0;
		}
	}
	else if (type == 0xBB)
	{
		onoffBufferRec[wrOnoffRecIndex][0] = sysTime >> 24; // 记录自主指令时间
		onoffBufferRec[wrOnoffRecIndex][1] = sysTime >> 16;
		onoffBufferRec[wrOnoffRecIndex][2] = sysTime >> 8;
		onoffBufferRec[wrOnoffRecIndex][3] = sysTime;
		onoffBufferRec[wrOnoffRecIndex][4] = number; // 记录指令码
		onoffBufferRec[wrOnoffRecIndex][5] = number;

		wrOnoffRecIndex++;
		if (wrOnoffRecIndex > 23) // 指令记录范围
		{
			wrOnoffRecIndex = 0;
		}
	}
}

// ****************************************************
// 函数名称 : RamCheck
// 功能描述 : RAM自检函数
// *****************************************************
unsigned char RamCheck(void)
{
	unsigned char i;
	unsigned char xdata check[64];

	for (i = 0; i < 64; i++) //向SRAM写入0x55
	{
		check[i] = 0x55;
	}
	for (i = 0; i < 64; i++)
	{
		if (check[i] != 0x55)
		{
			return (TRUE); // 返回错误20170815
		}
	}
	for (i = 0; i < 64; i++) //向SRAM写入0xAA
	{
		check[i] = 0xAA;
	}
	for (i = 0; i < 64; i++)
	{
		if (check[i] != 0xAA)
		{
			return (TRUE); // 返回错误
		}
	}
	return (FALSE); //返回正确
}

/***************************************************************************************/
/*  函数名称:  PowerSupplyCheck                                                        */
/*  功能描述:  供电函数                                                                */
/*  修改记录:                                                                          */
/***************************************************************************************/
void PowerSupplyCheck(void)
{
	if (selectAB == 0xAA) // A机 时    区分AB电压判读
	{
		if ((anData[84] > (0x3E8 + 120)) || (anData[84] < (0x3E8 - 120))) // 5V基准电压 主机5V 0.3V
		{
			checkCount8++; // 计数增加
			if (checkCount8 > 16)
			{
				fgCheck8 = TRUE; // 自检异常 返回 TRUE - 异常范围
			}
		}
		else
		{
			checkCount8 = 0;  // 计数清零
			fgCheck8 = FALSE; //
		}
	}
	else // B机 时
	{
		if ((anData[85] > (0x3E8 + 120)) || (anData[85] < (0x3E8 - 120))) // 5V基准电压  备机5V  0.3V
		{
			checkCount8++; // 计数增加
			if (checkCount8 > 16)
			{
				fgCheck8 = TRUE; // 自检异常 返回 TRUE - 异常范围
			}
		}
		else
		{
			checkCount8 = 0;  // 计数清零
			fgCheck8 = FALSE; //
		}
	}
}

//****************************************************
// 函数名称 : OverChargeProtect
// 功能描述 : 过充保护函数
// *****************************************************
void OverChargeProtect(void)
{
	unsigned char i;
	float xdata batteryVminus1;
	float xdata batteryVminus2;
	float xdata tempfloat;
	unsigned int xdata batteryValueCunt;
	unsigned int xdata batteryChannel;

	if ((overChargeProtectEn == TRUE) && (chargeFlag == TRUE)) // 过充保护允许
	{
		if ((equVRefPhy >= 4.9) && (equVRefPhy <= 5.1)) // 均衡器基准能否正常工作
		{
			if ((accVAEquPyh >= batteryVAWardValueUP) && (accVBEquPyh >= batteryVAWardValueUP)) // 蓄电池组电压主、备都不小于蓄电池电压上限保护值
			{
				fgOverChargeprotect = TRUE;
			}
			else if ((accVAEquPyh >= batteryVAWardValueUP) || (accVBEquPyh >= batteryVAWardValueUP)) // 只存在1个不小于
			{
				batteryVminus1 = fabs(accVAEquPyh - accVAHardPyh);
				batteryVminus2 = fabs(accVBEquPyh - accVAHardPyh);
				tempfloat = (batteryVminus1 < batteryVminus2) ? accVAEquPyh : accVBEquPyh;
				if (tempfloat >= batteryVAWardValueUP) // 差值小的是否超过设定阈值
				{
					fgOverChargeprotect = TRUE;
				}
				else
				{
					fgOverChargeprotect = FALSE; // 增加过充保护状态清零操作
				}
			}
			else
			{
				fgOverChargeprotect = FALSE; // 增加过充保护状态清零操作
			}

			if (fgOverChargeprotect == TRUE) // 是否可以判断为过充状态下
			{
				for (i = 0; i < 10; i++) // 需要判断A组充电电路是否有且只有一路故障充电
				{
					if (chargeArrayCurrent[i] > 4.0) // 充电阵电流大于4A
					{
						chargeArrayCount[i]++;
					}
					else
					{
						chargeArrayCount[i] = 0;
					}
				}
				batteryValueCunt = 0;
				batteryChannel = 0;

				if (batteryValueCunt == 1) // 有且仅有一路充电阵故障
				{
					fgOverChargeShort = TRUE; //

					if ((pcuState1 == 0xFF) && ((pcuState2 & 0xC0) == 0xC0)) // 充电阵全部接通条件下 AB组
					{
						switch (batteryChannel) // 故障路数
						{
						case 0:							 // A1故障
							ONOFFOutput(5);				 // 充电阵A1断开
							OnoffSelfExeRec(0xBB, 0x11); // 自主指令记录
							break;

						case 1:							 // A2故障
							ONOFFOutput(14);			 // 充电阵A2断开
							OnoffSelfExeRec(0xBB, 0x12); // 自主指令记录
							break;

						case 2:							 // A3故障
							ONOFFOutput(6);				 // 充电阵A3断开
							OnoffSelfExeRec(0xBB, 0x13); // 自主指令记录
							break;

						case 3:							 // A4故障
							ONOFFOutput(15);			 // 充电阵A4断开
							OnoffSelfExeRec(0xBB, 0x14); // 自主指令记录
							break;

						case 4:							 // A5故障
							ONOFFOutput(7);				 // 充电阵A5断开
							OnoffSelfExeRec(0xBB, 0x15); // 自主指令记录
							break;

						case 5:							 // B1故障
							ONOFFOutput(21);			 // 充电阵A5断开
							OnoffSelfExeRec(0xBB, 0x17); // 自主指令记录
							break;

						case 6:							 // B2故障
							ONOFFOutput(30);			 // 充电阵B1断开
							OnoffSelfExeRec(0xBB, 0x18); // 自主指令记录
							break;

						case 7:							 // B3故障
							ONOFFOutput(22);			 // 充电阵B2断开
							OnoffSelfExeRec(0xBB, 0x19); // 自主指令记录
							break;

						case 8:							 // B4故障
							ONOFFOutput(31);			 // 充电阵B3断开
							OnoffSelfExeRec(0xBB, 0x1A); // 自主指令记录
							break;

						case 9:							 // B5故障
							ONOFFOutput(23);			 // 充电阵B4断开
							OnoffSelfExeRec(0xBB, 0x1B); // 自主指令记录
							break;

						default:
							break;
						}

						selfControlCount++;			// 过充自主控制计数
						overChargeProCount++;		// 过充保护自主计数
						if (overChargeProCount > 3) // bit2 有效范围
						{
							overChargeProCount = 0;
						}
					}
				}
			}
			else //
			{
				for (i = 0; i < 10; i++) // 不过充则清零计数
				{
					chargeArrayCount[i] = 0;
				}
			}
		}
	}
}

//****************************************************
// **函数名称 : ChargeCheck
// **功能描述 : 充电控制模块
// *****************************************************
void ChargeCheck(void)
{
	unsigned char temp;
	unsigned char ttflag;

	ttflag = FALSE;
	temp = ((aqStateContrl >> 5) & 0x03); // 判断有几节单体参与控制

	switch (temp)
	{
	case 0x03:
		aqControlNumber = 0x09;
		break;
	case 0x02:
		aqControlNumber = 0x08;
		break;
	case 0x01:
		aqControlNumber = 0x07;
		break;
	case 0x00:
		aqControlNumber = 0x06;
		break;
	default:
		break;
	}

	chargeFlag = FALSE; //充电状态默认不充电
	if (meaV > 7.5)		// MEA大于7.5V
	{
		currentModeCountB[0]++;	  //MEA计数
		currentModeCountA[0] = 0; //MEA计数
	}
	else
	{
		currentModeCountA[0]++;	  // MEA计数
		currentModeCountB[0] = 0; // MEA计数
	}
	if (chargeCurrent1 > 4) // 充电电流大于4A
	{
		currentModeCountB[1]++; //MEA计数
	}
	else
	{
		currentModeCountB[1] = 0; //MEA计数
	}
	if (chargeCurrent2 > 4) //B组充电电流大于4A
	{
		currentModeCountB[2]++; //MEA计数
	}
	else
	{
		currentModeCountB[2] = 0; //MEA计数
	}
	if (dischargeCurrent1 > 5) // 放电电流1计数
	{
		currentModeCountA[1]++;
	}
	else
	{
		currentModeCountA[1] = 0;
	}
	if (dischargeCurrent2 > 5) // 放电电流2计数
	{
		currentModeCountA[2]++;
	}
	else
	{
		currentModeCountA[2] = 0;
	}
	// 判断是否处于放电状态
	if (currentModeCountA[0] >= 28)
	{
		if ((currentModeCountA[1] > 59) || (currentModeCountA[2] > 59))
		{
			ttflag = TRUE;
		}
	}
	else
	{
		if ((currentModeCountA[1] > 59) && (currentModeCountA[2] > 59))
		{
			ttflag = TRUE;
		}
	}

	// 判断是否处于充电状态
	if (currentModeCountB[0] >= 28)
	{
		if ((currentModeCountB[1] > 27) || (currentModeCountB[2] > 27)) //充电状态
		{
			chargeFlag = TRUE; // 设置充电状态
		}
	}
	else
	{
		if ((currentModeCountB[1] > 27) && (currentModeCountB[2] > 27)) //充电状态
		{
			chargeFlag = TRUE; // 设置充电状态
		}
	}

	if (ttflag == TRUE) // 处于放电
	{
		if (singlesaveflag == TRUE) // 单体保护充电控制自主切换曲线
		{
			if ((singlesaveVLevelA > 0) && (singlesaveVLevelA < 9)) //档位范围1~8
			{
				chargeVolKickA = singlesaveVLevelA;	  //恢复保护前状态
				chargeVolKickA_B = singlesaveVLevelA; // A组电压
				chargeVolKickA_C = singlesaveVLevelA; // A组电压

				chargeVolKickB = singlesaveVLevelB;	  // 恢复保护前状态
				chargeVolKickB_B = singlesaveVLevelB; // B组电压
				chargeVolKickB_C = singlesaveVLevelB; // B组电压

				singlesaveflag = FALSE; //保存状态

				chargeVolKickRsA = chargeVolKickA;
				chargeVolKickRsB = chargeVolKickB;

				DA_A_VOL = volCsTab[chargeVolKickRsA - 1]; // A组电压 切换执行
				DA_B_VOL = volCsTab[chargeVolKickRsA - 1]; // B组电压 切换执行

				OnoffSelfExeRec(0xAA, chargeVolKickRsA); // 自主指令记录 - A组 切换过压切换记录
				OnoffSelfExeRec(0xAB, chargeVolKickRsB); // 自主指令记录 - B组 切换过压切换记录

				selfControlCount++; // 自主控制计数
			}
		}
	}
}

//---------------------------------------------------------------------------------------------------------------//
