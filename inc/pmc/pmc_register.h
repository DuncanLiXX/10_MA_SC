/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2020/10/29
 *@brief 本头文件包含PMC中寄存器数据类的声明
 *@version
 */
#ifndef PMCREGISTER_H_
#define PMCREGISTER_H_

#include <stdint.h>

#include "global_definition.h"
#include "hmi_shared_data.h"

class MICommunication;

#pragma pack(1)

//单通道F寄存器点位映射定义, 单通道256字节
struct FRegBits{
	//F0
	uint8_t :4;
	uint8_t SPL:1;						//循环暂停灯信号   F0.4
	uint8_t STL:1;						//循环启动灯信号   F0.5
	uint8_t SA:1;						//伺服就绪信号   F0.6
	uint8_t OP:1;						//自动运行信号   F0.7
	//F1
	uint8_t AL:1;						//告警信号      F1.0
	uint8_t RST:1;						//复位信号		F1.1
	uint8_t BAL:1;						//电池告警信号    F1.2
	uint8_t DEN:1;						//分配结束信号   F1.3
	uint8_t ENB:1;                      //主轴使能信号   F1.4
	uint8_t TAP:1;                      //攻丝信号       F1.5
	uint8_t :1;
	uint8_t MA:1;						//CNC就绪信号	F1.7
	//F2
	uint8_t :8;
	//F3
	uint8_t MINC:1;					//步进选择检测信号   F3.0
	uint8_t MH:1;					//手轮进给信号   F3.1
	uint8_t MJ:1;					//手动JOG选择检测信号   F3.2
	uint8_t MMDI:1;					//MDI模式信号    F3.3
	uint8_t MREF:1;                 //原点模式     F3.4
	uint8_t MMEM:1;                 //自动运行模式信号    F3.5
	uint8_t MEDT:1;                 //存储器编辑选择检测信号   F3.6
	uint8_t :1;
	//F4
	uint8_t MBDT1:1;				//跳段选择检测信号   F4.0
    uint8_t MMLK:1;                 //所有轴机床锁住检测信号 F4.1
    uint8_t :1;
	uint8_t MSBK:1;					//单段选择检测信号   F4.3
	uint8_t :4;
	//F5
	uint8_t :8;
	//F6
	uint8_t :8;
	//F7
	uint8_t MF:1;						//辅助功能选通信号 F7.0
	uint8_t EFD:1;                      //高速接口的外部运行信号   F7.1
	uint8_t SF:1;						//主轴速度功能选通信号 F7.2
	uint8_t TF:1;						//刀具功能选通信号 F7.3
	uint8_t :4;
	//F8
	uint8_t :4;
	uint8_t MF2:1;						//第2M功能选通信号 F8.4
	uint8_t MF3:1;						//第3M功能选通信号 F8.5
	uint8_t :2;
	//F9
	uint8_t :2;
	uint8_t DM98:1;						//M98译码信号   F9.2
	uint8_t DM99:1;                     //M99译码信号   F9.3
	uint8_t DM30:1;						//M30译码信号   F9.4
	uint8_t DM02:1;						//M02译码信号   F9.5
	uint8_t DM01:1;						//M01译码信号   F9.6
	uint8_t DM00:1;						//M00译码信号   F9.7
	//F10
	uint8_t mcode_0:8;					//辅助功能代码信号，1组
	//F11
	uint8_t mcode_1:8;					//辅助功能代码信号，1组
	//F12
	uint8_t mcode_2:8;					//辅助功能代码信号，1组
	//F13
	uint8_t mcode_3:8;					//辅助功能代码信号，1组
	//F14
	uint8_t mcode_20:8;					//辅助功能代码信号，2组
	//F15
	uint8_t mcode_21:8;					//辅助功能代码信号，2组
	//F16
	uint8_t mcode_30:8;					//辅助功能代码信号，3组
	//F17
	uint8_t mcode_31:8;					//辅助功能代码信号，3组
	//F18
	uint8_t :8;
	//F19
	uint8_t :8;
	//F20
	uint8_t :8;
	//F21
	uint8_t :8;
	//F22
	uint8_t scode_0:8;					//主轴速度代码信号
	//F23
	uint8_t scode_1:8;					//主轴速度代码信号
	//F24
	uint8_t scode_2:8;					//主轴速度代码信号
	//F25
	uint8_t scode_3:8;					//主轴速度代码信号
	//F26
	uint8_t tcode_0:8;					//刀具功能代码信号，1组
	//F27
	uint8_t tcode_1:8;					//刀具功能代码信号，1组
	//F28
	uint8_t tcode_2:8;					//刀具功能代码信号，1组
	//F29
	uint8_t tcode_3:8;					//刀具功能代码信号，1组
	//F30
	uint8_t :8;
	//F31
	uint8_t :8;
	//F32
	uint8_t :8;
	//F33
	uint8_t :8;
	//F34
    uint8_t GR1O:1;                      //档位1选择信号
    uint8_t GR2O:1;                      //档位2选择信号
    uint8_t GR3O:1;                      //档位3选择信号
    uint8_t :5;
	//F35
	uint8_t :8;
    //F36~F37
    uint16_t RO:16;                      //主轴电机转速输出
	//F38
	uint8_t :6;
	uint8_t SPS:2;                       //主轴旋转状态  F38.6~F38.7  0--停转   1--正转  2--反转
	//F39
	uint8_t :8;
	//F40
	uint8_t :8;
	//F41
    uint8_t CLMLK:1;                     //机械锁住恢复中 F41.0
    uint8_t :7;
	//F42
	uint8_t :8;
	//F43
	uint8_t :8;
	//F44
	uint8_t :8;
	//F45
    uint8_t :1;
    uint8_t SST:1;     // 零速信号 F45.1
    uint8_t :1;
    uint8_t SAR:1;     // 速度到达信号 F45.3
    uint8_t :3;
    uint8_t ORAR:1;    // 定向结束信号 F45.7
	//F46
	uint8_t :8;
	//F47
	uint8_t :8;
	//F48
	uint8_t :8;
	//F49
	uint8_t :8;
	//F50
	uint8_t :8;
	//F51
	uint8_t :8;
	//F52
	uint8_t :8;
	//F53
	uint8_t :8;
	//F54
	uint8_t :8;
	//F55
	uint8_t :8;
	//F56
	uint8_t :8;
	//F57
	uint8_t :8;
	//F58
	uint8_t :8;
	//F59
	uint8_t :8;
	//F60
	uint8_t EREND:1;                //外部数据输入读取结束信号   F60.0
	uint8_t :7;
	//F61
    uint8_t :8;
	//F62
	uint8_t :8;
	//F63
	uint8_t :8;
	//F64
	uint8_t :7;
	uint8_t TAM:1;               //自动对刀信号      F64.7
	//F65
    uint8_t RGSPP:1;            //进入攻丝状态反馈 F65.0
    uint8_t :1;
    uint8_t RGMP:1;             //主轴模式切换 F65.2
    uint8_t :5;
	//F66
    uint8_t :1;
    uint8_t RTPT:1;         //刚性攻丝回退结束信号 F66.1
    uint8_t :6;
	//F67
	uint8_t :8;
	//F68
	uint8_t :8;
	//F69
	uint8_t :8;
	//F70
	uint8_t :8;
	//F71
	uint8_t :8;
	//F72
	uint8_t OUT0:1;
	uint8_t OUT1:1;
	uint8_t OUT2:1;
	uint8_t OUT3:1;
	uint8_t OUT4:1;
	uint8_t OUT5:1;
	uint8_t OUT6:1;
	uint8_t OUT7:1;
	//F73
	uint8_t OUT8:1;
	uint8_t OUT9:1;
	uint8_t OUT10:1;
	uint8_t OUT11:1;
	uint8_t OUT12:1;
	uint8_t OUT13:1;
	uint8_t OUT14:1;
	uint8_t OUT15:1;
	//F74
	uint8_t OUT16:1;
	uint8_t OUT17:1;
	uint8_t OUT18:1;
	uint8_t OUT19:1;
	uint8_t OUT20:1;
	uint8_t OUT21:1;
	uint8_t OUT22:1;
	uint8_t OUT23:1;
	//F75
	uint8_t OUT24:1;
	uint8_t OUT25:1;
	uint8_t OUT26:1;
	uint8_t OUT27:1;
	uint8_t OUT28:1;
	uint8_t OUT29:1;
	uint8_t OUT30:1;
	uint8_t OUT31:1;
	//F76
	uint8_t :8;
	//F77
	uint8_t :8;
	//F78
	uint8_t :8;
	//F79
	uint8_t :8;
	//F80
	uint8_t :8;
	//F81
	uint8_t :8;
	//F82
	uint8_t :8;
	//F83
    uint8_t :1;
    uint8_t :1;     // 自动对刀信号 F83.1
    uint8_t :6;
	//F84
	uint8_t :8;
	//F85
	uint8_t :8;
	//F86
	uint8_t :8;
	//F87
	uint8_t :8;
	//F88
	uint8_t :8;
	//F89
	uint8_t :8;
	//F90
	uint8_t :8;
	//F91
	uint8_t :8;
	//F92
	uint8_t :8;
	//F93
	uint8_t :8;
    //F94
	uint8_t ZP11:8;   //返回参考点结束信号   bit0-bit7：  轴1-轴8
	//F95
	uint8_t ZP12:8;   //返回参考点结束信号   bit0-bit7：  轴9-轴16
	//F96
	uint8_t ZP21:8;    //返回第二参考点结束信号   bit0-bit7：  轴1-轴8
	//F97
	uint8_t ZP22:8;   //返回第二参考点结束信号   bit0-bit7：  轴9-轴16
	//F98
	uint8_t ZP31:8;    //返回第三参考点结束信号   bit0-bit7：  轴1-轴8
	//F99
	uint8_t ZP32:8;    //返回第三参考点结束信号   bit0-bit7：  轴9-轴16
	//F100
	uint8_t ZP41:8;   //返回第四参考点结束信号   bit0-bit7：  轴1-轴8
	//F101
	uint8_t ZP42:8;    //返回第四参考点结束信号   bit0-bit7：  轴9-轴16
	//F102
	//F103
	uint16_t MV:16;     //轴移动信号   F102~F103         bit0~bit15：轴1~轴16
	//F104
	//F105
	uint16_t INP:16;    //轴到位信号     F104~F105       bit0~bit15：轴1~轴16
	//F106
	//F107
	uint16_t MVD:16;    //轴运动方向信号    F106~F107    bit0~bit15：轴1~轴16
	//F108
	uint8_t :8;
	//F109
	uint8_t :8;
	//F110
	uint8_t :8;
	//F111
	uint8_t :8;
	//F112
	uint8_t EADEN1:1;							//分配完成信号1(PMC轴控制，1组)     F112.0
	uint8_t EADEN2:1;                           //分配完成信号2(PMC轴控制，2组)     F112.1
	uint8_t EADEN3:1;							//分配完成信号3(PMC轴控制，3组)     F112.2
	uint8_t EADEN4:1;							//分配完成信号4(PMC轴控制，4组)     F112.3
	uint8_t :4;
	//F113
	uint8_t :8;
	//F114
	uint8_t :8;
	//F115
	uint8_t :8;
	//F116
	uint8_t :8;
	//F117
	uint8_t :8;
	//F118
	//F119
	uint16_t IRF:16;   //回零过程中信号   bit0~bit16:  轴1~轴16
	//F120
	uint8_t ZRF1:8;    //参考点建立信号   bit0-bit7：  轴1-轴8
	//F121
	uint8_t ZRF2:8;    //参考点建立信号   bit0-bit7：  轴9-轴16
	//F122
	uint8_t :8;
	//F123
	uint8_t :8;
	//F124
	uint8_t :8;
	//F125
	uint8_t :8;
	//F126
	uint8_t :8;
	//F127
	uint8_t :8;

	//F128
	uint8_t :8;
	//F129
	uint8_t :5;
	uint8_t EOV0:1;                 //倍率0%信号(PMC轴控制)     F129.5
	uint8_t :1;
	uint8_t _EAXSL:1; 				//控制轴选择状态信号（PMC轴控制）   F129.7    NC端PMC轴运动指令缓冲有数据或者CNC控制时为1，无数据时为0
	//F130
	uint8_t EINPA:1;				//到位信号，第一组(PMC轴控制)      F130.0
	uint8_t ECKZA:1;				//零跟随误差检测信号，第一组(PMC轴控制)     F130.1
	uint8_t EIALA:1;				//报警信号，第一组(PMC轴控制)     F130.2
	uint8_t EDENA:1;				//辅助功能执行信号，第一组(PMC轴控制)     F130.3
	uint8_t EGENA:1;				//轴移动信号，第一组(PMC轴控制)     F130.4
	uint8_t EOTPA:1;				//正向超程信号，第一组(PMC轴控制)     F130.5
	uint8_t EOTNA:1;				//负向超程信号，第一组(PMC轴控制)     F130.6
	uint8_t EBSYA:1;  				//轴控制指令读取完成信号，第一组（PMC轴控制）  F130.7
	//F131
	uint8_t EMFA:1;					//辅助功能选通信号，第一组（PMC轴控制）F131.0
	uint8_t EABUFA:1;				//PMC轴控制数据缓冲器满信号，第一组（PMC轴控制）F131.1
	uint8_t EMF2A:1;				//辅助功能2选通信号，第一组（PMC轴控制）F131.2
	uint8_t EMF3A:1;				//辅助功能3选通信号，第一组（PMC轴控制）F131.3
	uint8_t :4;
	//F132
	uint8_t :8;
	//F133
	uint8_t EINPB:1;				//到位信号，第二组(PMC轴控制)      F133.0
	uint8_t ECKZB:1;				//零跟随误差检测信号，第二组(PMC轴控制)     F133.1
	uint8_t EIALB:1;				//报警信号，第二组(PMC轴控制)     F133.2
	uint8_t EDENB:1;				//辅助功能执行信号，第二组(PMC轴控制)     F133.3
	uint8_t EGENB:1;				//轴移动信号，第二组(PMC轴控制)     F133.4
	uint8_t EOTPB:1;				//正向超程信号，第二组(PMC轴控制)     F133.5
	uint8_t EOTNB:1;				//负向超程信号，第二组(PMC轴控制)     F133.6
	uint8_t EBSYB:1;  				//轴控制指令读取完成信号，第二组（PMC轴控制）  F133.7
	//F134
	uint8_t EMFB:1;					//辅助功能选通信号，第二组（PMC轴控制）F134.0
	uint8_t EABUFB:1;				//PMC轴控制数据缓冲器满信号，第二组（PMC轴控制）F134.1
	uint8_t EMF2B:1;				//辅助功能2选通信号，第二组（PMC轴控制）F134.2
	uint8_t EMF3B:1;				//辅助功能3选通信号，第二组（PMC轴控制）F134.3
	uint8_t :4;
	//F135
	uint8_t :8;

	//F136
	uint8_t EINPC:1;				//到位信号，第三组(PMC轴控制)      F136.0
	uint8_t ECKZC:1;				//零跟随误差检测信号，第三组(PMC轴控制)     F136.1
	uint8_t EIALC:1;				//报警信号，第三组(PMC轴控制)     F136.2
	uint8_t EDENC:1;				//辅助功能执行信号，第三组(PMC轴控制)     F136.3
	uint8_t EGENC:1;				//轴移动信号，第三组(PMC轴控制)     F136.4
	uint8_t EOTPC:1;				//正向超程信号，第三组(PMC轴控制)     F136.5
	uint8_t EOTNC:1;				//负向超程信号，第三组(PMC轴控制)     F136.6
	uint8_t EBSYC:1;  				//轴控制指令读取完成信号，第三组（PMC轴控制）  F136.7
	//F137
	uint8_t EMFC:1;					//辅助功能选通信号，第三组（PMC轴控制）F137.0
	uint8_t EABUFC:1;				//PMC轴控制数据缓冲器满信号，第三组（PMC轴控制）F137.1
	uint8_t EMF2C:1;				//辅助功能2选通信号，第三组（PMC轴控制）F137.2
	uint8_t EMF3C:1;				//辅助功能3选通信号，第三组（PMC轴控制）F137.3
	uint8_t :4;
	//F138
	uint8_t :8;
	//F139
	uint8_t EINPD:1;				//到位信号，第四组(PMC轴控制)      F139.0
	uint8_t ECKZD:1;				//零跟随误差检测信号，第四组(PMC轴控制)     F139.1
	uint8_t EIALD:1;				//报警信号，第四组(PMC轴控制)     F139.2
	uint8_t EDEND:1;				//辅助功能执行信号，第四组(PMC轴控制)     F139.3
	uint8_t EGEND:1;				//轴移动信号，第四组(PMC轴控制)     F139.4
	uint8_t EOTPD:1;				//正向超程信号，第四组(PMC轴控制)     F139.5
	uint8_t EOTND:1;				//负向超程信号，第四组(PMC轴控制)     F139.6
	uint8_t EBSYD:1;  				//轴控制指令读取完成信号，第四组（PMC轴控制）  F139.7
	//F140
	uint8_t EMFD:1;					//辅助功能选通信号，第四组（PMC轴控制）F140.0
	uint8_t EABUFD:1;				//PMC轴控制数据缓冲器满信号，第四组（PMC轴控制）F140.1
	uint8_t EMF2D:1;				//辅助功能2选通信号，第四组（PMC轴控制）F140.2
	uint8_t EMF3D:1;				//辅助功能3选通信号，第四组（PMC轴控制）F140.3
	uint8_t :4;
	//F141
	uint8_t :8;
	//F142
	uint8_t :8;
	//F143
	uint8_t :8;

	//F144
	uint8_t mcode_40:8;					//辅助功能代码信号，4组
	//F145
	uint8_t mcode_41:8;					//辅助功能代码信号，4组
	//F146
	uint8_t mcode_50:8;					//辅助功能代码信号，5组
	//F147
	uint8_t mcode_51:8;					//辅助功能代码信号，5组
	//F148
	uint8_t mcode_60:8;					//辅助功能代码信号，6组
	//F149
	uint8_t mcode_61:8;					//辅助功能代码信号，6组
	//F150
	uint8_t mcode_70:8;					//辅助功能代码信号，7组
	//F151
	uint8_t mcode_71:8;					//辅助功能代码信号，7组
	//F152
	uint8_t mcode_80:8;					//辅助功能代码信号，8组
	//F153
	uint8_t mcode_81:8;					//辅助功能代码信号，8组
	//F154
	uint8_t mcode_90:8;					//辅助功能代码信号，9组
	//F155
	uint8_t mcode_91:8;					//辅助功能代码信号，9组
	//F156
	uint8_t mcode_100:8;					//辅助功能代码信号，10组
	//F157
	uint8_t mcode_101:8;					//辅助功能代码信号，10组
	//F158
	uint8_t mcode_110:8;					//辅助功能代码信号，11组
	//F159
	uint8_t mcode_111:8;					//辅助功能代码信号，11组
	//F160
	uint8_t mcode_120:8;					//辅助功能代码信号，12组
	//F161
	uint8_t mcode_121:8;					//辅助功能代码信号，12组
	//F162
	uint8_t mcode_130:8;					//辅助功能代码信号，13组
	//F163
	uint8_t mcode_131:8;					//辅助功能代码信号，13组
	//F164
	uint8_t mcode_140:8;					//辅助功能代码信号，14组
	//F165
	uint8_t mcode_141:8;					//辅助功能代码信号，14组
	//F166
	uint8_t mcode_150:8;					//辅助功能代码信号，15组
	//F167
	uint8_t mcode_151:8;					//辅助功能代码信号，15组
	//F168
	uint8_t mcode_160:8;					//辅助功能代码信号，16组
	//F169
	uint8_t mcode_161:8;					//辅助功能代码信号，16组
	//F170
	uint8_t MF4:1;                          //第4M功能选通信号 F170.0
	uint8_t MF5:1;                          //第5M功能选通信号 F170.1
	uint8_t MF6:1;                          //第6M功能选通信号 F170.2
	uint8_t MF7:1;                          //第7M功能选通信号 F170.3
	uint8_t MF8:1;                          //第8M功能选通信号 F170.4
	uint8_t MF9:1;                          //第9M功能选通信号 F170.5
	uint8_t MF10:1;                          //第10M功能选通信号 F170.6
	uint8_t MF11:1;                          //第11M功能选通信号 F170.7
	//F171
	uint8_t MF12:1;                          //第12M功能选通信号 F171.0
	uint8_t MF13:1;                          //第13M功能选通信号 F171.1
	uint8_t MF14:1;                          //第14M功能选通信号 F171.2
	uint8_t MF15:1;                          //第15M功能选通信号 F171.3
	uint8_t MF16:1;                          //第16M功能选通信号 F171.4
	uint8_t :1;
	uint8_t :1;
	uint8_t :1;
	//F172
	uint8_t :8;
	//F173
	uint8_t :8;
	//F174
	uint8_t :8;
	//F175
	uint8_t PPI:4;                         //当前工艺参数组号
	uint8_t :4;
	//F176
	uint8_t :8;
	//F177
	uint8_t :8;
	//F178
	uint8_t :8;
	//F179
	uint8_t :8;
	//F180
	uint8_t :8;
	//F181
	uint8_t :8;
	//F182
	uint8_t EACNT1:1;					//控制信号，第一组（PMC轴控制）   F182.0
	uint8_t EACNT2:1;					//控制信号，第二组（PMC轴控制）   F182.1
	uint8_t EACNT3:1;					//控制信号，第三组（PMC轴控制）   F182.2
	uint8_t EACNT4:1;					//控制信号，第四组（PMC轴控制）   F182.3
	uint8_t :4;
	//F183
	uint8_t :8;
	//F184
	uint8_t :8;
    //F185-F186
    int16_t SRS:16;                     //主轴设置的定向角度
    //F187-F188
    int16_t SRG:16;                     //主轴当前角度
	//F189
	uint8_t :8;
	//F190~F193
	int32_t wnd_data:32;         //窗口返回数据     F190~F193   鲁匠项目临时使用
	//F194
	uint8_t :8;
	//F195
    uint8_t MERS:1; // MDI复位请求 F195.0
    uint8_t :7;
	//F196
	uint8_t :8;
	//F197
	uint8_t :8;
	//F198
	uint8_t :8;
	//F199
	uint8_t :8;
	//F200~F207
	uint64_t in_ref_point:64;    //回参考点信号，临时方案
#ifdef USES_GRIND_MACHINE
	//F208
	uint8_t change_tray_left:1;       //左侧请求更换料盘   F208.0   左机
	uint8_t change_tray_right:1;       //右侧请求更换料盘  F208.1 左机
	uint8_t :3;
	uint8_t m66_runover_right:1;      //通知右侧上下料完毕  F208.5   左机
	uint8_t m66_req_right:1;          //右侧上下料请求   F208.6   右机
	uint8_t new_tray_right:1;         //右侧通知已换新料盘 F208.7   右机
	//F209
	uint8_t work_vac:1;          //加工位真空阀     F209.0   左/右机
	uint8_t work_hold:1;         //加工位压紧气缸   F209.1   左/右机
	uint8_t :5;
	uint8_t work_vac_right:1;    //通知右机开真空    F209.7   左机
	//F210
	uint8_t left_claw_vac:1;     //左爪真空阀    F210.0    左机
	uint8_t left_claw_down:1;    //左爪下            F210.1   左机
	uint8_t right_claw_vac:1;    //右爪真空阀    F210.2   左机
	uint8_t right_claw_down:1;   //右爪下            F210.3  左机
	uint8_t :4;
	//F211
	uint8_t correct_pos_x:1;     //对位平台X轴夹紧   F211.0    左机
	uint8_t correct_pos_y:1;     //对位平台Y轴夹紧   F211.1  左机
	uint8_t :6;
	//F212
	uint8_t :8;
	//F213
	uint8_t :8;
	//F214
	uint8_t :8;
	//F215
	uint8_t :8;
#else
	//F208~F215
	uint64_t :64;
#endif
	//F216
	uint8_t MPCO:1;           //PMC宏程序调用结束     F216.0
	uint8_t :7;
	//F217
	uint8_t :8;
	//F218
	uint8_t :8;
	//F219
	uint8_t CHNC:4;           //当前通道号
	uint8_t : 4;
	//F220
	uint8_t TF2:1;           //刀具功能选通信号2 F220.0
	uint8_t TF3:1;           //刀具功能选通信号3 F220.1
	uint8_t TF4:1;           //刀具功能选通信号4 F220.2
	uint8_t TF5:1;           //刀具功能选通信号5 F220.3
	uint8_t TF6:1;           //刀具功能选通信号6 F220.4
	uint8_t TF7:1;           //刀具功能选通信号7 F220.5
	uint8_t TF8:1;           //刀具功能选通信号8 F220.6
	uint8_t TF9:1;           //刀具功能选通信号9 F220.7
	//F221
	uint8_t TF10:1;           //刀具功能选通信号10 F221.0
	uint8_t TF11:1;           //刀具功能选通信号11 F221.1
	uint8_t TF12:1;           //刀具功能选通信号12 F221.2
	uint8_t TF13:1;           //刀具功能选通信号13 F221.3
	uint8_t TF14:1;           //刀具功能选通信号14 F221.4
	uint8_t TF15:1;           //刀具功能选通信号15 F221.5
	uint8_t TF16:1;           //刀具功能选通信号16 F221.6
	uint8_t :1;
	//F222
	uint8_t :8;
	//F223
	uint8_t :8;
	//F224
	uint8_t tcode_20:8;            //刀具功能代码信号，2组
	//F225
	uint8_t tcode_21:8;            //刀具功能代码信号，2组
	//F226
	uint8_t tcode_30:8;            //刀具功能代码信号，3组
	//F227
	uint8_t tcode_31:8;            //刀具功能代码信号，3组
	//F228
	uint8_t tcode_40:8;            //刀具功能代码信号，4组
	//F229
	uint8_t tcode_41:8;            //刀具功能代码信号，4组
	//F230
	uint8_t tcode_50:8;            //刀具功能代码信号，5组
	//F231
	uint8_t tcode_51:8;            //刀具功能代码信号，5组
	//F232
	uint8_t tcode_60:8;            //刀具功能代码信号，6组
	//F233
	uint8_t tcode_61:8;            //刀具功能代码信号，6组
	//F234
	uint8_t tcode_70:8;            //刀具功能代码信号，7组
	//F235
	uint8_t tcode_71:8;            //刀具功能代码信号，7组
	//F236
	uint8_t tcode_80:8;            //刀具功能代码信号，8组
	//F237
	uint8_t tcode_81:8;            //刀具功能代码信号，8组
	//F238
	uint8_t tcode_90:8;            //刀具功能代码信号，9组
	//F239
	uint8_t tcode_91:8;            //刀具功能代码信号，9组
	//F240
	uint8_t tcode_100:8;            //刀具功能代码信号，10组
	//F241
	uint8_t tcode_101:8;            //刀具功能代码信号，10组
	//F242
	uint8_t tcode_110:8;            //刀具功能代码信号，11组
	//F243
	uint8_t tcode_111:8;            //刀具功能代码信号，11组
	//F244
	uint8_t tcode_120:8;            //刀具功能代码信号，12组
	//F245
	uint8_t tcode_121:8;            //刀具功能代码信号，12组
	//F246
	uint8_t tcode_130:8;            //刀具功能代码信号，13组
	//F247
	uint8_t tcode_131:8;            //刀具功能代码信号，13组
	//F248
	uint8_t tcode_140:8;            //刀具功能代码信号，14组
	//F249
	uint8_t tcode_141:8;            //刀具功能代码信号，14组
	//F250
	uint8_t tcode_150:8;            //刀具功能代码信号，15组
	//F251
	uint8_t tcode_151:8;            //刀具功能代码信号，15组
	//F252
	uint8_t tcode_160:8;            //刀具功能代码信号，16组
	//F253
	uint8_t tcode_161:8;            //刀具功能代码信号，16组
	//F254
	uint8_t :8;
	//F255
	uint8_t :8;

};

//单通道G寄存器点位映射定义, 单通道256字节
struct GRegBits{
	//G0
	uint8_t ED1:8;					//外部数据输入用数据信号1  G0.0~G0.7
	//G1
	uint8_t ED2:8;					//外部数据输入用数据信号2  G1.0~G1.7
	//G2
	uint8_t EA:7;					//外部数据输入用地址信号  G2.0~G2.6
	uint8_t ESTB:1;                 //外部输入数据读取信号   G2.7
	//G3
	uint8_t :8;
	//G4
	uint8_t :3;
	uint8_t FIN:1;					//FIN信号，结束信号  G4.3
	uint8_t MFIN2:1;				//MFIN2信号，第2M功能结束信号  G4.4
	uint8_t MFIN3:1;				//MFIN3信号，第3M功能结束信号  G4.5
	uint8_t :2;
	//G5
	uint8_t MFIN:1;                 //辅助功能结束信号   G5.0
	uint8_t EFIN:1;                 //外部运行功能结束信号   G5.1
	uint8_t SFIN:1;                 //主轴功能结束信号    G5.2
	uint8_t TFIN:1;                 //刀具功能结束信号    G5.3
	uint8_t :4;
	//G6
	uint8_t :6;
	uint8_t SKIPP:1;			    //SKIPP信号  G6.6  用于G31跳转
	uint8_t :1;
	//G7
	uint8_t :1;
	uint8_t STLK:1;					//启动锁住信号   G7.1
	uint8_t ST:1;					//循环启动信号   G7.2
    uint8_t :3;
    uint8_t EXLM:1;                     //软限位选择信号 0:软限位1  1:软限位2
    uint8_t RLSOT:1;                     //解除软限位限制信号
	//G8
	uint8_t :4;
	uint8_t _ESP:1;					//急停信号  G8.4
	uint8_t _SP:1;					//进给暂停信号    G8.5
	uint8_t RRW:1;					//复位和倒带信号  G8.6
	uint8_t ERS:1;                  //外部复位信号    G8.7
	//G9
	uint8_t :8;
	//G10
	//G11
	uint16_t _JV:16;               //手动移动速度倍率信号   *JV0~*JV15   低有效，单位：0.01%
	//G12
	uint8_t _FV:8;					//进给速度倍率信号    *FV0~*FV7   低有效，单位：1%
	//G13
	uint8_t :8;
	//G14
	uint8_t ROV:8;                 //G00快速倍率信号  G14.0~G14.7  ROV1~ROV7   单位：1%
	//G15
	uint8_t :8;
	//G16
	uint8_t :8;
	//G17
	uint8_t :8;
	//G18
	uint8_t :8;
	//G19
	uint8_t :4;
	uint8_t MP:2;				//手动步长信号 MP1~MP2  G19.4~G19.5
	uint8_t :1;
	uint8_t RT:1;				//手动快速进给选择信号  G19.7
	//G20
#ifdef USES_WUXI_BLOOD_CHECK
	uint8_t ret_home:1;       //回参考点信号   为无锡项目定制  G20.0
	uint8_t reset:1;		//复位信号  G20.1
	uint8_t :6;
#else
	uint8_t :8;
#endif

	//G21
	uint8_t :8;
	//G22
	uint8_t :8;
	//G23
	uint8_t :8;
	//G24
	uint8_t :8;
	//G25
	uint8_t :8;
	//G26
    uint8_t GTC:8;     //获取当前刀号 G26.8
	//G27
	uint8_t :8;
	//G28
	uint8_t :1;
	uint8_t GR1:1;      //齿轮选择信号（输入）     G28.1
	uint8_t GR2:1;      //齿轮选择信号（输入）     G28.2
	uint8_t :1;
	uint8_t _SUCPF:1;   //主轴松开完成信号    低有效     G28.4
	uint8_t _SCPF:1;    //主轴夹紧完成信号    低有效     G28.5
	uint8_t SPSTP:1;    //主轴停止完成信号         G28.6
    uint8_t :1;
	//G29
	uint8_t GR21:1;     //齿轮选择信号（输入）     G29.0
    uint8_t :4;
    uint8_t SOR:1;      //主轴准停信号           G29.5
	uint8_t _SSTP:1;    //主轴停信号             G29.6
	uint8_t :1;
	//G30
	uint8_t SOV:8;      //主轴倍率信号    SOV0~SOV7     G30.0~G30.7     单位：1%
    //G31~G32
    uint16_t RI:16;     //主轴转速输入 R01I~R16I
	//G33
    uint8_t :5;
    uint8_t SGN:1;      //PMC输入的主轴方向    G33.5   0：正 1：负
    uint8_t SSIN:1;     //主轴方向由CNC决定还是PMC决定  G33.6   0：CNC 1：PMC
    uint8_t SIND:1;     //主轴速度由CNC决定还是PMC决定 G33.7   0：CNC 1：PMC
	//G34
	uint8_t :8;
	//G35
	uint8_t :8;
	//G36
	uint8_t :8;
	//G37
	uint8_t :8;
	//G38
	uint8_t :8;
	//G39
	uint8_t :8;
	//G40
	uint8_t :8;
	//G41
    uint8_t HSIA:8;     //手轮插入信号 G41
	//G42
	uint8_t :8;
	//G43
	uint8_t MD:3;       //方式选择，等同于fanuc的MD1/MD2/MD4   G43.0~G43.2
	uint8_t :5;
	//G44
	uint8_t BDT1:1;     //跳段信号    G44.0
    uint8_t MLK:1;      //所有轴机床锁住信号 G44.1
    uint8_t :6;
	//G45
	uint8_t :8;
	//G46
	uint8_t :1;
	uint8_t SBK:1;     //单程序段信号    G46.1
	uint8_t SBS:1;     //选停信号         G46.2
    uint8_t KEY:1;     //程序锁住信号  G46.3
    uint8_t :4;
	//G47
	uint8_t :8;
	//G48
	uint8_t :8;
	//G49
	uint8_t :8;
	//G50
	uint8_t :8;
	//G51
	uint8_t :8;
	//G52
	uint8_t :8;
	//G53
	uint8_t :8;
	//G54
	uint8_t :8;
	//G55
	uint8_t :8;
	//G56
	uint8_t :8;
	//G57
	uint8_t :8;
	//G58
	uint8_t :8;
	//G59
	uint8_t :8;
	//G60
	uint8_t :8;
	//G61
    uint8_t RGTAP:1;    // 刚性攻丝信号 G61.0
    uint8_t RGMD:1;     // 刚性攻丝模式切换信号 G61.1
    uint8_t :6;
	//G62
    uint8_t :6;
    uint8_t RTNT:1;     // 刚性攻丝回退启动信号 G62.6
    uint8_t :1;
	//G63
	uint8_t :8;
	//G64
	uint8_t :8;
	//G65
	uint8_t :8;
	//G66
	uint8_t :8;
	//G67
	uint8_t :2;
	uint8_t HWT:1;     //手轮跟踪信号    G67.2
	uint8_t :5;
	//G68
	uint8_t :8;
    //G69
	uint8_t :8;
	//G70
    uint8_t :4;
    uint8_t SRV:1;      //主轴反转 G70.4
    uint8_t SFR:1;      //主轴正转 G70.5
    uint8_t ORCMA:1;    //主轴定向 G70.6
    uint8_t :1;
	//G71
	uint8_t :8;
	//G72
	uint8_t :8;
	//G73
	uint8_t :8;
	//G74
	uint8_t :8;
	//G75
	uint8_t :8;
	//G76
	uint8_t :8;
	//G77
	uint8_t :8;
	//G78
	uint8_t :8;
	//G79
	uint8_t :8;
	//G80
	uint8_t :8;
	//G81
	uint8_t :8;
	//G82
	uint8_t :8;
	//G83
	uint8_t :8;
	//G84
	uint8_t :8;
	//G85
	uint8_t :8;
	//G86
	uint8_t :8;
	//G87
	uint8_t :8;
	//G88
	uint8_t :8;
	//G89
	uint8_t :8;
	//G90
	uint8_t :8;
	//G91
	uint8_t :8;
	//G92
	uint8_t :8;
	//G93
	uint8_t :8;
	//G94
	uint8_t :8;
	//G95
	uint8_t :8;
	//G96
	uint8_t :8;
	//G97
	uint8_t :8;
	//G98
	uint8_t :8;
	//G99
	uint8_t :8;
	//G100
    uint8_t JP:8;				//进给轴和方向选择信号  +J1    轴1正向
//	uint8_t JP2:1;				//进给轴和方向选择信号  +J2    轴2正向
//	uint8_t JP3:1;				//进给轴和方向选择信号  +J3    轴3正向
//	uint8_t JP4:1;				//进给轴和方向选择信号  +J4    轴4正向
//	uint8_t JP5:1;				//进给轴和方向选择信号  +J5    轴5正向
//	uint8_t JP6:1;				//进给轴和方向选择信号  +J6    轴6正向
//	uint8_t JP7:1;				//进给轴和方向选择信号  +J7    轴7正向
//	uint8_t JP8:1;				//进给轴和方向选择信号  +J8    轴8正向
	//G101
	uint8_t :8;
	//G102
    uint8_t JN:8;				//进给轴和方向选择信号  -J1    轴1负向
//	uint8_t JN2:1;				//进给轴和方向选择信号  -J2    轴2负向
//	uint8_t JN3:1;				//进给轴和方向选择信号  -J3    轴3负向
//	uint8_t JN4:1;				//进给轴和方向选择信号  -J4    轴4负向
//	uint8_t JN5:1;				//进给轴和方向选择信号  -J5    轴5负向
//	uint8_t JN6:1;				//进给轴和方向选择信号  -J6    轴6负向
//	uint8_t JN7:1;				//进给轴和方向选择信号  -J7    轴7负向
//	uint8_t JN8:1;				//进给轴和方向选择信号  -J8    轴8负向
	//G103
    uint8_t :8;
	//G104~G105
	uint16_t REFE:16;           //物理轴回参考点使能信号，  bit0~bit15  轴1~轴15
	//G106
	uint8_t :8;
	//G107
	uint8_t :8;
	//G108
    uint8_t MLKI:8;     //机械锁住信号 轴1~轴8
    //G109
	uint8_t :8;
	//G110
	uint8_t :8;
	//G111
	uint8_t :8;
	//G112
	uint8_t :8;
	//G113
	uint8_t :8;
	//G114
	uint8_t axis_limit_postive1:8;	//正向超程信号， G114.0-G114.7代表轴1~轴8
	//G115
	uint8_t axis_limit_postive2:8;	//正向超程信号， G115.0-G115.7代表轴9~轴16
	//G116
	uint8_t axis_limit_negative1:8;	//负向超程信号， G116.0-G116.7代表轴1~轴8
	//G117
	uint8_t axis_limit_negative2:8;	//负向超程信号， G117.0-G117.7代表轴9~轴16
	//G118
	uint8_t AXIS_PDEC1:8;          //正向轴减速信号，G118.0-G118.7代表轴1~轴8
	//G119
	uint8_t AXIS_PDEC2:8;         //正向轴减速信号，G119.0-G119.7代表轴9~轴16
	//G120
	uint8_t AXIS_NDEC1:8;          //负向轴减速信号，G120.0-G120.7代表轴1~轴8
	//G121
	uint8_t AXIS_NDEC2:8;          //负向轴减速信号，G121.0-G121.7代表轴9~轴16
	//G122
	uint8_t :8;
	//G123
	uint8_t :8;
	//G124
	uint8_t :8;
	//G125
	uint8_t :8;
	//G126
    uint8_t SVF:8;                 //伺服关断信号 每一位表示一个轴 1代表下使能
	//G127
	uint8_t :8;

	//G128~G135
	uint64_t :64;
	//G136
	uint8_t EAX1:1;					//控制轴选择信号1（PMC轴控制，第一组）  G136.0
	uint8_t EAX2:1;					//控制轴选择信号2（PMC轴控制，第二组）  G136.1
	uint8_t EAX3:1;					//控制轴选择信号3（PMC轴控制，第三组）  G136.2
	uint8_t EAX4:1;					//控制轴选择信号4（PMC轴控制，第四组）  G136.3
	uint8_t :4;
	//G137
	uint8_t :8;
	//G138
    uint8_t SYNC:8;                //同步控制功能可在自动和MDI方式下运行
    //G139
	uint8_t :8;
	//G140
    uint8_t SYNCJ:8;                //在JOG、手轮或增量进给方式下执行同步控制
    //G141
	uint8_t :8;
	//G142
	uint8_t EFINA:1;                //辅助功能结束信号（PMC轴控制，第一组）    G142.0
	uint8_t :1;
	uint8_t EMBUFA:1;				//缓冲禁止信号（PMC轴控制，第一组）    G142.2
	uint8_t ESBKA:1;                //程序段停止信号（PMC轴控制，第一组）    G142.3
	uint8_t ESOFA:1;                //伺服关断信号（PMC轴控制，第一组）    G142.4
	uint8_t ESTPA:1;                //轴控制暂停信号（PMC轴控制，第一组）    G142.5
	uint8_t ECLRA:1;				//复位信号（第一组）    G142.6
	uint8_t EBUFA:1;				//轴控制指令读取信号（第一组）  G142.7
	//G143
	uint8_t ECA:7;					//EC0A~EC6A  轴控制指令信号（第一组）   G143.0~G143.6
	uint8_t EMSBKA:1;				//程序段停止禁止信号（第一组）  G143.7

	//G144~G145
	uint16_t EIFA:16;               //EIF0A~EIF15A  轴控制进给速度信号（第一组）   G144~G145
	//G146~G149
	int32_t EIDA:32;  				//EID0A~EID31A  轴控制数据信号（第一组）    G146~G149
	//G150
	uint8_t :6;
	uint8_t ERT:1;					//手动快速进给选择信号（PMC轴控制）    G150.6
	uint8_t :1;
	//G151
	uint8_t :8;

	//G152
	uint8_t :8;
	//G153
	uint8_t :8;
	//G154
	uint8_t EFINB:1;                //辅助功能结束信号（PMC轴控制，第二组）    G154.0
	uint8_t :1;
	uint8_t EMBUFB:1;				//缓冲禁止信号（PMC轴控制，第二组）    G154.2
	uint8_t ESBKB:1;                //程序段停止信号（PMC轴控制，第二组）    G154.3
	uint8_t ESOFB:1;                //伺服关断信号（PMC轴控制，第二组）    G154.4
	uint8_t ESTPB:1;                //轴控制暂停信号（PMC轴控制，第二组）    G154.5
	uint8_t ECLRB:1;				//复位信号（第二组）    G154.6
	uint8_t EBUFB:1;				//轴控制指令读取信号（第二组）  G154.7
	//G155
//	uint8_t EC0B:1;					//轴控制指令信号（第二组）   G155.0~G155.6
//	uint8_t EC1B:1;
//	uint8_t EC2B:1;
//	uint8_t EC3B:1;
//	uint8_t EC4B:1;
//	uint8_t EC5B:1;
//	uint8_t EC6B:1;
	uint8_t ECB:7;					//EC0B~EC6B  轴控制指令信号（第二组）   G155.0~G155.6
	uint8_t EMSBKB:1;				//程序段停止禁止信号（第二组）  G155.7

	//G156~G157
	uint16_t EIFB:16;               //EIF0B~EIF15B  轴控制进给速度信号（第二组）   G156~G157

	//G158~G161
	int32_t EIDB:32;  				//EID0B~EID31B  轴控制数据信号（第二组）       G158~G161
	//G162
	uint8_t :8;
	//G163
	uint8_t :8;
	//G164
	uint8_t :8;
	//G165
	uint8_t :8;
	//G166
	uint8_t EFINC:1;                //辅助功能结束信号（PMC轴控制，第三组）    G166.0
	uint8_t :1;
	uint8_t EMBUFC:1;				//缓冲禁止信号（PMC轴控制，第三组）    G166.2
	uint8_t ESBKC:1;                //程序段停止信号（PMC轴控制，第三组）    G166.3
	uint8_t ESOFC:1;                //伺服关断信号（PMC轴控制，第三组）    G166.4
	uint8_t ESTPC:1;                //轴控制暂停信号（PMC轴控制，第三组）    G166.5
	uint8_t ECLRC:1;				//复位信号（第三组）    G166.6
	uint8_t EBUFC:1;				//轴控制指令读取信号（第三组）  G166.7
	//G167
//	uint8_t EC0C:1;					//轴控制指令信号（第三组）   G167.0~G167.6
//	uint8_t EC1C:1;
//	uint8_t EC2C:1;
//	uint8_t EC3C:1;
//	uint8_t EC4C:1;
//	uint8_t EC5C:1;
//	uint8_t EC6C:1;
	uint8_t ECC:7;					//EC0C~EC6C  轴控制指令信号（第三组）   G167.0~G167.6
	uint8_t EMSBKC:1;				//程序段停止禁止信号（第三组）  G167.7

	//G168~G169
	uint16_t EIFC:16;               //EIF0C~EIF15C  轴控制进给速度信号（第三组）   G168~G169
	//G170~G173
	int32_t EIDC:32;  				//EID0C~EID31C  轴控制数据信号（第三组）    G170~G173
	//G174
	uint8_t :8;
	//G175
#ifdef USES_WOOD_MACHINE
	uint8_t BDM:1;                 //门板模式，木工专用   G175.0
	uint8_t BOXM:1;                //柜体模式，木工专用   G175.1
	uint8_t :6;
#else
	uint8_t :8;
#endif

	//G176
	uint8_t :8;
	//G177
	uint8_t :8;
	//G178
	uint8_t EFIND:1;                //辅助功能结束信号（PMC轴控制，第四组）    G178.0
	uint8_t :1;
	uint8_t EMBUFD:1;				//缓冲禁止信号（PMC轴控制，第四组）    G178.2
	uint8_t ESBKD:1;                //程序段停止信号（PMC轴控制，第四组）    G178.3
	uint8_t ESOFD:1;                //伺服关断信号（PMC轴控制，第四组）    G178.4
	uint8_t ESTPD:1;                //轴控制暂停信号（PMC轴控制，第四组）    G178.5
	uint8_t ECLRD:1;				//复位信号（第四组）    G178.6
	uint8_t EBUFD:1;				//轴控制指令读取信号（第四组）  G178.7
	//G179
//	uint8_t EC0D:1;					//轴控制指令信号（第四组）   G179.0~G179.6
//	uint8_t EC1D:1;
//	uint8_t EC2D:1;
//	uint8_t EC3D:1;
//	uint8_t EC4D:1;
//	uint8_t EC5D:1;
//	uint8_t EC6D:1;
	uint8_t ECD:7;					//EC0D~EC6D  轴控制指令信号（第四组）   G179.0~G179.6
	uint8_t EMSBKD:1;				//程序段停止禁止信号（第四组）  G179.7

	//G180~G181
	uint16_t EIFD:16;               //EIF0D~EIF15D  轴控制进给速度信号（第四组）   G180~G181
	//G182~G185
	int32_t EIDD:32;  				//EID0D~EID31D  轴控制数据信号（第四组）    G182~G185
	//G186
	uint8_t :8;
	//G187
	uint8_t :8;
	//G188
	uint8_t :8;
	//G189
	uint8_t :8;
	//G190
	uint8_t :8;
	//G191
	uint8_t :8;

	//G192
	uint8_t :8;
	//G193
	uint8_t :8;
	//G194
	uint8_t :8;
	//G195
	uint8_t :8;
	//G196~G197
	uint16_t DEC:16;            //轴回零减速信号，用户轴回参考点
	//G198~G199
	uint16_t REF:16;             //轴回零的原点信号（精基准），用于双基准回零
	//G200~G207
	uint64_t :64;
#ifdef USES_GRIND_MACHINE
	//G208
	uint8_t work_vac_check:1;    //加工位真空检测                  G208.0    左、右机
	uint8_t work_up_check:1;     //加工位气缸上升到位          G208.1   左、右机
	uint8_t work_down_check:1;   //加工位气缸下降到位          G208.2  左、右机
	uint8_t right_work_vac_check:1;  //右加工位真空检测     G208.3  左机
	uint8_t op_right_work_vac:1;    //左机请求操作右机加工位吸真空   G208.4    右机
	uint8_t m66_runover_right:1;  //右侧上下料执行完毕        G208.5   右机
	uint8_t m66_req_right:1;     //右侧请求上下料                  G208.6  左机
	uint8_t new_tray_right:1;    //右侧通知已换新料盘          G208.7  左机
	//G209
	uint8_t left_claw_vac_check:1;    //左爪真空检测           G209.0   左机
	uint8_t left_claw_up_check:1;     //左爪上升到位检测   G209.1  左机
	uint8_t right_claw_vac_check:1;    //右爪真空检测         G209.2  左机
	uint8_t right_claw_up_check:1;     //右爪上升到位检测 G209.3  左机
	uint8_t :3;
	uint8_t right_work_up_check:1;     //右机工位气缸上升到位信号       G209.7   左机
	//G210
	uint8_t correct_pos_x_check:1;      //对位X轴气缸松开到位      G210.0   左机
	uint8_t correct_pos_y_check:1;      //对位Y轴气缸松开到位      G210.1  左机
	uint8_t :1;
	uint8_t change_new_tray_right_cancel:1;   //左机取消通知右机换新料盘   G210.3   //右机
	uint8_t change_new_tray_right:1;    //左机通知右机换新料盘   G210.4    右机
	uint8_t left_tray_inpos:1;			//左料盘在位    G210.5     左机
	uint8_t right_tray_inpos:1;         //右料盘在位    G210.6  左机    //在位为0，不在位为1
	uint8_t safe_door:1;                //安全门打开      G210.7   左机
	//G211
	uint8_t main_chn:1;          //0--主机     1--从机      G211.0   左、右机
	uint8_t :7;
	//G212
	uint8_t :8;
	//G213
	uint8_t :8;
	//G214
	uint8_t :8;
	//G215
	uint8_t :8;
#else
	//G208~G215
	uint64_t :64;
#endif
	//G216
	uint8_t EMPC:1;         //PMC调用宏程序使能    G216.0
#ifdef USES_WOOD_MACHINE
	uint8_t :6;
	uint8_t QDE:1;             //快钻功能(Quick Drill)使能信号   G216.7
#else
	uint8_t :7;
#endif
	//G217~G218
	uint16_t MPCS:16;       //PMC调用宏程序编号     G217~G218
	//G219
	uint8_t CHNC:4;         //当前通道号
	uint8_t :4;
	//G220
	uint8_t MFIN4:1;				//MFIN4信号，第4M功能结束信号  G220.0
	uint8_t MFIN5:1;				//MFIN5信号，第5M功能结束信号  G220.1
	uint8_t MFIN6:1;				//MFIN6信号，第6M功能结束信号  G220.2
	uint8_t MFIN7:1;				//MFIN7信号，第7M功能结束信号  G220.3
	uint8_t MFIN8:1;				//MFIN8信号，第8M功能结束信号  G220.4
	uint8_t MFIN9:1;				//MFIN9信号，第9M功能结束信号  G220.5
	uint8_t MFIN10:1;				//MFIN10信号，第10M功能结束信号  G220.6
	uint8_t MFIN11:1;				//MFIN11信号，第11M功能结束信号  G220.7
	//G221
	uint8_t MFIN12:1;				//MFIN12信号，第12M功能结束信号  G221.0
	uint8_t MFIN13:1;				//MFIN13信号，第13M功能结束信号  G221.1
	uint8_t MFIN14:1;				//MFIN14信号，第14M功能结束信号  G221.2
	uint8_t MFIN15:1;				//MFIN15信号，第15M功能结束信号  G221.3
	uint8_t MFIN16:1;				//MFIN16信号，第16M功能结束信号  G221.4
	uint8_t :3;
	//G222
	uint8_t TFIN2:1;                 //T功能结束信号2    G222.0
	uint8_t TFIN3:1;                 //T功能结束信号3    G222.1
	uint8_t TFIN4:1;                 //T功能结束信号4    G222.2
	uint8_t TFIN5:1;                 //T功能结束信号5    G222.3
	uint8_t TFIN6:1;                 //T功能结束信号6    G222.4
	uint8_t TFIN7:1;                 //T功能结束信号7    G222.5
	uint8_t TFIN8:1;                 //T功能结束信号8    G222.6
	uint8_t TFIN9:1;                 //T功能结束信号9    G222.7
	//G223
	uint8_t TFIN10:1;                 //T功能结束信号10    G222.0
	uint8_t TFIN11:1;                 //T功能结束信号11    G222.1
	uint8_t TFIN12:1;                 //T功能结束信号12    G222.2
	uint8_t TFIN13:1;                 //T功能结束信号13    G222.3
	uint8_t TFIN14:1;                 //T功能结束信号14    G222.4
	uint8_t TFIN15:1;                 //T功能结束信号15    G222.5
	uint8_t TFIN16:1;                 //T功能结束信号16    G222.6
	uint8_t :1;
	//G224
	uint8_t MEXC1:1;                  //MEXC1信号，第1组M指令的执行中信号   G224.0
	uint8_t MEXC2:1;                  //MEXC2信号，第2组M指令的执行中信号   G224.1
	uint8_t MEXC3:1;                  //MEXC3信号，第3组M指令的执行中信号   G224.2
	uint8_t MEXC4:1;                  //MEXC4信号，第4组M指令的执行中信号   G224.3
	uint8_t MEXC5:1;                  //MEXC5信号，第5组M指令的执行中信号   G224.4
	uint8_t MEXC6:1;                  //MEXC6信号，第6组M指令的执行中信号   G224.5
	uint8_t MEXC7:1;                  //MEXC7信号，第7组M指令的执行中信号   G224.6
	uint8_t MEXC8:1;                  //MEXC8信号，第8组M指令的执行中信号   G224.7
	//G225
	uint8_t MEXC9:1;                  //MEXC9信号，第9组M指令的执行中信号   G225.0
	uint8_t MEXC10:1;                  //MEXC10信号，第10组M指令的执行中信号   G225.1
	uint8_t MEXC11:1;                  //MEXC11信号，第11组M指令的执行中信号   G225.2
	uint8_t MEXC12:1;                  //MEXC12信号，第12组M指令的执行中信号   G225.3
	uint8_t MEXC13:1;                  //MEXC13信号，第13组M指令的执行中信号   G225.4
	uint8_t MEXC14:1;                  //MEXC14信号，第14组M指令的执行中信号   G225.5
	uint8_t MEXC15:1;                  //MEXC15信号，第15组M指令的执行中信号   G225.6
	uint8_t MEXC16:1;                  //MEXC16信号，第16组M指令的执行中信号   G225.7
	//G226
	uint8_t :8;
	//G227
	uint8_t :8;
	//G228
	uint8_t :8;
	//G229
	uint8_t :8;
	//G230
	uint8_t :8;
	//G231
	uint8_t :8;
	//G232~G239
	uint64_t :64;
	//G240~G247
	uint64_t :64;
	//G248~G255
	uint64_t :64;
};
#pragma pack()

//F地址寄存器定义
union FRegister{
	uint8_t all[F_REG_COUNT];
	FRegBits bits[kMaxChnCount];
};


//G地址寄存器定义
union GRegister{
	uint8_t all[G_REG_COUNT];
	GRegBits bits[kMaxChnCount];
};



//寄存器类型定义
enum PmcRegSection{
	PMC_REG_X = 0,		//X寄存器
	PMC_REG_Y,			//Y寄存器
	PMC_REG_F,			//F寄存器
	PMC_REG_G,			//G寄存器
	PMC_REG_K,			//K寄存器
	PMC_REG_R,			//R寄存器
	PMC_REG_A,			//A寄存器
	PMC_REG_D,			//D寄存器
	PMC_REG_C,			//C寄存器
	PMC_REG_T,			//T寄存器
#ifndef USES_PMC_2_0
	PMC_REG_DC,			//DC寄存器
	PMC_REG_DT			//DT寄存器
#else
	PMC_REG_E           //E寄存器
#endif
};

enum PmcKeepMaskBit{
	K_REG_BIT = 0,      //K寄存器
	D_REG_BIT,			//D寄存器
	DC_REG_BIT,			//DC寄存器
	DT_REG_BIT,			//DT寄存器


	KEEP_REG_COUNT     //保持型寄存器数量
};


/**
 * @brief PMC寄存器类定义，包含PMC模块的所有寄存器
 */
class PmcRegister {
public:
	PmcRegister();
	virtual ~PmcRegister();

	bool SetRegValue(PmcRegSection sec, uint16_t index, uint8_t value);	//设置寄存器值
	bool SetRegValue(PmcRegSection sec, uint16_t index, uint16_t value);	//设置寄存器值
	bool SetRegBitValue(PmcRegSection sec, uint16_t index, uint8_t bit, uint8_t count, uint32_t value);   //设置寄存器bit值
	bool GetRegValue(PmcRegSection sec, uint16_t index, uint8_t &value);	//获取寄存器值
	bool GetRegValue(PmcRegSection sec, uint16_t index, uint16_t &value);	//获取寄存器值
	bool GetRegValueMulti(PmcRegSection sec, uint16_t index, uint16_t count, uint8_t *value);   //获取多个寄存器值
	bool GetRegValueMulti(PmcRegSection sec, uint16_t index, uint16_t count, uint16_t *value);   //获取多个寄存器值

	bool GetRegValue(PmcRegSection sec, uint16_t index, int32_t &value);   //获取四字节寄存器值
//	bool IsKeepRegChanged(){return this->m_mask_save_keep==0x00?false:true;}  //保持型寄存器是否改变


	FRegister &FReg(){return m_f_reg;}		//返回F寄存器
	const GRegister &GReg(){return m_g_reg;}		//返回G寄存器
	uint8_t *GetRegPtr8(PmcRegSection sec);	//返回寄存器段指针
#ifndef USES_PMC_2_0
	uint16_t *GetRegPtr16(PmcRegSection sec);	//返回寄存器段指针
#endif

	void SaveRegData();   //保存非易失性数据


private:
	void Initialize();	//初始化
	void InitRegFile();    //初始化寄存器数据文件


private:
	uint8_t m_x_reg[X_REG_COUNT];		//输入信号寄存器
#ifdef USES_PMC_2_0
	uint8_t m_reserved_1[7];            //为了解决字节对齐问题占位
#endif
	uint8_t m_y_reg[Y_REG_COUNT];		//输出信号寄存器
#ifdef USES_PMC_2_0
	uint8_t m_reserved_2[4];            //为了解决字节对齐问题占位
#endif
	FRegister m_f_reg;					//NC->PMC， F地址
	GRegister m_g_reg;			    	//PMC->NC， G地址
	uint8_t m_a_reg[A_REG_COUNT];		//告警寄存器
#ifdef USES_PMC_2_0
	uint8_t m_reserved_3[7];            //为了解决字节对齐问题占位
#endif
	uint8_t m_r_reg[R_REG_COUNT];		//内部寄存器
	uint8_t m_k_reg[K_REG_COUNT];		//非易失寄存器
#ifdef USES_PMC_2_0
	uint8_t m_reserved_4[4];            //为了解决字节对齐问题占位
#endif
#ifndef USES_PMC_2_0
	uint16_t m_d_reg[D_REG_COUNT];		//数据寄存器，非易失
	uint16_t m_t_reg[T_REG_COUNT];		//计时器
	uint16_t m_c_reg[C_REG_COUNT];		//计数器
	uint16_t m_dt_reg[T_REG_COUNT];		//计时器预置值寄存器，非易失
	uint16_t m_dc_reg[C_REG_COUNT];		//计数器预置值寄存器，非易失
#else
	uint8_t m_d_reg[D_REG_COUNT];		//数据寄存器，非易失
	uint8_t m_t_reg[T_REG_COUNT*2];		//计时器
	uint8_t m_c_reg[C_REG_COUNT*4];		//计数器
	uint8_t m_e_reg[E_REG_COUNT];       //扩展寄存器
#endif


	MICommunication *m_p_mi_comm;	//MI通讯接口
//	uint8_t m_mask_save_keep;     //保存保持型寄存器  bit0-bit3依次表示K/D/DT/DC寄存器需要保存到文件
	int m_n_fp;         //寄存器文件打开句柄
};


#endif /* PMCREGISTER_H_ */
