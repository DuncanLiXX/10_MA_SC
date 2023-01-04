/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2020/10/29
 *@brief ��ͷ�ļ�����PMC�мĴ��������������
 *@version
 */
#ifndef PMCREGISTER_H_
#define PMCREGISTER_H_

#include <stdint.h>

#include "global_definition.h"
#include "hmi_shared_data.h"

class MICommunication;

#pragma pack(1)

//��ͨ��F�Ĵ�����λӳ�䶨��, ��ͨ��256�ֽ�
struct FRegBits{
	//F0
	uint8_t :4;
	uint8_t SPL:1;						//ѭ����ͣ���ź�   F0.4
	uint8_t STL:1;						//ѭ���������ź�   F0.5
	uint8_t SA:1;						//�ŷ������ź�   F0.6
	uint8_t OP:1;						//�Զ������ź�   F0.7
	//F1
	uint8_t AL:1;						//�澯�ź�      F1.0
	uint8_t RST:1;						//��λ�ź�		F1.1
	uint8_t BAL:1;						//��ظ澯�ź�    F1.2
	uint8_t DEN:1;						//��������ź�   F1.3
	uint8_t ENB:1;                      //����ʹ���ź�   F1.4
	uint8_t TAP:1;                      //��˿�ź�       F1.5
	uint8_t :1;
	uint8_t MA:1;						//CNC�����ź�	F1.7
	//F2
	uint8_t :8;
	//F3
	uint8_t MINC:1;					//����ѡ�����ź�   F3.0
	uint8_t MH:1;					//���ֽ����ź�   F3.1
	uint8_t MJ:1;					//�ֶ�JOGѡ�����ź�   F3.2
	uint8_t MMDI:1;					//MDIģʽ�ź�    F3.3
	uint8_t MREF:1;                 //ԭ��ģʽ     F3.4
	uint8_t MMEM:1;                 //�Զ�����ģʽ�ź�    F3.5
	uint8_t MEDT:1;                 //�洢���༭ѡ�����ź�   F3.6
	uint8_t :1;
	//F4
	uint8_t MBDT1:1;				//����ѡ�����ź�   F4.0
    uint8_t MMLK:1;                 //�����������ס����ź� F4.1
    uint8_t :1;
	uint8_t MSBK:1;					//����ѡ�����ź�   F4.3
	uint8_t :4;
	//F5
	uint8_t :8;
	//F6
	uint8_t :8;
	//F7
	uint8_t MF:1;						//��������ѡͨ�ź� F7.0
	uint8_t EFD:1;                      //���ٽӿڵ��ⲿ�����ź�   F7.1
	uint8_t SF:1;						//�����ٶȹ���ѡͨ�ź� F7.2
	uint8_t TF:1;						//���߹���ѡͨ�ź� F7.3
	uint8_t :4;
	//F8
	uint8_t :4;
	uint8_t MF2:1;						//��2M����ѡͨ�ź� F8.4
	uint8_t MF3:1;						//��3M����ѡͨ�ź� F8.5
	uint8_t :2;
	//F9
	uint8_t :2;
	uint8_t DM98:1;						//M98�����ź�   F9.2
	uint8_t DM99:1;                     //M99�����ź�   F9.3
	uint8_t DM30:1;						//M30�����ź�   F9.4
	uint8_t DM02:1;						//M02�����ź�   F9.5
	uint8_t DM01:1;						//M01�����ź�   F9.6
	uint8_t DM00:1;						//M00�����ź�   F9.7
	//F10
	uint8_t mcode_0:8;					//�������ܴ����źţ�1��
	//F11
	uint8_t mcode_1:8;					//�������ܴ����źţ�1��
	//F12
	uint8_t mcode_2:8;					//�������ܴ����źţ�1��
	//F13
	uint8_t mcode_3:8;					//�������ܴ����źţ�1��
	//F14
	uint8_t mcode_20:8;					//�������ܴ����źţ�2��
	//F15
	uint8_t mcode_21:8;					//�������ܴ����źţ�2��
	//F16
	uint8_t mcode_30:8;					//�������ܴ����źţ�3��
	//F17
	uint8_t mcode_31:8;					//�������ܴ����źţ�3��
	//F18
	uint8_t :8;
	//F19
	uint8_t :8;
	//F20
	uint8_t :8;
	//F21
	uint8_t :8;
	//F22
	uint8_t scode_0:8;					//�����ٶȴ����ź�
	//F23
	uint8_t scode_1:8;					//�����ٶȴ����ź�
	//F24
	uint8_t scode_2:8;					//�����ٶȴ����ź�
	//F25
	uint8_t scode_3:8;					//�����ٶȴ����ź�
	//F26
	uint8_t tcode_0:8;					//���߹��ܴ����źţ�1��
	//F27
	uint8_t tcode_1:8;					//���߹��ܴ����źţ�1��
	//F28
	uint8_t tcode_2:8;					//���߹��ܴ����źţ�1��
	//F29
	uint8_t tcode_3:8;					//���߹��ܴ����źţ�1��
	//F30
	uint8_t :8;
	//F31
	uint8_t :8;
	//F32
	uint8_t :8;
	//F33
	uint8_t :8;
	//F34
    uint8_t GR1O:1;                      //��λ1ѡ���ź�
    uint8_t GR2O:1;                      //��λ2ѡ���ź�
    uint8_t GR3O:1;                      //��λ3ѡ���ź�
    uint8_t :5;
	//F35
	uint8_t :8;
    //F36~F37
    uint16_t RO:16;                      //������ת�����
	//F38
	uint8_t :6;
	uint8_t SPS:2;                       //������ת״̬  F38.6~F38.7  0--ͣת   1--��ת  2--��ת
	//F39
	uint8_t :8;
	//F40
	uint8_t :8;
	//F41
    uint8_t CLMLK:1;                     //��е��ס�ָ��� F41.0
    uint8_t :7;
	//F42
	uint8_t :8;
	//F43
	uint8_t :8;
	//F44
	uint8_t :8;
	//F45
    uint8_t :1;
    uint8_t SST:1;     // �����ź� F45.1
    uint8_t :1;
    uint8_t SAR:1;     // �ٶȵ����ź� F45.3
    uint8_t :3;
    uint8_t ORAR:1;    // ��������ź� F45.7
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
	uint8_t EREND:1;                //�ⲿ���������ȡ�����ź�   F60.0
	uint8_t :7;
	//F61
    uint8_t :8;
	//F62
	uint8_t :8;
	//F63
	uint8_t :8;
	//F64
	uint8_t :7;
	uint8_t TAM:1;               //�Զ��Ե��ź�      F64.7
	//F65
    uint8_t RGSPP:1;            //���빥˿״̬���� F65.0
    uint8_t :1;
    uint8_t RGMP:1;             //����ģʽ�л� F65.2
    uint8_t :5;
	//F66
    uint8_t :1;
    uint8_t RTPT:1;         //���Թ�˿���˽����ź� F66.1
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
    uint8_t :1;     // �Զ��Ե��ź� F83.1
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
	uint8_t ZP11:8;   //���زο�������ź�   bit0-bit7��  ��1-��8
	//F95
	uint8_t ZP12:8;   //���زο�������ź�   bit0-bit7��  ��9-��16
	//F96
	uint8_t ZP21:8;    //���صڶ��ο�������ź�   bit0-bit7��  ��1-��8
	//F97
	uint8_t ZP22:8;   //���صڶ��ο�������ź�   bit0-bit7��  ��9-��16
	//F98
	uint8_t ZP31:8;    //���ص����ο�������ź�   bit0-bit7��  ��1-��8
	//F99
	uint8_t ZP32:8;    //���ص����ο�������ź�   bit0-bit7��  ��9-��16
	//F100
	uint8_t ZP41:8;   //���ص��Ĳο�������ź�   bit0-bit7��  ��1-��8
	//F101
	uint8_t ZP42:8;    //���ص��Ĳο�������ź�   bit0-bit7��  ��9-��16
	//F102
	//F103
	uint16_t MV:16;     //���ƶ��ź�   F102~F103         bit0~bit15����1~��16
	//F104
	//F105
	uint16_t INP:16;    //�ᵽλ�ź�     F104~F105       bit0~bit15����1~��16
	//F106
	//F107
	uint16_t MVD:16;    //���˶������ź�    F106~F107    bit0~bit15����1~��16
	//F108
	uint8_t :8;
	//F109
	uint8_t :8;
	//F110
	uint8_t :8;
	//F111
	uint8_t :8;
	//F112
	uint8_t EADEN1:1;							//��������ź�1(PMC����ƣ�1��)     F112.0
	uint8_t EADEN2:1;                           //��������ź�2(PMC����ƣ�2��)     F112.1
	uint8_t EADEN3:1;							//��������ź�3(PMC����ƣ�3��)     F112.2
	uint8_t EADEN4:1;							//��������ź�4(PMC����ƣ�4��)     F112.3
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
	uint16_t IRF:16;   //����������ź�   bit0~bit16:  ��1~��16
	//F120
	uint8_t ZRF1:8;    //�ο��㽨���ź�   bit0-bit7��  ��1-��8
	//F121
	uint8_t ZRF2:8;    //�ο��㽨���ź�   bit0-bit7��  ��9-��16
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
	uint8_t EOV0:1;                 //����0%�ź�(PMC�����)     F129.5
	uint8_t :1;
	uint8_t _EAXSL:1; 				//������ѡ��״̬�źţ�PMC����ƣ�   F129.7    NC��PMC���˶�ָ��������ݻ���CNC����ʱΪ1��������ʱΪ0
	//F130
	uint8_t EINPA:1;				//��λ�źţ���һ��(PMC�����)      F130.0
	uint8_t ECKZA:1;				//�����������źţ���һ��(PMC�����)     F130.1
	uint8_t EIALA:1;				//�����źţ���һ��(PMC�����)     F130.2
	uint8_t EDENA:1;				//��������ִ���źţ���һ��(PMC�����)     F130.3
	uint8_t EGENA:1;				//���ƶ��źţ���һ��(PMC�����)     F130.4
	uint8_t EOTPA:1;				//���򳬳��źţ���һ��(PMC�����)     F130.5
	uint8_t EOTNA:1;				//���򳬳��źţ���һ��(PMC�����)     F130.6
	uint8_t EBSYA:1;  				//�����ָ���ȡ����źţ���һ�飨PMC����ƣ�  F130.7
	//F131
	uint8_t EMFA:1;					//��������ѡͨ�źţ���һ�飨PMC����ƣ�F131.0
	uint8_t EABUFA:1;				//PMC��������ݻ��������źţ���һ�飨PMC����ƣ�F131.1
	uint8_t EMF2A:1;				//��������2ѡͨ�źţ���һ�飨PMC����ƣ�F131.2
	uint8_t EMF3A:1;				//��������3ѡͨ�źţ���һ�飨PMC����ƣ�F131.3
	uint8_t :4;
	//F132
	uint8_t :8;
	//F133
	uint8_t EINPB:1;				//��λ�źţ��ڶ���(PMC�����)      F133.0
	uint8_t ECKZB:1;				//�����������źţ��ڶ���(PMC�����)     F133.1
	uint8_t EIALB:1;				//�����źţ��ڶ���(PMC�����)     F133.2
	uint8_t EDENB:1;				//��������ִ���źţ��ڶ���(PMC�����)     F133.3
	uint8_t EGENB:1;				//���ƶ��źţ��ڶ���(PMC�����)     F133.4
	uint8_t EOTPB:1;				//���򳬳��źţ��ڶ���(PMC�����)     F133.5
	uint8_t EOTNB:1;				//���򳬳��źţ��ڶ���(PMC�����)     F133.6
	uint8_t EBSYB:1;  				//�����ָ���ȡ����źţ��ڶ��飨PMC����ƣ�  F133.7
	//F134
	uint8_t EMFB:1;					//��������ѡͨ�źţ��ڶ��飨PMC����ƣ�F134.0
	uint8_t EABUFB:1;				//PMC��������ݻ��������źţ��ڶ��飨PMC����ƣ�F134.1
	uint8_t EMF2B:1;				//��������2ѡͨ�źţ��ڶ��飨PMC����ƣ�F134.2
	uint8_t EMF3B:1;				//��������3ѡͨ�źţ��ڶ��飨PMC����ƣ�F134.3
	uint8_t :4;
	//F135
	uint8_t :8;

	//F136
	uint8_t EINPC:1;				//��λ�źţ�������(PMC�����)      F136.0
	uint8_t ECKZC:1;				//�����������źţ�������(PMC�����)     F136.1
	uint8_t EIALC:1;				//�����źţ�������(PMC�����)     F136.2
	uint8_t EDENC:1;				//��������ִ���źţ�������(PMC�����)     F136.3
	uint8_t EGENC:1;				//���ƶ��źţ�������(PMC�����)     F136.4
	uint8_t EOTPC:1;				//���򳬳��źţ�������(PMC�����)     F136.5
	uint8_t EOTNC:1;				//���򳬳��źţ�������(PMC�����)     F136.6
	uint8_t EBSYC:1;  				//�����ָ���ȡ����źţ������飨PMC����ƣ�  F136.7
	//F137
	uint8_t EMFC:1;					//��������ѡͨ�źţ������飨PMC����ƣ�F137.0
	uint8_t EABUFC:1;				//PMC��������ݻ��������źţ������飨PMC����ƣ�F137.1
	uint8_t EMF2C:1;				//��������2ѡͨ�źţ������飨PMC����ƣ�F137.2
	uint8_t EMF3C:1;				//��������3ѡͨ�źţ������飨PMC����ƣ�F137.3
	uint8_t :4;
	//F138
	uint8_t :8;
	//F139
	uint8_t EINPD:1;				//��λ�źţ�������(PMC�����)      F139.0
	uint8_t ECKZD:1;				//�����������źţ�������(PMC�����)     F139.1
	uint8_t EIALD:1;				//�����źţ�������(PMC�����)     F139.2
	uint8_t EDEND:1;				//��������ִ���źţ�������(PMC�����)     F139.3
	uint8_t EGEND:1;				//���ƶ��źţ�������(PMC�����)     F139.4
	uint8_t EOTPD:1;				//���򳬳��źţ�������(PMC�����)     F139.5
	uint8_t EOTND:1;				//���򳬳��źţ�������(PMC�����)     F139.6
	uint8_t EBSYD:1;  				//�����ָ���ȡ����źţ������飨PMC����ƣ�  F139.7
	//F140
	uint8_t EMFD:1;					//��������ѡͨ�źţ������飨PMC����ƣ�F140.0
	uint8_t EABUFD:1;				//PMC��������ݻ��������źţ������飨PMC����ƣ�F140.1
	uint8_t EMF2D:1;				//��������2ѡͨ�źţ������飨PMC����ƣ�F140.2
	uint8_t EMF3D:1;				//��������3ѡͨ�źţ������飨PMC����ƣ�F140.3
	uint8_t :4;
	//F141
	uint8_t :8;
	//F142
	uint8_t :8;
	//F143
	uint8_t :8;

	//F144
	uint8_t mcode_40:8;					//�������ܴ����źţ�4��
	//F145
	uint8_t mcode_41:8;					//�������ܴ����źţ�4��
	//F146
	uint8_t mcode_50:8;					//�������ܴ����źţ�5��
	//F147
	uint8_t mcode_51:8;					//�������ܴ����źţ�5��
	//F148
	uint8_t mcode_60:8;					//�������ܴ����źţ�6��
	//F149
	uint8_t mcode_61:8;					//�������ܴ����źţ�6��
	//F150
	uint8_t mcode_70:8;					//�������ܴ����źţ�7��
	//F151
	uint8_t mcode_71:8;					//�������ܴ����źţ�7��
	//F152
	uint8_t mcode_80:8;					//�������ܴ����źţ�8��
	//F153
	uint8_t mcode_81:8;					//�������ܴ����źţ�8��
	//F154
	uint8_t mcode_90:8;					//�������ܴ����źţ�9��
	//F155
	uint8_t mcode_91:8;					//�������ܴ����źţ�9��
	//F156
	uint8_t mcode_100:8;					//�������ܴ����źţ�10��
	//F157
	uint8_t mcode_101:8;					//�������ܴ����źţ�10��
	//F158
	uint8_t mcode_110:8;					//�������ܴ����źţ�11��
	//F159
	uint8_t mcode_111:8;					//�������ܴ����źţ�11��
	//F160
	uint8_t mcode_120:8;					//�������ܴ����źţ�12��
	//F161
	uint8_t mcode_121:8;					//�������ܴ����źţ�12��
	//F162
	uint8_t mcode_130:8;					//�������ܴ����źţ�13��
	//F163
	uint8_t mcode_131:8;					//�������ܴ����źţ�13��
	//F164
	uint8_t mcode_140:8;					//�������ܴ����źţ�14��
	//F165
	uint8_t mcode_141:8;					//�������ܴ����źţ�14��
	//F166
	uint8_t mcode_150:8;					//�������ܴ����źţ�15��
	//F167
	uint8_t mcode_151:8;					//�������ܴ����źţ�15��
	//F168
	uint8_t mcode_160:8;					//�������ܴ����źţ�16��
	//F169
	uint8_t mcode_161:8;					//�������ܴ����źţ�16��
	//F170
	uint8_t MF4:1;                          //��4M����ѡͨ�ź� F170.0
	uint8_t MF5:1;                          //��5M����ѡͨ�ź� F170.1
	uint8_t MF6:1;                          //��6M����ѡͨ�ź� F170.2
	uint8_t MF7:1;                          //��7M����ѡͨ�ź� F170.3
	uint8_t MF8:1;                          //��8M����ѡͨ�ź� F170.4
	uint8_t MF9:1;                          //��9M����ѡͨ�ź� F170.5
	uint8_t MF10:1;                          //��10M����ѡͨ�ź� F170.6
	uint8_t MF11:1;                          //��11M����ѡͨ�ź� F170.7
	//F171
	uint8_t MF12:1;                          //��12M����ѡͨ�ź� F171.0
	uint8_t MF13:1;                          //��13M����ѡͨ�ź� F171.1
	uint8_t MF14:1;                          //��14M����ѡͨ�ź� F171.2
	uint8_t MF15:1;                          //��15M����ѡͨ�ź� F171.3
	uint8_t MF16:1;                          //��16M����ѡͨ�ź� F171.4
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
	uint8_t PPI:4;                         //��ǰ���ղ������
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
	uint8_t EACNT1:1;					//�����źţ���һ�飨PMC����ƣ�   F182.0
	uint8_t EACNT2:1;					//�����źţ��ڶ��飨PMC����ƣ�   F182.1
	uint8_t EACNT3:1;					//�����źţ������飨PMC����ƣ�   F182.2
	uint8_t EACNT4:1;					//�����źţ������飨PMC����ƣ�   F182.3
	uint8_t :4;
	//F183
	uint8_t :8;
	//F184
	uint8_t :8;
    //F185-F186
    int16_t SRS:16;                     //�������õĶ���Ƕ�
    //F187-F188
    int16_t SRG:16;                     //���ᵱǰ�Ƕ�
	//F189
	uint8_t :8;
	//F190~F193
	int32_t wnd_data:32;         //���ڷ�������     F190~F193   ³����Ŀ��ʱʹ��
	//F194
	uint8_t :8;
	//F195
    uint8_t MERS:1; // MDI��λ���� F195.0
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
	uint64_t in_ref_point:64;    //�زο����źţ���ʱ����
#ifdef USES_GRIND_MACHINE
	//F208
	uint8_t change_tray_left:1;       //��������������   F208.0   ���
	uint8_t change_tray_right:1;       //�Ҳ������������  F208.1 ���
	uint8_t :3;
	uint8_t m66_runover_right:1;      //֪ͨ�Ҳ����������  F208.5   ���
	uint8_t m66_req_right:1;          //�Ҳ�����������   F208.6   �һ�
	uint8_t new_tray_right:1;         //�Ҳ�֪ͨ�ѻ������� F208.7   �һ�
	//F209
	uint8_t work_vac:1;          //�ӹ�λ��շ�     F209.0   ��/�һ�
	uint8_t work_hold:1;         //�ӹ�λѹ������   F209.1   ��/�һ�
	uint8_t :5;
	uint8_t work_vac_right:1;    //֪ͨ�һ������    F209.7   ���
	//F210
	uint8_t left_claw_vac:1;     //��צ��շ�    F210.0    ���
	uint8_t left_claw_down:1;    //��צ��            F210.1   ���
	uint8_t right_claw_vac:1;    //��צ��շ�    F210.2   ���
	uint8_t right_claw_down:1;   //��צ��            F210.3  ���
	uint8_t :4;
	//F211
	uint8_t correct_pos_x:1;     //��λƽ̨X��н�   F211.0    ���
	uint8_t correct_pos_y:1;     //��λƽ̨Y��н�   F211.1  ���
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
	uint8_t MPCO:1;           //PMC�������ý���     F216.0
	uint8_t :7;
	//F217
	uint8_t :8;
	//F218
	uint8_t :8;
	//F219
	uint8_t CHNC:4;           //��ǰͨ����
	uint8_t : 4;
	//F220
	uint8_t TF2:1;           //���߹���ѡͨ�ź�2 F220.0
	uint8_t TF3:1;           //���߹���ѡͨ�ź�3 F220.1
	uint8_t TF4:1;           //���߹���ѡͨ�ź�4 F220.2
	uint8_t TF5:1;           //���߹���ѡͨ�ź�5 F220.3
	uint8_t TF6:1;           //���߹���ѡͨ�ź�6 F220.4
	uint8_t TF7:1;           //���߹���ѡͨ�ź�7 F220.5
	uint8_t TF8:1;           //���߹���ѡͨ�ź�8 F220.6
	uint8_t TF9:1;           //���߹���ѡͨ�ź�9 F220.7
	//F221
	uint8_t TF10:1;           //���߹���ѡͨ�ź�10 F221.0
	uint8_t TF11:1;           //���߹���ѡͨ�ź�11 F221.1
	uint8_t TF12:1;           //���߹���ѡͨ�ź�12 F221.2
	uint8_t TF13:1;           //���߹���ѡͨ�ź�13 F221.3
	uint8_t TF14:1;           //���߹���ѡͨ�ź�14 F221.4
	uint8_t TF15:1;           //���߹���ѡͨ�ź�15 F221.5
	uint8_t TF16:1;           //���߹���ѡͨ�ź�16 F221.6
	uint8_t :1;
	//F222
	uint8_t :8;
	//F223
	uint8_t :8;
	//F224
	uint8_t tcode_20:8;            //���߹��ܴ����źţ�2��
	//F225
	uint8_t tcode_21:8;            //���߹��ܴ����źţ�2��
	//F226
	uint8_t tcode_30:8;            //���߹��ܴ����źţ�3��
	//F227
	uint8_t tcode_31:8;            //���߹��ܴ����źţ�3��
	//F228
	uint8_t tcode_40:8;            //���߹��ܴ����źţ�4��
	//F229
	uint8_t tcode_41:8;            //���߹��ܴ����źţ�4��
	//F230
	uint8_t tcode_50:8;            //���߹��ܴ����źţ�5��
	//F231
	uint8_t tcode_51:8;            //���߹��ܴ����źţ�5��
	//F232
	uint8_t tcode_60:8;            //���߹��ܴ����źţ�6��
	//F233
	uint8_t tcode_61:8;            //���߹��ܴ����źţ�6��
	//F234
	uint8_t tcode_70:8;            //���߹��ܴ����źţ�7��
	//F235
	uint8_t tcode_71:8;            //���߹��ܴ����źţ�7��
	//F236
	uint8_t tcode_80:8;            //���߹��ܴ����źţ�8��
	//F237
	uint8_t tcode_81:8;            //���߹��ܴ����źţ�8��
	//F238
	uint8_t tcode_90:8;            //���߹��ܴ����źţ�9��
	//F239
	uint8_t tcode_91:8;            //���߹��ܴ����źţ�9��
	//F240
	uint8_t tcode_100:8;            //���߹��ܴ����źţ�10��
	//F241
	uint8_t tcode_101:8;            //���߹��ܴ����źţ�10��
	//F242
	uint8_t tcode_110:8;            //���߹��ܴ����źţ�11��
	//F243
	uint8_t tcode_111:8;            //���߹��ܴ����źţ�11��
	//F244
	uint8_t tcode_120:8;            //���߹��ܴ����źţ�12��
	//F245
	uint8_t tcode_121:8;            //���߹��ܴ����źţ�12��
	//F246
	uint8_t tcode_130:8;            //���߹��ܴ����źţ�13��
	//F247
	uint8_t tcode_131:8;            //���߹��ܴ����źţ�13��
	//F248
	uint8_t tcode_140:8;            //���߹��ܴ����źţ�14��
	//F249
	uint8_t tcode_141:8;            //���߹��ܴ����źţ�14��
	//F250
	uint8_t tcode_150:8;            //���߹��ܴ����źţ�15��
	//F251
	uint8_t tcode_151:8;            //���߹��ܴ����źţ�15��
	//F252
	uint8_t tcode_160:8;            //���߹��ܴ����źţ�16��
	//F253
	uint8_t tcode_161:8;            //���߹��ܴ����źţ�16��
	//F254
	uint8_t :8;
	//F255
	uint8_t :8;

};

//��ͨ��G�Ĵ�����λӳ�䶨��, ��ͨ��256�ֽ�
struct GRegBits{
	//G0
	uint8_t ED1:8;					//�ⲿ���������������ź�1  G0.0~G0.7
	//G1
	uint8_t ED2:8;					//�ⲿ���������������ź�2  G1.0~G1.7
	//G2
	uint8_t EA:7;					//�ⲿ���������õ�ַ�ź�  G2.0~G2.6
	uint8_t ESTB:1;                 //�ⲿ�������ݶ�ȡ�ź�   G2.7
	//G3
	uint8_t :8;
	//G4
	uint8_t :3;
	uint8_t FIN:1;					//FIN�źţ������ź�  G4.3
	uint8_t MFIN2:1;				//MFIN2�źţ���2M���ܽ����ź�  G4.4
	uint8_t MFIN3:1;				//MFIN3�źţ���3M���ܽ����ź�  G4.5
	uint8_t :2;
	//G5
	uint8_t MFIN:1;                 //�������ܽ����ź�   G5.0
	uint8_t EFIN:1;                 //�ⲿ���й��ܽ����ź�   G5.1
	uint8_t SFIN:1;                 //���Ṧ�ܽ����ź�    G5.2
	uint8_t TFIN:1;                 //���߹��ܽ����ź�    G5.3
	uint8_t :4;
	//G6
	uint8_t :6;
	uint8_t SKIPP:1;			    //SKIPP�ź�  G6.6  ����G31��ת
	uint8_t :1;
	//G7
	uint8_t :1;
	uint8_t STLK:1;					//������ס�ź�   G7.1
	uint8_t ST:1;					//ѭ�������ź�   G7.2
    uint8_t :3;
    uint8_t EXLM:1;                     //����λѡ���ź� 0:����λ1  1:����λ2
    uint8_t RLSOT:1;                     //�������λ�����ź�
	//G8
	uint8_t :4;
	uint8_t _ESP:1;					//��ͣ�ź�  G8.4
	uint8_t _SP:1;					//������ͣ�ź�    G8.5
	uint8_t RRW:1;					//��λ�͵����ź�  G8.6
	uint8_t ERS:1;                  //�ⲿ��λ�ź�    G8.7
	//G9
	uint8_t :8;
	//G10
	//G11
	uint16_t _JV:16;               //�ֶ��ƶ��ٶȱ����ź�   *JV0~*JV15   ����Ч����λ��0.01%
	//G12
	uint8_t _FV:8;					//�����ٶȱ����ź�    *FV0~*FV7   ����Ч����λ��1%
	//G13
	uint8_t :8;
	//G14
	uint8_t ROV:8;                 //G00���ٱ����ź�  G14.0~G14.7  ROV1~ROV7   ��λ��1%
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
	uint8_t MP:2;				//�ֶ������ź� MP1~MP2  G19.4~G19.5
	uint8_t :1;
	uint8_t RT:1;				//�ֶ����ٽ���ѡ���ź�  G19.7
	//G20
#ifdef USES_WUXI_BLOOD_CHECK
	uint8_t ret_home:1;       //�زο����ź�   Ϊ������Ŀ����  G20.0
	uint8_t reset:1;		//��λ�ź�  G20.1
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
    uint8_t GTC:8;     //��ȡ��ǰ���� G26.8
	//G27
	uint8_t :8;
	//G28
	uint8_t :1;
	uint8_t GR1:1;      //����ѡ���źţ����룩     G28.1
	uint8_t GR2:1;      //����ѡ���źţ����룩     G28.2
	uint8_t :1;
	uint8_t _SUCPF:1;   //�����ɿ�����ź�    ����Ч     G28.4
	uint8_t _SCPF:1;    //����н�����ź�    ����Ч     G28.5
	uint8_t SPSTP:1;    //����ֹͣ����ź�         G28.6
    uint8_t :1;
	//G29
	uint8_t GR21:1;     //����ѡ���źţ����룩     G29.0
    uint8_t :4;
    uint8_t SOR:1;      //����׼ͣ�ź�           G29.5
	uint8_t _SSTP:1;    //����ͣ�ź�             G29.6
	uint8_t :1;
	//G30
	uint8_t SOV:8;      //���ᱶ���ź�    SOV0~SOV7     G30.0~G30.7     ��λ��1%
    //G31~G32
    uint16_t RI:16;     //����ת������ R01I~R16I
	//G33
    uint8_t :5;
    uint8_t SGN:1;      //PMC��������᷽��    G33.5   0���� 1����
    uint8_t SSIN:1;     //���᷽����CNC��������PMC����  G33.6   0��CNC 1��PMC
    uint8_t SIND:1;     //�����ٶ���CNC��������PMC���� G33.7   0��CNC 1��PMC
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
    uint8_t HSIA:8;     //���ֲ����ź� G41
	//G42
	uint8_t :8;
	//G43
	uint8_t MD:3;       //��ʽѡ�񣬵�ͬ��fanuc��MD1/MD2/MD4   G43.0~G43.2
	uint8_t :5;
	//G44
	uint8_t BDT1:1;     //�����ź�    G44.0
    uint8_t MLK:1;      //�����������ס�ź� G44.1
    uint8_t :6;
	//G45
	uint8_t :8;
	//G46
	uint8_t :1;
	uint8_t SBK:1;     //��������ź�    G46.1
	uint8_t SBS:1;     //ѡͣ�ź�         G46.2
    uint8_t KEY:1;     //������ס�ź�  G46.3
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
    uint8_t RGTAP:1;    // ���Թ�˿�ź� G61.0
    uint8_t RGMD:1;     // ���Թ�˿ģʽ�л��ź� G61.1
    uint8_t :6;
	//G62
    uint8_t :6;
    uint8_t RTNT:1;     // ���Թ�˿���������ź� G62.6
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
	uint8_t HWT:1;     //���ָ����ź�    G67.2
	uint8_t :5;
	//G68
	uint8_t :8;
    //G69
	uint8_t :8;
	//G70
    uint8_t :4;
    uint8_t SRV:1;      //���ᷴת G70.4
    uint8_t SFR:1;      //������ת G70.5
    uint8_t ORCMA:1;    //���ᶨ�� G70.6
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
    uint8_t JP:8;				//������ͷ���ѡ���ź�  +J1    ��1����
//	uint8_t JP2:1;				//������ͷ���ѡ���ź�  +J2    ��2����
//	uint8_t JP3:1;				//������ͷ���ѡ���ź�  +J3    ��3����
//	uint8_t JP4:1;				//������ͷ���ѡ���ź�  +J4    ��4����
//	uint8_t JP5:1;				//������ͷ���ѡ���ź�  +J5    ��5����
//	uint8_t JP6:1;				//������ͷ���ѡ���ź�  +J6    ��6����
//	uint8_t JP7:1;				//������ͷ���ѡ���ź�  +J7    ��7����
//	uint8_t JP8:1;				//������ͷ���ѡ���ź�  +J8    ��8����
	//G101
	uint8_t :8;
	//G102
    uint8_t JN:8;				//������ͷ���ѡ���ź�  -J1    ��1����
//	uint8_t JN2:1;				//������ͷ���ѡ���ź�  -J2    ��2����
//	uint8_t JN3:1;				//������ͷ���ѡ���ź�  -J3    ��3����
//	uint8_t JN4:1;				//������ͷ���ѡ���ź�  -J4    ��4����
//	uint8_t JN5:1;				//������ͷ���ѡ���ź�  -J5    ��5����
//	uint8_t JN6:1;				//������ͷ���ѡ���ź�  -J6    ��6����
//	uint8_t JN7:1;				//������ͷ���ѡ���ź�  -J7    ��7����
//	uint8_t JN8:1;				//������ͷ���ѡ���ź�  -J8    ��8����
	//G103
    uint8_t :8;
	//G104~G105
	uint16_t REFE:16;           //������زο���ʹ���źţ�  bit0~bit15  ��1~��15
	//G106
	uint8_t :8;
	//G107
	uint8_t :8;
	//G108
    uint8_t MLKI:8;     //��е��ס�ź� ��1~��8
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
	uint8_t axis_limit_postive1:8;	//���򳬳��źţ� G114.0-G114.7������1~��8
	//G115
	uint8_t axis_limit_postive2:8;	//���򳬳��źţ� G115.0-G115.7������9~��16
	//G116
	uint8_t axis_limit_negative1:8;	//���򳬳��źţ� G116.0-G116.7������1~��8
	//G117
	uint8_t axis_limit_negative2:8;	//���򳬳��źţ� G117.0-G117.7������9~��16
	//G118
	uint8_t AXIS_PDEC1:8;          //����������źţ�G118.0-G118.7������1~��8
	//G119
	uint8_t AXIS_PDEC2:8;         //����������źţ�G119.0-G119.7������9~��16
	//G120
	uint8_t AXIS_NDEC1:8;          //����������źţ�G120.0-G120.7������1~��8
	//G121
	uint8_t AXIS_NDEC2:8;          //����������źţ�G121.0-G121.7������9~��16
	//G122
	uint8_t :8;
	//G123
	uint8_t :8;
	//G124
	uint8_t :8;
	//G125
	uint8_t :8;
	//G126
    uint8_t SVF:8;                 //�ŷ��ض��ź� ÿһλ��ʾһ���� 1������ʹ��
	//G127
	uint8_t :8;

	//G128~G135
	uint64_t :64;
	//G136
	uint8_t EAX1:1;					//������ѡ���ź�1��PMC����ƣ���һ�飩  G136.0
	uint8_t EAX2:1;					//������ѡ���ź�2��PMC����ƣ��ڶ��飩  G136.1
	uint8_t EAX3:1;					//������ѡ���ź�3��PMC����ƣ������飩  G136.2
	uint8_t EAX4:1;					//������ѡ���ź�4��PMC����ƣ������飩  G136.3
	uint8_t :4;
	//G137
	uint8_t :8;
	//G138
    uint8_t SYNC:8;                //ͬ�����ƹ��ܿ����Զ���MDI��ʽ������
    //G139
	uint8_t :8;
	//G140
    uint8_t SYNCJ:8;                //��JOG�����ֻ�����������ʽ��ִ��ͬ������
    //G141
	uint8_t :8;
	//G142
	uint8_t EFINA:1;                //�������ܽ����źţ�PMC����ƣ���һ�飩    G142.0
	uint8_t :1;
	uint8_t EMBUFA:1;				//�����ֹ�źţ�PMC����ƣ���һ�飩    G142.2
	uint8_t ESBKA:1;                //�����ֹͣ�źţ�PMC����ƣ���һ�飩    G142.3
	uint8_t ESOFA:1;                //�ŷ��ض��źţ�PMC����ƣ���һ�飩    G142.4
	uint8_t ESTPA:1;                //�������ͣ�źţ�PMC����ƣ���һ�飩    G142.5
	uint8_t ECLRA:1;				//��λ�źţ���һ�飩    G142.6
	uint8_t EBUFA:1;				//�����ָ���ȡ�źţ���һ�飩  G142.7
	//G143
	uint8_t ECA:7;					//EC0A~EC6A  �����ָ���źţ���һ�飩   G143.0~G143.6
	uint8_t EMSBKA:1;				//�����ֹͣ��ֹ�źţ���һ�飩  G143.7

	//G144~G145
	uint16_t EIFA:16;               //EIF0A~EIF15A  ����ƽ����ٶ��źţ���һ�飩   G144~G145
	//G146~G149
	int32_t EIDA:32;  				//EID0A~EID31A  ����������źţ���һ�飩    G146~G149
	//G150
	uint8_t :6;
	uint8_t ERT:1;					//�ֶ����ٽ���ѡ���źţ�PMC����ƣ�    G150.6
	uint8_t :1;
	//G151
	uint8_t :8;

	//G152
	uint8_t :8;
	//G153
	uint8_t :8;
	//G154
	uint8_t EFINB:1;                //�������ܽ����źţ�PMC����ƣ��ڶ��飩    G154.0
	uint8_t :1;
	uint8_t EMBUFB:1;				//�����ֹ�źţ�PMC����ƣ��ڶ��飩    G154.2
	uint8_t ESBKB:1;                //�����ֹͣ�źţ�PMC����ƣ��ڶ��飩    G154.3
	uint8_t ESOFB:1;                //�ŷ��ض��źţ�PMC����ƣ��ڶ��飩    G154.4
	uint8_t ESTPB:1;                //�������ͣ�źţ�PMC����ƣ��ڶ��飩    G154.5
	uint8_t ECLRB:1;				//��λ�źţ��ڶ��飩    G154.6
	uint8_t EBUFB:1;				//�����ָ���ȡ�źţ��ڶ��飩  G154.7
	//G155
//	uint8_t EC0B:1;					//�����ָ���źţ��ڶ��飩   G155.0~G155.6
//	uint8_t EC1B:1;
//	uint8_t EC2B:1;
//	uint8_t EC3B:1;
//	uint8_t EC4B:1;
//	uint8_t EC5B:1;
//	uint8_t EC6B:1;
	uint8_t ECB:7;					//EC0B~EC6B  �����ָ���źţ��ڶ��飩   G155.0~G155.6
	uint8_t EMSBKB:1;				//�����ֹͣ��ֹ�źţ��ڶ��飩  G155.7

	//G156~G157
	uint16_t EIFB:16;               //EIF0B~EIF15B  ����ƽ����ٶ��źţ��ڶ��飩   G156~G157

	//G158~G161
	int32_t EIDB:32;  				//EID0B~EID31B  ����������źţ��ڶ��飩       G158~G161
	//G162
	uint8_t :8;
	//G163
	uint8_t :8;
	//G164
	uint8_t :8;
	//G165
	uint8_t :8;
	//G166
	uint8_t EFINC:1;                //�������ܽ����źţ�PMC����ƣ������飩    G166.0
	uint8_t :1;
	uint8_t EMBUFC:1;				//�����ֹ�źţ�PMC����ƣ������飩    G166.2
	uint8_t ESBKC:1;                //�����ֹͣ�źţ�PMC����ƣ������飩    G166.3
	uint8_t ESOFC:1;                //�ŷ��ض��źţ�PMC����ƣ������飩    G166.4
	uint8_t ESTPC:1;                //�������ͣ�źţ�PMC����ƣ������飩    G166.5
	uint8_t ECLRC:1;				//��λ�źţ������飩    G166.6
	uint8_t EBUFC:1;				//�����ָ���ȡ�źţ������飩  G166.7
	//G167
//	uint8_t EC0C:1;					//�����ָ���źţ������飩   G167.0~G167.6
//	uint8_t EC1C:1;
//	uint8_t EC2C:1;
//	uint8_t EC3C:1;
//	uint8_t EC4C:1;
//	uint8_t EC5C:1;
//	uint8_t EC6C:1;
	uint8_t ECC:7;					//EC0C~EC6C  �����ָ���źţ������飩   G167.0~G167.6
	uint8_t EMSBKC:1;				//�����ֹͣ��ֹ�źţ������飩  G167.7

	//G168~G169
	uint16_t EIFC:16;               //EIF0C~EIF15C  ����ƽ����ٶ��źţ������飩   G168~G169
	//G170~G173
	int32_t EIDC:32;  				//EID0C~EID31C  ����������źţ������飩    G170~G173
	//G174
	uint8_t :8;
	//G175
#ifdef USES_WOOD_MACHINE
	uint8_t BDM:1;                 //�Ű�ģʽ��ľ��ר��   G175.0
	uint8_t BOXM:1;                //����ģʽ��ľ��ר��   G175.1
	uint8_t :6;
#else
	uint8_t :8;
#endif

	//G176
	uint8_t :8;
	//G177
	uint8_t :8;
	//G178
	uint8_t EFIND:1;                //�������ܽ����źţ�PMC����ƣ������飩    G178.0
	uint8_t :1;
	uint8_t EMBUFD:1;				//�����ֹ�źţ�PMC����ƣ������飩    G178.2
	uint8_t ESBKD:1;                //�����ֹͣ�źţ�PMC����ƣ������飩    G178.3
	uint8_t ESOFD:1;                //�ŷ��ض��źţ�PMC����ƣ������飩    G178.4
	uint8_t ESTPD:1;                //�������ͣ�źţ�PMC����ƣ������飩    G178.5
	uint8_t ECLRD:1;				//��λ�źţ������飩    G178.6
	uint8_t EBUFD:1;				//�����ָ���ȡ�źţ������飩  G178.7
	//G179
//	uint8_t EC0D:1;					//�����ָ���źţ������飩   G179.0~G179.6
//	uint8_t EC1D:1;
//	uint8_t EC2D:1;
//	uint8_t EC3D:1;
//	uint8_t EC4D:1;
//	uint8_t EC5D:1;
//	uint8_t EC6D:1;
	uint8_t ECD:7;					//EC0D~EC6D  �����ָ���źţ������飩   G179.0~G179.6
	uint8_t EMSBKD:1;				//�����ֹͣ��ֹ�źţ������飩  G179.7

	//G180~G181
	uint16_t EIFD:16;               //EIF0D~EIF15D  ����ƽ����ٶ��źţ������飩   G180~G181
	//G182~G185
	int32_t EIDD:32;  				//EID0D~EID31D  ����������źţ������飩    G182~G185
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
	uint16_t DEC:16;            //���������źţ��û���زο���
	//G198~G199
	uint16_t REF:16;             //������ԭ���źţ�����׼��������˫��׼����
	//G200~G207
	uint64_t :64;
#ifdef USES_GRIND_MACHINE
	//G208
	uint8_t work_vac_check:1;    //�ӹ�λ��ռ��                  G208.0    ���һ�
	uint8_t work_up_check:1;     //�ӹ�λ����������λ          G208.1   ���һ�
	uint8_t work_down_check:1;   //�ӹ�λ�����½���λ          G208.2  ���һ�
	uint8_t right_work_vac_check:1;  //�Ҽӹ�λ��ռ��     G208.3  ���
	uint8_t op_right_work_vac:1;    //�����������һ��ӹ�λ�����   G208.4    �һ�
	uint8_t m66_runover_right:1;  //�Ҳ�������ִ�����        G208.5   �һ�
	uint8_t m66_req_right:1;     //�Ҳ�����������                  G208.6  ���
	uint8_t new_tray_right:1;    //�Ҳ�֪ͨ�ѻ�������          G208.7  ���
	//G209
	uint8_t left_claw_vac_check:1;    //��צ��ռ��           G209.0   ���
	uint8_t left_claw_up_check:1;     //��צ������λ���   G209.1  ���
	uint8_t right_claw_vac_check:1;    //��צ��ռ��         G209.2  ���
	uint8_t right_claw_up_check:1;     //��צ������λ��� G209.3  ���
	uint8_t :3;
	uint8_t right_work_up_check:1;     //�һ���λ����������λ�ź�       G209.7   ���
	//G210
	uint8_t correct_pos_x_check:1;      //��λX�������ɿ���λ      G210.0   ���
	uint8_t correct_pos_y_check:1;      //��λY�������ɿ���λ      G210.1  ���
	uint8_t :1;
	uint8_t change_new_tray_right_cancel:1;   //���ȡ��֪ͨ�һ���������   G210.3   //�һ�
	uint8_t change_new_tray_right:1;    //���֪ͨ�һ���������   G210.4    �һ�
	uint8_t left_tray_inpos:1;			//��������λ    G210.5     ���
	uint8_t right_tray_inpos:1;         //��������λ    G210.6  ���    //��λΪ0������λΪ1
	uint8_t safe_door:1;                //��ȫ�Ŵ�      G210.7   ���
	//G211
	uint8_t main_chn:1;          //0--����     1--�ӻ�      G211.0   ���һ�
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
	uint8_t EMPC:1;         //PMC���ú����ʹ��    G216.0
#ifdef USES_WOOD_MACHINE
	uint8_t :6;
	uint8_t QDE:1;             //���깦��(Quick Drill)ʹ���ź�   G216.7
#else
	uint8_t :7;
#endif
	//G217~G218
	uint16_t MPCS:16;       //PMC���ú������     G217~G218
	//G219
	uint8_t CHNC:4;         //��ǰͨ����
	uint8_t :4;
	//G220
	uint8_t MFIN4:1;				//MFIN4�źţ���4M���ܽ����ź�  G220.0
	uint8_t MFIN5:1;				//MFIN5�źţ���5M���ܽ����ź�  G220.1
	uint8_t MFIN6:1;				//MFIN6�źţ���6M���ܽ����ź�  G220.2
	uint8_t MFIN7:1;				//MFIN7�źţ���7M���ܽ����ź�  G220.3
	uint8_t MFIN8:1;				//MFIN8�źţ���8M���ܽ����ź�  G220.4
	uint8_t MFIN9:1;				//MFIN9�źţ���9M���ܽ����ź�  G220.5
	uint8_t MFIN10:1;				//MFIN10�źţ���10M���ܽ����ź�  G220.6
	uint8_t MFIN11:1;				//MFIN11�źţ���11M���ܽ����ź�  G220.7
	//G221
	uint8_t MFIN12:1;				//MFIN12�źţ���12M���ܽ����ź�  G221.0
	uint8_t MFIN13:1;				//MFIN13�źţ���13M���ܽ����ź�  G221.1
	uint8_t MFIN14:1;				//MFIN14�źţ���14M���ܽ����ź�  G221.2
	uint8_t MFIN15:1;				//MFIN15�źţ���15M���ܽ����ź�  G221.3
	uint8_t MFIN16:1;				//MFIN16�źţ���16M���ܽ����ź�  G221.4
	uint8_t :3;
	//G222
	uint8_t TFIN2:1;                 //T���ܽ����ź�2    G222.0
	uint8_t TFIN3:1;                 //T���ܽ����ź�3    G222.1
	uint8_t TFIN4:1;                 //T���ܽ����ź�4    G222.2
	uint8_t TFIN5:1;                 //T���ܽ����ź�5    G222.3
	uint8_t TFIN6:1;                 //T���ܽ����ź�6    G222.4
	uint8_t TFIN7:1;                 //T���ܽ����ź�7    G222.5
	uint8_t TFIN8:1;                 //T���ܽ����ź�8    G222.6
	uint8_t TFIN9:1;                 //T���ܽ����ź�9    G222.7
	//G223
	uint8_t TFIN10:1;                 //T���ܽ����ź�10    G222.0
	uint8_t TFIN11:1;                 //T���ܽ����ź�11    G222.1
	uint8_t TFIN12:1;                 //T���ܽ����ź�12    G222.2
	uint8_t TFIN13:1;                 //T���ܽ����ź�13    G222.3
	uint8_t TFIN14:1;                 //T���ܽ����ź�14    G222.4
	uint8_t TFIN15:1;                 //T���ܽ����ź�15    G222.5
	uint8_t TFIN16:1;                 //T���ܽ����ź�16    G222.6
	uint8_t :1;
	//G224
	uint8_t MEXC1:1;                  //MEXC1�źţ���1��Mָ���ִ�����ź�   G224.0
	uint8_t MEXC2:1;                  //MEXC2�źţ���2��Mָ���ִ�����ź�   G224.1
	uint8_t MEXC3:1;                  //MEXC3�źţ���3��Mָ���ִ�����ź�   G224.2
	uint8_t MEXC4:1;                  //MEXC4�źţ���4��Mָ���ִ�����ź�   G224.3
	uint8_t MEXC5:1;                  //MEXC5�źţ���5��Mָ���ִ�����ź�   G224.4
	uint8_t MEXC6:1;                  //MEXC6�źţ���6��Mָ���ִ�����ź�   G224.5
	uint8_t MEXC7:1;                  //MEXC7�źţ���7��Mָ���ִ�����ź�   G224.6
	uint8_t MEXC8:1;                  //MEXC8�źţ���8��Mָ���ִ�����ź�   G224.7
	//G225
	uint8_t MEXC9:1;                  //MEXC9�źţ���9��Mָ���ִ�����ź�   G225.0
	uint8_t MEXC10:1;                  //MEXC10�źţ���10��Mָ���ִ�����ź�   G225.1
	uint8_t MEXC11:1;                  //MEXC11�źţ���11��Mָ���ִ�����ź�   G225.2
	uint8_t MEXC12:1;                  //MEXC12�źţ���12��Mָ���ִ�����ź�   G225.3
	uint8_t MEXC13:1;                  //MEXC13�źţ���13��Mָ���ִ�����ź�   G225.4
	uint8_t MEXC14:1;                  //MEXC14�źţ���14��Mָ���ִ�����ź�   G225.5
	uint8_t MEXC15:1;                  //MEXC15�źţ���15��Mָ���ִ�����ź�   G225.6
	uint8_t MEXC16:1;                  //MEXC16�źţ���16��Mָ���ִ�����ź�   G225.7
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

//F��ַ�Ĵ�������
union FRegister{
	uint8_t all[F_REG_COUNT];
	FRegBits bits[kMaxChnCount];
};


//G��ַ�Ĵ�������
union GRegister{
	uint8_t all[G_REG_COUNT];
	GRegBits bits[kMaxChnCount];
};



//�Ĵ������Ͷ���
enum PmcRegSection{
	PMC_REG_X = 0,		//X�Ĵ���
	PMC_REG_Y,			//Y�Ĵ���
	PMC_REG_F,			//F�Ĵ���
	PMC_REG_G,			//G�Ĵ���
	PMC_REG_K,			//K�Ĵ���
	PMC_REG_R,			//R�Ĵ���
	PMC_REG_A,			//A�Ĵ���
	PMC_REG_D,			//D�Ĵ���
	PMC_REG_C,			//C�Ĵ���
	PMC_REG_T,			//T�Ĵ���
#ifndef USES_PMC_2_0
	PMC_REG_DC,			//DC�Ĵ���
	PMC_REG_DT			//DT�Ĵ���
#else
	PMC_REG_E           //E�Ĵ���
#endif
};

enum PmcKeepMaskBit{
	K_REG_BIT = 0,      //K�Ĵ���
	D_REG_BIT,			//D�Ĵ���
	DC_REG_BIT,			//DC�Ĵ���
	DT_REG_BIT,			//DT�Ĵ���


	KEEP_REG_COUNT     //�����ͼĴ�������
};


/**
 * @brief PMC�Ĵ����ඨ�壬����PMCģ������мĴ���
 */
class PmcRegister {
public:
	PmcRegister();
	virtual ~PmcRegister();

	bool SetRegValue(PmcRegSection sec, uint16_t index, uint8_t value);	//���üĴ���ֵ
	bool SetRegValue(PmcRegSection sec, uint16_t index, uint16_t value);	//���üĴ���ֵ
	bool SetRegBitValue(PmcRegSection sec, uint16_t index, uint8_t bit, uint8_t count, uint32_t value);   //���üĴ���bitֵ
	bool GetRegValue(PmcRegSection sec, uint16_t index, uint8_t &value);	//��ȡ�Ĵ���ֵ
	bool GetRegValue(PmcRegSection sec, uint16_t index, uint16_t &value);	//��ȡ�Ĵ���ֵ
	bool GetRegValueMulti(PmcRegSection sec, uint16_t index, uint16_t count, uint8_t *value);   //��ȡ����Ĵ���ֵ
	bool GetRegValueMulti(PmcRegSection sec, uint16_t index, uint16_t count, uint16_t *value);   //��ȡ����Ĵ���ֵ

	bool GetRegValue(PmcRegSection sec, uint16_t index, int32_t &value);   //��ȡ���ֽڼĴ���ֵ
//	bool IsKeepRegChanged(){return this->m_mask_save_keep==0x00?false:true;}  //�����ͼĴ����Ƿ�ı�


	FRegister &FReg(){return m_f_reg;}		//����F�Ĵ���
	const GRegister &GReg(){return m_g_reg;}		//����G�Ĵ���
	uint8_t *GetRegPtr8(PmcRegSection sec);	//���ؼĴ�����ָ��
#ifndef USES_PMC_2_0
	uint16_t *GetRegPtr16(PmcRegSection sec);	//���ؼĴ�����ָ��
#endif

	void SaveRegData();   //�������ʧ������


private:
	void Initialize();	//��ʼ��
	void InitRegFile();    //��ʼ���Ĵ��������ļ�


private:
	uint8_t m_x_reg[X_REG_COUNT];		//�����źżĴ���
#ifdef USES_PMC_2_0
	uint8_t m_reserved_1[7];            //Ϊ�˽���ֽڶ�������ռλ
#endif
	uint8_t m_y_reg[Y_REG_COUNT];		//����źżĴ���
#ifdef USES_PMC_2_0
	uint8_t m_reserved_2[4];            //Ϊ�˽���ֽڶ�������ռλ
#endif
	FRegister m_f_reg;					//NC->PMC�� F��ַ
	GRegister m_g_reg;			    	//PMC->NC�� G��ַ
	uint8_t m_a_reg[A_REG_COUNT];		//�澯�Ĵ���
#ifdef USES_PMC_2_0
	uint8_t m_reserved_3[7];            //Ϊ�˽���ֽڶ�������ռλ
#endif
	uint8_t m_r_reg[R_REG_COUNT];		//�ڲ��Ĵ���
	uint8_t m_k_reg[K_REG_COUNT];		//����ʧ�Ĵ���
#ifdef USES_PMC_2_0
	uint8_t m_reserved_4[4];            //Ϊ�˽���ֽڶ�������ռλ
#endif
#ifndef USES_PMC_2_0
	uint16_t m_d_reg[D_REG_COUNT];		//���ݼĴ���������ʧ
	uint16_t m_t_reg[T_REG_COUNT];		//��ʱ��
	uint16_t m_c_reg[C_REG_COUNT];		//������
	uint16_t m_dt_reg[T_REG_COUNT];		//��ʱ��Ԥ��ֵ�Ĵ���������ʧ
	uint16_t m_dc_reg[C_REG_COUNT];		//������Ԥ��ֵ�Ĵ���������ʧ
#else
	uint8_t m_d_reg[D_REG_COUNT];		//���ݼĴ���������ʧ
	uint8_t m_t_reg[T_REG_COUNT*2];		//��ʱ��
	uint8_t m_c_reg[C_REG_COUNT*4];		//������
	uint8_t m_e_reg[E_REG_COUNT];       //��չ�Ĵ���
#endif


	MICommunication *m_p_mi_comm;	//MIͨѶ�ӿ�
//	uint8_t m_mask_save_keep;     //���汣���ͼĴ���  bit0-bit3���α�ʾK/D/DT/DC�Ĵ�����Ҫ���浽�ļ�
	int m_n_fp;         //�Ĵ����ļ��򿪾��
};


#endif /* PMCREGISTER_H_ */
