/*************************************************************************
 * 	��������LITRAN-0MB �����ó���
 * 	���ʱ�䣺2022-10-15
 * 	����ˣ�	���
 * ***********************************************************************/

#include "tool_comp_data.h"


//  ��ַ����Ĵ���
ADRSREG_DEF      Car;
ADRSREG_DEF      Bar;
ADRSREG_DEF      Dar;
ADRSREG_DEF      Aar;


//  �岹����Ĵ����Ĵ���
//  BOR,AOR: 0,1
uint8_t   r_OutReg[2][154];

uint32_t  r_OfsVal;       //  ����ƫ����


int32_t   r_C1;   //  Բ��
int32_t   r_C2;   //  Բ��

//  ϵͳ�����Ĵ���
uint8_t   f_AlmBf;      //  ������־
uint8_t   r_PsNo;       //  ������


ADRSREG_DEF CompBuffer[2];

ADRSREG_DEF * pBar;
ADRSREG_DEF * pCar;

static bool first_comp = true;
static int index = 0;

void pushComp(ADRSREG_DEF block)
{

}
