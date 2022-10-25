/*************************************************************************
 * 	本程序是LITRAN-0MB 刀补用程序
 * 	编程时间：2022-10-15
 * 	编程人：	侯长合
 * ***********************************************************************/

#include "tool_comp_data.h"


//  地址译码寄存器
ADRSREG_DEF      Car;
ADRSREG_DEF      Bar;
ADRSREG_DEF      Dar;
ADRSREG_DEF      Aar;


//  插补输出寄存器寄存器
//  BOR,AOR: 0,1
uint8_t   r_OutReg[2][154];

uint32_t  r_OfsVal;       //  刀具偏置量


int32_t   r_C1;   //  圆心
int32_t   r_C2;   //  圆心

//  系统报警寄存器
uint8_t   f_AlmBf;      //  报警标志
uint8_t   r_PsNo;       //  报警号
