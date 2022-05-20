/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/08
 *@brief 本头文件包含刀具半径补偿相关数据结构的实现
 *@version
 */

#include "global_include.h"
#include "tool_comp_data.h"


/**
 * @brief  计算直线长度
 * @return
 */
double DataLine::GetLength()
{
	double  mm = source.x - target.x,
	            nn = source.y - target.y;
	return (mm * mm + nn * nn);
}

/**
 * @brief
 * @param offset
 * @return
 */
DataLine& DataLine::Offset(const ToolOffset& offset )
{
//	double angle = arcmath(target, source) * M_RAD + M_PI_2,
//	           off  = offset.radius;
//
//	if(fabs(off) < 1e-4)
//		return *this;
//
//	DPlane move( off * cos(angle), off * sin(angle) );
//	source += move;
//	target += move;
	return *this;
}
