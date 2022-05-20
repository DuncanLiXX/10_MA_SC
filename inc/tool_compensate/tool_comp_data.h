/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/08
 *@brief 本头文件包含刀具半径补偿相关数据结构的声明
 *@version
 */

#ifndef INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_
#define INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_

#include "geometry_data.h"

struct ToolOffset
{
	double radius;
};

struct CoordRec
{
	DPoint target;
	int code;
};

struct DataPoint
{
	DPlane target;
};

struct DataLine
{
	double GetLength();
	void InitData();//初始化起始点坐标为(0,0)
	DPlane source;
	DPlane target;
	DataLine& Offset(const ToolOffset& off);
	DataLine& LengthOffset(ToolOffset& offset, int flag) ;
};
struct DataCircle
{
	int     flag;
	double  radius;
	DPlane  target;
	DPlane  center;
	DataCircle& offset(const ToolOffset& off, void *troop = nullptr, bool *bErrFlag = nullptr);
};
struct DataArc
{
	int    flag;
	double radius;
	double   getAngle();
	//2012.9.1, GongLihu, G5
	double  getLength();
	DPlane source, target, center;
	DataArc& offset(const ToolOffset& off, void *troop = nullptr, bool *bErrFlag = nullptr);
};

struct PointRec
{
	DPoint target;
	DataPoint getData();
	void setData( const DataPoint& );
};
struct LineRec
{
	int     flag;     // 0:line,1:screw
	DPoint  source;
	DPoint  target;
	DataLine getData();
	void setData(const DataLine& );
};
struct CircleRec
{
	int     flag;    // 0:clockwise,1:anticlockwise
	double  radius;
	DPoint  source;
	DPoint  target;
	DPoint  center;

	DataCircle getData();
	void setData(const DataCircle&);
};
struct ArcRec
{
	int     flag;    //0:clockwise,1:anticlockwise
	double  radius;
	DPoint  source;
	DPoint  target;
	DPoint  center;

	DataArc getData();
	void setData(const DataArc& );
};

struct ModeRec{

};

struct GeometryRec
{
	int what;
	ushort usFlag; //0-普通数据；1--插入数据；2--优化数据
	ushort usSmoothFlag;  //0--无平滑  2--样条平滑 199--非G5.1数据
	union
	{
		CoordRec     coordinate;
		PointRec     point;
		LineRec      line;
		ArcRec       arc;
		CircleRec    circle;
#ifdef Uses_NURBS
		NurbsRec		nurbs;
#endif
		ModeRec       mcode;
//		ErrRec       error;
#ifdef Uses_CbsSmooth
		smoothRec   smooth;
		cbsRec      cbs;
#endif
	};
	GeometryRec();
	GeometryRec(int aWhat);
};


#endif /* INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_ */
