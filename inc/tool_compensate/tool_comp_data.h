/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/08
 *@brief ��ͷ�ļ��������߰뾶����������ݽṹ������
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
	void InitData();//��ʼ����ʼ������Ϊ(0,0)
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
	ushort usFlag; //0-��ͨ���ݣ�1--�������ݣ�2--�Ż�����
	ushort usSmoothFlag;  //0--��ƽ��  2--����ƽ�� 199--��G5.1����
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
