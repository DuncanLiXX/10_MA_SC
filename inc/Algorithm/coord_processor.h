/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mi_communication.h
 *@author gonghao
 *@date 2020/06/16
 *@brief 本头文件为坐标系处理器类的声明
 *@version
 */

#ifndef COORD_PROCESSOR_H_
#define COORD_PROCESSOR_H_

#include <stdint.h>
#include "global_structure.h"
#include "geometry_data.h"

enum CoordTransType{
	NULL_TRANS    =   0,	//非法转换
	MOVE_TRANS    =   1,	//平移
	REVOLVE_TRANS =   2,	//旋转
	RATIO_TRANS   =   3,	//比例缩放
	XMIRROR_TRANS =   4,	//X轴镜像
	YMIRROR_TRANS =   5,	//Y轴镜像
	ZMIRROR_TRANS =   6		//Z轴镜像
};

/**
 * @brief 坐标系转换基类
 */
class DataTrans
{
public:
	DataTrans()
		: what( NULL_TRANS ),
		  next(nullptr),
		  prev(nullptr)
	{
	}
	virtual ~DataTrans(){}
	virtual void trans(void* )  = 0;
	virtual void trans(DPoint& ) = 0;
	virtual void change(DPoint ) = 0;

public:
	uint8_t what;  //变换类型
	DataTrans* next;  //下一个变换对象
	DataTrans* prev;  //上一个变换对象
};

/**
 * @brief 坐标系镜像
 */
class DataMirror: public DataTrans
{
public:
	DataMirror(uint8_t mirr_type , double v, int aPlane);
	void trans(void* rec);
	void trans(DPoint& );
    void change(DPoint );
public:
	double OriValue;  //原始镜像轴坐标
	double value;  //镜像轴坐标
	int    plane;    //镜像平面
};

/**
 * @brief 坐标系平移
 */
class DataMove : public DataTrans
{
public:
	DataMove(const DPoint& aMove);
	void    trans(void* );
	void    trans(DPoint& );
public:
	DPoint  move;   //平移矢量
};

/**
 * @brief 坐标系缩放
 */
class DataRatio : public DataTrans
{
public:
	DataRatio(const DPoint& aCenter, double aRatio);
	DataRatio(const DPoint& aCenter, DPoint aRatio);
	void    trans(void* );
	void    trans(DPoint& );
    void    change(DPoint );

public:
	DPoint  OriCenter;  //原始缩放中心
	DPoint  center;		//缩放中心
//	double  ratio;  	//缩放比例
	DPoint  ratio1;		//缩放比例，支持各轴不等比例缩放
};

/**
 * @brief 坐标系旋转
 */
class DataRevolve : public DataTrans
{
public:
	DataRevolve(const DPoint& aCenter,double aAngle, int aPlane);  //DataRevolve(const DPoint& aCenter, double aAngle, int aPlane);
	void    trans(void* );
	void    trans(DPoint& );
    void    change(DPoint );

public:
	DPoint  center;   //旋转中心
	DPoint  OriCenter;	//初始旋转中心
	double  OriAngle;	//初始角度
	double  angle;		//旋转角度
	int     plane;		//旋转平面
};

/**
 * @brief 坐标系处理器，主要处理两个问题：1.工件坐标系到机械坐标系的转换； 2.坐标系的变换，包括缩放、镜像、旋转
 */
class CoordProcessor {
public:
	CoordProcessor();
	virtual ~CoordProcessor();

private:
	CoordInfo m_st_coord_info;   //坐标系数据
	DataTrans *m_p_trans;   //转换链
};

#endif /* COORD_PROCESSOR_H_ */
