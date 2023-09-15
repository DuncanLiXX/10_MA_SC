/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mi_communication.h
 *@author gonghao
 *@date 2020/06/16
 *@brief ��ͷ�ļ�Ϊ����ϵ���������ʵ��
 *@version
 */

#include "coord_processor.h"
#include "compile_message.h"

/**
 * @brief ���캯��
 */
CoordProcessor::CoordProcessor() {
	// TODO Auto-generated constructor stub

}

/**
 * @brief ��������
 */
CoordProcessor::~CoordProcessor() {
	// TODO Auto-generated destructor stub
}




/******************************************************************DataMirror class*************************************************************************/
/**
 * @brief ����ת���֧࣬��X�᾵��Y�᾵��Z�᾵��
 * @param mirr_type ����������
 * @param v : �������ֵ
 * @param aPlane: ����ƽ��
 */
DataMirror::DataMirror(uint8_t mirr_type, double v, int aPlane)
{
	what = mirr_type;
	value = OriValue = v;
	plane = aPlane;
}

void DataMirror::trans(void* rec )
{
	RecordMsg* msg = (RecordMsg* )rec;

	if( what == XMIRROR_TRANS ){

	}
	else if( what == YMIRROR_TRANS ){

	}
	else if( what == ZMIRROR_TRANS ){

	}
}


void DataMirror::trans(DPoint& obj )
{
	if( what == XMIRROR_TRANS )
		obj.x =  2*value - obj.x;
	else if( what == YMIRROR_TRANS )
		obj.y =  2*value - obj.y;
	else if( what == ZMIRROR_TRANS )
		obj.z =  2*value - obj.z;

}

void DataMirror::change(DPoint doff )
{
	if( what == XMIRROR_TRANS )
	{
            OriValue += doff.x;
            value += doff.x;
	}
	else if( what == YMIRROR_TRANS )
	{
            OriValue += doff.y;
            value += doff.y;
	}
	else if( what == ZMIRROR_TRANS )
	{
            OriValue += doff.z;
            value += doff.z;
	}
}

/******************************************************************DataMove class*************************************************************************/
DataMove::DataMove(const DPoint& aMove)
	: DataTrans(),
	  move( aMove )
{
	what = MOVE_TRANS;
}

void DataMove::trans(void* rec )
{
	RecordMsg* msg = (RecordMsg* )rec;
//	info->forEach(DataMessage::moveTo, &move );
}
void DataMove::trans(DPoint& obj )
{
	obj += move;
}

/******************************************************************DataRatio class*************************************************************************/
DataRatio::DataRatio(const DPoint& aCenter, double aRatio)  //p����
	: DataTrans(),
	  OriCenter( aCenter )
//	  ratio( aRatio)
{
	what = RATIO_TRANS;
	center = OriCenter;
	ratio1.x = aRatio;
	ratio1.y = aRatio;
	ratio1.z = aRatio;
	ratio1.a4 = ratio1.a5 = ratio1.a6 = ratio1.a7 = ratio1.a8 = 1.0;
}

DataRatio::DataRatio(const DPoint& aCenter, DPoint aRatio)  // i j k����
	: DataTrans(),
	  OriCenter( aCenter ),
	  ratio1( aRatio)
{
	what = RATIO_TRANS;
	center = OriCenter;
//	ratio = 1.0;
}
void DataRatio::trans(void* rec )
{
	RecordMsg* msg = (RecordMsg* )rec;
//	paraRatio p(center, ratio1);
//	info->forEach(DataMessage::ratioTo, &p);
}
void DataRatio::trans(DPoint& obj )
{
	DPoint p = obj - center;
	p.x *= ratio1.x;
	p.y *= ratio1.y;
	p.z *= ratio1.z;
	obj = p + center;
}
void DataRatio::change(DPoint doff )
{

	center     +=doff;
	OriCenter+=doff;

}


/******************************************************************DataRevolve class*************************************************************************/
/**
 * @brief ��ת�任��Ĺ��캯��
 * @param aCenter
 * @param aCenter1
 * @param aAngle �� ��ת�Ƕȣ���ʱ��Ϊ����˳ʱ��Ϊ��
 * @param aPlane �� ��תƽ��
 */
DataRevolve::DataRevolve(const DPoint& aCenter, double aAngle, int aPlane)
	: DataTrans(),
	  OriCenter( aCenter ),
	  OriAngle( aAngle ),
	  plane( aPlane)

{
	what = REVOLVE_TRANS;
	center = OriCenter;
	angle = OriAngle;
}
void DataRevolve::trans(void* rec)
{
	RecordMsg* msg = (RecordMsg* )rec;
//	paraRevolve p(center, angle, plane);
//	info->forEach(DataMessage::revolveTo, &p);
}
void DataRevolve::trans(DPoint& obj )
{
	DPlane p =::Point2Plane(obj, plane), c =::Point2Plane(center, plane);
	p.revolve(c, angle);
	::Plane2Point(p, obj, plane);
}
void DataRevolve::change(DPoint doff )
{
//        AbsCenter +=doff;
        OriCenter +=doff;
	center      +=doff;
}
