/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mi_communication.h
 *@author gonghao
 *@date 2020/06/16
 *@brief ��ͷ�ļ�Ϊ����ϵ�������������
 *@version
 */

#ifndef COORD_PROCESSOR_H_
#define COORD_PROCESSOR_H_

#include <stdint.h>
#include "global_structure.h"
#include "geometry_data.h"

enum CoordTransType{
	NULL_TRANS    =   0,	//�Ƿ�ת��
	MOVE_TRANS    =   1,	//ƽ��
	REVOLVE_TRANS =   2,	//��ת
	RATIO_TRANS   =   3,	//��������
	XMIRROR_TRANS =   4,	//X�᾵��
	YMIRROR_TRANS =   5,	//Y�᾵��
	ZMIRROR_TRANS =   6		//Z�᾵��
};

/**
 * @brief ����ϵת������
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
	uint8_t what;  //�任����
	DataTrans* next;  //��һ���任����
	DataTrans* prev;  //��һ���任����
};

/**
 * @brief ����ϵ����
 */
class DataMirror: public DataTrans
{
public:
	DataMirror(uint8_t mirr_type , double v, int aPlane);
	void trans(void* rec);
	void trans(DPoint& );
    void change(DPoint );
public:
	double OriValue;  //ԭʼ����������
	double value;  //����������
	int    plane;    //����ƽ��
};

/**
 * @brief ����ϵƽ��
 */
class DataMove : public DataTrans
{
public:
	DataMove(const DPoint& aMove);
	void    trans(void* );
	void    trans(DPoint& );
public:
	DPoint  move;   //ƽ��ʸ��
};

/**
 * @brief ����ϵ����
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
	DPoint  OriCenter;  //ԭʼ��������
	DPoint  center;		//��������
//	double  ratio;  	//���ű���
	DPoint  ratio1;		//���ű�����֧�ָ��᲻�ȱ�������
};

/**
 * @brief ����ϵ��ת
 */
class DataRevolve : public DataTrans
{
public:
	DataRevolve(const DPoint& aCenter,double aAngle, int aPlane);  //DataRevolve(const DPoint& aCenter, double aAngle, int aPlane);
	void    trans(void* );
	void    trans(DPoint& );
    void    change(DPoint );

public:
	DPoint  center;   //��ת����
	DPoint  OriCenter;	//��ʼ��ת����
	double  OriAngle;	//��ʼ�Ƕ�
	double  angle;		//��ת�Ƕ�
	int     plane;		//��תƽ��
};

/**
 * @brief ����ϵ����������Ҫ�����������⣺1.��������ϵ����е����ϵ��ת���� 2.����ϵ�ı任���������š�������ת
 */
class CoordProcessor {
public:
	CoordProcessor();
	virtual ~CoordProcessor();

private:
	CoordInfo m_st_coord_info;   //����ϵ����
	DataTrans *m_p_trans;   //ת����
};

#endif /* COORD_PROCESSOR_H_ */
