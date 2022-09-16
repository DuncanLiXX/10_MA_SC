/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file geometry_data.h
 *@author gonghao
 *@date 2020/04/16
 *@brief �йؼ��οռ���Ϣ�������ඨ���ļ�
 *@version
 */

#ifndef INC_GEOMETRY_DATA_H_
#define INC_GEOMETRY_DATA_H_

//#include "global_definition.h"
#include "hmi_shared_data.h"

extern const double M_RAD;
extern const double M_1_RAD;

/**
 * @brief ��άƽ���������
 */
class DPlane
{
public:
    DPlane(): x(0), y(0) {}
    DPlane( const DPlane& p): x(p.x), y(p.y) {}
    DPlane( double X, double Y): x(X), y(Y) {}

    DPlane& operator=( const DPlane& c);
    DPlane& operator+=( const DPlane& adder  );
    DPlane& operator-=( const DPlane& subber );
    DPlane& operator*=( const double& modulus);
    DPlane& operator/=( const double& modulus);
    DPlane& revolve(const DPlane& source, double aAngle);  //��ת����
    friend DPlane operator - ( const DPlane& one, const DPlane& two);
    friend DPlane operator + ( const DPlane& one, const DPlane& two);
    friend int    operator ==( const DPlane& one, const DPlane& two);
    friend int    operator !=( const DPlane& one, const DPlane& two);

    double x;  //X����ֵ
    double y;  //Y����ֵ
};

/**
 * @brief ��ά�������࣬���8��
 */
class DPoint
{
public:
    DPoint();
    DPoint(const DPoint& b );
    DPoint(double x1, double y1, double z1, double a4 = 0, double a5 = 0, double a6 = 0, double a7 = 0, double a8 = 0); //

    void Round(uint8_t round_type = 0);   //��������  round_type = 0 : 0.05um����  round_type = 1 �� 0.5um����
    double GetAxisValue(int index) const;    //��ȡ��������

    DPoint& operator=( const DPoint& c);
    DPoint& operator=( const double& value);
    DPoint& operator+=( const DPoint& adder  );
    DPoint& operator-=( const DPoint& subber );
    DPoint& operator*=( const double& modulus);
    DPoint& operator/=( const double& modulus);

    friend DPoint operator - ( const DPoint& one, const DPoint& two);
    friend DPoint operator + ( const DPoint& one, const DPoint& two);
    friend bool    operator == ( const DPoint& one, const DPoint& two);
    friend bool    operator != ( const DPoint& one, const DPoint& two);

    void CopyData(DPoint &c) const;
    double x, y, z, a4, a5, a6, a7, a8;   //��������ֵ
};

/**
 * @brief ��ά�������࣬�����������ͨ���������ȷ��
 */
class DPointChn
{
public:
	DPointChn();
	DPointChn(const DPointChn& b );
	DPointChn(double x, double y, double z); //

    void Round(uint8_t round_type = 0);   //��������  round_type = 0 : 0.05um����  round_type = 1 �� 0.5um����
    double GetAxisValue(int index);       //��ȡ��������
    void SetAxisValue(int index, double value);      //���õ�������

    DPointChn& operator=( const DPointChn& c);
    DPointChn& operator=( const double& value);
    DPointChn& operator+=( const DPointChn& adder  );
    DPointChn& operator-=( const DPointChn& subber );
    DPointChn& operator*=( const double& modulus);
    DPointChn& operator/=( const double& modulus);

    friend DPointChn operator - ( const DPointChn& one, const DPointChn& two);
    friend DPointChn operator + ( const DPointChn& one, const DPointChn& two);
    friend bool    operator == ( const DPointChn& one, const DPointChn& two);
    friend bool    operator != ( const DPointChn& one, const DPointChn& two);

    void CopyData(DPointChn &c) const;   //�������ݵ�c
    double m_df_point[kMaxAxisChn];   //��������ֵ
};


DPlane Point2Plane( const DPoint& p, int plane);//����ά������ӳ��Ϊ��άƽ������
DPlane Point2Plane(const DPointChn &p, int plane); //����ά������ӳ��Ϊ��άƽ������

void Plane2Point( const DPlane& p, DPoint& ret, int plane); //����άƽ������ת��Ϊ��ά������
void Plane2Point( const DPlane& p, DPointChn& ret, int plane); //����άƽ������ת��Ϊ��ά������

double GetVectAngle(const DPlane& end, const DPlane& start);  //��ƽ���ڼ���һ�������ķ���ǣ���X������ļн�

double GetVectLength(const DPlane &end, const DPlane &start); //��ƽ���ڼ���һ�������ĳ��ȣ�����������



//T���ٶȹ滮������
//class TPlanSimulator{
//public:
//	TPlanSimulator(int freq, int acc);   //
//	virtual ~TPlanSimulator();
//
//	void Reset();    //��λ
//	void Start(int dist, int vel);  	  	//���������þ����Ŀ���ٶ�
//	bool GetPos(int &dist);  	//��ȡÿ�����ڵľ���
//
//private:
//	int m_n_acc_target;    //Ŀ����ٶȣ���λ��plus/T^2,����20
//	int m_n_vel_target;    //Ŀ���ٶȣ���λ��plus/T
//	int m_n_dist_target;   //Ŀ�����, ��λ��plus
//	int m_n_freq;  //�岹Ƶ�ʣ���λHZ����������125us��Ƶ����Ϊ8000hz
//
//	int m_n_period_count;   //���ڼ���
//
//	int m_n_dist_cur;  //��ǰ�˶�����
//	int m_n_vel_cur;  //��ǰ�ٶ�
// //	int m_n_acc_cur;  //ʵ�ʼ��ٶ�
//	bool m_flag_dir;  //�˶�����true��ʾ����false��ʾ����
//
//	int m_n_acc_period;   //���ٶ�������
//	int m_n_uni_period;  //���ٶ�������
//	int m_n_dcc_period;  //���ٶ�������
//	int m_n_total_period;   //���˶�����
//
//};

#endif /* INC_GEOMETRY_DATA_H_ */
