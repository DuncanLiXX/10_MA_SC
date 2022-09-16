/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file geometry_data.h
 *@author gonghao
 *@date 2020/04/16
 *@brief 有关几何空间信息的数据类定义文件
 *@version
 */

#ifndef INC_GEOMETRY_DATA_H_
#define INC_GEOMETRY_DATA_H_

//#include "global_definition.h"
#include "hmi_shared_data.h"

extern const double M_RAD;
extern const double M_1_RAD;

/**
 * @brief 二维平面点坐标类
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
    DPlane& revolve(const DPlane& source, double aAngle);  //旋转函数
    friend DPlane operator - ( const DPlane& one, const DPlane& two);
    friend DPlane operator + ( const DPlane& one, const DPlane& two);
    friend int    operator ==( const DPlane& one, const DPlane& two);
    friend int    operator !=( const DPlane& one, const DPlane& two);

    double x;  //X坐标值
    double y;  //Y坐标值
};

/**
 * @brief 多维点坐标类，最大8轴
 */
class DPoint
{
public:
    DPoint();
    DPoint(const DPoint& b );
    DPoint(double x1, double y1, double z1, double a4 = 0, double a5 = 0, double a6 = 0, double a7 = 0, double a8 = 0); //

    void Round(uint8_t round_type = 0);   //四舍五入  round_type = 0 : 0.05um精度  round_type = 1 ： 0.5um精度
    double GetAxisValue(int index) const;    //获取单轴坐标

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
    double x, y, z, a4, a5, a6, a7, a8;   //各轴坐标值
};

/**
 * @brief 多维点坐标类，最大轴数根据通道最大轴数确定
 */
class DPointChn
{
public:
	DPointChn();
	DPointChn(const DPointChn& b );
	DPointChn(double x, double y, double z); //

    void Round(uint8_t round_type = 0);   //四舍五入  round_type = 0 : 0.05um精度  round_type = 1 ： 0.5um精度
    double GetAxisValue(int index);       //获取单轴坐标
    void SetAxisValue(int index, double value);      //设置单轴坐标

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

    void CopyData(DPointChn &c) const;   //拷贝数据到c
    double m_df_point[kMaxAxisChn];   //各轴坐标值
};


DPlane Point2Plane( const DPoint& p, int plane);//将三维点坐标映射为二维平面坐标
DPlane Point2Plane(const DPointChn &p, int plane); //将三维点坐标映射为二维平面坐标

void Plane2Point( const DPlane& p, DPoint& ret, int plane); //将二维平面坐标转换为三维点坐标
void Plane2Point( const DPlane& p, DPointChn& ret, int plane); //将二维平面坐标转换为三维点坐标

double GetVectAngle(const DPlane& end, const DPlane& start);  //在平面内计算一个向量的方向角，与X轴正向的夹角

double GetVectLength(const DPlane &end, const DPlane &start); //在平面内计算一个向量的长度，即两点间距离



//T型速度规划仿真器
//class TPlanSimulator{
//public:
//	TPlanSimulator(int freq, int acc);   //
//	virtual ~TPlanSimulator();
//
//	void Reset();    //复位
//	void Start(int dist, int vel);  	  	//启动，设置距离和目标速度
//	bool GetPos(int &dist);  	//获取每个周期的距离
//
//private:
//	int m_n_acc_target;    //目标加速度，单位：plus/T^2,比如20
//	int m_n_vel_target;    //目标速度，单位：plus/T
//	int m_n_dist_target;   //目标距离, 单位：plus
//	int m_n_freq;  //插补频率，单位HZ，例如周期125us，频率则为8000hz
//
//	int m_n_period_count;   //周期计数
//
//	int m_n_dist_cur;  //当前运动距离
//	int m_n_vel_cur;  //当前速度
// //	int m_n_acc_cur;  //实际加速度
//	bool m_flag_dir;  //运动方向，true表示正向，false表示负向
//
//	int m_n_acc_period;   //加速段周期数
//	int m_n_uni_period;  //匀速段周期数
//	int m_n_dcc_period;  //减速段周期数
//	int m_n_total_period;   //总运动周期
//
//};

#endif /* INC_GEOMETRY_DATA_H_ */
