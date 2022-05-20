/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file geometry_data.cpp
 *@author gonghao
 *@date 2020/04/16
 *@brief 有关几何空间信息的数据类实现文件
 *@version
 */
//#include "geometry_data.h"
#include "global_include.h"

const double M_RAD   = M_PI / 180.; //0.0174532925199433  1度对应的弧度
const double M_1_RAD = 180. / M_PI; //57.2957795130823;   1弧度对应的度



/*****************************************************DPlane类******************************************************/

/**
 * @brief 重载赋值运算符
 * @param c :
 * @return
 */
DPlane& DPlane::operator=( const DPlane& c ){
	if(&c == this)
		return *this;
	x = c.x;
	y = c.y;
	return *this;
}

/**
 * @brief 重载运算符
 * @param adder : 加数
 * @return
 */
DPlane& DPlane::operator+=( const DPlane& adder  ){
	x += adder.x;
	y += adder.y;
	return *this;
}

/**
 * @brief 重载运算符
 * @param subber : 减数
 * @return
 */
DPlane& DPlane::operator-=( const DPlane& subber )
{
	x -= subber.x;
	y -= subber.y;
	return *this;
}

/**
 * @brief 重载运算符
 * @param modulus ：乘数
 * @return
 */
DPlane& DPlane::operator*=( const double& modulus)
{
	x *= modulus;
	y *= modulus;
	return *this;
}

/**
 * @brief 重载运算符
 * @param modulus ： 除数
 * @return
 */
DPlane& DPlane::operator/=( const double& modulus)
{
	x /= modulus;
	y /= modulus;
	return *this;
}

/**
 * @brief 平面旋转
 * @param source ： 旋转中心
 * @param aAngle ： 旋转角度，逆时针旋转角度为正，顺时针旋转角度为负
 * @return
 */
DPlane& DPlane::revolve(const DPlane& source, double aAngle)
{
	double x1 = x - source.x,
	       y1 = y - source.y;
	double cs = cos(aAngle * M_RAD), ss = sin(aAngle * M_RAD);  //转换为弧度
	x = x1 * cs - y1 * ss + source.x;
	y = x1 * ss + y1 * cs + source.y;
	return *this;
}

/**
 * @brief 重载运算符
 * @param one
 * @param two
 * @return
 */
DPlane operator - ( const DPlane& one, const DPlane& two)
{
	return DPlane( one.x - two.x, one.y - two.y);
}

/**
 * @brief 重载运算符
 * @param one
 * @param two
 * @return
 */
DPlane operator + ( const DPlane& one, const DPlane& two)
{
	return DPlane( one.x + two.x, one.y + two.y);
}

/**
 * @brief 重载运算符
 * @param one
 * @param two
 * @return
 */
int operator ==( const DPlane& one, const DPlane& two)
{
	if( fabs(one.x - two.x) <= 1e-4 &&
	        fabs(one.y - two.y) <= 1e-4    ) return 1; //修改精度为1e-4，原为1e-5

	return 0;
}

/**
 * @brief 重载运算符
 * @param one
 * @param two
 * @return
 */
int operator !=( const DPlane& one, const DPlane& two)
{
	if( fabs(one.x - two.x) > 1e-4 ||
	        fabs(one.y - two.y) > 1e-4    ) return 1;  //修改精度为1e-4，原为1e-5

	return 0;
}

/*****************************************************DPoint类******************************************************/

/**
 * @brief 默认构造函数
 */
DPoint::DPoint()
{
    x = y = z = a4 = a5 = a6 = a7 = a8 = 0;
}

/**
 * @brief 构造函数
 * @param b
 */
DPoint::DPoint(const DPoint& b )
{
    x = b.x;
    y = b.y;
    z = b.z;
    a4 = b.a4;
    a5 = b.a5;
    a6 = b.a6;
    a7 = b.a7;
    a8 = b.a8;
}

/**
 * @brief 构造函数
 * @param x1
 * @param y1
 * @param z1
 * @param c4
 * @param c5
 * @param c6
 * @param c7
 * @param c8
 */
DPoint::DPoint(double x1, double y1, double z1, double c4, double c5, double c6, double c7, double c8)
{
    x = x1;
    y = y1;
    z = z1;
    a4 = c4;
    a5 = c5;
    a6 = c6;
    a7 = c7;
    a8 = c8;
}

/**
 * @brief 四舍五入
 * @param rount_type ： round_type = 0 : 0.05um精度  round_type = 1 ： 0.5um精度
 */
void DPoint::Round(uint8_t rount_type)
{
    double precision = 0.00005;   //精度 0.05um

    if(rount_type != 0)
    {
        precision = 0.0005;
    }

    if( fabs(x) > 1e-6 )
    {
        if( x < 0 ) x -= precision;
        else      x += precision;
    }

    if( fabs(y) > 1e-6 )
    {
        if( y < 0 ) y -= precision;
        else      y += precision;
    }

    if( fabs(z) > 1e-6 )
    {
        if( z < 0 ) z -= precision;
        else      z += precision;
    }

    if( fabs(a4) > 1e-6 )
    {
        if( a4 < 0 ) a4 -= precision;
        else      a4 += precision;
    }

    if( fabs(a5) > 1e-6 )
    {
        if( a5 < 0 ) a5 -= precision;
        else      a5 += precision;
    }

    if( fabs(a6) > 1e-6 )
    {
        if( a6 < 0 ) a6 -= precision;
        else      a6 += precision;
    }

    if( fabs(a7) > 1e-6 )
    {
        if( a7 < 0 ) a7 -= precision;
        else      a7 += precision;
    }

    if( fabs(a8) > 1e-6 )
    {
        if( a8 < 0 ) a8 -= precision;
        else      a8 += precision;
    }
}

/**
 * @brief 获取单轴坐标
 * @param index
 * @return
 */
double DPoint::GetAxisValue(int index) const{
	const double *val = &this->x;
	return val[index];
}


/**
 * @brief 重载赋值运算符
 * @param value
 * @return
 */
DPoint& DPoint::operator=( const double& value)
{
    x = y = z = a4 = a5 = a6 = a7 = a8 = value;
    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param a1
 * @return
 */
DPoint& DPoint::operator=( const DPoint& a1)
{
	if(&a1 == this)
		return *this;
    x = a1.x;
    y = a1.y;
    z = a1.z;
    a4 = a1.a4;
    a5 = a1.a5;
    a6 = a1.a6;
    a7 = a1.a7;
    a8 = a1.a8;
    return *this;
}

/**
 * @brief 值拷贝函数
 * @param c
 */
void DPoint::CopyData(DPoint & c) const
{
    c.x = x;
    c.y = y;
    c.z = z;
    c.a4 = a4;
    c.a5 = a5;
    c.a6 = a6;
    c.a7 = a7;
    c.a8 = a8;
}

/**
 * @brief 重载赋值运算符
 * @param adder
 * @return
 */
DPoint& DPoint::operator += ( const DPoint& adder )
{
    x += adder.x;
    y += adder.y;
    z += adder.z;
    a4 += adder.a4;
    a5 += adder.a5;
    a6 += adder.a6;
    a7 += adder.a7;
    a8 += adder.a8;
    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param subber
 * @return
 */
DPoint& DPoint::operator -= ( const DPoint& subber )
{
    x -= subber.x;
    y -= subber.y;
    z -= subber.z;
    a4 -= subber.a4;
    a5 -= subber.a5;
    a6 -= subber.a6;
    a7 -= subber.a7;
    a8 -= subber.a8;
    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param modulus
 * @return
 */
DPoint& DPoint::operator *= ( const double& modulus )
{
    x *= modulus;
    y *= modulus;
    z *= modulus;
    a4 *= modulus;
    a5 *= modulus;
    a6 *= modulus;
    a7 *= modulus;
    a8 *= modulus;
    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param modulus
 * @return
 */
DPoint& DPoint::operator /= ( const double& modulus )
{
    x /= modulus;
    y /= modulus;
    z /= modulus;
    a4 /= modulus;
    a5 /= modulus;
    a6 /= modulus;
    a7 /= modulus;
    a8 /= modulus;
    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param one
 * @param two
 * @return
 */
DPoint operator - ( const DPoint& one, const DPoint& two )
{
    DPoint result;
    result.x = one.x - two.x;
    result.y = one.y - two.y;
    result.z = one.z - two.z;
    result.a4 = one.a4 - two.a4;
    result.a5 = one.a5 - two.a5;
    result.a6 = one.a6 - two.a6;
    result.a7 = one.a7 - two.a7;
    result.a8 = one.a8 - two.a8;
    return result;
}

/**
 * @brief 重载赋值运算符
 * @param one
 * @param two
 * @return
 */
DPoint operator + ( const DPoint& one, const DPoint& two )
{
    DPoint result;
    result.x = one.x + two.x;
    result.y = one.y + two.y;
    result.z = one.z + two.z;
    result.a4 = one.a4 + two.a4;
    result.a5 = one.a5 + two.a5;
    result.a6 = one.a6 + two.a6;
    result.a7 = one.a7 + two.a7;
    result.a8 = one.a8 + two.a8;
    return result;
}

/**
 * @brief 重载赋值运算符
 * @param one
 * @param two
 * @return
 */
bool operator == ( const DPoint& one, const DPoint& two )
{
    return      ( fabs(one.x - two.x) <= 1e-4 &&
                  fabs(one.y - two.y) <= 1e-4 &&
                  fabs(one.z - two.z) <= 1e-4 &&
                  fabs(one.a4 - two.a4) <= 1e-4 &&
                  fabs(one.a5 - two.a5) <= 1e-4 &&
                  fabs(one.a6 - two.a6) <= 1e-4 &&
                  fabs(one.a7 - two.a7) <= 1e-4 &&
                  fabs(one.a8 - two.a8) <= 1e-4
                );
}

/**
 * @brief 重载赋值运算符
 * @param one
 * @param two
 * @return
 */
bool operator!= ( const DPoint& one, const DPoint& two )
{
    return !( one == two );
}


/*****************************************************DPointChn类******************************************************/

/**
 * @brief 默认构造函数
 */
DPointChn::DPointChn()
{
    memset(m_df_point, 0x00, sizeof(double)*kMaxAxisChn);
}

/**
 * @brief 构造函数
 * @param b
 */
DPointChn::DPointChn(const DPointChn& b )
{
	for(int i = 0; i < kMaxAxisChn; i++){
		m_df_point[i] = b.m_df_point[i];
	}
}

/**
 * @brief 构造函数
 * @param point : 轴坐标指针

 */
DPointChn::DPointChn(double x, double y, double z)
{
	memset(m_df_point, 0x00, sizeof(double)*kMaxAxisChn);

	m_df_point[0] = x;
	m_df_point[1] = y;
	m_df_point[2] = z;

}

/**
 * @brief 四舍五入
 * @param rount_type ： round_type = 0 : 0.05um精度  round_type = 1 ： 0.5um精度
 */
void DPointChn::Round(uint8_t rount_type)
{
    double precision = 0.00005;   //精度 0.05um

    if(rount_type != 0)
    {
        precision = 0.0005;
    }

    for(int i = 0; i < kMaxAxisChn; i++){
        if( fabs(this->m_df_point[i]) > 1e-6 )
        {
            if( m_df_point[i] < 0 )
            	m_df_point[i] -= precision;
            else
            	m_df_point[i] += precision;
        }
    }

}

/**
 * @brief 获取单轴坐标
 * @param index : 轴顺序号，从0开始
 * @return
 */
double DPointChn::GetAxisValue(int index){

	return m_df_point[index];
}

/**
 * @brief 设置单轴坐标
 * @param index  : 轴顺序号，从0开始
 * @param value : 轴坐标
 */
void DPointChn::SetAxisValue(int index, double value){
	if(index < kMaxAxisChn)
		m_df_point[index] = value;
}


/**
 * @brief 重载赋值运算符
 * @param value
 * @return
 */
DPointChn& DPointChn::operator=( const double& value)
{
	for(int i = 0; i < kMaxAxisChn; i++){
		m_df_point[i] = value;
	}
    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param a1
 * @return
 */
DPointChn& DPointChn::operator=( const DPointChn& a1)
{
	if(&a1 == this)
		return *this;

	memcpy(m_df_point, a1.m_df_point, sizeof(double)*kMaxAxisChn);
//	for(int i = 0; i < kMaxAxisChn; i++){
//		m_df_point[i] = a1.m_df_point[i];
//	}
    return *this;
}

/**
 * @brief 值拷贝函数
 * @param c
 */
void DPointChn::CopyData(DPointChn & c) const
{

	memcpy(c.m_df_point, m_df_point, sizeof(double)*kMaxAxisChn);
//	for(int i = 0; i < kMaxAxisChn; i++){
//		c.m_df_point[i] = m_df_point[i];
//	}
}

/**
 * @brief 重载赋值运算符
 * @param adder
 * @return
 */
DPointChn& DPointChn::operator += ( const DPointChn& adder )
{
	for(int i = 0; i < kMaxAxisChn; i++){
		m_df_point[i] += adder.m_df_point[i];
	}

    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param subber
 * @return
 */
DPointChn& DPointChn::operator -= ( const DPointChn& subber )
{
	for(int i = 0; i < kMaxAxisChn; i++){
		m_df_point[i] -= subber.m_df_point[i];
	}

    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param modulus
 * @return
 */
DPointChn& DPointChn::operator *= ( const double& modulus )
{
	for(int i = 0; i < kMaxAxisChn; i++){
		m_df_point[i] *= modulus;
	}
    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param modulus
 * @return
 */
DPointChn& DPointChn::operator /= ( const double& modulus )
{
	for(int i = 0; i < kMaxAxisChn; i++){
		m_df_point[i] /= modulus;
	}

    return *this;
}

/**
 * @brief 重载赋值运算符
 * @param one
 * @param two
 * @return
 */
DPointChn operator - ( const DPointChn& one, const DPointChn& two )
{
	DPointChn result;
	for(int i = 0; i < kMaxAxisChn; i++){
		result.m_df_point[i] = one.m_df_point[i] - two.m_df_point[i];
	}

    return result;
}

/**
 * @brief 重载赋值运算符
 * @param one
 * @param two
 * @return
 */
DPointChn operator + ( const DPointChn& one, const DPointChn& two )
{
	DPointChn result;
	for(int i = 0; i < kMaxAxisChn; i++){
		result.m_df_point[i] = one.m_df_point[i] + two.m_df_point[i];
	}

    return result;
}

/**
 * @brief 重载赋值运算符
 * @param one
 * @param two
 * @return
 */
bool operator == ( const DPointChn& one, const DPointChn& two )
{
	for(int i = 0; i < kMaxAxisChn; i++){
		if(fabs(one.m_df_point[i] - two.m_df_point[i]) > 1e-4)
			return false;
	}
    return  true;
}

/**
 * @brief 重载赋值运算符
 * @param one
 * @param two
 * @return
 */
bool operator!= ( const DPointChn& one, const DPointChn& two )
{
    return !( one == two );
}


/*********************************************************************全局函数实现*******************************************************************/
/**
 * @brief 将三维点坐标映射为二维平面坐标
 * @param p ： 三维点
 * @param plane ： 指定平面
 * @return 平面坐标
 */
DPlane Point2Plane( const DPoint& p, int plane){
	DPlane ret(p.x, p.y);

	switch( plane )
	{
	case PLANE_YZ:
		ret.x = p.y;
		ret.y = p.z;
		break;

	case PLANE_ZX:
		ret.x = p.z,
		ret.y = p.x;
		break;
	}

	return ret;
}

/**
 * @brief 将三维点坐标映射为二维平面坐标
 * @param p ：  多维维点
 * @param plane ： 指定平面
 * @return 平面坐标
 */
DPlane Point2Plane( const DPointChn& p, int plane){
	DPlane ret(p.m_df_point[0], p.m_df_point[1]);

	switch( plane )
	{
	case PLANE_YZ:
		ret.x = p.m_df_point[1];
		ret.y = p.m_df_point[2];
		break;

	case PLANE_ZX:
		ret.x = p.m_df_point[2],
		ret.y = p.m_df_point[0];
		break;
	}

	return ret;
}

/**
 * @brief 将二维平面坐标转换为三维点坐标
 * @param p ：平面点
 * @param[out] ret : 返回的三维点
 * @param plane ： 指定平面
 */
void Plane2Point( const DPlane& p, DPoint& ret, int plane){
	switch( plane )
	{
	case PLANE_XY:
		ret.x = p.x,
		ret.y = p.y;
		break;

	case PLANE_YZ:
		ret.y = p.x,
		ret.z = p.y;
		break;

	case PLANE_ZX:
		ret.x = p.y,
		ret.z = p.x;
		break;
	}
}

/**
 * @brief 将二维平面坐标转换为三维点坐标
 * @param p ：平面点
 * @param[out] ret : 返回的三维点
 * @param plane ： 指定平面
 */
void Plane2Point( const DPlane& p, DPointChn& ret, int plane){
	switch( plane )
	{
	case PLANE_XY:
		ret.m_df_point[0] = p.x,
		ret.m_df_point[1] = p.y;
		break;

	case PLANE_YZ:
		ret.m_df_point[1] = p.x,
		ret.m_df_point[2] = p.y;
		break;

	case PLANE_ZX:
		ret.m_df_point[0] = p.y,
		ret.m_df_point[2] = p.x;
		break;
	}
}

/**
 * @brief 在平面内计算一个向量的方向角，与X轴正向的夹角
 * @param end : 向量终点
 * @param start : 向量起点
 * @return 向量方向角[-pi,pi]，单位：弧度
 */
double GetVectAngle(const DPlane& end, const DPlane& start){

	double   dx = end.x - start.x,
			 dy = end.y - start.y;
	double   dl = sqrt(dx * dx + dy * dy),
			 angle = 0;

	if( dl < EPSINON )   return 0;

	double cos = dx / dl;


	if( (cos-1.0) >= -EPSINON)
	{
		return 0;
	}

	if((cos+1.0) < EPSINON)
	{
		return M_PI;
	}

	angle = acos(cos);

	if( angle > M_PI ) angle = 2 * M_PI - angle;

	if(dy < -1e-4)     angle *= -1;

	return angle;
}

/**
 * @brief 在平面内计算一个向量的长度，即两点间距离
 * @param end : 向量终点
 * @param start : 向量起点
 * @return
 */
double GetVectLength(const DPlane &end, const DPlane &start){
	double   dx = end.x - start.x,
			 dy = end.y - start.y;
	double   dl = sqrt(dx * dx + dy * dy);
	return dl;
}

/**************************************************************TPlanSimulator类*********************************************************************/
//TPlanSimulator::TPlanSimulator(int freq, int acc){
//	this->m_n_acc_target = acc;
//	this->m_n_freq = freq;
//	m_n_period_count = 0;
//	Reset();
//}
//
//TPlanSimulator::~TPlanSimulator(){
//
//}
//
//void TPlanSimulator::Reset(){
//	m_n_period_count = 0;
//	m_n_dist_cur = 0;
//	m_n_vel_cur = 0;
//	m_n_acc_period = 0;
//	m_n_uni_period = 0;
//	m_n_dcc_period = 0;
//	m_n_total_period = 0;
//	m_flag_dir = true;
//}
//
///**
// * @brief 启动，设置距离和目标速度，并计算相关参数，进行速度规划
// * @param dist ： 目标距离，单位：plus数量
// * @param vel ： 目标速度，单位：转每秒
// */
//void TPlanSimulator::Start(int dist, int vel){
//	this->m_n_dist_target = abs(dist);
//	this->m_n_vel_target = vel*2^23/m_n_freq;  //单位转换，将速度单位转换为：plus/T，每周期脉冲数
//	if(dist < 0)
//		m_flag_dir = false;
//
//	Reset();
//
//	int da = m_n_vel_target*m_n_vel_target/(2*m_n_acc_target);  //s=v^2/2a
//	if(2*da >= m_n_dist_target){//距离不够，达不到目标速度，取消匀速阶段，调整目标速度
//		da = m_n_dist_target/2;
//		m_n_vel_target = sqrt(2*m_n_acc_target*da);
//
//		m_n_uni_period = 0;
//	}else{
//		m_n_uni_period = (m_n_dist_target-2*da)/m_n_vel_target;
//	}
//	m_n_acc_period = m_n_vel_target/m_n_acc_target +1;
//	m_n_dcc_period = m_n_acc_period;
//	m_n_total_period = m_n_acc_period+m_n_dcc_period+m_n_uni_period;
//}
//
///**
// * @brief 获取每个周期的距离和速度
// * @param dist:返回每个周期的运动距离，单位：脉冲数
// * @return 插补完成返回true，否则返回false
// */
//bool TPlanSimulator::GetPos(int &dist){
//	if(m_n_period_count < m_n_acc_period){//加速阶段
//		m_n_vel_cur += m_n_acc_target;
//
//	}
//	else if(m_n_period_count > (m_n_acc_period+m_n_uni_period)){ //减速阶段
//		m_n_vel_cur -= m_n_acc_target;
//	}
//	m_n_dist_cur += m_n_vel_cur;
//
//	dist = m_n_vel_cur;
//
//	if(m_n_dist_cur >= m_n_dist_target){
//		dist -= (m_n_dist_cur-m_n_dist_target);
//		m_n_dist_cur = m_n_dist_target;
//	}
//
//	if(!m_flag_dir)
//		dist *= -1;
//
//	if(++m_n_period_count >= m_n_total_period ||
//			m_n_dist_cur == m_n_dist_target	)
//		return true;
//	return false;
//}
