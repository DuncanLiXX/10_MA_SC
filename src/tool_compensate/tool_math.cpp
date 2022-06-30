/*------------------------------------------------------------*/
/* filename -       AE_tmath.cpp                              */
/*                                                            */
/*    Copyright (c) 2011.11.11 by OuDeSiShuKong                 */
/*    All Rights Reserved.                                    */
/* function(s)                                                */
/*    Mathe Function for tool compensate                      */
/*------------------------------------------------------------*/

//#define Uses_DPlane
//#define Uses_DataMessage
//#include <Ods.h>
#include "math.h"
#include "tool_comp_data.h"
#include "tools_comp.h"


double Sqrt( double ss )
{
	if( ss < EPSINON) return 0;

	return sqrt(ss);
}

double MyRoundPos( double ss,double s0 )
{
	if(ss > 1e-8) 
		return (ss+s0);
    else if(ss < -(1e-8))
		return (ss-s0);
	return (ss);
}

/**
 *@brief 输出角以2*PI为周期，确定离当前角最近的角
 *@param double Ang0 当前角，double& Ang1 原始输出角
 *return 输出角与当前角的最小角度差的绝对值
 */
double NearestAng(double Ang0, double& Ang1)
{
/*
	while(Ang1-Ang0 < -M_PI)
		Ang1 += 2*M_PI;
	while(Ang1-Ang0 > M_PI)
		Ang1 -= 2*M_PI;
	return fabs(Ang1-Ang0);
*/	
	while(Ang1-Ang0 < -180.0)
		Ang1 += 360.0;
	while(Ang1-Ang0 > 180.0)
		Ang1 -= 360.0;
	return fabs(Ang1-Ang0);
}

double AngChangeTo0_360(double Angle)
{
    double Ang0 = Angle;
    if(Ang0<0)
    {
       while(Ang0<0) Ang0 += 360.0;	   
	}
	else if(Ang0>=0)
	{
       while(Ang0>=360.00) Ang0 -= 360.0;	
	}
	return Ang0;
}

/**
 *@brief 
 *@param 
 *return 
 */
short AngleJudgeBySoftLmt(double* lmtAngle, double& Ang1)
{
    if(Ang1<lmtAngle[0])   //小于付限位坐标
    {
      while(Ang1<lmtAngle[0])
	  	 Ang1 += 360.0;
	  if(Ang1 > lmtAngle[1])
	  	 return 0;
	  return 1;

	}
	else if(Ang1 > lmtAngle[1])  //大于正限位坐标
	{
      while(Ang1>lmtAngle[1])
	  	 Ang1 -= 360.0;
	  if(Ang1 < lmtAngle[0])
	  	 return 0;
	  return 1;
	}
	else
	    return 1;
    
}


/** 
 *@brief 根据受限轴(软限位值)确定轴的移动目标位置
 *@param 
 *@return 1-- 选第一组  2--选第二组    3--无法确定
 */
 
short JudgeRotateAngBySoftlmtdistance(double* lmtAngle, double Ang1, double Ang2)
{
   double Ang1Distance,Ang2Distance;
   double fpos1,fpos2;

   fpos1 = fabs(Ang1 - lmtAngle[0]);
   fpos2 = fabs(Ang1 - lmtAngle[1]);
   Ang1Distance = (fpos1<=fpos2)? fpos1:fpos2;
   
   fpos1 = fabs(Ang2 - lmtAngle[0]);
   fpos2 = fabs(Ang2 - lmtAngle[1]);
   Ang2Distance = (fpos1<=fpos2)? fpos1:fpos2;
	
   if(Ang1Distance > Ang2Distance)
   	  return 1;
   else if(Ang1Distance < Ang2Distance)
   	  return 2;
   else 
   	  return 3;
    
}

/** 
 *@brief 进行移动判定以确定旋转轴最终输出角(以弧度为单位),Axis1为主动轴(第一旋转轴)，Axis2为从动轴(第二旋转轴)
 *@param 
 *@return 无
 */
void JudgeRotateAngByMovedistance(double Axis1Angle0, double Axis2Angle0, double Axis1Angle1, double Axis2Angle1, double Axis1Angle2, 
                          double Axis2Angle2, double& Axis1AngleOut, double& Axis2AngleOut)
{
	double Axis1AngleNew1 = Axis1Angle1, Axis1AngleNew2 = Axis1Angle2;
	double Axis1MinGap1 = fabs(Axis1AngleNew1 - Axis1Angle0);  // NearestAng(Axis1Angle0,Axis1AngleNew1);//优先比较主动轴
	double Axis1MinGap2 = fabs(Axis1AngleNew2 - Axis1Angle0);  //  NearestAng(Axis1Angle0,Axis1AngleNew2);
	
	if(Axis1MinGap1<Axis1MinGap2)
	{//采用第一组角
		Axis1AngleOut = Axis1AngleNew1;
		Axis2AngleOut = Axis2Angle1;
//		NearestAng(Axis2Angle0,Axis2AngleOut);
		return;
	} 
	else if(Axis1MinGap1>Axis1MinGap2)
	{//采用第二组角
		Axis1AngleOut = Axis1AngleNew2;
		Axis2AngleOut = Axis2Angle2;
//		NearestAng(Axis2Angle0,Axis2AngleOut);
		return;
	} 
	else 
	{//主动轴角度差相等，判断从动轴角度差
		double Axis2AngleNew1 = Axis2Angle1, Axis2AngleNew2 = Axis2Angle2;
		double Axis2MinGap1 = fabs(Axis2AngleNew1 - Axis2Angle0);  // NearestAng(Axis2Angle0, Axis2AngleNew1);
		double Axis2MinGap2 = fabs(Axis2AngleNew1 - Axis2Angle0);  // NearestAng(Axis2Angle0, Axis2AngleNew2);
		
		if(Axis2MinGap1<Axis2MinGap2) 
		{//采用第一组角
			Axis1AngleOut = Axis1AngleNew1;
			Axis2AngleOut = Axis2AngleNew1;
			return;
		} 
		else if(Axis2MinGap1>Axis2MinGap2) 
		{//采用第二组角
			Axis1AngleOut = Axis1AngleNew2;
			Axis2AngleOut = Axis2AngleNew2;
			return;
		}
		else 
		{
			//主、从动轴角度差相等，判断主动轴角度是否更接近0度(360度)
		//	double CmpAng=0;
		//	double Axis1Tmp1 = Axis1AngleNew1, Axis1Tmp2 = Axis1AngleNew2;
			Axis1MinGap1 = fabs(Axis1AngleNew1);  // NearestAng(CmpAng,Axis1Tmp1);
			Axis1MinGap2 = fabs(Axis1AngleNew2);  // NearestAng(CmpAng,Axis1Tmp2);
			
			if(Axis1MinGap1<Axis1MinGap2) 
			{//采用第一组角
			    Axis1AngleOut = Axis1AngleNew1;
				Axis2AngleOut = Axis2AngleNew1;
				return;
			} 
			else if(Axis1MinGap1>Axis1MinGap2) 
			{//采用第二组角
				Axis1AngleOut = Axis1AngleNew2;
				Axis2AngleOut = Axis2AngleNew2;
				return;
			} 
			else 
			{
	//			double Axis2Tmp1 = Axis2AngleNew1, Axis2Tmp2 = Axis2AngleNew2;
				Axis2MinGap1 = fabs(Axis2AngleNew1);   // NearestAng(CmpAng,Axis2Tmp1);
				Axis2MinGap2 = fabs(Axis2AngleNew2);   // NearestAng(CmpAng,Axis2Tmp2);//判断从动轴角度是否更接近0度(360度)
				
				if(Axis2MinGap1<Axis2MinGap2) 
				{
					Axis1AngleOut = Axis1AngleNew1;
					Axis2AngleOut = Axis2AngleNew1;
					return;
				} 
				else if(Axis2MinGap1>Axis2MinGap2) 
				{
					Axis1AngleOut = Axis1AngleNew2;
					Axis2AngleOut = Axis2AngleNew2;
					return;
				} 
				else 
				{
					Axis1AngleOut = Axis1AngleNew1;
					Axis2AngleOut = Axis2AngleNew1;
					return;
				}
			}
		}
	}
}

//矩阵乘法，4*4矩阵
int MatrixMultiplication(double aa[][4],double bb[][4],double (*cc)[4])
{
   int i,j,k; 
   
   for(i=0; i<4; i++)
   {
     for(j=0; j<4; j++)
     {
      cc[i][j] = aa[i][0]*(bb[0][j]) + aa[i][1]*(bb[1][j]) + aa[i][2]*(bb[2][j]) + aa[i][3]* (bb[3][j]);
	 }
  
   }    
   return 1;
}


//矩阵乘法，4*4矩阵
int MatrixMultiplication1(double *aa,double *bb,double *cc)
{
   int i,j,k,ii,jj,kk;
   double *p,*p1,*p2;

   if(!aa ||!bb||!cc) 
   	   return 0;
   
   for(i=0; i<4; i++)
   {
     for(j=0; j<4; j++)
     {
       ii = i*4 + j;
       *(cc + ii) = 0;
       for(k=0; k<4; k++)
       {
	      jj = i*4 + k;
		  kk = 4*k + j;
          *(cc+ii) +=  *(aa+jj) + *(bb+kk);
	   }
	 }
  
   }    
   return 1;
}

int getDirection(const DPlane& point1, const DPlane& point2, const DPlane& point3 )
{
	double s0, s1, s2;
	s0 = (point1.y - point2.y) * point3.x;
	s1 = (point2.y - point3.y) * point1.x;
	s2 = (point3.y - point1.y) * point2.x;
	double s3 = s0 + s1 + s2;

	if( s3 <= -1e-6 )  return  -1; /* s_circle */

	if( s3 >= 1e-6  ) return   1;  /* n_circle */

	return 0;
}

/*
DPlane varyPlane( const DPoint& p, int plane)
{
	DPlane ret(p.x, p.y);

	switch( plane )
	{
	case yzPlane:
		ret.x = p.y;
		ret.y = p.z;
		break;

	case zxPlane:
		ret.x = p.z,
		    ret.y = p.x;
		break;
	}

	return ret;
}
*/

DPlane VaryPlane( const DPointChn& p, int plane)
{
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

/*
void Plane2Point( const DPlane& p, BPoint& ret, int plane)
{
	switch( plane )
	{
	case xyPlane:
		ret.x = p.x,
		    ret.y = p.y;
		break;

	case yzPlane:
		ret.y = p.x,
		    ret.z = p.y;
		break;

	case zxPlane:
		ret.x = p.y,
		    ret.z = p.x;
		break;
	}
}
*/
//***********************************************//
//Author;zhanghui
//Purpose:-180---180
//Modify:2012.8.15   合并两个函数
//*********************************************//
double arcmath(const DPlane& point, const DPlane& center)
{
	double p1[2], c1[2];
	p1[0] = point.x,
	        p1[1] = point.y,
	                c1[0] = center.x,
	                        c1[1] = center.y;
	double   dx = p1[0] - c1[0],
	         dy = p1[1] - c1[1];
	double   dl = Sqrt(dx * dx + dy * dy),
	         angle = 0;

	if( dl < 1e-6 )   return 0;

	double cos = dx / dl;

	//if( cos >= 1.0 )
	if( (cos-1.0) >= -EPSINON)
	{
		return 0;
	}

	//if( cos <= -1.0)
	if((cos+1.0) < EPSINON)
	{
		return 180.;
	}

	angle = acos(cos);

	if( angle > M_PI ) angle = 2 * M_PI - angle;

	if(dy < -1e-4)     angle *= -1;

	return angle * 180. / M_PI;
}
//******************************************************************//
//Author:zhanghui
//Data:2012.9.24
//Purpose:计算角度范围为0--360度,仅用于计算圆弧的角度
//*****************************************************************//
double arcmath1(const DPlane& point, const DPlane& center) 
{
	double p1[2], c1[2];
	p1[0] = point.x,
	        p1[1] = point.y,
	                c1[0] = center.x,
	                        c1[1] = center.y;
	double   dx = p1[0] - c1[0],
	         dy = p1[1] - c1[1];
	double   dl = Sqrt(dx * dx + dy * dy),
	         angle = 0;

	if( dl < 1e-6 )   return 0;

	double cos = dx / dl;

	if( cos >= 1.0 )
	{
		return 0;
	}

	if( cos <= -1.0)
	{
		return 180.;
	}

	angle = acos(cos);

	if(dy < -1e-4)     angle = 2 * M_PI - angle;

	return angle * 180. / M_PI;
}

//******************************************************************//
//Author:huangxiangjun
//Data:2018.4.24
//Purpose:计算角度范围为0--360度,仅用于计算圆弧的角度
//*****************************************************************//
/*
DPoint arcmath2(DPoint centerP, double radius,double angle,char plane,DPoint startP, DPoint offP) 
{
    DPoint position;	
    double p1[2];
	DPlane center;

	position = startP;
	position += offP;
	
//	tprintf("\n plane=%d",(int)plane);
//	tprintf("\n ----->arcmath2:startP  (%f %f %f %f %f %f)",startP.x,startP.y,startP.z,startP.a4,startP.a5,startP.a6);
//	tprintf("\n ----->arcmath2:offP    (%f %f %f %f %f %f)",offP.x,offP.y,offP.z,offP.a4,offP.a5,offP.a6);
//	tprintf("\n ----->arcmath2:position(%f %f %f %f %f %f)",position.x,position.y,position.z,position.a4,position.a5,position.a6);
	if(plane == yzPlane) 
	{
       center.x = centerP.y;
       center.y = centerP.z;
	}
	else if(plane == zxPlane)	
	{
       center.x = centerP.z;
       center.y = centerP.x;
	}
	else	//xyPlane
	{
       center.x = centerP.x;
       center.y = centerP.y;
	}		

	p1[0] = radius * cos(angle*M_PI/180.) + center.x;
	p1[1] = radius * sin(angle*M_PI/180.) + center.y;	
//tprintf("\n ----->arcmath2:	p1(%f %f)",p1[0],p1[1]);

	if(plane == yzPlane) 
	{
       position.y = p1[0];
       position.z = p1[1];
	}
	else if(plane == zxPlane)	
	{
       position.z = p1[0];
       position.x = p1[1];
	}
	else	//xyPlane
	{
       position.x = p1[0];
       position.y = p1[1];
	}
//tprintf("\n ----->arcmath2:position(%f %f %f %f %f %f)",position.x,position.y,position.z,position.a4,position.a5,position.a6);

	return position;
}
*/
// 根据起点 终点 圆心 和方向  计算圆心角
double GetAngle(const DPlane& source,const DPlane& target,const DPlane& center,int nDir)
{
	double aStart = arcmath1(source, center),
	       aEnd = arcmath1(target, center);
	double angle = 0, dir = ( nDir == 0 ) ? -1 : 1; //顺时针(-1)，逆时针(1)
	angle = (aEnd - aStart) * dir;

    if(angle < -1e-6)
	{
		angle += 360.;
	}
	return fabs(angle);


}


//**************************************************//
//Purpose:用于表示坐标点在坐标系中的那个区域
//dir=0时,0->0->1->1->2->2->3->3
//dir!=0时,0->1->1->2->2->3->3->0
//**************************************************//
int getQuadrant(const DPlane& point, const DPlane& center, int dir)
{
	DPlane dd = point - center;
	int    m = (dd.x > 1e-5) ? 0 : ((dd.x < -1e-5) ? 1 : 2),
	       n = (dd.y > 1e-5) ? 0 : ((dd.y < -1e-5) ? 1 : 2);
	int8_t   flag[3][3] = { {0, 3, 3}, {1, 2, 1}, {0, 2, -1} };
	int8_t   flag1[3][3] = { {0, 3, 0}, {1, 2, 2}, {1, 3, -1} };
	return ((dir == 0) ? flag[m][n] : flag1[m][n]);
}
//*********************************************//
//Purpose:包围盒
//如果起点和终点在同一区间则起点和终点的坐标就是边界
//不在同一区间则需求出其最大点的坐标
//*********************************************//
void getArcField(const DataArc& arc, DPlane& min, DPlane& max )
{
	min = arc.source,
	max = arc.target;

    double val=0;
	if( min.x > max.x )
	{
        val = min.x, min.x= max.x, max.x=val;
	}

	if( min.y > max.y )
	{
        val = min.y, min.y= max.y, max.y=val;
	}

	if( arc.radius <= 1e-5 ) return;

	int startn = getQuadrant(arc.source, arc.center, arc.flag),
	    endn  = getQuadrant(arc.target, arc.center, arc.flag);

	if( startn == -1 || endn == -1 ) return;

	if( startn == endn )
	{
		double start = arcmath( arc.source, arc.center),
		       end = arcmath( arc.target, arc.center);
		int dir = (arc.flag == 0) ? -1 : 1;
		double angle = (end - start) * dir;

		if( angle < -EPSINON) angle += 360.0;

		if( angle < 90 ) return;
	}

	int    flag[2][4] = { {3, 0, 1, 2}, {1, 2, 3, 0} },
	m[2][4]   = { {0, 2, 1, 3}, {2, 1, 3, 0} },
	state = 0;
	DPlane field(0, 0);

	do
	{
		state = m[arc.flag][startn];
		startn = flag[arc.flag][startn];

		switch(state)
		{
		case 0:
			field.x = arc.center.x + arc.radius;

			if( max.x < field.x ) max.x = field.x;

			break;

		case 1:
			field.x = arc.center.x - arc.radius;

			if( min.x > field.x ) min.x = field.x;

			break;

		case 2:
			field.y = arc.center.y + arc.radius;

			if( max.y < field.y ) max.y = field.y;

			break;

		case 3:
			field.y = arc.center.y - arc.radius;

			if( min.y > field.y ) min.y = field.y;

			break;
		}
	}
	while( startn != endn );
}

//**************************************************//
//圆心调整
/*
void ArcAdjust0(DataArc& arc, int flag ) // flag=1 or -1
{
	DPlane mid = arc.source + arc.target;
	mid /= 2;
	DPlane dd = mid - arc.source;
	double dxy = dd.x * dd.x + dd.y * dd.y,
	       dr2 = arc.radius * arc.radius - dxy;
	double dr = Sqrt(dr2);
	double  angle = arcmath(arc.target, arc.source) * M_RAD + flag * M_PI_2;
	DPlane  move(dr * cos(angle), dr * sin(angle) );
	arc.center = mid + move;
//   arc.radius=Sqrt(dxy+dr*dr);
}
*/
//圆心与半径调整
void ArcAdjust(DataArc& arc, int flag ) // flag=1 or -1
{
	DPlane mid = arc.source + arc.target;
	mid /= 2;
	DPlane dd = mid - arc.source;
	double dxy = dd.x * dd.x + dd.y * dd.y,
	       dr2 = arc.radius * arc.radius - dxy;
	double dr = Sqrt(dr2);
	double  angle = arcmath(arc.target, arc.source) * M_RAD + flag * M_PI_2;
	DPlane  move(dr * cos(angle), dr * sin(angle) );
	arc.center = mid + move;
	arc.radius = Sqrt(dxy + dr * dr);
}
void ArcAdjust( DataArc& arc )
{
	DPlane  mid = arc.source + arc.target;
	mid /= 2;
	DPlane  dd = mid - arc.source;
	double  dxy = dd.x * dd.x + dd.y * dd.y;
	double  dr = (arc.radius * arc.radius - dxy);
	int    flag = (arc.flag == 0) ? -1 : 1; //-1顺1为逆
	double end  = arcmath(arc.target, arc.center),
	       start = arcmath(arc.source, arc.center),
	       len  = (end - start) * flag;

	if( len <=  1e-4) len += 360;

	if( (len - 180.) >= 1e-4) flag *= -1;

	if(fabs(len -180.) < 1e-4 || fabs(arc.radius-Sqrt(dxy)) < 1e-3)
	{//180度取中点为圆心
		arc.center = mid;
		return;
	}

	dr = Sqrt(dr);
	
	double  angle = arcmath(arc.target, arc.source)* M_RAD + flag * M_PI_2 ;
	dd.x = dr * cos(angle),
	   dd.y = dr * sin(angle);
	//DPlane cc = arc.center;
	arc.center = mid + dd;
}
void ArcAdjust( DataCircle& circle)
{
	double angle = arcmath(circle.center, circle.target) * M_RAD;
	DPlane adjust(circle.radius * cos(angle), circle.radius * sin(angle) );
	circle.center = circle.target + adjust;
}

/*
double Round(double x)
{
        double precision = 0.000005;
	double y = x;	
	if( fabs(y) > EPSINON)
	{
		if( y < 0 ) y -= precision;
		else      y += precision;
	}
	return y;
}
*/

/*
//获得一个点在2维极坐标空间的角度
double getPolarAngle(DPoint point, char plane)
{
       double   dx,dy,dl,angle;	   

       if(plane== yzPlane)
       {
           dx = point.y;
           dy = point.z;
       }
       else if(plane== zxPlane)
       {
           dx = point.z;
           dy = point.x;
       }
       else
       {
           dx = point.x;
           dy = point.y;
       }

	dl = Sqrt(dx * dx + dy * dy);
	
	if( dl < 1e-6 )   return 0;

	double cos = dx / dl;

	if( cos >= 1.0 )
	{
		return 0;
	}

	if( cos <= -1.0)
	{
		return 180.;
	}

	angle = acos(cos);

	if(dy < -1e-4)     angle = 2 * M_PI - angle;

	return angle * 180. / M_PI;
}
*/

//获得一个点在2维极坐标空间的角度
/*
double getPolarAngle2(DPoint point,DPoint center, char plane)
{
       double   dx,dy,dl,angle;	   

       if(plane== yzPlane)
       {
           dx = point.y - center.y;
           dy = point.z - center.z;
       }
       else if(plane== zxPlane)
       {
           dx = point.z - center.z;
           dy = point.x - center.x;
       }
       else
       {
           dx = point.x - center.x;
           dy = point.y - center.y;
       }

	dl = Sqrt(dx * dx + dy * dy);
	
	if( dl < 1e-6 )   return 0;

	double cos = dx / dl;

	if( cos >= 1.0 )
	{
		return 0;
	}

	if( cos <= -1.0)
	{
		return 180.;
	}

	angle = acos(cos);

	if(dy < -1e-4)     angle = 2 * M_PI - angle;

	return angle * 180. / M_PI;
}
*/
//获得一个点在2维极坐标空间的极半径
/*
double getPolarRadius(DPoint point,DPoint center, char plane)
{
       double   dx,dy,dl;	   

       if(plane== yzPlane)
       {
           dx = point.y - center.y;
           dy = point.z - center.z;
       }
       else if(plane== zxPlane)
       {
           dx = point.z - center.z;
           dy = point.x - center.x;
       }
       else
       {
           dx = point.x - center.x;
           dy = point.y - center.y;
       }

	dl = Sqrt(dx * dx + dy * dy);
	
	return dl;
}
*/
#ifdef USES_TOOL_COMPENSATE

//**************************************************//
//Purpose:圆和直线的标准形式
//Created:zhanghui 2011.5.7
//**************************************************//
int stdcircle(const DPlane& center, double r, double* cir)
{
	if(r < 1e-4) return 0; //2012.8.31

	cir[0] = center.x,
	         cir[1] = center.y,
	                  cir[2] = r;
	return 1;
}
int stdline(const DPlane& source, const DPlane& target, double* line)
{
	double dd = (target.y - source.y) * (target.y - source.y) + (target.x - source.x) * (target.x - source.x);

	if(dd < 1e-6)
		return 0;
	else
	{
	        double ss, sina, cosa;
		ss = fabs(dd);
		sina = (target.y - source.y) / ss;
		cosa = (target.x - source.x) / ss;
		line[0] = -sina;
		line[1] = cosa;
		line[2] = -((source.y) * cosa - (source.x) * sina);
		return 1;
	}
}

//*****************zhanghui2011.5.7*************//
void vertline(const double *line1,
              const DPlane& point, double *line2)
{
	int m = 0, i = 0;
	line2[0] = (line1[1] != 0) ? line1[1] : 0;
	line2[1] = (line1[0] != 0) ? -line1[0] : 0;
	line2[2] = -(line2[0] * point.x + line2[1] * point.y);

	if(line2[1])
		m = (line2[1] > 0) ? 1 : -1;
	else
		m = (line2[0] > 0) ? 1 : -1;

	for(i = 0; i < 3; i++)
		line2[i] *= m;
}
//*****************************************************//
//Purpose:直线和直线求交点
//Created:zhanghui2011.5.7
//Modify:zhanghui 2011.9.13
//返回:平行0,有一个交点1,重合2
//Modify:2012.8.15   合并两个函数
//*****************************************************//

int linecrfline(double* line1, double* line2, DPlane& point)
{
	double m, n, k;
	m = line2[2] * line1[1] - line2[1] * line1[2]; //c2*b1-b2*c1
	n = line2[1] * line1[0] - line2[0] * line1[1]; //b2*a1-a2*b1
	k = line1[2] * line2[0] - line2[2] * line1[0]; //c1*a2-c2*a1

	//cout<<"m = "<<m<<", n = "<<n<<", k = "<<k<<endl;
	if(fabs(n) < 1e-6) //分平行0和重合2两种情况
	{
		if(fabs(k) < 1e-6)
			return 2;
		else
			return 0;
	}
	else
	{
		point.x = m / n;
		point.y = k / n;
		return 1;
	}
}
int linecrfline(const DataLine& l1, const DataLine& l2, DPlane& point)
{
	double line1[3], line2[3];
	int ret = 0;

	if( stdline( l1.source, l1.target, line1) == 0 ) return 0;

	if( stdline( l2.source, l2.target, line2) == 0 ) return 0;

	ret = linecrfline(line1, line2, point);
	return ret;
}
//***************************************************//
//Author:zhanghui
//Modify:2011.5.9
//Purpose:直线和圆求交点,0/1/2
//**************************************************//
int linecrfcir(const DataLine& line, double* circle, DPlane* point)
{
	double  line1[3], line2[3];
	DPlane  center(circle[0], circle[1]);
	double  radius = circle[2];

	if(stdline( line.source, line.target, line1) == 0 )
		return 0;

	vertline(line1, center, line2);
	DPlane mid, dd;

	if(linecrfline(line1, line2, mid) != 1 )
		return 0;

	dd = mid - center;
	double ds = Sqrt(dd.x * dd.x + dd.y * dd.y);
	double dis = ds - radius;

	if(fabs(dis) < 1e-6) //相切
	{
		point[0] = mid;
		return 1;
	}
	else if(dis > 1e-6) //相离
	{
		return 0;
	}
	else   //相交
	{
		double dr = radius * radius - ds * ds;
		dr = Sqrt(dr);
		double angle = arcmath( line.target, line.source ) * M_RAD;
		dd.x = dr * cos(angle);
		dd.y = dr * sin(angle);
		point[0] = mid + dd;
		point[1] = mid - dd;
		return 2;
	}
}
//**********************************************//
//Author:zhanghui
//Modify:2011.5.7
//Purpose:圆和圆的 交点circrf/circrfcir,0/1/2
//**********************************************//
int circrfcir(double* cir1, double* cir2, DPlane* point)
{
	DPlane   c1(cir1[0], cir1[1]),
	         c2(cir2[0], cir2[1]);
	double   r1 = cir1[2], r2 = cir2[2];
	DPlane   dd = c2 - c1, ds = c1 + c2;
	double   ss = dd.x * dd.x + dd.y * dd.y;
	double dis = (cir1[0] - cir2[0]) * (cir1[0] - cir2[0]) + (cir1[1] - cir2[1]) * (cir1[1] - cir2[1]);
	double  temp1 = dis - (r1 + r2) * (r1 + r2),
	        temp2 = (r1 - r2) * (r1 - r2) - dis;

	if(temp1 > 1e-6) //外离
	{
		return  0;
	}

	if(temp2 > 1e-6) //内离
	{
		return  0;
	}

	double   line1[3], line2[3];

	if( stdline(c1, c2, line1) == 0 )
	{
		if(fabs(r1-r2) < 1e-4)  //20131023 burke
			return 5;
		return 0;
	}

	line2[0] = 2 * dd.x,
	           line2[1] = 2 * dd.y,
	                      line2[2] = r2 * r2 - r1 * r1 - ds.x * dd.x - ds.y * dd.y;
	DPlane mid;

	if( linecrfline(line1, line2, mid) == 0 )
	{
		return 0;
	}
	else  //有交点
	{
		dd = mid - c1;
		ss = Sqrt(r1 * r1 - dd.x * dd.x - dd.y * dd.y);

		if(ss < 1e-4)   //20131023 burke
		{
			point[0] = mid;
			point[1] = mid;

			if(fabs(temp1) < 1e-6)return 3; //外切

			if(fabs(temp2) < 1e-6)return 4; //内切

			return 5;
		}
		else
		{
			double angle = arcmath(c2, c1) * M_RAD - M_PI_2; //2012.8.30
			DPlane move( ss * cos(angle), ss * sin(angle) );
			point[0] = mid + move; //出圆
			point[1] = mid - move; //入圆
			return 2;
		}
	}

}

//**************************************************//
//Author:zhanghui
//return:0---无交点 1----有交点
//Created:2012.2.10
//Purpose:干涉检测
//Modify:2012.9.23
//*************************************************//
int veccrfvec( DataLine*  line1,  DataLine* line2)
{
	DPlane point;
//	double aa, bb, mm, nn;
	double len1 = line1->GetLength(),
	       len2 = line2->GetLength();

	if((len1 < 1e-6) || (len2 < 1e-6))  return 0; ///2012.8.29

	int ret = linecrfline(*line1, *line2, point);
        
	if(ret == 0)return 0;
	else if(ret == 2)return 0;
	else
	{//存在一个交点，判断交点是否在延长线上
		double m, mm, mmm, n, nn, nnn, k, kk, kkk, p, pp, ppp;
		m = line1->target.x - line1->source.x;
		mm = (m > 1e-6) ? line1->target.x : line1->source.x; 
		mmm = (m < -1e-6) ? line1->target.x : line1->source.x;
		n = line1->target.y - line1->source.y;
		nn = (n > 1e-6) ? line1->target.y : line1->source.y; 
		nnn = (n < -1e-6) ? line1->target.y : line1->source.y;

		k = line2->target.x - line2->source.x;
		kk = (k > 1e-6) ? line2->target.x : line2->source.x; 
		kkk = (k< -1e-6) ? line2->target.x : line2->source.x;
		p = line2->target.y - line2->source.y;
		pp = (p > 1e-6) ? line2->target.y : line2->source.y; 
		ppp = (p < -1e-6) ? line2->target.y : line2->source.y;

		if(((point.x - mmm) > -1e-6) && ((point.x - mm) < 1e-6) && ((point.y - nnn) > -1e-6) && ((point.y - nn) < 1e-6))
		{
			if(((point.x - kkk) > -1e-6) && ((point.x - kk) < 1e-6) && ((point.y - ppp) > -1e-6) && ((point.y - pp) < 1e-6))
			{
				return 1;
			}
		}

		return 0;
	}
}

#endif

