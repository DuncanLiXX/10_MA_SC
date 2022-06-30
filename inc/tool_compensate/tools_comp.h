/* ------------------------------------------------------------------------*/
/*                                                                         */
/*   AE_TOOLS.H                                                            */
/*                                                                         */
/*   Copyright (c) 2011.11.11 by OuDeSiShuKong                               */
/*   All Rights Reserved.                                                  */
/*                                                                         */
/*   defines the classes                                                   */
/*   CompensateTroop                                                       */
/*                                                                         */
/* ------------------------------------------------------------------------*/


#include "list_buffer.h"
#include "compile_message.h"

//extern char g_compensateError0; // wch 2012.06.15

#ifndef __CompensateTroop
#define __CompensateTroop

//const int
//xyPlane =         0,
//yzPlane =         1,
//zxPlane =         2;

//class DataTroop;
class GeometryRec;
class CompensateTroop   //主要完成刀具补偿
{
public:
	CompensateTroop();
	virtual ~CompensateTroop();
	void SetToollistbuffer(ToolCompMsgList* m_p_list){m_p_toolcomp_list_msg=m_p_list;}
	int Compensate();
	int ResetAllDatas();
	int GetCount(){return this->m_p_toolcomp_list_msg->GetLength();}
	short GetToolCompensateState(){return tool_compensate_state;}
	void setStartCompState();

	/**************车床刀补函数********/
	int isCompensateEndData(int type);
	int isRunData(int type);
	int isPointLineData(int type);
	int isNoCompensateData(int type);
	int getDataType(int index);
	int getRunDataIndex(int from);
	int getCompensateEndDataIndex(int from);
	int notCompensateDataCount(int from);
	int getCompensateDir(int index);
	double getCompensateRadius(int index);
	int compensateRadiusValid(double r);
	int hasCompensateRadius(int index);
	DPointChn getDataTarget(GeometryRec      pdata);
	DPointChn getDataTarget(int index);
	GeometryRec GetADataAt(int indx);
	bool GetAPlaneMoveData(int indx,int num,GeometryRec& data);
	bool GetAMoveData(int indx,int num,GeometryRec& data);
	int RefreshDataAt(int indx, GeometryRec data);
	bool GetFeedLineBlockAt(uint32_t indx,double &feed,uint64_t &line,bool &block); // 

	LineRec getLineRec(DPointChn& point, int index);
	double getLatheOffset1(DataLine& line, ToolRec& tool);
	double getLatheOffset2(DataLine& line, ToolRec& tool);
	int modifyLineCross(LineRec& line1, LineRec& line2);
	void latheCompensateHandle(int i1, int i2);
	int latheCompensate();

	int InsertErrorData(int err_falg, int index=0);
	/*******************************************/
	
	enum {  COMPTYPES = 5 };
//	GeometryRec* at( int aIndex );
//	GeometryRec* takeItem();
	virtual void empty();
	void setStart(const DPointChn& );
private:
	int point_point(PointRec& data1, PointRec&  data2);
	int point_line(PointRec& data1, LineRec& data2);
	int point_line(LineRec& data1, PointRec& data2);
	int point_arc(PointRec& data1, ArcRec& data2);
	int point_arc(ArcRec& data1, PointRec& data2);
	int point_circle(PointRec& data1, CircleRec& data2);
	int point_circle(CircleRec& data1, PointRec& data2);
	int point_end(PointRec& data1 );

	int line_line(LineRec& data1, LineRec& data2);
	int line_arc(LineRec& data1, ArcRec& data2);
	int line_arc(ArcRec& data1, LineRec& data2);
	int line_cir(LineRec& data1, CircleRec& data2);
	int line_cir(CircleRec& data1, LineRec& data2);
	int line_end(LineRec& data1 );

	int arc_arc( ArcRec& data1, ArcRec& data2   );
	int arc_cir( ArcRec& data1, CircleRec& data2);
	int arc_cir( CircleRec& data1, ArcRec& data2);
	int arc_end( ArcRec& data1 );

	int cir_cir( CircleRec& data1, CircleRec& data2);
	int cir_end( CircleRec& );
	int findNextMove(double x, double y);
	double GetLength2(double* point1, double* point2);
	LineRec pointtoline(DPointChn& point, PointRec& data1);
	double  lathe_tool_radius(const ToolRec& tool);
	double  lathe_tool_dir(const ToolRec& tool);
	double  tool_radius(const ToolRec& tool);
	ToolOffset tool_offset(ToolRec& tool );

private:
	char select_type[COMPTYPES][COMPTYPES];
	short tool_compensate_state ; //0 --无刀补 ；1 --刀补建立；// 2 --刀补进行中  3 --刀补撤消 4--撤消前一段  其它 --异常，无刀补   -1--不用判断刀补
	short tool_compensate_type;  //0圆弧插入,1为直线插入
	char TypeFlag;                      ///1--缩短型0----为非缩短型号2012.2.2
	double PreAngle,AfterAngle;
	DataLine *vecline1, *vecline2,*vecline3, *vecline4;
//	DPoint   m_t_target_old, m_t_target, m_t_target_new; //2012.2.15
	DPointChn   m_t_Target_old, m_t_Target, m_t_Target_new; //2012.2.15

	ToolCompMsgList* m_p_toolcomp_list_msg;

public:	
	char g_compensateError; // wch 2012.06.15

private:
	virtual void freeItem(void* item)
	{
		delete (GeometryRec* )item;
	}
};
#endif

//DPlane varyPlane( const DPoint&, int plane);
DPlane VaryPlane( const DPointChn&, int plane);
void Plane2Point( const DPlane&, DPoint&, int plane);
double arcmath(const DPlane& point, const DPlane& center);
double arcmath1(const DPlane& point, const DPlane& center);
//DPoint arcmath2(DPoint centerP, double radius,double angle,char plane,DPoint startP, DPoint offP);
double GetAngle(const DPlane& source,const DPlane& target,const DPlane& center,int dir);
int getQuadrant(const DPlane& point, const DPlane& center, int dir);
void getArcField(const DataArc& arc, DPlane& min, DPlane& max );
//void ArcAdjust0(DataArc&, int flag);
void ArcAdjust(DataArc&, int flag);
void ArcAdjust(DataArc& );
void ArcAdjust(DataCircle&);
int stdline(const DPlane& p1, const DPlane& p2, double* line);
int stdcircle(const DPlane & center, double r, double * cir);
void vertline(const double *line1, const DPlane& point, double *line2);
int linecrfline(const DataLine & l1, const DataLine & l2, DPlane & point);
int linecrfline(double * line1, double * line2, DPlane & point);
int linecrfcir(const DataLine & line, double * circle, DPlane * point);
int circrfcir(double * cir1, double * cir2, DPlane * point);
double GetVecScalarProduct(DPlane &vec1, DPlane &vec2); //added by burke 20120316计算两个向量的点积
int  lineVector(const DataLine& line, DPlane& vector);
int  getDirection( const DPlane& point1, const DPlane& point2, const DPlane& point3  );

int veccrfvec( DataLine*  line1,  DataLine* line2);//终点补偿矢量求交

//double Round(double);

//double getPolarRadius(DPoint,DPoint,char);
//double getPolarAngle(DPoint,char);
//double getPolarAngle2(DPoint,DPoint,char);