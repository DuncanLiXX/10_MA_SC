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

#include "compiler_data.h"
#include "geometry_data.h"

const int
coordData     = -2,
invalidData   = -1,
pointData     = 0,
lineData      = 1,
arcData       = 2,
circleData    = 3,
rapidData     = 4,
endData       = 5,
compData      = 6,
jumpData      = 7,
errorData     = 8,
nurbsData	 = 9,
endCompensateData = 10,
codeData      = 11,
cbsData         = 12,   //三次B样条数据gonghao 20161125
smoothData    = 13;   //平滑插补开关gonghao 20161205

enum
{
    MILL_SYSTEM = 0,   //雕铣机
    CNC_SYSTEM = 1,   // 加工中心
    GRIND_SYSTEM = 2,  // 磨床
    LATHE_SYSTEM = 3, //车床
    SCREW_SYSTEM= 4,  // 螺丝机
    Z1Z2_SYSTEM = 5,  // 多Z系统
    AUTOMATION_SYSTEM = 6,  //自动化设备
    GUNCHI_SYSTEM =7          //  滚齿机系统
};




#define ERR_COMPENSATE 15 //burke 20120412 刀补错误
#define ERR_CREATETOOL 16 //burke 20120414 创建刀补失败
#define ERR_CANCELTOOL 17 //burke 20120414 撤销刀补失败
#define ERR_TOOLRADIUS 18 //	burke 20120414 刀补半径过大
#define ERR_SUBCOMPENSATE 19 //burke 20120425 刀补进行中不允许调用子程序

const int MAX_D_TOOL        = 120;
const int MAX_LATHE_TOOL = 32;
const int MAX_OFFSET_NUMBER = 14;// zhanghui 2012.09.22 //  10--->14; //6; //burke 刀补队列长度


/*
struct TroopRec
{
	void* rec;
	TroopRec* next;
	TroopRec* pre;  //gonghao 20161207改为双向链表
	TroopRec(): rec(0), next(0), pre(0) {}
};
class DataTroop
{
public:
	DataTroop(int aLimit);
	virtual ~DataTroop();
	virtual int  addItem(void* rec);
	virtual void empty();
	void* at( int aIndex);
	int   atInsert(int aIndex, void* rec);
	void* takeItem();
	int   getCount();
	bool isEmpty();
	bool isFull();
	//2012.9.1, GongLihui, G1
//	//void* headItem()
//	{
//		return head->rec;
//	}
	void* tailItem();
	//protected:
	void buildlink();
	TroopRec* head;
	TroopRec* tail;
	TroopRec* items;
	int       count;
	int       limit;
private:
	virtual void freeItem( void* item );
};

*/

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
	DataLine& offset(const ToolOffset& off);
	DataLine& length_offset(ToolOffset& offset, int flag) ;
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
//	DPoint target;
	DPointChn  Target;
	ToolRec tool;
//	DataPoint getData();
//	void setData( const DataPoint& );
	DataPoint GetData();
	void SetData( const DataPoint& );
};
struct LineRec
{
	int     flag;     // 0:line,1:screw
//	DPoint  source;
//	DPoint  target;
	DPointChn  Source;
	DPointChn  Target;
	ToolRec tool;
//	DataLine getData();
//	void setData(const DataLine& );
	DataLine GetData();
	void SetData(const DataLine& );
};
struct CircleRec
{
	int     flag;    // 0:clockwise,1:anticlockwise
	double  radius;
//	DPoint  source;
//	DPoint  target;
//	DPoint  center;
	DPointChn  Source;
	DPointChn  Target;
	DPointChn  Center;
	ToolRec tool;

//	DataCircle getData();
//	void setData(const DataCircle&);
	DataCircle GetData();
	void SetData(const DataCircle&);

};
struct ArcRec
{
	int     flag;    //0:clockwise,1:anticlockwise
	double  radius;
//	DPoint  source;
//	DPoint  target;
//	DPoint  center;
	DPointChn  Source;
	DPointChn  Target;
	DPointChn  Center;
	ToolRec tool;

//	DataArc getData();
//	void setData(const DataArc& );
	DataArc GetData();
	void SetData(const DataArc& );

};

//added by burke 20120412
struct ErrRec
{
	uint32_t errflag;
	int errType; // 1--过切； 2--刀补半径过大
};

struct ModeRec{

};

struct GeometryRec
{
	int what;
	uint16_t usFlag; //0-普通数据；1--插入数据；2--优化数据
	uint16_t usSmoothFlag;  //0--无平滑  2--样条平滑 199--非G5.1数据
	//union
	//{
		CoordRec     coordinate;
		PointRec     point;
	    LineRec      line;
		ArcRec       arc;
		CircleRec    circle;
#ifdef Uses_NURBS
		NurbsRec	  nurbs;
#endif
		ModeRec       mcode;
		ErrRec        error;
#ifdef Uses_CbsSmooth
		smoothRec   smooth;
		cbsRec      cbs;
#endif
	//};
	ToolRec getTool();
	GeometryRec();
	GeometryRec(int aWhat);
};

#endif /* INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_ */
