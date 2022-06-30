/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/08
 *@brief 本头文件包含刀具半径补偿相关数据结构的实现
 *@version
 */

#include "global_include.h"
#include "tool_comp_data.h"
#include "tools_comp.h"					   


/**
 * @brief  计算直线长度
 * @return
 */
double DataLine::GetLength()
{
	double  mm = source.x - target.x,
	            nn = source.y - target.y;
	return (mm * mm + nn * nn);
}

/**
 * @brief
 * @param offset
 * @return
 */
DataLine& DataLine::offset(const ToolOffset& offset )
{
	double angle = arcmath(target, source) * M_RAD + M_PI_2,
	           off  = offset.radius;
    
//	printf("+++++++offset.radius  %lf\n",offset.radius);
	if(fabs(off) < 1e-4)  //20131023 burke
		return *this;

	DPlane move( off * cos(angle), off * sin(angle) );
	source += move;
	target += move;
	return *this;
}


/*
DataPoint PointRec::getData()
{
	DataPoint p;
	p.target = varyPlane( target, tool.plane);
	return p;
}
void PointRec::setData(const DataPoint& p)
{
	Plane2Point(p.target, target, tool.plane);
}
*/

DataPoint PointRec::GetData()
{
	DataPoint p;
	p.target = VaryPlane( Target, tool.plane);
	return p;
}
void PointRec::SetData(const DataPoint& p)
{
	Plane2Point(p.target, Target, tool.plane);
}

/*
DataLine LineRec::getData()
{
	DataLine p;
	p.source = varyPlane(source, tool.plane);
	p.target = varyPlane(target, tool.plane);
	return p;
}
void LineRec::setData(const DataLine& p)
{
	Plane2Point(p.source, source, tool.plane);
	Plane2Point(p.target, target, tool.plane);
}
*/

DataLine LineRec::GetData()
{
	DataLine p;
	p.source = VaryPlane(Source, tool.plane);
	p.target = VaryPlane(Target, tool.plane);
	return p;
}
void LineRec::SetData(const DataLine& p)
{
//	printf("<<<<<<<<<SetData  tool.plane=%d \n", (int)tool.plane);	
	Plane2Point(p.source, Source, tool.plane);
	Plane2Point(p.target, Target, tool.plane);
}

/*
DataCircle CircleRec::getData()
{
	DataCircle p;
	p.flag      = flag;
	p.radius    = radius;
	p.center    = varyPlane(center, tool.plane);
	p.target    = varyPlane(target, tool.plane);
	return p;
}
void CircleRec::setData(const DataCircle& p)
{
	radius = p.radius;
	flag  = p.flag;
	Plane2Point(p.center, center, tool.plane);
	Plane2Point(p.target, target, tool.plane);
}
*/

DataCircle CircleRec::GetData()
{
	DataCircle p;
	p.flag      = flag;
	p.radius    = radius;
	p.center    = VaryPlane(Center, tool.plane);
	p.target    = VaryPlane(Target, tool.plane);
	return p;
}
void CircleRec::SetData(const DataCircle& p)
{
	radius = p.radius;
	flag  = p.flag;
	Plane2Point(p.center, Center, tool.plane);
	Plane2Point(p.target, Target, tool.plane);
}

/*
DataArc ArcRec::getData()
{
	DataArc p;
	p.radius = radius;
	p.flag   = flag;
	p.center = varyPlane(center, tool.plane);
	p.source = varyPlane(source, tool.plane);
	p.target = varyPlane(target, tool.plane);
	return p;
}
void ArcRec::setData(const DataArc& p)
{
	flag  = p.flag;
	radius = p.radius;
	Plane2Point(p.center, center, tool.plane);
	Plane2Point(p.source, source, tool.plane);
	Plane2Point(p.target, target, tool.plane);
}
*/

DataArc ArcRec::GetData()
{
	DataArc p;
	p.radius = radius;
	p.flag   = flag;
	p.center = VaryPlane(Center, tool.plane);
	p.source = VaryPlane(Source, tool.plane);
	p.target = VaryPlane(Target, tool.plane);
	return p;
}
void ArcRec::SetData(const DataArc& p)
{
	flag  = p.flag;
	radius = p.radius;
	Plane2Point(p.center, Center, tool.plane);
	Plane2Point(p.source, Source, tool.plane);
	Plane2Point(p.target, Target, tool.plane);
}



/*------------------------------------------------------------*/
/* function(s)                                                */
/*                                                            */
/*               DataTroop member functions                   */
/*------------------------------------------------------------*/
/*
DataTroop::DataTroop( int aLimit )
	: limit( aLimit ),
	  count(0)
{
	items = new TroopRec[limit];
	buildlink();
	head = items;
	tail = items;
}
DataTroop::~DataTroop()
{
	delete []items;
}

//构造环形队列 marked by gonghao
void DataTroop::buildlink()
{
	if( items )
	{
		int i = 0;
		for( i = 0; i < limit - 1; i++ )
		{
			items[i].next = &(items[i + 1]);
			items[i+1].pre = &(items[i]);    //gongaho 20161207
		}

		items[i].next = &(items[0]);
		items[0].pre = &(items[i]);   //gonghao 20161207
	}
}
int DataTroop::getCount()
{
	return count;
}
bool DataTroop::isEmpty()
{
	return bool( count == 0 );
}
bool DataTroop::isFull()
{
	return bool( count == limit );
}
void* DataTroop::takeItem()
{
	void* rec = 0;

	if( !isEmpty() )
	{
		rec = head->rec;
		head->rec = 0;
		head = head->next;
		count--;
	}

	return rec;
}
int DataTroop::addItem(void* rec)
{
	if( !isFull() && rec)
	{
		tail->rec = rec;
		tail = tail->next;
		count++;
		return count;
	}

	freeItem( rec );
	return -1;
}
void* DataTroop::tailItem()
{
	return DataTroop::at( count - 1 );
}
void* DataTroop::at(int aIndex)
{
	if( aIndex < 0 || aIndex >= count ) return 0;

	TroopRec* p = head;

	while( aIndex-- )
	{
		p = p->next;
	}

	return p->rec;
}
int DataTroop::atInsert(int aIndex, void* rec)
{
	if( rec == 0 || aIndex > count )
	{
		freeItem(rec);
		return -1;
	}

	TroopRec* p = head;
	int n = aIndex;

	while( n-- )
		p = p->next;

	count++;
	void* temp = rec;

	for( int i = aIndex; i < count; i++ )
	{
		void* tmp = p->rec;
		p->rec   = temp;
		temp     = tmp;
		p        = p->next;
	}

	tail = tail->next;
	return count;
}
void DataTroop::empty()
{
	while( !isEmpty() ) freeItem( DataTroop::takeItem() );

	tail = head = items;
}
void DataTroop::freeItem( void* rec)
{
	delete (char *)rec;
}
*/
/*------------------------------------------------------------*/
/* function(s)                                                */
/*                                                            */
/*               GeometryRec member functions                 */
/*------------------------------------------------------------*/

GeometryRec::GeometryRec()
	: what( invalidData )
{
	usFlag = 0;    //gonghao 20161206
	usSmoothFlag = 199;
}
GeometryRec::GeometryRec(int aWhat)
	: what( aWhat )
{
	usFlag = 0;   //gonghao 20161206
	usSmoothFlag = 199;
}

ToolRec GeometryRec::getTool()
{
	ToolRec tool;
	
	switch( what )
	{
	case pointData:
	case rapidData:
		return point.tool;

	case lineData:
		return line.tool;

	case arcData:
		return arc.tool;

	case circleData:
		return circle.tool;
	}

	tool.G41G42dir   = 0,
	tool.D = tool.H = 0;
	tool.Radius = 0;
	return tool;
}
