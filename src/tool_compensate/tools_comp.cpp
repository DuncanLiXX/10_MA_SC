/************************************************************************
*    COPYRIGHT NOTICE
*    Copyright (c) 2011.11.11 by OuDeSiShuKong
*     All rights reserved.
*
*  @file: AE_tools.cpp
*  @brief: ���߰뾶����������
*  @version:1.0
*  @author:zhanghui
*  @date:2011-05-09
*  @Modification History:
*  (name, date, modification action)
*************************************************************************/

#include "math.h"
#include "tool_comp_data.h"
#include "tools_comp.h"

/*------------------------------------------------------------*/
/*                                                                               */
/* function(s)                                                             */
/*                                                                               */
/*           plane data record member functions            */
/*------------------------------------------------------------*/
//const double EPSINON = 1e-6;
//char g_compensateError0 = 0; // wch 2012.06.15
char nJumpFlag=0;  // hxj 2021.12.20 ��ʱ���ã����Ż� ??????????????????????????????????
//**************************************************//
//Author:zhanghui
//Modify:2011.6.16
//Purpose:ֱ��ƫ��
//**************************************************//
/*
double DataLine::GetLength()
{
	double  mm = source.x - target.x,
	            nn = source.y - target.y;
	return (mm * mm + nn * nn);
}

DataLine& DataLine::Offset(const ToolOffset& offset )
{
	double angle = arcmath(target, source) * M_RAD + M_PI_2,
	           off  = offset.radius;

	if(fabs(off) < 1e-4)  //20131023 burke
		return *this;

	DPlane move( off * cos(angle), off * sin(angle) );
	source += move;
	target += move;
	return *this;
}
*/

//added by burke 20120412
//��ʼ��ErrorMessage
/*
GeometryRec* InitErrorData(uint32_t errFlag)
{

	GeometryRec* info = new GeometryRec;

	if(!info)
		return nullptr;

	info->what = errorData;
	info->error.errflag = errFlag;
	g_compensateError0 = 1; // wch 2012.06.15
	return info;
}
*/
//end 20120412

//******************************************************//
//Author:zhanghui
//Data:2011.6.16
//Purpose:����ֱ�߹���
//******************************************************//
struct DataLine&  DataLine::length_offset(ToolOffset& offset, int flag)
{
	double angle = arcmath(target, source) * M_RAD,
	       off  = fabs(offset.radius);

	if(off < 9e-4)  //20131022 burke ��ֹ�����뾶С��1u
		return *this;

	DPlane move(flag * off * cos(angle), flag * off * sin(angle));
	source += move;
	target += move;
	return *this;
}
//******************************************************//
//Author:burkegong
//Data:2012.4.18
//Purpose:initialize the start and end point to (0,0)
//******************************************************//
void DataLine::InitData()
{
	source.x = 0;
	source.y = 0;
	target.x = 0;
	target.y = 0;
}


//***************************************************//
//Purpose:����Բ��ƫ��
//flag=0Ϊ˳ʱ�룬offΪ�����󵶲�
//***************************************************//
DataCircle& DataCircle::offset(const ToolOffset& offset , void *troop, bool *bErrFlag)
{
	if(bErrFlag != nullptr)
		*bErrFlag = false;

	double angle = arcmath(target, center) * M_RAD,
	       off  = offset.radius;

	if(fabs(off) < 1e-4)  //20131023 burke
		return *this;

	radius += (flag == 0) ? off : -off;


	//if( radius < 1e-3 )
	if((radius-1e-3) < EPSINON)    //20131023 burke
	{
		if(troop != nullptr) //added by burke
		{
		/*
			GeometryRec* err = InitErrorData(ERR_TOOLRADIUS);
			if(!err)	return *this;
			((DataTroop *)troop)->atInsert(0, err);  */
			
            ((CompensateTroop *)troop)->g_compensateError = 1;
            CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

			if(bErrFlag != nullptr)
				*bErrFlag = true;
		}

		return *this;
	}

	DPlane move( radius * cos(angle), radius * sin(angle));
	target = move + center;
	return *this;
}
//***********************************************//
//Purpose:Բ��ƫ��
//Modify:
//***********************************************//
DataArc& DataArc::offset(const ToolOffset& offset, void *troop, bool *bErrFlag)
{
	if(bErrFlag != nullptr)
		*bErrFlag = false;

	double angle = arcmath(target, center) * M_RAD,
	       off  = offset.radius;
//zhanghui2012.9.13
	double arcangle = getAngle();

	//if(arcangle > 180.0)
	if((arcangle-180.0) > EPSINON)  //20131022 burke
	{
		if((flag == 0 && off < -9e-4) || (flag == 1 && off > 9e-4))
		{
			double dis = sqrt((target.x - source.x) * (target.x - source.x) + (target.y - source.y) * (target.y - source.y));

			if(dis < 2 * fabs(off))
			{
				if(troop != nullptr)
				{
				/*
					GeometryRec* err = InitErrorData(ERR_TOOLRADIUS);
					if(!err)
						return *this;
					((CompensateTroop *)troop)->atInsert(0, err);*/
                    ((CompensateTroop *)troop)->g_compensateError = 1;
                    CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

					if(bErrFlag != nullptr)
						*bErrFlag = true;
				}

				return *this;
			}
		}
	}

//end
	if(fabs(off) < 9e-4)  //20131023 burke
		return *this;

	radius += (flag == 0) ? off : -off;

	//if( radius < 1e-3)
	if((radius-1e-3) < EPSINON)    //20131023 burke
	{
		if(troop != nullptr) //added by burke
		{
		/*
			GeometryRec* err = InitErrorData(ERR_TOOLRADIUS);
			if(!err)	return *this;
			((CompensateTroop *)troop)->atInsert(0, err); */
			
            ((CompensateTroop *)troop)->g_compensateError = 1;
            CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

			if(bErrFlag != nullptr)
				*bErrFlag = true;
		}

		return *this;
	}

	target = DPlane( radius * cos(angle), radius * sin(angle) ) + center;
	angle = arcmath(source, center) * M_RAD;
	source = DPlane( radius * cos(angle), radius * sin(angle) ) + center;
	return *this;
}
/*
double DataArc::getLength()   //����Բ���ĳ���
{
	double aStart = arcmath( source, center),
	       aEnd  = arcmath( target, center);
	int dir = ( flag == 0 ) ? -1 : 1;
	double angle = (aEnd - aStart) * dir;
       if( angle<-0.1 )
		angle += 360;
	return fabs(radius * angle * M_RAD);
}
*/
/*
double DataArc::getAngle() //����Բ���Ƕȼ��
{
	double aStart = arcmath(source, center),
	       aEnd = arcmath(target, center);
	double angle = 0, dir = ( flag == 0 ) ? -1 : 1; //˳ʱ��(-1)����ʱ��(1)
	angle = (aEnd - aStart) * dir;
	if(angle < -0.1)
	{
		angle += 360.;
	}

	return fabs(angle);
//	return getLength()/(radius*M_RAD);
}*/
double DataArc::getAngle() //����Բ���Ƕȼ��
{
	double aStart = arcmath1(source, center),
	       aEnd = arcmath1(target, center);
	double angle = 0, dir = ( flag == 0 ) ? -1 : 1; //˳ʱ��(-1)����ʱ��(1)
	angle = (aEnd - aStart) * dir;

       if(angle < -1e-6)
	{
		angle += 360.;
	}
	return fabs(angle);
}

/*------------------------------------------------------------*/
/*                                                            */
/* function(s)                                                */
/*                                                            */
/*           CompensateTroop member functions                 */
/*------------------------------------------------------------*/
CompensateTroop::CompensateTroop()
{
	vecline1 = new DataLine;
	vecline2 = new DataLine;//���ڸ�����
	vecline3 = new DataLine;
	vecline4 = new DataLine;//���ڸ�����
	TypeFlag = 0;
	tool_compensate_state = 0;
	tool_compensate_type = 0;
	g_compensateError = 0;
	int  m = 0;

	for( int i = 0;  i < COMPTYPES; i++)
		for( int j = i;  j < COMPTYPES; j++)
		{
			select_type[i][j] = select_type[j][i] = m++;
		}
}
CompensateTroop::~CompensateTroop()
{
	empty();
	delete vecline1;
	delete vecline2;
	delete vecline3 ;
	delete vecline4;
}

/*
ϵͳ��λʱ����
*/
int CompensateTroop::ResetAllDatas()
{
	TypeFlag = 0;
	tool_compensate_state = -1;
	tool_compensate_type = 0;
	g_compensateError = 0;

	return 0;
}


/*
*  ��������������Ϣ�������뵶������
*/
int CompensateTroop::InsertErrorData(int errFlag, int index)
{    
	RecordMsg *err_msg = new ErrorMsg(errFlag);
	if(err_msg == nullptr){
		           //�ڴ����ʧ�ܣ��澯
		  CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		  return 0;
	}
	
	ListNode<RecordMsg *> *node = nullptr; 
	node = this->m_p_toolcomp_list_msg->HeadNode();  //ȡ��һ����Ϣ
	if(index>0)
	{ 
	   int listcnt = this->m_p_toolcomp_list_msg->GetLength();
	   if(index>listcnt) index= listcnt;
	   while(index--)
	   {
	   	   node = node->next;
	   }
	}
	
	RecordMsg *msg = nullptr;
	msg = static_cast<RecordMsg *>(node->data);
	err_msg->SetLineNo(msg->GetLineNo());  //���õ�ǰ�к�
	this->m_p_toolcomp_list_msg->InsertBefore(err_msg,node);
	return 1;
}


double CompensateTroop::GetLength2(double* point1, double* point2)
{
	double tmp = *point1 - *point2,
	       tmp1 = *(point1 + 1) - *(point2 + 1);
	double mm = tmp * tmp,
	       nn = tmp1 * tmp1;
	return (mm + nn);
}

LineRec CompensateTroop::pointtoline(DPointChn& point, PointRec& data1)
{
	LineRec lineRec1;
	lineRec1.Source = point;
	lineRec1.Target = data1.Target;
	lineRec1.tool =   data1.tool;
	return lineRec1;
}

double  CompensateTroop::lathe_tool_radius(const ToolRec& tool)
{
	int index = tool.H;

	if(index < 0 || index > MAX_LATHE_TOOL)
		return 0;

    // hxj 2021.12.18  ��ʱ�޸�    ������Ҫ��ȷ�޸�
//	double radius = (!index) ? 0 : pSys->gLatheTool[1][index].rOffset / 2000. ;
//	if (pSys->gLatheTool[1][index].rDir == 0 || pSys->gLatheTool[1][index].rDir == 9)
//		radius = 0;
    double radius = 0;  //????????????????????????

	int dir[10] = {0, 1, -1, 0, 0, 0, 0, 0, 0, 0 };
	int m = tool.G41G42dir;

	if(m==41)
		return radius;
	else if(m==42)
		return (-radius);
	else
		return 0;
}


double  CompensateTroop::lathe_tool_dir(const ToolRec& tool)
{
	int index = tool.H;

	if(index < 0 || index > MAX_LATHE_TOOL)
		return 0;

    // hxj 2021.12.18  ��ʱ�޸�    ������Ҫ��ȷ�޸�
	//if (pSys->gLatheTool[1][index].rDir <= 0 || pSys->gLatheTool[1][index].rDir >= 9)
	//	return 0;
	//return pSys->gLatheTool[1][index].rDir;
	
		return 0;
}

double  CompensateTroop::tool_radius(const ToolRec& tool)
{
	int index = tool.D;

	if(index < 0 || index > MAX_D_TOOL)
		return 0;

	double radius = (!index) ? 0 : (tool.Radius);  //  / 2.

	int m = tool.G41G42dir;

	if( m == 42 )  
		return (-radius * ((tool.plane == PLANE_ZX) ? -1 : 1)) ; //burke 20120322 �޸�G18ƽ�棬���ҵ������õ�����
	else if( m == 41)
		return radius * ((tool.plane == PLANE_ZX) ? -1 : 1); //burke 20120322 �޸�G18ƽ�棬���ҵ������õ�����
	else	
		return 0;
}

ToolOffset CompensateTroop::tool_offset(ToolRec& tool )
{
	ToolOffset offset;
	offset.radius = tool_radius(tool);;
	return offset;
}

/* 
  �� m_t_target m_t_target_old ��ֵ
*/
void CompensateTroop::setStart(const DPointChn& p)
{
    p.CopyData(m_t_Target);
    m_t_Target_old = m_t_Target;
}
void CompensateTroop::empty()
{
	TypeFlag = 0; //burke 20120416 ��ʼ��	
	g_compensateError = 0;

	//added by burke 20120418
	if(vecline1 != nullptr)
		vecline1->InitData();

	if(vecline2 != nullptr)
		vecline2->InitData();
	if(vecline3 != nullptr)
		vecline3->InitData();

	if(vecline4 != nullptr)
		vecline4->InitData();

	//end 20120418
	tool_compensate_state = 0;
//	m_t_target.x = m_t_target.y = m_t_target.z = m_t_target.a4 = m_t_target.a5 = m_t_target.a6 = m_t_target.a7 = m_t_target.a8 = 0; //changed by burke 20120424
	memset(m_t_Target.m_df_point,0,sizeof(DPointChn)*kMaxAxisChn);
	m_t_Target_old = m_t_Target;
//	DataTroop::empty();
		
}


void CompensateTroop:: setStartCompState()
{
    if(tool_compensate_state < 0)  
		tool_compensate_state=0;
}


/*
GeometryRec* CompensateTroop::at( int aIndex )
{
	return (GeometryRec* )(DataTroop::at( aIndex ) );
}
GeometryRec* CompensateTroop::takeItem()
{
	return (GeometryRec* )(DataTroop::takeItem() );
}
*/

//*****************************************************//
//�������߰뾶����
//*****************************************************//

#define DOUBLE_ZERO 1e-5

//�жϸ������Ƿ����
int DoubleEqual(double x, double y)
{
	if (x==y)
		return 1;
	if (x>y && x<= y+DOUBLE_ZERO)
		return 1;
	if (y>x && y<=x+DOUBLE_ZERO)
		return 1;
	return 0;
}

//�ж��Ƿ��ǵ�����������
int CompensateTroop::isCompensateEndData(int type)
{
	if (type == endData || type == compData ||
		type == endCompensateData)
		return 1;
	return 0;
}

//�ж��Ƿ��ǵ�����������
int CompensateTroop::isRunData(int type)
{
	if (type == pointData || type == lineData ||
		type == arcData || type == circleData ||
		type == rapidData)
		return 1;
	return 0;
}

//�ж��Ƿ����߶���������
int CompensateTroop::isPointLineData(int type)
{
	if (type == pointData || type == lineData ||
		type == rapidData)
		return 1;
	return 0;
}

//�ж��Ƿ��Ƿǵ�������
int CompensateTroop::isNoCompensateData(int type)
{
	if (isCompensateEndData(type))
		return 0;
	if (isRunData(type))
		return 0;
	return 1;
}

//��ȡ�˶�ָ��������
int CompensateTroop::getRunDataIndex(int from)
{
	int num = GetCount();
	
	for (int i=from; i<num; i++)
	{
	    GeometryRec data = GetADataAt(i);
		if (isRunData(data.what))
			return i;
	}
	return -1;
}

//��ȡ��������ָ��������
int CompensateTroop::getCompensateEndDataIndex(int from)
{
	int num = GetCount();
	
	for (int i=from; i<num; i++)
	{   
	    GeometryRec data = GetADataAt(i);
		if (isCompensateEndData(data.what))
			return i;
	}
	return -1;
}

//��ȡ�ǵ���ָ�����
int CompensateTroop::notCompensateDataCount(int from)
{
	int num = GetCount();
	int count=0;
	
	for (int i=from; i<num; i++)
	{
	    GeometryRec data = GetADataAt(i);
		if (isNoCompensateData(data.what))
			count++;
	}
	return count;
}

//��ȡ��������
int CompensateTroop::getCompensateDir(int index)
{
	int num = GetCount();

	if (index<0 || index>=num)
		return 0;
	 GeometryRec data = GetADataAt(index);
	return lathe_tool_dir(data.getTool());
}

//��ȡ�����뾶
double CompensateTroop::getCompensateRadius(int index)
{
	int num = GetCount();

	if (index<0 || index>=num)
		return 0;
	 GeometryRec data = GetADataAt(index);
	return lathe_tool_radius(data.getTool());
}


//�����뾶�Ƿ���Ч
int CompensateTroop::compensateRadiusValid(double r)
{
	if (fabs(r) < 9e-4)
		return 0;
	return 1;
}

//ָ�����ݵ����뾶�Ƿ���Ч
int CompensateTroop::hasCompensateRadius(int index)
{
	double r = getCompensateRadius(index);
	return compensateRadiusValid(r);
}

DPointChn CompensateTroop::getDataTarget(GeometryRec      pdata)  
{
	DPointChn p;
	int type = pdata.what;

	if (type == pointData || type == rapidData)
		p = pdata.point.Target;
	else if (type == lineData)
		p = pdata.line.Target;
	else if (type == arcData)
		p = pdata.arc.Target;
	else if (type == circleData)
		p = pdata.circle.Target;
	
	return p;
}

DPointChn CompensateTroop::getDataTarget(int index)
{
	DPointChn p;
	int num = GetCount();

	if (index<0 || index>=num)
		return p;
	return getDataTarget(GetADataAt(index));
	
}


GeometryRec CompensateTroop::GetADataAt(int indx)
{
    GeometryRec data;	
    
	RecordMsg *msg0 = nullptr;  
	msg0 = static_cast<RecordMsg *>(this->m_p_toolcomp_list_msg->at(indx)->data);	
	CodeMsgType type0 = msg0->GetMsgType();

    if(type0==RAPID_MSG)
    {
         RapidMsg *msg1 = static_cast<RapidMsg *>(msg0);
	     data.what = pointData;
		 data.point.Target = msg1->GetTargetPos();
		 data.point.tool = msg1->GetAToolRec();
    }
    else if(type0==LINE_MSG)
    {
         LineMsg *msg1 = static_cast<LineMsg *>(msg0);
	     data.what = lineData;
		 data.line.Source = msg1->GetSourcePos();
		 data.line.Target = msg1->GetTargetPos();
		 data.line.tool = msg1->GetAToolRec();
    }
    else if(type0==ARC_MSG)
    {
         ArcMsg *msg1 = static_cast<ArcMsg *>(msg0);
		 int8_t dir,  major, circle;
		 msg1->getArcFlags(dir,major,circle);
		 if(circle==1){
		     data.what = circleData;
			 data.circle.Source = msg1->GetSourcePos();
			 data.circle.Target = msg1->GetTargetPos();
			 data.circle.Center = msg1->GetCenterPos();
			 data.circle.tool   = msg1->GetAToolRec();
		 }
		 else{
		     data.what = arcData;
			 data.arc.Source = msg1->GetSourcePos();
			 data.arc.Target = msg1->GetTargetPos();
			 data.arc.Center = msg1->GetCenterPos();
			 data.arc.tool = msg1->GetAToolRec();
		 }
    }
    else if(type0==COMPENSATE_MSG)
    {
        CompensateMsg *tmp = (CompensateMsg *)msg0;
		int gcode = tmp->GetGCode();
		if(gcode==G40_CMD)
		{
	       data.what = endCompensateData;
		}
		else
		{
	       data.what = invalidData;
		}
    }
    else if(type0==AUX_MSG)
    {
	     AuxMsg *tmp = (AuxMsg *)msg0;
	     int mcode = tmp->GetMCode();
//		 printf("*&^%$#@  mcode = %d\n",mcode);
		 if(mcode==99 )
		 {                
	         data.what = compData;	
		 }
		 else if(mcode==30 || mcode==2  )  // 
		 {                
	         data.what = endData;		 
		 }
		 else
		 {
	         data.what = invalidData;
		 }
    }
    else
    {
	     data.what = invalidData;
    }
    return data;
}

bool CompensateTroop::GetAPlaneMoveData(int indx,int num,GeometryRec& Gdata)
{
    return true;
}

bool CompensateTroop::GetAMoveData(int indx,int num,GeometryRec& Gdata)
{
    GeometryRec data;	

    int i=0;
	for(i=indx; i<num; i++)
	{
        data = GetADataAt(i);
		if(data.what>=pointData && data.what<=rapidData)
		{
			Gdata = data;
			return true;
		}
	}	
    return false;
}


bool CompensateTroop::GetFeedLineBlockAt(uint32_t indx,double &feed,uint64_t &line,bool &block)
{    
	RecordMsg *msg0 = nullptr;  
	msg0 = static_cast<RecordMsg *>(this->m_p_toolcomp_list_msg->at(indx)->data);	
	CodeMsgType type0 = msg0->GetMsgType();

    if(type0==RAPID_MSG)
    {
         RapidMsg *msg1 = static_cast<RapidMsg *>(msg0);
		 feed = 0;
		 line = msg1->GetLineNo();
		 block = msg1->CheckFlag(FLAG_BLOCK_OVER);
    }
    else if(type0==LINE_MSG)
    {
         LineMsg *msg1 = static_cast<LineMsg *>(msg0);
		 feed = msg1->GetFeed();
		 line = msg1->GetLineNo();
		 block = msg1->CheckFlag(FLAG_BLOCK_OVER);
    }
    else if(type0==ARC_MSG)
    {
         ArcMsg *msg1 = static_cast<ArcMsg *>(msg0);
		 feed = msg1->GetFeed();
		 line = msg1->GetLineNo();
		 block = msg1->CheckFlag(FLAG_BLOCK_OVER);
    }
    else
    {
		 feed = 0;
		 line = 0;
		 block = 0;
    }
    return true;
}


int CompensateTroop::RefreshDataAt(int indx,GeometryRec data)
{
	RecordMsg *msg0 = nullptr;  
	msg0 = static_cast<RecordMsg *>(this->m_p_toolcomp_list_msg->at(indx)->data);	

//	printf(" ############RefreshDataAt indx = %d data.what=%d \n",indx, data.what);

    if(data.what == pointData)
    {
//         printf(" #####0000###data.point.Target %lf  %lf  %lf \n",data.point.Target.m_df_point[0],data.point.Target.m_df_point[1],data.point.Target.m_df_point[2]);
         RapidMsg *msg1 = static_cast<RapidMsg *>(msg0);  
		 msg1->SetTargetPos(data.point.Target);
    }
    else if(data.what==lineData)
    {
//         printf(" #####1111###data.line.Source %lf  %lf  %lf \n",data.line.Source.m_df_point[0],data.line.Source.m_df_point[1],data.line.Source.m_df_point[2]);     
//         printf(" #####1111###data.line.Target %lf  %lf  %lf \n",data.line.Target.m_df_point[0],data.line.Target.m_df_point[1],data.line.Target.m_df_point[2]);
         LineMsg *msg1 = static_cast<LineMsg *>(msg0);		 
		 msg1->SetSourcePos(data.line.Source);
		 msg1->SetTargetPos(data.line.Target);
    }
    else if(data.what==arcData)
    {
//         printf(" #####2222###data.arc.Source %lf  %lf  %lf \n",data.arc.Source.m_df_point[0],data.arc.Source.m_df_point[1],data.arc.Source.m_df_point[2]);     
//         printf(" #####2222###data.arc.Target %lf  %lf  %lf \n",data.arc.Target.m_df_point[0],data.arc.Target.m_df_point[1],data.arc.Target.m_df_point[2]);  
//         printf(" #####2222###data.arc.Center %lf  %lf  %lf \n",data.arc.Center.m_df_point[0],data.arc.Center.m_df_point[1],data.arc.Center.m_df_point[2]);
         ArcMsg *msg1 = static_cast<ArcMsg *>(msg0);
		 msg1->SetSourcePos(data.arc.Source);
		 msg1->SetTargetPos(data.arc.Target);
		 msg1->SetCenterPos(data.arc.Center);
    }
    else if(data.what==circleData)
    {
//         printf(" #####3333###data.circle.Source %lf  %lf  %lf \n",data.circle.Source.m_df_point[0],data.circle.Source.m_df_point[1],data.circle.Source.m_df_point[2]);     
//         printf(" #####3333###data.circle.Target %lf  %lf  %lf \n",data.circle.Target.m_df_point[0],data.circle.Target.m_df_point[1],data.circle.Target.m_df_point[2]);  
//         printf(" #####3333###data.circle.Target %lf  %lf  %lf \n",data.circle.Center.m_df_point[0],data.circle.Center.m_df_point[1],data.circle.Center.m_df_point[2]);
         ArcMsg *msg1 = static_cast<ArcMsg *>(msg0);
		 msg1->SetSourcePos(data.circle.Source);
		 msg1->SetTargetPos(data.circle.Target);
		 msg1->SetCenterPos(data.circle.Center);
    }
 /*   else if(type0==ARC_MSG)
    {
         ArcMsg *msg1 = static_cast<ArcMsg *>(msg0);
	     data.what = circleData;
		 data.circle.Source = msg1->GetSourcePos();
		 data.circle.Target = msg1->GetTargetPos();
		 data.circle.Center = msg1->GetCenterPos();
    }*/

    return 1;
}



int CompensateTroop::getDataType(int index)
{
	int num = GetCount();

	if (index<0 || index>=num)
		return invalidData;
	GeometryRec data = GetADataAt(index);
	return data.what;
}

LineRec CompensateTroop::getLineRec(DPointChn& point, int index)
{
	LineRec lineRec1;
	lineRec1.Source = point;
	lineRec1.Target = getDataTarget(index);
	lineRec1.tool = GetADataAt(index).getTool();
	return lineRec1;
}

int lathe_dir_sign[10][2] = {{0, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, -1}, {-1, 1}, {-1, 0}, {0, 0}};

double CompensateTroop::getLatheOffset1(DataLine& line, ToolRec& tool)
{
	double dx = line.target.x - line.source.x;
	double dy = line.target.y - line.source.y;
	double len = sqrt(dx*dx + dy*dy);

	if (len <= 9e-4) return 0;

	double r = lathe_tool_radius(tool);
	if (DoubleEqual(r, 0))
		return 0;

	int index = tool.H;
	double off = 0;

        // hxj 2021.12.18  ��ʱ�޸�    ������Ҫ��ȷ�޸�
//	int dir = pSys->gLatheTool[1][index].rDir;
	int dir = 1;
	if (dir <=0 || dir >= 9)
		off = 0;
	else
		off = r*(1-(dx/len))*lathe_dir_sign[dir][0];
	return off;
}

double CompensateTroop::getLatheOffset2(DataLine& line, ToolRec& tool)
{
	double dx = line.target.x - line.source.x;
	double dy = line.target.y - line.source.y;
	double len = sqrt(dx*dx + dy*dy);

	if (len <= 9e-4) return 0;

	double r = lathe_tool_radius(tool);
	if (DoubleEqual(r, 0))
		return 0;

	int index = tool.H;
	double off = 0;

    // hxj 2021.12.18  ��ʱ�޸�    ������Ҫ��ȷ�޸�

//	int dir = pSys->gLatheTool[1][index].rDir;
	int dir = 1;
	if (dir <=0 || dir >= 9)
		off = 0;
	else if (dir==1 || dir==3 || dir==5 || dir==7)
		off = r*(1-(dy/len))*lathe_dir_sign[dir][1];
	else
		off = r*(dy/len);
	return off;
}

//����߶�ʸ��
DPlane GetLineVector(DataLine& line)
{
	DPlane v;
	v.x = line.target.x - line.source.x;
	v.y = line.target.y - line.target.y;
	return v;
}

//���ʸ�����
double GetVectorDotProduct(DPlane v1, DPlane v2)
{
	return v1.x*v2.x + v1.y*v2.y;
}

//��ö�ά���
double GetVectorCrossProduct(DPlane v1, DPlane v2)
{
	return v1.x*v2.y - v1.y*v2.x;
}
	
//�޸��߶ν��㣬����1�޸ĳɹ�0�޸�ʧ��û�н���
int CompensateTroop::modifyLineCross(LineRec& line1, struct LineRec& line2)
{
	DataLine planeline1 = line1.GetData();
	DataLine planeline2 = line2.GetData();
	if (planeline1.target == planeline2.source) //�����ཻ
		return 1;

	DPlane v1 = GetLineVector(planeline1);
	DPlane v2 = GetLineVector(planeline2);

	double d = GetVectorDotProduct(v1, v2);
	if (DoubleEqual(d, 1)) { //ƽ��
		planeline2.source = planeline1.target;
		line2.SetData(planeline2);
		return 1;
	} else if (DoubleEqual(d, -1)) { //�౳
		return 0;
	}

	DPlane v3;
	v3.x = planeline2.source.x - planeline1.source.x;
	v3.y = planeline2.source.y - planeline1.source.y;

	double s1 = GetVectorCrossProduct(v1, v2);
	double s2 = GetVectorCrossProduct(v3, v2);

	if (DoubleEqual(s1, 0))
		return 0;

	double ratio = s2/s1;
	planeline1.target.x = planeline1.source.x + v1.x*ratio;
	planeline1.target.y = planeline1.source.y + v1.y*ratio;
	planeline2.source = planeline1.target;
	line1.SetData(planeline1);
	line2.SetData(planeline2);
	return 1;
}


void CompensateTroop::latheCompensateHandle(int i1, int i2)
{
	LineRec lineRec1, lineRec2; //����ֱ��
	double r = getCompensateRadius(i1);
	int dir = getCompensateDir(i1);
	
	if (isPointLineData(getDataType(i1))) {
		if (isPointLineData(getDataType(i2))) {
			//ֱ�߽�ֱ��
			lineRec1 = getLineRec(m_t_Target_old, i1);
			DPointChn p = getDataTarget(i1);
			lineRec2 = getLineRec(p, i2);
			DataLine line1 = lineRec1.GetData(), 
				line2 = lineRec2.GetData();

			double line1_off1 = getLatheOffset1(line1, lineRec1.tool);
			double line1_off2 = getLatheOffset2(line1, lineRec1.tool);
			double line2_off1 = getLatheOffset1(line2, lineRec2.tool);
			double line2_off2 = getLatheOffset2(line2, lineRec2.tool);

			line1.source.x += line1_off1;
			line1.target.x += line1_off1;
			line1.source.y += line1_off2;
			line1.target.y += line1_off2;

			line2.source.x += line2_off1;
			line2.target.x += line2_off1;
			line2.source.y += line2_off2;
			line2.target.y += line2_off2;

			lineRec1.SetData(line1);
			lineRec2.SetData(line2);

			modifyLineCross(lineRec1, lineRec2);
		} else {
			//ֱ�߽�Բ��	�������е���������ԭ����
		}
	} else {
		if (isPointLineData(getDataType(i2))) {
			//Բ����ֱ�ߣ������е���������ԭ����
		} else {
			//Բ����Բ���������е���������ԭ����
		}
	}
}

//��������wch 2020.09.15
int CompensateTroop::latheCompensate()
{
	int index;
	GeometryRec data[2];
	double offset1, offset2;
	int filterCount = 0;
	DPointChn tmpTarget;
	
	int num = GetCount();

	if(num <= 0) return 0;


	if (tool_compensate_state == 0) //�޵����׶�
	{
		//���˷ǵ�������
		filterCount = notCompensateDataCount(0);
		if (filterCount > 0)
			return filterCount;

		//����ǽ������������򷵻�
		if (isCompensateEndData(getDataType(0)))
			return 1;

		//��������С��0�����
		if (!hasCompensateRadius(0)) {
			return 1;
		}

		//����ʼ���ݲ���ֱ�������򱨴�
		if (!isPointLineData(getDataType(0))) {
/*			
			GeometryRec* err = InitErrorData(ERR_CREATETOOL);
			if(!err)				return 0;
			atInsert(0, err);
*/
            g_compensateError = 1;
            CreateError(ERR_CHANGE_R_COMP_STAT_CMD, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
			return 2;
		}

		//���뽨�������׶�
		m_t_Target = getDataTarget(0);
		tool_compensate_state = 1;
		return 1;
	}
	else if (tool_compensate_state == 1) //���������׶�
	{
		//���˷ǵ�������
		filterCount = notCompensateDataCount(0);
		if (filterCount > 0)
			return filterCount;

		//����ǽ����������ݻ򵶲�����С��0 ������޵����׶β�����
		if (isCompensateEndData(getDataType(0)) || !hasCompensateRadius(0)) {
			tool_compensate_state = 0;
			return 1;
		}

		//�������ͬ����ֱ�ӷ���
		tmpTarget = getDataTarget(0);
		if (tmpTarget == m_t_Target) {
			return 1;
		}

		//ת�����׶�
		m_t_Target_old = m_t_Target;
		m_t_Target = tmpTarget;
		tool_compensate_state = 2;
		return 0;
	}
	else if (tool_compensate_state == 2) //�����׶�
	{
		//�ҵ��ڶ�������
		filterCount = notCompensateDataCount(1);
		index = filterCount+1;
		if (index >= num)
			return 0;

		//����ǽ���������ת����������״̬
		if (isCompensateEndData(getDataType(index)) || !hasCompensateRadius(index)) {
			if (!isPointLineData(getDataType(0))) {
/*				
				GeometryRec* err = InitErrorData(ERR_CANCELTOOL);
				if(!err)					return 0;
				atInsert(0, err);
*/
               
                g_compensateError = 1; 
                CreateError(ERR_CHANGE_R_COMP_STAT_CMD, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
				return 2;
			}
			
			tool_compensate_state = 3;
			return 0;
		}

		//�õ��ڶ�������
		m_t_Target_new = getDataTarget(index);

		//��������
		latheCompensateHandle(0, index);
		return index;
	}
	else if (tool_compensate_state == 3) //���������׶�
	{
		tool_compensate_state = 0;
		return 1;
	}
	return num;
}

//*****************************************************//
//Author:zhanghui
//Purpose:���߰뾶����
//Modify:2011.7.25
//Note:�ѵ����Ľ���/����/�����ֿ�
//�������:��
//�������:������Ϣ�ĸ���
//*****************************************************//
int CompensateTroop::Compensate()
{
	int num = GetCount();//	int num = this->m_p_toolcomp_list_msg->GetLength();   // ��ȡ��ǰ�������л������ݸ���

	if(num < 1) return 0;

#ifndef USES_TOOL_COMPENSATE
	return num;
#else

    if(tool_compensate_state < 0)  // -1
	   return num;
		

    // hxj 2021.12.18  ��ʱ�޸�    ������Ҫ��ȷ�޸�
//	if (pSys->cCNCType == GRIND_SYSTEM) //ĥ���޵���
//		return num;
	
	 // hxj 2021.12.18  ��ʱ�޸�    ������Ҫ��ȷ�޸�
//	if (pSys->cCNCType == LATHE_SYSTEM) //��������
//		return latheCompensate();
	/*
	if (g_compensateError & 0x01) 
	{
		if(nJumpFlag != 0)
		{
			//gonghao 20121217 �ӹ���λ����������������
			//begin		
            g_compensateError = 1;
            InsertErrorData(ERR_COMPENSATE);
			return (num>0)?1:0;
		}
		
		return 0; // wch 2012.06.15
	}
    */
	// ��л�2012.08.28
	// �������ݲ����뵶������ֱ�ӷ���
	// begin

    /*
	if (g_compensateError & 0x02) 
		return (num>0)?1:0; //�������ݲ����뵶����gudekun 2013.04.09 G10�޸ĵ�ƫ�����뵶��[

	*/	

//    printf("++++ num=%d \n",num);
	

	GeometryRec data[2];
	data[0] = GetADataAt(0);
//    printf("++++ data[0].what = %d \n",data[0].what);
	
//	printf("++++ ");
    int ii=0;
	for(ii=0; ii<num; ii++)
	{
      data[1] = GetADataAt(ii);
	  printf("  data[%d].what = %d \n",ii,data[1].what);
	}
//	printf("\n");    

	/*
          ��������׸�����Ϊ M2 M30 M99��ֱ�ӳ����в����õ���״̬Ϊ0
	*/
    if( data[0].what == endData || data[0].what == compData)   //   ��������׸�����Ϊ M2 M30 M99
    {
		tool_compensate_state = -1;
		return 1;
    }
	
	/*
	        ��������׸�����Ϊ ��G01-G03�������ݿ���ֱ�ӳ�����
	*/
    if(data[0].what < pointData || data[0].what>rapidData)
		return 1;
	
	int  m[2] = {0, 0}, ret = 1;
	double offset1, offset2;
	offset1 = offset2 = 0;
	
	m[0] = data[0].what;
	if( m[0] == rapidData)		m[0] = pointData;
	offset1 = tool_radius( data[0].getTool());


	if(tool_compensate_state < 0 || tool_compensate_state > 4) // �쳣ֵ
	{
        tool_compensate_state = 0;
		return 0;
	}
	else if(tool_compensate_state == 1)  // ��һ�����ڽ�������
	{
		// ���¶������ݾ�ΪG00-G03
		if(num < 2)   return 0;  //  ��������ΪG00--G03�������ݸ�������2���޷����в��������Է���Ϊ0���Ⱥ�������ݽ�������������		
		
		if(num < 10 && data[1].what != endData && data[1].what != compData	/* && data[1].what != endCompensateData*/)
			     return 0;	   
		
		if(fabs(offset1)< 9e-4)     // 1------->>> 3
		{
		     tool_compensate_state = 3;
		}
		else                        // 1------->>> 2
		{		  
            tool_compensate_state = 2;			
		}	
	} 
	else if(tool_compensate_state == 2)  // ��һ�����ڵ�����
	{
		// ���¶������ݾ�ΪG00-G03
		if(num < 2)   return 0;  //  ��������ΪG00--G03�������ݸ�������2���޷����в��������Է���Ϊ0���Ⱥ�������ݽ�������������		
		
		if(num < 10 && data[1].what != endData && data[1].what != compData	/* && data[1].what != endCompensateData*/)
			     return 0;	   
		
		if(fabs(offset1)< 9e-4)     // 2------->>> 3
		{
		     tool_compensate_state = 3;
		}
		else                        // 2------->>> 2
		{ 
            tool_compensate_state = 2;	

		}	
	}
	else if(tool_compensate_state == 3)  // ��һ�����ڳ���������
	{  		
		if(fabs(offset1)< 9e-4)     // 3------->>> 0
		{
		     tool_compensate_state = 0;
			 return 1;
		}
		else                        // 3------->>> 1
		{	
		     // ���¶������ݾ�ΪG00-G03
		     if(num < 2)   return 0;  //  ��������ΪG00--G03�������ݸ�������2���޷����в��������Է���Ϊ0���Ⱥ�������ݽ�������������		
		
		     if(num < 10 && data[1].what != endData && data[1].what != compData	 /*&& data[1].what != endCompensateData*/)
			     return 0;		  
            tool_compensate_state = 1;			
		}	
	}
	else if(tool_compensate_state == 4)  // ��һ�����ڳ���������ǰһ��
	{ 		
		if(fabs(offset1)< 9e-4)     // 2------->>> 3
		{
		     tool_compensate_state = 3;
		}
		else                        // 2------->>> 2
		{	
		   // ���¶������ݾ�ΪG00-G03
		   if(num < 2)   return 0;  //  ��������ΪG00--G03�������ݸ�������2���޷����в��������Է���Ϊ0���Ⱥ�������ݽ�������������		
		
		   if(num < 10 && data[1].what != endData && data[1].what != compData	 /*&& data[1].what != endCompensateData*/)
			     return 0;	  
		   
            tool_compensate_state = 2;			
		}	
	}
	else   // tool_compensate_state == 0   // ��һ�������޵�����
	{
        // δ���뵶��״̬����ʹG00-G03��Ҳ���Գ�վ
		if(fabs(offset1)< 9e-4)     // 0------->>> 0
		{
		     tool_compensate_state = 0;
			 return 1; //2011.9.20
		}
		else                        // 0------->>> 1
		{
			// ���¶������ݾ�ΪG00-G03
		    if(num < 2) 
				return 0;  //  ��������ΪG00--G03�������ݸ�������2���޷����в��������Է���Ϊ0���Ⱥ�������ݽ�������������		
				
		    data[1] = GetADataAt(num - 1);			
		    if(num < 10 && data[1].what != endData && data[1].what != compData	/* && data[1].what != endCompensateData*/)
			     return 0;	   
            tool_compensate_state = 1;			
		}		
	}
	
	if(num>1)
	{ 
	    bool retflag=false;
	    /*
	    ��ȡ��һ������ƽ�������ƶ�������
	    */
	   
		/*
	    �������û�л�ȡ������ƽ�������ƶ������ݣ����ȡ��һ���ƶ�����(����XYƽ�浶���Ļ���ֻ��Z���ƶ�������)
		*/
	    retflag = GetAMoveData(1,num,data[1]);
		/*
	    ������涼û��get�����ݣ���ֻ��ȡ��һ����������
		*/
		if(false == retflag)
		{	    
		    data[1] = GetADataAt(1);
		}
		
		m[1] = data[1].what;
		if( m[1] == rapidData)					m[1] = pointData;
		else if(m[1] < pointData)				m[1] = endData;
		else if(m[1] > rapidData)		        m[1] = endData;		
		
		if(tool_compensate_state == 2)
		{
	        offset2 = tool_radius( data[1].getTool()); 
			if(fabs(offset2) < 9e-4)  // ��2�����ݵ���ֵΪ0
			{
				tool_compensate_state = 4;
			}
		}
	}
	else
	{
         data[1] = GetADataAt(0);
		 m[1] = endData;
	}

	

	if(data[0].what == lineData)
	{
		vecline1->source = data[0].line.GetData().source;
		vecline2->source = data[0].line.GetData().target;
	}
	else if(data[0].what == arcData)  //zhanghui2012.9.28
	{
		vecline3->source = data[0].arc.GetData().source;
		vecline4->source = data[0].arc.GetData().target;
		PreAngle=data[0].arc.GetData().getAngle();
	}	

//****************************************************************************************************//
	if(tool_compensate_state == 1 && (m[0] > lineData || m[0] < pointData)) //��������ʧ��
	{
	/*
		GeometryRec* err = InitErrorData(ERR_CREATETOOL);
		if(!err)			return 0;
		atInsert(0, err); */		
		
        g_compensateError = 1;

        //20220720 lidianqiang: ԭ������������У�����û�������澯���ĳ�������
        CreateError(ERR_CHANGE_R_COMP_STAT_CMD, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
		return 2;
	}

	if(tool_compensate_state == 3 && (m[0] > lineData || m[0] < pointData)) //��������ʧ��
	{
	/*
		GeometryRec* err = InitErrorData(ERR_CANCELTOOL);
		if(!err)return 0;
		atInsert(0, err);  */
		
        g_compensateError = 1;
        CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
		return 2;
	}

	// wch 2012.05.29
	if (tool_compensate_state == 1)  // ��������
	{
		if (data[0].what == pointData)
		{
			setStart(data[0].point.Target);  // ��m_t_target_old  m_t_target ��ֵ
		}
		else if (data[0].what == lineData)
		{
			setStart(data[0].line.Source);  // ��m_t_target_old  m_t_target ��ֵ
		}
	}

	// end 2012.05.29
	DPointChn tmpTarget;

	char compType = select_type[m[0]][m[1]];
//	printf("###################### tool_compensate_state = %d  compType=%d\n",(int)tool_compensate_state,(int)compType);

	switch( select_type[m[0]][m[1]] )
	{
	case 0:   // G00-G00
		tmpTarget = data[0].point.Target;
		ret = point_point(data[0].point, data[1].point);
		break;

	case 1:   // G00-G01  G01-G00
		tmpTarget = (m[0] <= m[1]) ?
		            data[0].point.Target :
		            data[0].line.Target;
		ret = (m[0] <= m[1]) ?
		      point_line(data[0].point, data[1].line) :
		      point_line(data[0].line, data[1].point);
		break;

	case 2:  // G00-arc  arc-G00
		tmpTarget = (m[0] <= m[1]) ?
		            data[0].point.Target :
		            data[0].arc.Target;
		ret = (m[0] <= m[1]) ?
		      point_arc(data[0].point, data[1].arc) :
		      point_arc(data[0].arc, data[1].point);
		break;

	case 3:
		tmpTarget = (m[0] <= m[1]) ?
		            data[0].point.Target :
		            data[0].circle.Target;
		ret = (m[0] <= m[1]) ?
		      point_circle(data[0].point, data[1].circle) :
		      point_circle(data[0].circle, data[1].point);
		break;

	case 4:
		tmpTarget = data[0].point.Target;
		ret = point_end(data[0].point);
		break;

	case 5:
		tmpTarget = data[0].line.Target;
		ret = line_line(data[0].line, data[1].line);
		break;

	case 6:

		tmpTarget = (m[0] <= m[1]) ?
		            data[0].line.Target :
		            data[0].arc.Target;
		ret = (m[0] <= m[1]) ?
		      line_arc(data[0].line, data[1].arc) :
		      line_arc(data[0].arc, data[1].line);
		break;
      
	case 7:
		tmpTarget = (m[0] <= m[1]) ?
		            data[0].line.Target :
		            data[0].circle.Target;
		ret = (m[0] <= m[1]) ?
		      line_cir(data[0].line, data[1].circle) :
		      line_cir(data[0].circle, data[1].line);
		break;

	case 8:
		tmpTarget = data[0].line.Target;
		ret = line_end(data[0].line);
		break;

	case 9:
		tmpTarget = data[0].arc.Target;
		ret = arc_arc( data[0].arc, data[1].arc);
		break;

	case 10:
		tmpTarget = (m[0] <= m[1]) ?
		            data[0].arc.Target :
		            data[0].circle.Target;
		ret = (m[0] <= m[1]) ?
		      arc_cir( data[0].arc, data[1].circle ) :
		      arc_cir( data[0].circle, data[1].arc );
		break;

	case 11:
		tmpTarget = data[0].arc.Target;
		ret = arc_end( data[0].arc );
		break;

	case 12:
		tmpTarget = data[0].circle.Target;
		ret = cir_cir( data[0].circle, data[1].circle);
		break;

	case 13:
		tmpTarget = data[0].circle.Target;
		ret = cir_end(data[0].circle);
		break;
	}

	
//	printf(" @@@@@@@@@@@@@@@@@@@@@@@@@@ ret = %d  \n",ret);

	if( ret )
	{
		GeometryRec Recdata = GetADataAt(0);  //  (int indx) GeometryRec *pt = at(0);

		if(Recdata.what != errorData)
		{
			RefreshDataAt(0,data[0]);  // *at(0) = data[0];
			m_t_Target_old = tmpTarget;

//			printf(" --------> data[0].what = %d  \n",data[0].what);

			if(data[0].what == lineData)
			{
				if(vecline1->source != vecline2->source)//gonghao 20121218 �ų�̧��������ͶӰΪһ��������
				{
					vecline1->target = data[0].line.GetData().source;
					vecline2->target = data[0].line.GetData().target;
					//cout<<"v1s("<<vecline1->source.x<<", "<<vecline1->source.y<<")"<<endl;
					//cout<<"v1t("<<vecline1->target.x<<", "<<vecline1->target.y<<")"<<endl;
					//cout<<"v2s("<<vecline2->source.x<<", "<<vecline2->source.y<<")"<<endl;
					//cout<<"v2t("<<vecline2->target.x<<", "<<vecline2->target.y<<")"<<endl;
					int tt = veccrfvec(vecline1, vecline2);
					if((tt == 1) &&( (tool_compensate_state == 2 )||(tool_compensate_state == 4 )))
					{
					/*
						GeometryRec* err = InitErrorData(ERR_TOOLRADIUS);
						//cout<<"error1"<<endl;
						if(!err)  return ret;
						atInsert(0, err); */
						
						g_compensateError = 1;
					    CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

						return (ret + 1);
					}
				}
			}
			else if(data[0].what == arcData)
			{
				vecline3->target = data[0].arc.GetData().source;
				vecline4->target = data[0].arc.GetData().target;
				int ttt = veccrfvec(vecline3, vecline4);
				if((ttt == 1) &&( (tool_compensate_state == 2 )||(tool_compensate_state == 4 )))
				{
				/*
					GeometryRec* err = InitErrorData(ERR_TOOLRADIUS);
					if(!err) return ret;
					atInsert(0, err); */
					
					g_compensateError = 1;
				    CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
					return (ret + 1);
				}
				AfterAngle=data[0].arc.GetData().getAngle();
				//if(((AfterAngle - PreAngle) >= 180.0)&& ( (tool_compensate_state == 2 )||(tool_compensate_state == 4 )))
				if(((AfterAngle - PreAngle - 180.0) >= EPSINON)&& ( (tool_compensate_state == 2 )||(tool_compensate_state == 4 ))) //20131022 burke
				{
				/*
					GeometryRec* err = InitErrorData(ERR_TOOLRADIUS);
					if(!err)  return 1;
					atInsert(0, err);  */
					
                    g_compensateError = 1;
				    CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

					return (ret + 1);
				}
			}
		}
	}

	return ret;
#endif
}

#ifdef USES_TOOL_COMPENSATE
int CompensateTroop::point_point(PointRec& data1, PointRec&  data2)
{
	int ret = 1;
	LineRec lineRec1, lineRec2; //����ֱ��
	lineRec1 = pointtoline(m_t_Target_old, data1);
	lineRec2 = pointtoline(data1.Target, data2);
	DataLine line1 = lineRec1.GetData(),
	         line2 = lineRec2.GetData();
	ToolOffset  offset = tool_offset(data1.tool);

	if(tool_compensate_state == 2)   //����������
	{
		ret = line_line(lineRec1, lineRec2) ;
		data1.Target = lineRec1.Target;
		return ret;
	}
	else if(tool_compensate_state == 1) //��������
	{
		if(line2.source == line2.target)
		{
			int nNext = findNextMove(line1.target.x, line1.target.y);

			if(nNext > 1)
			{
				GeometryRec Nextdata = GetADataAt(nNext);    //    *at(nNext);

				if(Nextdata.what == pointData)
				{
					ret = (point_point(data1, Nextdata.point) );
					return ret;
				}
				else if(Nextdata.what == lineData)
				{
					ret = ( point_line(data1, Nextdata.line) );
					return ret;
				}
				else if(Nextdata.what == arcData)
				{
					ret = (point_arc(data1, Nextdata.arc) );
					return ret;
				}
				else if(Nextdata.what == circleData)
				{
					ret = (point_circle(data1, Nextdata.circle) );
					return ret;
				}
			}
		}
		else
		{
			line2.offset( offset );
			line1.target = line2.source;
			lineRec1.SetData( line1);
			data1.Target = lineRec1.Target;
			return 1;
		}
	}
	else if(tool_compensate_state == 4)//��������ǰһ��
	{
		line1.offset(offset);
		lineRec1.SetData( line1);
		data1.Target = lineRec1.Target;
		return 1;
	}

	m_t_Target = data1.Target;
	return ret;
}

//***********************************************************//
int CompensateTroop::point_line(PointRec& data1,LineRec& data2)
{
	int ret = 1;
	LineRec lineRec1;//����ֱ��
	lineRec1 = pointtoline(m_t_Target_old, data1);
	ToolOffset  offset = tool_offset(data1.tool);

	if(tool_compensate_state == 2)  //����������
	{
		ret = line_line(lineRec1, data2) ;
		data1.Target = lineRec1.Target;
		return ret;
	}
	else if(tool_compensate_state == 1)  //��������
	{
		DataPoint point = data1.GetData();
		DataLine  line = data2.GetData();

		if(line.source == line.target)
		{
			int nNext = findNextMove(point.target.x, point.target.y);

			if(nNext > 1)
			{
				GeometryRec Nextdata =  GetADataAt(nNext);    //    *at(nNext);

				if(Nextdata.what == pointData)
				{
					ret = (point_point(data1, Nextdata.point) );
					return ret;
				}
				else if(Nextdata.what == lineData)
				{
					ret = ( point_line(data1, Nextdata.line) );
					return ret;
				}
				else if(Nextdata.what == arcData)
				{
					ret = (point_arc(data1, Nextdata.arc) );
					return ret;
				}
				else if(Nextdata.what == circleData)
				{
					ret = (point_circle(data1, Nextdata.circle) );
					return ret;
				}
			}
		}
		else
		{
			line.offset(offset );
			point.target = line.source;
			data1.SetData( point );
			m_t_Target = data1.Target;
			return 1;
		}
	}
	else if(tool_compensate_state == 4)  //��������ǰһ��
	{
		DataLine line = lineRec1.GetData();
		line.offset(offset);
		lineRec1.SetData(line);
		data1.Target = lineRec1.Target;
		return 1;
	}

	m_t_Target = data1.Target;
	return ret;
}

int CompensateTroop::point_line(LineRec& data1, PointRec& data2)
{
	int ret = 1;
	LineRec lineRec2;//����ֱ��
	lineRec2 = pointtoline(data1.Target, data2);
	DataLine line1 = data1.GetData();
	DataPoint point = data2.GetData();
	DataLine  line2;
	ToolOffset  offset = tool_offset(data1.tool);

	if(tool_compensate_state == 2)   //����������
	{
		ret = line_line(data1, lineRec2) ;
		return ret;
	}
	else if(tool_compensate_state == 1)  //����������
	{
		if(line1.target == point.target)
		{
			int nNext = findNextMove(point.target.x, point.target.y);

			if(nNext > 1)
			{
				GeometryRec Nextdata = GetADataAt(nNext);    //    *at(nNext);

				if(Nextdata.what == pointData)
				{
					ret = (point_line(data1, Nextdata.point) );
					return ret;
				}
				else if(Nextdata.what == lineData)
				{
					ret = ( line_line(data1, Nextdata.line) );
					return ret;
				}
				else if(Nextdata.what == arcData)
				{
					ret = (line_arc(data1, Nextdata.arc) );
					return ret;
				}
				else if(Nextdata.what == circleData)
				{
					ret = (line_cir(data1, Nextdata.circle) );
					return ret;
				}
			}
		}
		else
		{
			line2.source = line1.target;
			line2.target = point.target;
			line2.offset(offset);
			line1.target = line2.source;
			data1.SetData(line1);
		}
	}
	else if(tool_compensate_state == 4)  //��������ǰһ��
	{
		if(line1.source == line1.target)
		{
			data1.Source = m_t_Target;
			data1.Target.m_df_point[0] = data1.Source.m_df_point[0];
			data1.Target.m_df_point[1] = data1.Source.m_df_point[1];
			m_t_Target = data1.Target;
			return 1;
		}

		line1.offset(offset);
		data1.SetData(line1);
	}
	else if(tool_compensate_state == 3) //��������
	{
		line2.source.x = m_t_Target.m_df_point[0];
		line2.source.y = m_t_Target.m_df_point[1];
		line2.target = point.target;
		data1.SetData( line2 );
	}

	data1.Source = m_t_Target;
	m_t_Target = data1.Target;
	return 1;
}
//**********************************************************//
//Author:zhanghui
//Modify:2012.8.16
//**********************************************************//
int CompensateTroop::point_arc(PointRec& data1, ArcRec& data2)
{
	int ret = 1;
	bool bError = false; //burke 20120416

	if(tool_compensate_state == 2)//����������
	{
		LineRec lineRec1;//����ֱ��
		lineRec1 = pointtoline(m_t_Target_old, data1);
		ret = line_arc(lineRec1, data2) ;
		data1.Target = lineRec1.Target;
		return ret;
	}
	else if(tool_compensate_state == 1)//��������
	{
		DataPoint point = data1.GetData();
		DataArc   arc  = data2.GetData();
		arc.offset( tool_offset(data1.tool), this, &bError);
		point.target = arc.source;
		data1.SetData( point );
		m_t_Target = data1.Target;

		if(bError)
			return 2;
		else
			return 1;
	}

	m_t_Target = data1.Target;
	return ret;
}

int CompensateTroop::point_arc(ArcRec& data1, PointRec& data2)
{

	int ret = 1;
	bool bError = false; //burke 20120416

	if(tool_compensate_state == 2)//����������
	{
		LineRec lineRec2;//����ֱ��
		lineRec2 = pointtoline(data1.Target, data2);
		ret = line_arc(data1, lineRec2) ;
		return ret;
	}
	else if(tool_compensate_state == 4)//��������ǰһ��
	{
		DataArc arc = data1.GetData();
		arc.offset(tool_offset( data1.tool), this, &bError);
		data1.SetData(arc);
		data1.Source = m_t_Target;
		//arc = data1.getData();
		//ArcAdjust( arc );
		//data1.setData( arc );
		m_t_Target = data1.Target;

		if(bError)
			return 2;
		else
			return 1;
	}

	data1.Source = m_t_Target;
	//DataArc arc0 = data1.getData();
	//ArcAdjust( arc0 );
	//data1.setData( arc0 );
	m_t_Target = data1.Target;
	return ret;
}
int CompensateTroop::point_circle(PointRec& data1, CircleRec& data2)
{
	int ret = 1;
	bool bError = false;
	DataPoint point1 = data1.GetData();
	DataCircle cir2 = data2.GetData();
	LineRec lineRec1;//����ֱ��
	lineRec1 = pointtoline(m_t_Target_old, data1);
	DataLine line1 = lineRec1.GetData();

	if(tool_compensate_state == 2)
	{
		ret = line_cir(lineRec1, data2) ;
		data1.Target = lineRec1.Target;
		return ret;
	}
	else if(tool_compensate_state == 1)
	{
		cir2.offset(tool_offset( data1.tool), this, &bError);
		point1.target = cir2.target;
		data1.SetData( point1 );
	}

	m_t_Target = data1.Target;

	if(bError)
		return ret + 1;
	else
		return ret;
}

int CompensateTroop::point_circle(CircleRec& data1, PointRec& data2)
{
	bool bError = false; //burke 20120416
	int ret = 1;
	DataCircle circle = data1.GetData();
	LineRec lineRec2;   //����ֱ��
	lineRec2 = pointtoline(data1.Target, data2);

	if(tool_compensate_state == 2)
	{
		ret = line_cir(data1, lineRec2) ;
		return ret;
	}
	else if(tool_compensate_state == 4)
	{
		circle.offset(tool_offset( data1.tool), this, &bError);
		data1.SetData(circle);
		data1.Target = m_t_Target;
		//circle = data1.getData();
		//ArcAdjust( circle );
		//data1.setData( circle );
		m_t_Target = data1.Target;

		if(bError)
			return 2;
		else
			return 1;
	}

	data1.Target = m_t_Target;
	//DataCircle circle0 = data1.getData();
	//ArcAdjust( circle0 );
	//data1.setData( circle0 );
	m_t_Target = data1.Target;
	return ret;
}
//************************************************//
//Author:zhanghui
//Modify:2011.5.9
//Purpose:Բ������/ֱ�߹���
//************************************************//
RecordMsg* initCorner( const ArcRec& arc )
{
/*
	GeometryRec* info = new GeometryRec;

	if( !info  ) return 0;

	info->what  = arcData;
	ArcRec& arc = info->arc;
	arc.flag    = dir;
	arc.radius  = radius;
	arc.Source  = point;
	arc.Target  = point;
	arc.Center  = point;
	arc.tool    = tool;

	if(arc.tool.feed < 1)
		arc.tool.feed = 500000l;

	return info;
	*/
	int gcode=G02_CMD;
	if(arc.flag==1) gcode=G03_CMD;     //��Ϊ1,˳Ϊ-1
	double feed = arc.tool.feed;
	if(feed<1) feed=500.0;
	int axis_mask=0x3;	
	if(arc.tool.plane == PLANE_YZ) axis_mask=0x6;
	else if(arc.tool.plane == PLANE_ZX) axis_mask=0x5;
	int8_t major_flag = 1;  //�Ż���־�� 1--�ӻ�          -1--�Ż�
	int8_t circle_flag = 0;	//��Բ��־�� 0--Բ��    1--��Բ
	int8_t dir_flag = arc.flag;  //�����־��-1:clockwise,1:anticlockwise
	
	RecordMsg *arc_msg = new ArcMsg(gcode, arc.Source, arc.Target, arc.Center, arc.radius, feed, axis_mask,
				dir_flag, major_flag, circle_flag);
		if(arc_msg == nullptr){
			//TODO �ڴ����ʧ�ܣ��澯
			CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
			return nullptr;
		}
	return arc_msg; 
	
}
RecordMsg* initCorner( const LineRec& Line)
{
    /*
	GeometryRec* info = new GeometryRec;

	if( !info  ) return 0;

	info->what  = lineData;
	LineRec& Line = info->line;
	Line.flag = dir;
	Line.Source = point;
	Line.Target = point;
	Line.tool = tool;

	if(Line.tool.feed < 1)
		Line.tool.feed = 500000l;

	return info;
	*/
	

	RecordMsg *line_msg = nullptr;
	int feed = Line.tool.feed;
	if(feed<1) feed=500000l;
	line_msg = new LineMsg(Line.Source, Line.Target,feed, 0xff);
	if(line_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return nullptr;
	}
//	line_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	return line_msg; 
}

RecordMsg* initCorner( const ToolRec& tool, const DPointChn& point,int dir)
{
    /*
	GeometryRec* info = new GeometryRec;

	if( !info  ) return 0;

	info->what  = lineData;
	LineRec& Line = info->line;
	Line.flag = dir;
	Line.Source = point;
	Line.Target = point;
	Line.tool = tool;

	if(Line.tool.feed < 1)
		Line.tool.feed = 500000l;

	return info;
	*/
	

	RecordMsg *line_msg = nullptr;
	int feed = tool.feed;
	if(feed<1) feed=500000l;
	line_msg = new LineMsg(point, point,feed, 0xff);
	if(line_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return nullptr;
	}
//	line_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	return line_msg; 
}
//****************************************************//
//Author:zhanghui
//Modify:2011.5.21
//Purpose:ֱ�߹���
//****************************************************//
int  lineVector(const DataLine& line, DPlane& vector) //ֱ�ߵķ�������
{
	double x0, y0, x1, y1, s;
	x0 = line.source.x;
	y0 = line.source.y;
	x1 = line.target.x;
	y1 = line.target.y;
	s = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
	s = sqrt(s);

	
	//if(fabs(s) > 1e-3)
	if(s > 1e-3)  //20131022 burke
	{
		vector.x = (x1 - x0) / s;
		vector.y = (y1 - y0) / s;
		return 1;
	}

	//burke 20120419
	vector.x = 0.;
	vector.y = 0.;
	//end 20120419
	return 0;
}

//�������������ĵ�� burke 20120316
double GetVecScalarProduct(DPlane &vec1, DPlane &vec2)
{
	return vec1.x * vec2.x + vec1.y * vec2.y;
}
double arcc(DPlane& point1, DPlane &point2)
{
	double dx = point2.x - point1.x,
	       dy = point2.y - point1.y,
	       dl = sqrt(dx * dx + dy * dy),
	       angle = 0;

	if( dl < EPSINON)
		return 0;

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

	//if(dy < 0)
	if( dy < -1e-6 )       //20131022 burke
		angle *= -1;

	return angle * 180. / M_PI;
}

//����Բ�� �����յ�ĵ�λ����
int ArcVector(const DataArc& arc, const DPlane& point, DPlane& vector)
{
	double x, y, x0, y0;

	if(point == arc.center) return 0;

	x = point.x;
	y = point.y;
	x0 = arc.center.x;
	y0 = arc.center.y;
	int k = arc.flag == 0 ? 1 : -1; //k=1Ϊ˳,k=-1Ϊ��
	double R = arc.radius;

	if(arc.radius > 9e-4)  //20131023 burke ��ֹԲ���뾶1u����
	{
		vector.x = (y - y0) / (k * R);
		vector.y = -(x - x0) / (k * R);
		return 1;
	}

	return 0;
}
//��Բ�������յ��ֱ��
int ArcLine(const DataArc& arc, const DPlane& point, double *line)
{
	DPlane vector;
	double k, b;

	if(ArcVector(arc, point, vector) == 0)
		return 0;
	else
	{
		if(fabs(vector.x) < EPSINON)
		{
			line[0] = 1;
			line[1] = 0;
			line[2] = -(point.x);
			return 1;
		}
		else
		{
			k = (vector.y) / (vector.x);
			b = point.y - k * (point.x);
			line[0] = k;
			line[1] = -1;
			line[2] = b;
			return 1;
		}
	}
}

//*********************************************************//
//Author:zhanghui
//Modify:2011.7.25
//Purpose:�������α�̹켣��ʸ���н�
//Note:flag=1Ϊ�󵶲�,flag=-1Ϊ�ҵ���
//vec1/vec2Ϊ��λ����
///1-������2-������3-�쳤��4-�������
//����ת�ӽ�����ȡ��Ӧ��ת�ӹ�ϵ
//Modify:1e-3---->1e-2
//******************************************************//
int Transition_Type(DPlane* Vec1, DPlane* Vec2, int flag)
{
	DPlane *vec1 = Vec1;
	DPlane *vec2 = Vec2;
	double p = -(vec1->x) * (vec2->x) - (vec1->y) * (vec2->y),
	       q = flag * ((vec2->x) * (vec1->y) - (vec1->x) * (vec2->y));
       //cout<<"vec1=("<<vec1->x<<", "<<vec1->y<<")"<<endl;
       //	cout<<"vec2=("<<vec2->x<<", "<<vec2->y<<")"<<endl;
       //	cout<<"p="<<p<<", q = "<<q<<endl;
 
	if(q < -1e-2) //(180,360)
	{
		return 1;
	}
	else if(fabs(q) < 1e-2 && fabs(p + 1) < 0.1) //=180
	{
		return 5;
	}
	else if(q > 1e-2)
	{
		if(p > 1e-2)
			return 2; //(0,90)
		else
			return 3;//[90,180)
	}
	else if(fabs(q) < 1e-2&& fabs(p - 1) < 0.1) //0/360������Ϊһ����������������
	{
		return 4;    //���Է�Ϊ���̺Ͳ���
	}
	else
	{
		return 5;  ///=180
	}
}
int CompensateTroop::findNextMove(double x, double y)
{
	GeometryRec data;
	int cnt = GetCount();

	if(cnt < 3) return 0;

	for(int kk = 2; kk < cnt; kk++)
	{
		data = GetADataAt(kk); // *at(kk);  

		if( data.what == arcData || data.what == circleData )
		{
			return kk;
		}
		else if(data.what == pointData )
		{
			DataPoint point1 = data.point.GetData();

			if(x != point1.target.x || y != point1.target.y)
				return kk;
		}
		else if(data.what == lineData)
		{
			DataLine line1 = data.line.GetData();

			if(line1.source != line1.target)
				return kk;
		}
	}

	return 0;
}
void  planeconv(LineRec& data1, DPointChn& point)   //2012.8.17���ȫ�ֺ���
{
	data1.Source = point;

	//changed by burke 20120322 �޸�G18,G19ƽ�浶������
	if(data1.tool.plane == PLANE_YZ)
	{
		data1.Target.m_df_point[2] = data1.Source.m_df_point[2];
		data1.Target.m_df_point[1] = data1.Source.m_df_point[1];
	}
	else if(data1.tool.plane == PLANE_ZX)
	{
		data1.Target.m_df_point[0] = data1.Source.m_df_point[0];
		data1.Target.m_df_point[2] = data1.Source.m_df_point[2];
	}
	else  //(data1.tool.plane == PLANE_XY)
	{
		data1.Target.m_df_point[0] = data1.Source.m_df_point[0];
		data1.Target.m_df_point[1] = data1.Source.m_df_point[1];
	}

	point = data1.Target;
}


//**********************************************************//
//Auhor:zhanghui
//Create:2011.5.9
//Purpose:ֱ��ת��ֱ��
//Modify:2012.2.2
//Modify:2012.8.17�����Ż�
//**********************************************************//
int CompensateTroop::line_line(LineRec& data1, LineRec& data2)
{
	int ret = 1, dir;
	DataLine  line1 = data1.GetData(),
	          line2 = data2.GetData();
//	double len1 = line1.getlength(),
//	       len2 = line2.getlength();
	DPlane vec1, vec2;
	lineVector(line1, vec1) ;
	lineVector(line2, vec2);
	DPlane linesource = line1.source;
	DPlane linetarget = line1.target;
	
	dir = getDirection(line1.source, line1.target, line2.target); //��Ϊ1,˳Ϊ-1
	ToolOffset offset = tool_offset(data1.tool);
	double  radius = fabs( offset.radius);
	int tool_flag = offset.radius > 0 ? 1 : -1;
	int type = Transition_Type(&vec1, &vec2, tool_flag);

//	printf("*******************line_line******1**************\n");
//	printf("radius=%lf  tool_flag=%d type=%d\n",radius,tool_flag,type);
//	printf("line1: source %lf %lf target %lf %lf \n",line1.source.x,line1.source.y,line1.target.x,line1.target.y);
//	printf("line2: source %lf %lf target %lf %lf \n",line2.source.x,line2.source.y,line2.target.x,line2.target.y);
	
		
	if(type == 1)
	{
		switch(TypeFlag)
		{
		case 0:
			TypeFlag = 1;
			break;

		case 1:
			TypeFlag = 2;
			break;

		case 2:
			TypeFlag = 1;
			break;
		}
	}
	else
		TypeFlag = 0;

//************************************����������***************************************//
	if(tool_compensate_state == 2)
	{
		if(line1.source == line1.target)
		{
			planeconv(data1, m_t_Target);
			return 1;
			
//	printf("*******************line_line******1111******\n");
		}
		else if(line2.source == line2.target)//�����󽻵�,�޽��������ֱ�߹���
		{
			int nNext = findNextMove(line1.target.x, line1.target.y);
//	printf("*******************line_line******2222******\n");

			if(nNext > 1)
			{
				GeometryRec Nextdata = GetADataAt(nNext);   
				if(Nextdata.what == lineData)
				{
					//gonghao 20121205 ��̧����������£����¼��㵶��״̬
					double offset_t = tool_radius( Nextdata.getTool() );
					if(fabs(offset_t) <= 1e-3)
					{
						if(tool_compensate_state == 2)
							tool_compensate_state = 4;
					}
					//end 20121205
					return( line_line(data1, Nextdata.line) );
				}
				else if(Nextdata.what == arcData)
				{
					return(line_arc(data1, Nextdata.arc) );
				}
				else if(Nextdata.what == circleData)
				{
					return(line_cir(data1, Nextdata.circle) );
				}
			}

			line1.offset( offset );
			data1.SetData(line1);
			data1.Source = m_t_Target;
			m_t_Target = data1.Target;
			return 1;

		}
		

		line1.offset(offset);
		line2.offset(offset);
//	printf("*******************line_line******3333******\n");
//	printf("line1: source %lf %lf target %lf %lf \n",line1.source.x,line1.source.y,line1.target.x,line1.target.y);
//	printf("line2: source %lf %lf target %lf %lf \n",line2.source.x,line2.source.y,line2.target.x,line2.target.y);
//	printf("type=%d\n",type);

		if(type == 2)  //0--90Բ��������
		{
//	printf("*******************line_line******type == 2******\n");
			ArcRec insert_arc;  //  = corner->arc.GetData();
			insert_arc.flag = dir;
			insert_arc.Source = data1.Target;
			insert_arc.Target = data1.Target;
			insert_arc.Center = data1.Target;
			insert_arc.radius = radius;
			insert_arc.tool = data1.tool;

			DataArc data= insert_arc.GetData();
			data.source = line1.target;
			data.target = line2.source;			
			ArcAdjust( data );
			insert_arc.SetData(data);			
			data1.SetData( line1 );     // �޸� data1 �� Source �� Target
			data1.Source = m_t_Target; // �޸� data1 �� Source			
			m_t_Target = insert_arc.Target;

			uint64_t curline=0;
			bool block_flag=false;
			GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);
			
			RecordMsg* corner = initCorner(insert_arc);
			if(corner==nullptr)
				return 0;  
			corner->SetLineNo(curline);
			corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
//	printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@1111111 \n");            
			m_p_toolcomp_list_msg->InsertAt(1, corner);
			    return 2;
		}
		else if(type == 4) //0�ȵ����,ֱ�߲�����
		{	
//	printf("*******************line_line******type == 4******\n");		
			LineRec insert_line;
			insert_line.Source = data1.Target;
			insert_line.Target = data1.Target;

			DataLine data  = insert_line.GetData();
			data.source   = line1.length_offset(offset, 1).target;
			data.target   = line2.length_offset(offset, -1).source;
			insert_line.SetData(data);
			data1.SetData( line1 );
			data1.Source = m_t_Target;
			m_t_Target = insert_line.Target;
			
			uint64_t curline=0;
			bool block_flag=false;
			GetFeedLineBlockAt(0,insert_line.tool.feed,curline,block_flag);
			
			RecordMsg* corner = initCorner(insert_line);
			if(corner==nullptr)
				return 0;
			corner->SetLineNo(curline);
			corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
//	printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@22222 \n");  
			m_p_toolcomp_list_msg->InsertAt(1, corner);
			    return 2;
		}
		else if(3==type||1==type) //��չ�ͺ�������
		{
//	printf("*******************line_line******3==type||1==type******\n");	
			DPlane point = line1.target;
//	printf(">>>>>>>>>>>>>> point= %lf  %lf \n",point.x,point.y);	
			int ret = linecrfline(line1, line2, point);
//	printf(">>>>>>>>>>>>>> point= %lf  %lf ret=%d\n",point.x,point.y,ret);
			if(ret != 0)
			{
//	printf(">0>>>>>>>>>>>>>data1.Source %lf  %lf \n",data1.Source.m_df_point[0],data1.Source.m_df_point[1],data1.Source.m_df_point[2]);	
//	printf(">0>>>>>>>>>>>>>data1.Target %lf  %lf \n",data1.Target.m_df_point[0],data1.Target.m_df_point[1],data1.Target.m_df_point[2]);	
				line1.target = point;
				data1.SetData(line1);
				data1.Source = m_t_Target;
				m_t_Target = data1.Target;
//	printf(">1>>>>>>>>>>>>>data1.Source %lf  %lf \n",data1.Source.m_df_point[0],data1.Source.m_df_point[1],data1.Source.m_df_point[2]);	
//	printf(">1>>>>>>>>>>>>>data1.Target %lf  %lf \n",data1.Target.m_df_point[0],data1.Target.m_df_point[1],data1.Target.m_df_point[2]);	
				return 1;
			}
			else
			{
			/*
				GeometryRec* err = InitErrorData(ERR_TOOLRADIUS); 
				if(!err)	return 0;
				atInsert(0, err);*/
				
                g_compensateError = 1;
                CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
//	printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@33333 \n");  
				return 2;
			}
		}
		else if(type == 5)   //���Ե�һ��ֱ�߽���ƫ��
		{
//	printf("*******************line_line******type == 5******\n");
			data1.SetData( line1);
			data1.Source = m_t_Target;
			m_t_Target = data1.Target;
			return 1;
		}
	}
//*************************����������Ϊ����A/B******************//
	else if(tool_compensate_state == 1)
	{
		if(line1.source == line1.target)
		{
			//data1.source = target;
			//data1.target.x = data1.source.x;
			//data1.target.y = data1.source.y;
			//target = data1.target;
			planeconv(data1, m_t_Target); //gonghao 20121214 �˴�Ӧ��ȫ�̿���G17/G18/G19ƽ�� 
			return 1;
		}

		if(line2.source == line2.target)
		{
			int nNext = findNextMove(line1.target.x, line1.target.y);

			if(nNext > 1)
			{
				GeometryRec Nextdata =  GetADataAt(nNext); //  *at(nNext);

				if(Nextdata.what == pointData)
				{
					ret = (point_line(data1, Nextdata.point) );
					return ret;
				}
				else if(Nextdata.what == lineData)
				{
					ret = ( line_line(data1, Nextdata.line) );
					return ret;
				}
				else if(Nextdata.what == arcData)
				{
					ret = (line_arc(data1, Nextdata.arc) );
					return ret;
				}
				else if(Nextdata.what == circleData)
				{
					ret = (line_cir(data1, Nextdata.circle) );
					return ret;
				}
			}
		}

		double linelength = line1.GetLength();
		linelength = sqrt(linelength);

		if(fabs(offset.radius) > linelength)
		{
		/*
			GeometryRec* err = InitErrorData(ERR_CREATETOOL);
			if(!err)	return 0;
			atInsert(0, err);  */
			
            g_compensateError = 1;
		    CreateError(ERR_CHANGE_R_COMP_STAT_CMD, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

//	printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@44444 \n");  
			return 2;
		}

		if(type == 2 || type == 3) //||type==4)  //������
		{
			line1.offset(offset);
			line2.offset(offset);
			DPlane temp1 = line1.target;

			if((tool_compensate_type == 0) && (dir != 0)) //Բ��ת������B��
			{
				
				ArcRec insert_arc;	//	= corner->arc.GetData();
				insert_arc.flag = dir;
				insert_arc.Source = data1.Target;
				insert_arc.Target = data1.Target;
				insert_arc.Center = data1.Target;
				insert_arc.radius = radius;
				insert_arc.tool = data1.tool;
				
				DataArc data= insert_arc.GetData();
				data.source = line1.target;
				data.target = line2.source; 		
				ArcAdjust( data );
				insert_arc.SetData(data);	
				
				DataLine templine;
				templine.source = linesource;
				templine.target = line1.target;
				data1.SetData( templine );
				data1.Source = m_t_Target;
				m_t_Target =insert_arc.Target;	
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);		
				
				RecordMsg* corner = initCorner(insert_arc);
				if(corner==nullptr)
					return 0;  
				
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
//	printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@55555 \n");  
				m_p_toolcomp_list_msg->InsertAt(1, corner);				
				return 2;
			}
			else
			{
			/*
				GeometryRec* err = InitErrorData(ERR_COMPENSATE);
				if(!err) return 0;
				atInsert(0, err); */
				
                g_compensateError = 1;
                CreateError(ERR_R_COMP_GENERATE_FAIL, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
				return 2;
			}
		}
		else   //ֱ��ת������A��
		{
			line2.offset( offset);
			line1.target = line2.source;
			data1.SetData( line1 );
			data1.Source = m_t_Target;
			m_t_Target = data1.Target;
			return 1;
		}
	}
	//**************************** ����ǰһ��*****************************//
	else if(tool_compensate_state == 4)
	{
		//cout<<"type = "<<type<<"offset = "<<offset.radius<<endl;
		if(type == 2 || type == 3)	//type==4// 90��ǳ�����������������һ��typeΪ3�����
		{
			line1.offset(offset);
			line2.offset(offset);
			DPlane temp1 = line1.target;

			if((tool_compensate_type == 0) && (dir != 0)) //Բ��ת��
			{	
				ArcRec insert_arc;  //  = corner->arc.GetData();
				insert_arc.flag = dir;
				insert_arc.Source = data1.Target;
				insert_arc.Target = data1.Target;
				insert_arc.Center = data1.Target;
				insert_arc.radius = radius;
				insert_arc.tool = data1.tool;

				DataArc data= insert_arc.GetData();
				data.source = line1.target;
				data.target = line2.source;			
				ArcAdjust( data );
				insert_arc.SetData(data);			
				data1.SetData( line1 );     // �޸� data1 �� Source �� Target
				data1.Source = m_t_Target; // �޸� data1 �� Source			
				m_t_Target = insert_arc.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);	
				
				RecordMsg* corner = initCorner(insert_arc);
				if(corner==nullptr)
					return 0;  
				
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
//				printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@66666 \n");

				m_p_toolcomp_list_msg->InsertAt(1, corner);
				return 2;				
			}
			else
			{
			/*
				GeometryRec* err = InitErrorData(ERR_COMPENSATE);
				if(!err) return 0;
				atInsert(0, err); */
				
                g_compensateError = 1;
                CreateError(ERR_R_COMP_GENERATE_FAIL, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

				return 2;
			}
		}
		else
		{
			//gonghao 20121205 ����������ǰһ��Ϊ̧�������
			if(line1.source == line1.target)
			{
				planeconv(data1, m_t_Target);
				return 1;
			}
			//end 20121205
			line1.offset( offset);
			data1.SetData( line1);
			data1.Source = m_t_Target;
			m_t_Target = data1.Target;
			return 1;
		}
	}
///*********************************��������**************************************//
	else if(tool_compensate_state == 3)
	{
		double linelength = line2.GetLength(); //�����α�����ڵ��߰뾶
		linelength = sqrt(linelength);

		if(fabs(offset.radius) > linelength)
		{
		/*
			GeometryRec* err = InitErrorData(ERR_CANCELTOOL);
			if(!err)	return 0;
			atInsert(0, err); */ 
			
            g_compensateError = 1;
            CreateError(ERR_CHANGE_R_COMP_STAT_CMD, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
			return 2;
		}

		DataLine line2, line1 = data1.GetData();
		DPlane cc = line1.target;
		line2.source.x = m_t_Target.m_df_point[0];
		line2.source.y = m_t_Target.m_df_point[1];
		line2.target = cc;
		data1.SetData( line2 );
		data1.Source = m_t_Target;
		m_t_Target = data1.Target;
		return 1;
	}

	data1.Source = m_t_Target;
	m_t_Target = data1.Target;
	return 1;
}

//************************************************************************************//
//Author:zhanghui
//Modify:2011.5.10
//Purpose:ֱ�ߺ�Բ���νӵ����
//Modify:�����Ż�
//Modify:2012.9.12��Ҫ����Բ���ļ����������ξ����Ƿ���Ҫ����
//**********************************************************************************//
int CompensateTroop::line_arc(LineRec& data1, ArcRec& data2)
{
	bool bError = false;
	DataLine     line1 = data1.GetData();
	DataArc      arc = data2.GetData();
	double arcangle = arc.getAngle();
	double len = line1.GetLength();
	DPlane linesource = line1.source;
	DPlane linetarget = line1.target;
//����ȡ���ж�����
	double  linevecx, linevecy, ctpointx, ctpointy;
	linevecx = line1.target.x - line1.source.x;
	linevecy = line1.target.y - line1.source.y;
	ctpointx = arc.center.x - line1.target.x;
	ctpointy = arc.center.y - line1.target.y;
	double value = linevecx * ctpointx + linevecy * ctpointy;
	int typeflag;

	if(value > 1e-6)  //20131022 burke
		typeflag = 1;
	else if(value < -1e-6)
		typeflag = 2;
	else
		typeflag = 3;

//���� ����Բ���ķ���
	DataLine     line2;
	double angle;
    
	if(arcangle > 1)
		angle = 1;
	else
		angle = arcangle;
	int flag = (arc.flag == 0 ) ? -1 : 1;
	line2.source = line2.target = arc.source;
	(line2.target).revolve(arc.center, flag * angle);
	int dir = getDirection(line1.source, line1.target, line2.target);
//����ת������
	DPlane vec1, vec2;
	lineVector(line1, vec1) ;

	if(ArcVector(arc, arc.source, vec2) == 0)return 1;

	ToolOffset  offset = tool_offset(data1.tool);
	double radius = fabs( offset.radius );
	int tool_flag = offset.radius > 0 ? 1 : -1;
	int type = Transition_Type(&vec1, &vec2, tool_flag);

//�������ͽ��и�����
	if(type == 1)
	{
		switch(TypeFlag)
		{
		case 0:
			TypeFlag = 1;
			break;

		case 1:
			TypeFlag = 2;
			break;

		case 2:
			TypeFlag = 1;
			break;
		}
	}
	else
		TypeFlag = 0;

	//******************����������**************************************//
	if(tool_compensate_state == 2)
	{
		if(line1.source == line1.target)
		{
			planeconv(data1, m_t_Target);
			return 1;
		}

//2012.9.20

		double te[3];											//bo
		DPlane pointp[2];
		stdcircle( arc.center, arc.radius, te);
		int rett = linecrfcir(line1, te, pointp);

		if(rett > 1)
		{
			double sou, tar, poi, d1, d2, s1, s2;
			DPlane l1, l2;
			sou = arcc(arc.center, arc.source);
			tar = arcc(arc.center, arc.target);
			l1.x = line1.target.x - line1.source.x;
			l1.y = line1.target.y - line1.source.y;
			s1 = sqrt(l1.x * l1.x + l1.y * l1.y);

			if(pointp[0] == arc.source)
			{
				poi = arcc(arc.center, pointp[1]);
				l2.x = pointp[1].x - pointp[0].x;
				l2.y = pointp[1].y - pointp[0].y;
			}

			if(pointp[1] == arc.source)
			{
				poi = arcc(arc.center, pointp[0]);
				l2.x = pointp[0].x - pointp[1].x;
				l2.y = pointp[0].y - pointp[1].y;
			}

			s2 = sqrt(l2.x * l2.x + l2.y * l2.y);

			if(data2.flag == 1)
			{
				d1 = (sou - tar) > EPSINON ? (360 + tar - sou) : (tar - sou);
				d2 = (fabs(sou - poi) <= EPSINON) ? 360 : ((sou - poi) > EPSINON) ? (360 + poi - sou) : (poi - sou);
			}

			if(data2.flag == 0)
			{
				d1 = (sou - tar) > EPSINON ? (sou - tar) : (360 + sou - tar);
				d2 = (fabs(sou - poi) <= EPSINON) ? 360 : ((sou - poi) > EPSINON) ? (sou - poi) : (360 + sou - poi);
			}

			if((l2.x != 0 || l2.y != 0) && l1.x * l2.x <= 0 && l1.y * l2.y <= 0 && d1 > d2 && (s1 - s2) > -1e-6)
			{
			/*
				GeometryRec* err = InitErrorData(ERR_COMPENSATE);
				if(!err) return 0;
				atInsert(0, err);  */
				
				g_compensateError = 1;
				CreateError(ERR_R_COMP_GENERATE_FAIL, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
				return 2;
			}
		}

//end
		line1.offset( offset);
		line2.offset( offset);
		arc.offset(offset, this, &bError);
              
		if(bError)
			return 2;

		DPlane offsetlinesource = line1.source;
		double circle[3];
		DPlane point[2];

		if(stdcircle(arc.center, arc.radius, circle) == 0)return 1;

		int ret = linecrfcir(line1, circle, point);

		if(type == 2 && dir != 0) //������
		{			
			ArcRec insert_arc;  //  = corner->arc.GetData();
			insert_arc.flag = dir;
			insert_arc.Source = data1.Target;
			insert_arc.Target = data1.Target;
			insert_arc.Center = data1.Target;
			insert_arc.radius = radius;
			insert_arc.tool = data1.tool;

			DataArc data= insert_arc.GetData();
			data.source = line1.target;
			data.target = arc.source;			
			ArcAdjust( data );
			insert_arc.SetData(data);			
			data1.SetData( line1 );     // �޸� data1 �� Source �� Target
			data1.Source = m_t_Target; // �޸� data1 �� Source			
			m_t_Target = insert_arc.Target;
				
			uint64_t curline=0;
			bool block_flag=false;
			GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);	
			
			RecordMsg* corner = initCorner(insert_arc);
			if(corner==nullptr)
				return 0;  
			
			corner->SetLineNo(curline);
			corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
//			printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@7777 \n");

			m_p_toolcomp_list_msg->InsertAt(1, corner);
			return 2;
		}
		else if(type == 4) //0������
		{
			if(1 == ret)
			{
				line1.target = point[0];
				data1.SetData(line1);
			}
			else if(2 == ret)
			{
				if(((arc.flag == 0) && (offset.radius > 1e-4)) || ((arc.flag == 1) && (offset.radius < -1e-4)))
				{
					line1.target = point[1];
					data1.SetData(line1);
				}
				else
				{
					line1.target = point[0];
					data1.SetData(line1);
				}
			}
			else
			{				
				ArcRec insert_arc;  //  = corner->arc.GetData();
				insert_arc.flag = dir;
				insert_arc.Source = data1.Target;
				insert_arc.Target = data1.Target;
				insert_arc.Center = data1.Target;
				insert_arc.radius = radius;
				insert_arc.tool = data1.tool;

				DataArc data= insert_arc.GetData();
				data.source = line1.target;
				data.target = arc.source;			
				ArcAdjust( data );
				insert_arc.SetData(data);			
				data1.SetData( line1 );     // �޸� data1 �� Source �� Target
				data1.Source = m_t_Target; // �޸� data1 �� Source			
				m_t_Target = insert_arc.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);	
				
				RecordMsg* corner = initCorner(insert_arc);
				if(corner==nullptr)
					return 0; 
				
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
				m_p_toolcomp_list_msg->InsertAt(1, corner);
				return 2;
			}
		}
		else if(type == 1) //������
		{
			if(1 == ret)
			{
				
					line1.target = point[0];
					data1.SetData(line1);
					data1.Source = m_t_Target;
					m_t_Target = data1.Target;
					return 1;
				
			}
			else if(2 == ret)
			{
				if((1 == typeflag) || (3 == typeflag))
				{
					
						line1.target = point[1];
						data1.SetData(line1);
						data1.Source = m_t_Target;
						m_t_Target = data1.Target;
						return 1;
					
				}
				else if(2 == typeflag)
				{
					
						line1.target = point[0];
						data1.SetData(line1);
						data1.Source = m_t_Target;
						m_t_Target = data1.Target;
						return 1;
					
				}
			}
			else 
			{
			/*
				GeometryRec* err = InitErrorData(ERR_COMPENSATE);
				if(!err) return 0;
				atInsert(0, err);  */
				
                g_compensateError = 1;
                CreateError(ERR_R_COMP_GENERATE_FAIL, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
				return 2;
			}
		}
		else if(type == 3)
		{
			double line2[3], line3[3];
			DPlane point;

			if(stdline(line1.source, line1.target, line2) == 0)return 1;

			ArcLine(arc, arc.source, line3);

			int ret = linecrfline(line2, line3, point);

			if(ret != 0)
			{
				LineRec insert_line;
				insert_line.Source = data1.Target;
				insert_line.Target = data1.Target;

				DataLine data  = insert_line.GetData();
				data.source   = point;
				data.target   = arc.source;
				insert_line.SetData(data);
				line1.target = point;
				data1.SetData(line1);
				data1.Source = m_t_Target;
				m_t_Target = insert_line.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_line.tool.feed,curline,block_flag);	
				
				RecordMsg* corner = initCorner(insert_line);
				if(corner==nullptr)
					return 0;
//			printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@8888 \n");
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
				m_p_toolcomp_list_msg->InsertAt(1, corner);
				    return 2;
			}
			else
			{		
				/*
				GeometryRec* err = InitErrorData(ERR_TOOLRADIUS);
				if(!err) return 0;
				atInsert(0, err); */
				
                g_compensateError = 1;
    			CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

				return 2;
			}
		}
		else if(type == 5)
		{
			data1.SetData( line1);
			data1.Source = m_t_Target;
			m_t_Target = data1.Target;
			return 1;
		}
	}
///******************************��������*********************************************//
	else if(tool_compensate_state == 1) //ֱ��-Բ����������
	{
		double linelength = line1.GetLength();
		linelength = sqrt(linelength);

		if(fabs(offset.radius) > linelength)
		{
		/*
			GeometryRec* err = InitErrorData(ERR_CREATETOOL);
			if(!err)				return 0;
			atInsert(0, err); */

            g_compensateError = 1;
			CreateError(ERR_CHANGE_R_COMP_STAT_CMD, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

			return 2;
		}

		line1.offset(offset);
		arc.offset(offset, this, &bError);

		if(bError)
			return 2;

		if(type == 2)
		{	
		    DPlane temp1 = line1.target;	
			
			ArcRec insert_arc;  //  = corner->arc.GetData();
			insert_arc.flag = dir;
			insert_arc.Source = data1.Target;
			insert_arc.Target = data1.Target;
			insert_arc.Center = data1.Target;
			insert_arc.radius = radius;
			insert_arc.tool = data1.tool;

			DataArc data= insert_arc.GetData();
			data.source = line1.target;
			data.target = arc.source;			
			ArcAdjust( data );
			insert_arc.SetData(data);	
			DataLine templine;
			templine.source = linesource;
			templine.target = temp1;		
			data1.SetData( templine );     // �޸� data1 �� Source �� Target
			data1.Source = m_t_Target; // �޸� data1 �� Source			
			m_t_Target = insert_arc.Target;
				
			uint64_t curline=0;
			bool block_flag=false;
			GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);	
			
			RecordMsg* corner = initCorner(insert_arc);
			if(corner==nullptr)
				return 0;  

//			printf("NHLDJFNHLDJFNHLDJFNHLDJFNHLDJF%^&$#@9999\n");
			
			corner->SetLineNo(curline);
			corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
			m_p_toolcomp_list_msg->InsertAt(1, corner);
			return 2;
		}
		else if(type == 4) //����Ϊ180��
		{
			DPlane temp1 = line1.target;
			double circle[3];
			DPlane point[2];

			if(stdcircle(arc.center, arc.radius, circle) == 0)
			{
				return 1;
			}

			int ret = linecrfcir(line1, circle, point);

			if(ret == 0) //�޽���
			{
				if(tool_compensate_type == 0 && dir != 0) //Բ������
				{	
					ArcRec insert_arc;  //  = corner->arc.GetData();
					insert_arc.flag = dir;
					insert_arc.Source = data1.Target;
					insert_arc.Target = data1.Target;
					insert_arc.Center = data1.Target;
					insert_arc.radius = radius;
					insert_arc.tool = data1.tool;

					DataArc data= insert_arc.GetData();
					data.source = line1.target;
					data.target = arc.source;			
					ArcAdjust( data );
					insert_arc.SetData(data);	
					DataLine templine;
					templine.source = linesource;
					templine.target = temp1;		
					data1.SetData( templine );     // �޸� data1 �� Source �� Target
					data1.Source = m_t_Target; // �޸� data1 �� Source			
					m_t_Target = insert_arc.Target;
				
					uint64_t curline=0;
					bool block_flag=false;
					GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);	
					
					RecordMsg* corner = initCorner(insert_arc);
					if(corner==nullptr)
						return 0;  
					
					corner->SetLineNo(curline);
					corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
					m_p_toolcomp_list_msg->InsertAt(1, corner);
					return 2;
				}
			}
			else   //ֱ�ӹ�������A
			{
				DataLine line = data1.GetData();
				line.target = arc.source;
				data1.SetData( line);
				data1.Source = m_t_Target;
				m_t_Target = data1.Target;
			}
		}
		else   ///����A
		{
			line1.target = arc.source;
			data1.SetData( line1);
			data1.Source = m_t_Target;
			m_t_Target = data1.Target;
		}
                
		return 1;//gonghao 20121114
	}
//***************************************��������*************************************//
	else if(tool_compensate_state == 3)
	{
		DataLine line1 = data1.GetData();
		DPlane cc = line1.target;
		line1.offset(tool_offset(data1.tool));
		line1.source.x = m_t_Target.m_df_point[0];
		line1.source.y = m_t_Target.m_df_point[1];
		line1.target = cc;
		data1.SetData( line1 );
	}

	data1.Source = m_t_Target;
	m_t_Target = data1.Target;
	return 1;
}


//****************************************************//
//Author:zhanghui
//Modify:2011.5.10
//Purpose:ʵ��Բ����ֱ��
//Modify:�����Ż�
//****************************************************//
int CompensateTroop::line_arc(ArcRec& data1,LineRec& data2)
{
	bool bError = false;
	double tempangle;
	DataArc   arc = data1.GetData();
	DataLine  line2 = data2.GetData();
	double len = line2.GetLength();
	double arcangle = arc.getAngle();

	DPlane linesource = line2.source;
	DPlane linetarget = line2.target;
//����ȡ���ж�����
	double  linevecx, linevecy, ctpointx, ctpointy;
	linevecx = line2.target.x - line2.source.x;
	linevecy = line2.target.y - line2.source.y;
	ctpointx = arc.center.x - line2.source.x;
	ctpointy = arc.center.y - line2.source.y;
	double value = linevecx * ctpointx + linevecy * ctpointy;
	int  typeflag;

	if(value > 1e-6)  //20131022 burke
		typeflag = 1;
	else if(value < -1e-6)
		typeflag = 2;
	else
		typeflag = 3;

//���� ����Բ���ķ���
	DataLine     line1;
	double angle;

	if(arcangle > 1)
		angle = 1;
	else
		angle = arcangle;

	int flag = (arc.flag == 0 ) ? 1 : -1;
	line1.source = line1.target = arc.target;
	(line1.source).revolve(arc.center, flag * angle);
	int dir = getDirection(line1.source, line1.target, line2.target);
//����ת������
	DPlane vec1, vec2;
	lineVector(line2, vec2) ;

	if(ArcVector(arc, arc.target, vec1) == 0)return 1;

	ToolOffset  offset = tool_offset(data1.tool);
	double radius = fabs( offset.radius );
	int tool_flag = offset.radius > 0 ? 1 : -1;
	int type = Transition_Type(&vec1, &vec2, tool_flag);
	

////�������ͽ��и�����
	if(type == 1)
	{
		switch(TypeFlag)
		{
		case 0:
			TypeFlag = 1;
			break;

		case 1:
			TypeFlag = 2;
			break;

		case 2:
			TypeFlag = 1;
			break;
		}
	}
	else
		TypeFlag = 0;

//********************����������***************************************//
	if( tool_compensate_state == 2)
	{
		if(line2.source == line2.target)
		{
			int nNext = findNextMove(arc.target.x, arc.target.y);

			if(nNext > 1)
			{
				GeometryRec Nextdata =  GetADataAt(nNext);  //   *at(nNext);

				if(Nextdata.what == lineData)
					return( line_arc(data1, Nextdata.line) );
				else if(Nextdata.what == arcData)
					return(arc_arc(data1, Nextdata.arc) );
				else if(Nextdata.what == circleData)
					return(arc_cir(data1, Nextdata.circle) );
			}

			arc.offset(offset, this, &bError);
			data1.SetData(arc);
			data1.Source = m_t_Target;
			//arc = data1.getData();
			//ArcAdjust( arc );
			//data1.setData( arc );
			m_t_Target = data1.Target;
			if(bError)
				return 2;
			else
				return 1;
		}

//2012.9.20  

		double temp[3];
		DPlane pointp[2];
		stdcircle( arc.center, arc.radius, temp);
		int rett = linecrfcir(line2, temp, pointp);

		if(rett > 1)
		{
			double sou, tar, poi, d1, d2, s1, s2;
			DPlane l1, l2;
			sou = arcc(arc.center, arc.source);
			tar = arcc(arc.center, arc.target);
			l1.x = line2.target.x - line2.source.x;
			l1.y = line2.target.y - line2.source.y;
			s1 = sqrt(l1.x * l1.x + l1.y * l1.y);

			if(pointp[0] == line2.source)
			{
				poi = arcc(arc.center, pointp[1]);
				l2.x = pointp[1].x - pointp[0].x;
				l2.y = pointp[1].y - pointp[0].y;
			}
			else if(pointp[1] == line2.source)
			{
				poi = arcc(arc.center, pointp[0]);
				l2.x = pointp[0].x - pointp[1].x;
				l2.y = pointp[0].y - pointp[1].y;
			}

			s2 = sqrt(l2.x * l2.x + l2.y * l2.y);

			if(data1.flag == 1)
			{
				d1 = (sou - tar) > EPSINON ? (360 + tar - sou) : (tar - sou);
				d2 = (fabs(sou - poi) <= EPSINON) ? 360 : ((sou - poi) > EPSINON) ? (360 + poi - sou) : (poi - sou);
			}

			if(data1.flag == 0)
			{
				d1 = (sou - tar) > EPSINON ? (sou - tar) : (360 + sou - tar);
				d2 = (fabs(sou - poi) <= EPSINON) ? 360 : ((sou - poi) > EPSINON) ? (sou - poi) : (360 + sou - poi);
			}

			if((l2.x != 0 || l2.y != 0) && l1.x * l2.x >= 0 && l1.y * l2.y >= 0 && d1 > d2 && (s1 - s2) > -1e-6)
			{
			/*
				GeometryRec* err = InitErrorData(ERR_COMPENSATE);
				if(!err)	return 0;
				atInsert(0, err);  */
				
				g_compensateError = 1;
	            CreateError(ERR_R_COMP_GENERATE_FAIL, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
				return 2;
			}
		}

//end
		line2.offset( offset );
		line1.offset(offset);
		arc.offset( offset , this, &bError);
		double circle[3];
		DPlane point[2];
		
	       if(bError)                       //zhanghui2012.10.17
			return 2;

		if(stdcircle(arc.center, arc.radius, circle) == 0)return 1;

		int ret = linecrfcir(line2, circle, point);

		if(type == 2 && dir != 0) //����Բ����
		{            		
			ArcRec insert_arc;  //  = corner->arc.GetData();
			insert_arc.flag = dir;
			insert_arc.Source = data1.Target;
			insert_arc.Target = data1.Target;
			insert_arc.Center = data1.Target;
			insert_arc.radius = radius;
			insert_arc.tool = data1.tool;

			DataArc data= insert_arc.GetData();
			data.source = arc.target;
			data.target = line2.source;			
			ArcAdjust( data );
			insert_arc.SetData(data);	
			data1.SetData( arc );     // �޸� data1 �� Source �� Target
			data1.Source = m_t_Target;
			m_t_Target = insert_arc.Target;
				
			uint64_t curline=0;
			bool block_flag=false;
			GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);	
			
			RecordMsg* corner = initCorner(insert_arc);
			if(corner==nullptr)
				return 0;  
			
			corner->SetLineNo(curline);
			corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
			m_p_toolcomp_list_msg->InsertAt(1, corner);			

			if(bError)
				return 3;
			else
				return 2;
		}
		else if(type == 1) //������
		{
			if(1 == ret)
			{
					arc.target = point[0];
					data1.SetData( arc );
			}
			else if(2 == ret)
			{
				if(1 == typeflag)
				{
						arc.target = point[1];
						data1.SetData( arc );
				}
				else if((2 == typeflag) || (3 == typeflag))
				{
						arc.target = point[0];
						data1.SetData( arc );
				}
			}
			else   
			{
			/*
				GeometryRec* err = InitErrorData(ERR_COMPENSATE);
				if(!err)	return 0;
				atInsert(0, err); */				
				
				g_compensateError = 1;
				CreateError(ERR_R_COMP_GENERATE_FAIL, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
				return 2;
			}
		}
		else if(type == 4)  //0�����
		{
			if(1 == ret)
			{
				arc.target = point[0];
				data1.SetData( arc );
			}
			else if(2 == ret)
			{
				if(((offset.radius > 1e-4) && (arc.flag == 0)) || ((offset.radius < 1e-4) && (arc.flag == 1)))
				{
					arc.target = point[0];
					data1.SetData( arc );
				}
				else
				{
					arc.target = point[1];
					data1.SetData( arc );
				}
			}
			else
			{				        		
				ArcRec insert_arc;  //  = corner->arc.GetData();
				insert_arc.flag = dir;
				insert_arc.Source = data1.Target;
				insert_arc.Target = data1.Target;
				insert_arc.Center = data1.Target;
				insert_arc.radius = radius;
				insert_arc.tool = data1.tool;

				DataArc data= insert_arc.GetData();
				data.source = arc.target;
				data.target = line2.source;			
				ArcAdjust( data );
				insert_arc.SetData(data);	
				data1.SetData( arc );     // �޸� data1 �� Source �� Target
				data1.Source = m_t_Target;
				m_t_Target = insert_arc.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);	
				
				RecordMsg* corner = initCorner(insert_arc);
				if(corner==nullptr)
					return 0;  

			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
				m_p_toolcomp_list_msg->InsertAt(1, corner);	

				if(bError)
					return 3;
				else
					return 2;
			}
		}
		else if(type == 3) //ֱ�ߺ�Բ�������ߵĽ���
		{
			double line3[3], line4[3];
			DPlane point;

			if(stdline(line2.source, line2.target, line3) == 0)return 1;

			ArcLine(arc, arc.target, line4);

			int ret = linecrfline(line3, line4, point);

			if(ret != 0)
			{						
				LineRec insert_line;
				insert_line.Source = data1.Target;
				insert_line.Target = data1.Target;

				DataLine data  = insert_line.GetData();
				data.source   = arc.target;
				data.target   = point;
				insert_line.SetData(data);
				data1.SetData( arc );
				data1.Source = m_t_Target;
				m_t_Target = insert_line.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_line.tool.feed,curline,block_flag);	
				
				RecordMsg* corner = initCorner(insert_line);
				if(corner==nullptr)
					return 0;
				
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
				m_p_toolcomp_list_msg->InsertAt(1, corner);
				return 2;
			}
			else   ///����Ƿ����
			{
				/*
				GeometryRec* err = InitErrorData(ERR_COMPENSATE);
				if(!err)	return 0;
				atInsert(0, err); */					
				
				g_compensateError = 1;
    			CreateError(ERR_R_COMP_GENERATE_FAIL, FATAL_LEVEL, CLEAR_BY_MCP_RESET);

				return 2;
			}
		}
		else  //����ת������
		{
			data1.SetData(arc);
			data1.Source = m_t_Target;
			//arc = data1.getData();
			//ArcAdjust( arc );
			//data1.setData( arc );
			m_t_Target = data1.Target;
			return 1;
		}
	}
//************************************��������ǰһ��*******************************//
	else if(tool_compensate_state == 4)  //
	{
		line2.offset( offset );
		line1.offset(offset);
		arc.offset( offset , this, &bError);

		if(bError)
			return 2;
		if(type == 2)
		{
				if(dir != 0)
				{					
					ArcRec insert_arc;	//	= corner->arc.GetData();
					insert_arc.flag = dir;
					insert_arc.Source = data1.Target;
					insert_arc.Target = data1.Target;
					insert_arc.Center = data1.Target;
					insert_arc.radius = radius;
					insert_arc.tool = data1.tool;
					
					DataArc data= insert_arc.GetData();
					data.source = arc.target;
					data.target = line2.source; 		
					ArcAdjust( data );
					insert_arc.SetData(data);	
					data1.SetData( arc );	  // �޸� data1 �� Source �� Target
					data1.Source = m_t_Target;
					m_t_Target = insert_arc.Target;
				
					uint64_t curline=0;
					bool block_flag=false;
					GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);	
					
					RecordMsg* corner = initCorner(insert_arc);
					if(corner==nullptr)
						return 0;  
					
			        corner->SetLineNo(curline);
				    corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
					m_p_toolcomp_list_msg->InsertAt(1, corner); 
					return 2;					
				}
				else
				{
				/*
					GeometryRec* err = InitErrorData(ERR_COMPENSATE);
					if(!err)	return 0;
					atInsert(0, err);  */					
					
					g_compensateError = 1;
					CreateError(ERR_R_COMP_GENERATE_FAIL, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
					return 2;
				}
		}
		else if(type == 4)  //0��
		{
			double circle[3];
			DPlane point[2];
			if(stdcircle(arc.center, arc.radius, circle) == 0)return 1;

			int ret = linecrfcir(line2, circle, point);

			if(ret == 0) //�޽���
			{
				if(tool_compensate_type == 0 && dir != 0) //Բ��ת��
				{			
					ArcRec insert_arc;	//	= corner->arc.GetData();
					insert_arc.flag = dir;
					insert_arc.Source = data1.Target;
					insert_arc.Target = data1.Target;
					insert_arc.Center = data1.Target;
					insert_arc.radius = radius;
					insert_arc.tool = data1.tool;
					
					DataArc data= insert_arc.GetData();
					data.source = arc.target;
					data.target = line2.source; 		
					ArcAdjust( data );
					insert_arc.SetData(data);	
					data1.SetData( arc );	  // �޸� data1 �� Source �� Target
					data1.Source = m_t_Target;
					m_t_Target = insert_arc.Target;
				
					uint64_t curline=0;
					bool block_flag=false;
					GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);
					
					RecordMsg* corner = initCorner(insert_arc);
					if(corner==nullptr)
						return 0;  
					
			        corner->SetLineNo(curline);
				    corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
					m_p_toolcomp_list_msg->InsertAt(1, corner); 
					return 2;					
				}
			}
			else //�н���
			{
				data1.SetData( arc );
			}
		}
		else
		{
			data1.SetData( arc );
		}
	}

	data1.Source = m_t_Target;
	//arc = data1.getData();
	//ArcAdjust( arc );
	//data1.setData( arc );
	m_t_Target = data1.Target;
	return 1;
}


//****************************************************//
//Author:zhanghui
//Modify:2011.5.10
//Purpose:ֱ�߽�Բ/Բ��ֱ��
//ֻ�м�ǹ���,�ȵ����ٽ����ཻ,����������ת����
//****************************************************//
int CompensateTroop::line_cir(LineRec& data1, CircleRec& data2)
{
	bool bError = false;
	DataLine   line1 = data1.GetData();
	DataCircle circle = data2.GetData();
	ToolOffset offset = tool_offset(data1.tool);
	DPlane temppoint;
//����ȡ���ж�����
	double  linevecx, linevecy, ctpointx, ctpointy;
	linevecx = line1.target.x - line1.source.x;
	linevecy = line1.target.y - line1.source.y;
	ctpointx = circle.center.x - line1.target.x;
	ctpointy = circle.center.y - line1.target.y;
	double value = linevecx * ctpointx + linevecy * ctpointy;
	int typeflag;

	if(value > 1e-6)   //20121022 burke
		typeflag = 1;
	else if(value < -1e-6)
		typeflag = 2;
	else
		typeflag = 3;
//end
	if( tool_compensate_state == 2 )  //����������
	{
		if(line1.source == line1.target)
		{
			circle.offset( offset , this, &bError);
          /*           data1.source=target;
			if(data1.tool.plane == xyPlane)
			{
				data1.target.x = circle.target.x;
				data1.target.y = circle.target.y;
			}
			else if(data1.tool.plane == yzPlane)
			{
				data1.target.z = circle.target.x;
				data1.target.y = circle.target.y;
			}
			else if(data1.tool.plane == zxPlane)
			{
				data1.target.x = circle.target.x;
				data1.target.z = circle.target.y;
			}
			target= data1.target;*/
			planeconv(data1, m_t_Target);//gonghao 20121218 ����̧�����
                return 1; 
		}
		else
		{
			line1.offset(offset );
			circle.offset( offset , this, &bError);
			DPlane point[2];
			double cir[3];
			temppoint=point[0] = point[1] = line1.target;

			if(stdcircle( circle.center, circle.radius, cir) == 0)return 1;

			int ret = linecrfcir(line1, cir, point); //����ֱ�ߺ�Բƫ�ƺ�Ľ���
                   if(1 == ret)
			{
				
					line1.target = point[0];
					data1.SetData(line1);
					data1.Source = m_t_Target;
					m_t_Target = data1.Target;
					return 1;
				
			}
			else if(2 == ret)
			{
				if((1 == typeflag) || (3 == typeflag))
				{
					
						line1.target = point[1];
						data1.SetData(line1);
						data1.Source = m_t_Target;
						m_t_Target = data1.Target;
						return 1;
					
				}
				else if(2 == typeflag)
				{
					
						line1.target = point[0];
						data1.SetData(line1);
						data1.Source = m_t_Target;
						m_t_Target = data1.Target;
						return 1;
					
				}
			}
			else  //�޽�������
			{				 					
				LineRec insert_line;
				insert_line.Source = data1.Target;
				insert_line.Target = data1.Target;

				DataLine data  = insert_line.GetData();
				data.source   = temppoint;
				data.target   = circle.target;
				insert_line.SetData(data);
				data1.SetData( line1 );
				data1.Source = m_t_Target;
				m_t_Target = insert_line.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_line.tool.feed,curline,block_flag);
				
				RecordMsg* corner = initCorner(insert_line);
				if(corner==nullptr)
					return 0;
				
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
				m_p_toolcomp_list_msg->InsertAt(1, corner);
				return 2;
			}
		}
	}
	else if(tool_compensate_state == 1) //��������
	{
		circle.offset(offset, this, &bError);
		line1.target = circle.target;
		data1.SetData(line1);
	}
	else if(tool_compensate_state == 3) //��������
	{
		DataLine line2;
		DPlane cc = line1.target;
		line2.source.x = m_t_Target.m_df_point[0];
		line2.source.y = m_t_Target.m_df_point[1];
		line2.target = cc;
		data1.SetData( line2 );
	}

	data1.Source = m_t_Target;
	m_t_Target = data1.Target;

	if(bError)
	{
		return 2; //burke 20120416
	}
	else
		return 1;
}
//***********************************************************************//2012.11.8
int CompensateTroop::line_cir(CircleRec& data1, LineRec& data2)
{
	bool bError = false; //burke 20120416
	DPlane temppoint;
	DataCircle circle = data1.GetData();
	DataLine    line2 = data2.GetData();
	ToolOffset  offset = tool_offset(data1.tool);
//����ȡ���ж�����
	double  linevecx, linevecy, ctpointx, ctpointy;
	linevecx = line2.target.x - line2.source.x;
	linevecy = line2.target.y - line2.source.y;
	ctpointx = circle.center.x - line2.source.x;
	ctpointy = circle.center.y - line2.source.y;
	double value = linevecx * ctpointx + linevecy * ctpointy;
	int  typeflag;

	if(value > 1e-6)   //20131022 burke
		typeflag = 1;
	else if(value < -1e-6)
		typeflag = 2;
	else
		typeflag = 3;

///end
	if( tool_compensate_state == 2 )
	{
		if(line2.source == line2.target)
		{
			int nNext = findNextMove(circle.target.x, circle.target.y);

			if(nNext > 1)
			{
				GeometryRec Nextdata = GetADataAt(nNext);  //  *at(nNext);
				if(Nextdata.what == lineData)
					return( line_cir(data1, Nextdata.line) );
				else if(Nextdata.what == arcData)
					return(arc_cir(data1, Nextdata.arc) );
				else if(Nextdata.what == circleData)
					return(cir_cir(data1, Nextdata.circle) );
			}

			circle.offset( offset , this, &bError);
			data1.SetData(circle);
		}
		else
		{
			line2.offset(  offset );
			circle.offset( offset , this, &bError);
			DPlane point[2];
			double cir[3];
			temppoint=point[0] = point[1] = line2.source;

			if(stdcircle( circle.center, circle.radius, cir) == 0)return 1;

			int ret = linecrfcir(line2, cir, point);
			if(ret == 1)
			{
				temppoint= point[0];
			}
			else if(ret == 2)
			{
				if(1 == typeflag)
				{
					temppoint= point[1];
				}
				else if((2 == typeflag) || (3 == typeflag))
				{
					temppoint = point[0];
				}
			}
		 			 		 					
				LineRec insert_line;
				insert_line.Source = data1.Target;
				insert_line.Target = data1.Target;

				DataLine data  = insert_line.GetData();
				data.source   = circle.target;
				data.target   = temppoint;
				insert_line.SetData(data);
				data1.SetData( circle );
				data1.Source = m_t_Target;
				m_t_Target = insert_line.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_line.tool.feed,curline,block_flag);
				
				RecordMsg* corner = initCorner(insert_line);
				if(corner==nullptr)
					return 0;
				
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
				m_p_toolcomp_list_msg->InsertAt(1, corner);
				return 2;
			
		}
	}
	else if(tool_compensate_state == 4)
	{
		circle.offset(offset, this, &bError);
		data1.SetData(circle);
	}

	data1.Source = m_t_Target;
	//circle = data1.getData();
	//ArcAdjust( circle );
	//data1.setData(circle);
	m_t_Target = data1.Target;

	if(bError)
		return 2;
	else
		return 1;
}
//**********************************************************//
//Author:zhanghui
//Modify:2011.11.18
//Purpose:���Բ����Բ��180�Ƚ�ʱ������
//**********************************************************//
int CompensateTroop::arc_arc( ArcRec& data1, ArcRec& data2 )
{
	bool bError1 = false, bError2 = false;
	DataArc arc1 = data1.GetData(),
	        arc2 = data2.GetData();
	double arcangle1 = arc1.getAngle();
	double arcangle2 = arc2.getAngle();
//����ȡ������
	double ccvecx, ccvecy, pcx, pcy;
	ccvecx = arc2.center.x - arc1.center.x;
	ccvecy = arc2.center.y - arc1.center.y;
	pcx = arc2.center.x - arc1.target.x;
	pcy = arc2.center.y - arc1.target.y;
	double value = ccvecy * pcx - ccvecx * pcy;
	int  typeflag;

	if(value > 1e-6)   //20121022 burke
		typeflag = 1;
	else if(value < -1e-6)
		typeflag = 2;
	else
		typeflag = 3;

	DPlane temppoint[2];
	double circle1[3], circle2[3];

	if( stdcircle( arc1.center, arc1.radius, circle1) == 0)
	{
		return 1;
	}

	if( stdcircle( arc2.center, arc2.radius, circle2) == 0)
	{
		return 1;
	}

	int cirflag = circrfcir(circle1, circle2, temppoint);
///����Բ����������
	DPlane vec1, vec2, vec3, vec4;

	if(ArcVector(arc1, arc1.target, vec1) == 0)
	{
		return 1;
	}

	if(ArcVector(arc2, arc2.source, vec2) == 0)
	{
		return 1;
	}

//����Բ����ֱ������
	double angle1, angle2;

	//if(arcangle1 > 1.0)
	if((arcangle1-1.0)>EPSINON)   //20131022 burke
		angle1 = 1.0;
	else
		angle1 = arcangle1;

	//if(arcangle2 > 1.0)
	if((arcangle2-1.0)>EPSINON)   //20131022 burke
		angle2 =1.0;
	else
		angle2 = arcangle2;

	DataLine  line1, line2;
	int flag = (arc1.flag == 0 ) ? 1 : -1;
	line1.source = line1.target = arc1.target;
	(line1.source).revolve(arc1.center, flag * angle1);
	int flag1 = (arc2.flag == 0) ? -1 : 1;
	line2.source = line2.target = arc2.source;
	(line2.target).revolve(arc2.center, flag1 * angle2);
	int dir = getDirection( line1.source, line1.target, line2.target);

///�������������������ת������
	ToolOffset offset = tool_offset( data1.tool );
	double radius = fabs(offset.radius);
	int tool_flag = offset.radius > 0 ? 1 : -1;
	int type = Transition_Type(&vec1, &vec2, tool_flag);

//�������ͽ��и�����
	if(type == 1)
	{
		switch(TypeFlag)
		{
		case 0:
			TypeFlag = 1;
			break;

		case 1:
			TypeFlag = 2;
			break;

		case 2:
			TypeFlag = 1;
			break;
		}
	}
	else
		TypeFlag = 0;
//******************���������е��������**************************************//
	if(tool_compensate_state == 2)
	{
		arc1.offset( offset , this, &bError1);
		arc2.offset( offset , this, &bError2);

		if(bError1 || bError2)
			return 2;

		if(type == 2 && dir != 0) //�����Ͳ���Բ�����ɵķ�������????  �������ҪҪ���ؿ���
		{		
					ArcRec insert_arc;	//	= corner->arc.GetData();
					insert_arc.flag = dir;
					insert_arc.Source = data1.Target;
					insert_arc.Target = data1.Target;
					insert_arc.Center = data1.Target;
					insert_arc.radius = radius;
					insert_arc.tool = data1.tool;
					
					DataArc data= insert_arc.GetData();
					data.source = arc1.target;
					data.target = arc2.source; 		
					ArcAdjust( data );
					insert_arc.SetData(data);	
					data1.SetData( arc1 );	  // �޸� data1 �� Source �� Target
					data1.Source = m_t_Target;
					m_t_Target = insert_arc.Target;
				
					uint64_t curline=0;
					bool block_flag=false;
					GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);
					
					RecordMsg* corner = initCorner(insert_arc);
					if(corner==nullptr)
						return 0;  
					
			        corner->SetLineNo(curline);
				    corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
					m_p_toolcomp_list_msg->InsertAt(1, corner); 
					return 2;	
		}
		else if((type == 4)) //����4
		{
			DPlane point[2];
			double cir1[3], cir2[3];
			point[0] = point[1] = arc1.target;

			if(stdcircle( arc1.center, arc1.radius, cir1) == 0)return 1;

			if(stdcircle( arc2.center, arc2.radius, cir2) == 0)return 1;

			int ret = circrfcir(cir1, cir2, point);

			if(3 == ret || 4 == ret || 5 == ret)
			{
				arc1.target = point[0];
				data1.SetData(arc1);
			}
			else if (2 == ret)
			{
				if(1 == typeflag)
				{
					arc1.target = point[1];
					data1.SetData(arc1);
				}
				else if(2 == typeflag)
				{
					arc1.target = point[0];
					data1.SetData(arc1);
				}
				else  if(3 == typeflag)
				{
					if(cirflag == 3)
					{
						if(offset.radius > 1e-4)
						{
							arc1.target = point[1];
						}
						else
						{
							arc1.target = point[0];
						}
					}
					else if(cirflag == 4)
					{
						if(offset.radius > 1e-4)
						{
							arc1.target = point[0];
						}
						else
						{
							arc1.target = point[1];
						}
					}

					data1.SetData(arc1);
				}
			}
			else //�޽�������������ҲҪ���ؿ���
			{
				if(dir != 0)
				{						
					ArcRec insert_arc;	//	= corner->arc.GetData();
					insert_arc.flag = dir;
					insert_arc.Source = data1.Target;
					insert_arc.Target = data1.Target;
					insert_arc.Center = data1.Target;
					insert_arc.radius = radius;
					insert_arc.tool = data1.tool;
					
					DataArc data= insert_arc.GetData();
					data.source = arc1.target;
					data.target = arc2.source; 		
					ArcAdjust( data );
					insert_arc.SetData(data);	
					data1.SetData( arc1 );	  // �޸� data1 �� Source �� Target
					data1.Source = m_t_Target;
					m_t_Target = insert_arc.Target;
				
					uint64_t curline=0;
					bool block_flag=false;
					GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);
					
					RecordMsg* corner = initCorner(insert_arc);
					if(corner==nullptr)
						return 0;  
					
			        corner->SetLineNo(curline);
				    corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
					m_p_toolcomp_list_msg->InsertAt(1, corner); 
					return 2;	
				}
				else   //�����������
				{											
					ArcRec insert_arc;	//	= corner->arc.GetData();
					insert_arc.flag = dir;
					insert_arc.Source = data1.Target;
					insert_arc.Target = data1.Target;
					insert_arc.Center = data1.Target;
					insert_arc.radius = radius;
					insert_arc.tool = data1.tool;
					
					DataArc data= insert_arc.GetData();
					data.source = arc1.target;
					data.target = arc2.source; 		
					ArcAdjust( data );
					insert_arc.SetData(data);	
					data1.SetData( arc1 );	  // �޸� data1 �� Source �� Target
					data1.Source = m_t_Target;
					m_t_Target = insert_arc.Target;
				
					uint64_t curline=0;
					bool block_flag=false;
					GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);
					
					RecordMsg* corner = initCorner(insert_arc);
					if(corner==nullptr)
						return 0;  
					
			        corner->SetLineNo(curline);
				    corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
					m_p_toolcomp_list_msg->InsertAt(1, corner); 
					return 2;	
				}
			}
		}
		else if(type == 5) //180�ȴ���
		{
			data1.SetData(arc1);
			data1.Source = m_t_Target;
			//arc1 = data1.GetData();
			//ArcAdjust( arc1 );
			//data1.setData( arc1 );
			m_t_Target = data1.Target;
			return 1;
		}
		else if(type == 1) //������
		{
			DPlane point[2];
			double cir1[3], cir2[3];
			point[0] = point[1] = arc1.target;

			if(stdcircle( arc1.center, arc1.radius, cir1) == 0)return 1;

			if(stdcircle( arc2.center, arc2.radius, cir2) == 0)return 1;

			int ret = circrfcir(cir1, cir2, point);

			if(3 == ret || 4 == ret || 5 == ret)
			{
				arc1.target = point[0];
				data1.SetData(arc1);
			}
			else if (2 == ret)
			{
				if(1 == typeflag)//////////////
				{
					arc1.target = point[1];
					data1.SetData(arc1);
				
				}
				else if(2 == typeflag)
				{
					arc1.target = point[0];
					data1.SetData(arc1);
				}
				else  if(3 == typeflag)
				{
					if(cirflag == 3)
					{
						if(offset.radius > 1e-4)
						{
							arc1.target = point[1];
						}
						else
						{
							arc1.target = point[0];
						}
					}
					else if(cirflag == 4)
					{
						if(offset.radius > 1e-4)
						{
							arc1.target = point[0];
						}
						else
						{
							arc1.target = point[1];
						}
					}

					data1.SetData(arc1);
				}
			}

			else 
			{
			/*
				GeometryRec* err = InitErrorData(ERR_TOOLRADIUS);
					if(!err)						return 0;
					atInsert(0, err); */
					
            	g_compensateError = 1;
            	CreateError(ERR_TOOL_RADIUS_EXCEED, FATAL_LEVEL, CLEAR_BY_MCP_RESET);
				return 2;
			}
		}
		else if(type == 3) //��չ��
		{
			DPlane point[2];
			double cir1[3], cir2[3];
			point[0] = point[1] = arc1.target;

			if(stdcircle( arc1.center, arc1.radius, cir1) == 0)return 1;

			if(stdcircle( arc2.center, arc2.radius, cir2) == 0)return 1;

			int ret = circrfcir(cir1, cir2, point);

			if(3 == ret || 4 == ret || 5 == ret)
			{
				arc1.target = point[0];
				data1.SetData(arc1);
			}
			else if(2 == ret)
			{
				if(1 == typeflag)
				{
					arc1.target = point[1];
					data1.SetData(arc1);
				}
				else if(2 == typeflag)
				{
					arc1.target = point[0];
					data1.SetData(arc1);
				}
				else  if(3 == typeflag)
				{
					if(cirflag == 3)
					{
						if(offset.radius > 1e-4)
						{
							arc1.target = point[1];
						}
						else
						{
							arc1.target = point[0];
						}
					}
					else if(cirflag == 4)
					{
						if(offset.radius > 1e-4)
						{
							arc1.target = point[0];
						}
						else
						{
							arc1.target = point[1];
						}
					}

					data1.SetData(arc1);
				}
			}
			else  ///�޽���ҲҪ�������������
			{										
					ArcRec insert_arc;	//	= corner->arc.GetData();
					insert_arc.flag = dir;
					insert_arc.Source = data1.Target;
					insert_arc.Target = data1.Target;
					insert_arc.Center = data1.Target;
					insert_arc.radius = radius;
					insert_arc.tool = data1.tool;
					
					DataArc data= insert_arc.GetData();
					data.source = arc1.target;
					data.target = arc2.source; 		
					ArcAdjust( data );
					insert_arc.SetData(data);	
					data1.SetData( arc1 );	  // �޸� data1 �� Source �� Target
					data1.Source = m_t_Target;
					m_t_Target = insert_arc.Target;
				
					uint64_t curline=0;
					bool block_flag=false;
					GetFeedLineBlockAt(0,insert_arc.tool.feed,curline,block_flag);
					
					RecordMsg* corner = initCorner(insert_arc);
					if(corner==nullptr)
						return 0;  
					
			        corner->SetLineNo(curline);
				    corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
					m_p_toolcomp_list_msg->InsertAt(1, corner); 
					return 2;	
			}
		}
	}

	data1.Source = m_t_Target;
	//arc1 = data1.GetData();
	//ArcAdjust( arc1 );
	//data1.setData( arc1 );
	m_t_Target = data1.Target;
	return 1;
}

//*****************************************************//
//Author:zhanghui
//Modify:2011.5.10
//purpose:Բ��ת��Բ
//���õķ�����Ȼ���ȵ�����������ߵĽ���
//��Բ�����յ�ı�Ϊ������Ľ���
//Modify:2012.11.8
//Purpose:���Բ����Բ�Ľ�������
//**************************************************** //
int CompensateTroop::arc_cir( ArcRec& data1, CircleRec& data2 )
{
	bool bError1 = false, bError2 = false;
	DataArc    arc = data1.GetData();
	DataCircle circle = data2.GetData();
//����ȡ������
	double ccvecx, ccvecy, pcx, pcy;
	ccvecx = circle.center.x - arc.center.x;
	ccvecy = circle.center.y - arc.center.y;
	pcx = circle.center.x - arc.target.x;
	pcy = circle.center.y - arc.target.y;
	double value = ccvecy * pcx - ccvecx * pcy;
	int  typeflag;

	if(value > 1e-6)   //20131022 burke
		typeflag = 1;
	else if(value < -1e-6)
		typeflag = 2;
	else
		typeflag = 3;
	DPlane temppoint[2];
	double circle1[3], circle2[3];

	if( stdcircle( arc.center, arc.radius, circle1) == 0)return 1;

	if( stdcircle( circle.center, circle.radius, circle2) == 0)return 1;

	int cirflag = circrfcir(circle1, circle2, temppoint);
//end
	if( tool_compensate_state == 2 )//����������
	{
		DPlane point[2];
		double cir1[3], cir2[3];
		ToolOffset offset = tool_offset( data1.tool);
		arc.offset( offset , this, &bError1);
		circle.offset( offset , this, &bError2);
              point[0] = point[1] = arc.target;
			  
		if(bError1 || bError2)
			return 2; 
		if(stdcircle( arc.center, arc.radius, cir1) == 0)return 1;
		if(stdcircle( circle.center, circle.radius, cir2) == 0)return 1;

		int ret=circrfcir(cir1, cir2, point); 
		if(3 == ret || 4 == ret || 5 == ret)
		{
			arc.target = point[0];
			data1.SetData(arc);
		}
		else if (2 == ret)
		{
			if(1 == typeflag)
			{
				arc.target = point[1];
				data1.SetData(arc);
			}
			else if(2 == typeflag)
			{
				arc.target = point[0];
				data1.SetData(arc);
			}
			else  if(3 == typeflag)
			{
				if(cirflag == 3)
					{
						if(offset.radius > 1e-4)
						{
							arc.target = point[1];
						}
						else
						{
							arc.target = point[0];
						}
					}
					else if(cirflag == 4)
					{
						if(offset.radius > 1e-4)
						{
							arc.target = point[0];
						}
						else
						{
							arc.target = point[1];
						}
					}

					data1.SetData(arc);
				}
			}
		else
		{					 		 					
				LineRec insert_line;
				insert_line.Source = data1.Target;
				insert_line.Target = data1.Target;

				DataLine data  = insert_line.GetData();
				data.source   = arc.target;
				data.target   = circle.target;
				insert_line.SetData(data);
				data1.SetData( arc );
				data1.Source = m_t_Target;
				m_t_Target = insert_line.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_line.tool.feed,curline,block_flag);
				
				RecordMsg* corner = initCorner(insert_line);
				if(corner==nullptr)
					return 0;
				
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
				m_p_toolcomp_list_msg->InsertAt(1, corner);
				return 2;
			}
	}

	data1.Source = m_t_Target;
	//arc = data1.GetData();
	//ArcAdjust( arc );
	//data1.setData( arc );
	m_t_Target = data1.Target;
	return 1;
}

//**************************************************//
int CompensateTroop::arc_cir( CircleRec& data1, ArcRec& data2 )
{
	bool bError1 = false, bError2 = false; //burke 20120416
	DataCircle circle=data1.GetData();
	DataArc arc=data2.GetData();
       DPlane tempp;

//����ȡ������
	double ccvecx, ccvecy, pcx, pcy;
	ccvecx = arc.center.x - circle.center.x;
	ccvecy = arc.center.y - circle.center.y;
	pcx = arc.center.x - circle.target.x;
	pcy = arc.center.y - circle.target.y;
	double value = ccvecy * pcx - ccvecx * pcy;
	int  typeflag;

	if(value > 1e-6)  //20131022 burke
		typeflag = 1;
	else if(value < -1e-6)
		typeflag = 2;
	else
		typeflag = 3;
	
	DPlane temppoint[2];
	double circle1[3], circle2[3];

	if( stdcircle( circle.center, circle.radius, circle1) == 0)return 1;

	if( stdcircle( arc.center, arc.radius, circle2) == 0)return 1;

	int cirflag = circrfcir(circle1, circle2, temppoint);
//end
	if( tool_compensate_state == 2 ) //����������
	{
		DPlane point[2];
		double cir1[3], cir2[3];
		ToolOffset offset = tool_offset( data1.tool );
		circle.offset( offset , this, &bError1);
		arc.offset( offset , this, &bError2);
		tempp=point[0] = point[1] = arc.source;

		if(bError1 || bError2)
			return 2;

		if(stdcircle( circle.center, circle.radius, cir1) == 0)return 1;

		if(stdcircle( arc.center, arc.radius, cir2) == 0)return 1;

		int ret=circrfcir(cir1, cir2, point); 
		if(3 == ret || 4 == ret || 5 == ret)
		{
			tempp= point[0];
		}
		else if (2 == ret)
		{
			if(1 == typeflag)
			{
				tempp= point[1];
			}
			else if(2 == typeflag)
			{
				tempp = point[0];
			}
			else  if(3 == typeflag)
			{
				if(cirflag == 3)
					{
						if(offset.radius > 1e-4)
						{
							tempp= point[1];
						}
						else
						{
							tempp = point[0];
						}
					}
					else if(cirflag == 4)
					{
						if(offset.radius > 1e-4)
						{
							tempp= point[0];
						}
						else
						{
							tempp= point[1];
						}
					}
				}
			}	
			 			 		 					
				LineRec insert_line;
				insert_line.Source = data1.Target;
				insert_line.Target = data1.Target;

				DataLine data  = insert_line.GetData();
				data.source   = circle.target;
				data.target   = tempp;
				insert_line.SetData(data);
				data1.SetData( circle );
				data1.Source = m_t_Target;
				m_t_Target = insert_line.Target;
				
				uint64_t curline=0;
				bool block_flag=false;
				GetFeedLineBlockAt(0,insert_line.tool.feed,curline,block_flag);
				
				RecordMsg* corner = initCorner(insert_line);
				if(corner==nullptr)
					return 0;
				
			    corner->SetLineNo(curline);
				corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
				m_p_toolcomp_list_msg->InsertAt(1, corner);
				return 2;
	}

	data1.Target = m_t_Target;
	//circle = data1.GetData();
	//ArcAdjust( circle );
	//data1.setData( circle );
	m_t_Target = data1.Target;
	return 1;
}
//===================================================================//
//Modify:2012.11.8
int CompensateTroop::cir_cir(CircleRec& data1, CircleRec& data2)    
{
	bool bError1 = false, bError2 = false; 
	DataCircle circle ;

	if( tool_compensate_state == 2 )
	{
          DataCircle circle1=data1.GetData();
	      DataCircle circle2=data2.GetData();
	      ToolOffset offset = tool_offset( data1.tool );
		  circle1.offset( offset , this, &bError1);
		  circle2.offset( offset , this, &bError2);

		  if(bError1 || bError2) 
			    return 2;
		
    
		  LineRec insert_line;
		  insert_line.Source = data1.Target;
		  insert_line.Target = data1.Target;

		  DataLine data  = insert_line.GetData();
		  data.source   = circle1.target;
		  data.target   = circle2.target;
		  insert_line.SetData(data);
		  data1.SetData( circle1 );
		  data1.Source = m_t_Target;
		  m_t_Target = insert_line.Target;
				
		  uint64_t curline=0;
		  bool block_flag=false;
		  GetFeedLineBlockAt(0,insert_line.tool.feed,curline,block_flag);
			
		  RecordMsg* corner = initCorner(insert_line);
		  if(corner==nullptr)
			return 0;
		  
		  corner->SetLineNo(curline);
		  corner->SetFlag(FLAG_BLOCK_OVER, block_flag);
		  m_p_toolcomp_list_msg->InsertAt(1, corner);
			return 2;
	}

	data1.Source = m_t_Target;
	//circle = data1.GetData();
	//ArcAdjust( circle );
	//data1.setData( circle );
	m_t_Target = data1.Target;
	return 1;
}
//*********************������Ҫ�޸Ĳ���**************//
//Modify:2012.1.18
//����������ǰ
//end�����ǵ����뾶Ϊ�㣬����ֻ������״̬
//���ǰ����G40����ǰ��������ǰһ�δ���
//�����G40�򵱵�����������
//*********************************************************//
int CompensateTroop::line_end(LineRec& data1 )
{
	DataLine line1 = data1.GetData();

	if( tool_compensate_state == 3) //��������
	{
		DataLine line2;
		DPlane cc = line1.target;
		line2.source.x = m_t_Target.m_df_point[0];
		line2.source.y = m_t_Target.m_df_point[1];
		line2.target = cc;
		data1.SetData( line2 );
	}
	else if(tool_compensate_state == 4) //��������ǰһ��
	{
		line1.offset( tool_offset(data1.tool) );
		data1.SetData(line1);
	}

	data1.Source = m_t_Target;
	m_t_Target = data1.Target;
	return 1;
}
int CompensateTroop::arc_end(ArcRec& data1 )
{
	bool bError = false;
	DataArc arc;

	if( tool_compensate_state == 4)
	{
		arc = data1.GetData();
		arc.offset( tool_offset( data1.tool) , this, &bError);
		data1.SetData( arc );
	}

	data1.Source = m_t_Target;
	//arc = data1.GetData();
	//ArcAdjust( arc );
	//data1.setData( arc );
	m_t_Target = data1.Target;

	if(bError)
		return 2;
	else
		return 1;
}
int CompensateTroop::cir_end(CircleRec& data1 )
{
	bool bError = false;
	DataCircle cir;

	if( tool_compensate_state == 4 )
	{
		cir = data1.GetData();
		cir.offset( tool_offset( data1.tool ) , this, &bError);
		data1.SetData( cir );
	}

	data1.Source = m_t_Target;
	//cir = data1.GetData();
	//ArcAdjust( cir );
	//data1.setData( cir );
	m_t_Target = data1.Target;

	if(bError)
		return 2;
	else
		return 1;
}
int CompensateTroop::point_end( PointRec& data1 )
{
	if(tool_compensate_state == 4)//��������ǰһ��
	{
		LineRec lineRec1 = pointtoline(m_t_Target_old, data1);
		DataLine     line1 = lineRec1.GetData();
		line1.offset( tool_offset(lineRec1.tool) );
		lineRec1.SetData(line1);
		data1.Target = lineRec1.Target;
	}

	m_t_Target = data1.Target;
	return 1;
}
#endif
