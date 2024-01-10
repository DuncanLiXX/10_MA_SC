#include <sys/socket.h>
#include <netinet/tcp.h>

#include "servo_guide.h"
#include "global_include.h"
#include "channel_engine.h"
#include "channel_control.h"
#include "spindle_control.h"

using namespace std::chrono;

/**
 * @brief ���ݲɼ�������
 */
ServeGuide::ServeGuide()
{
}

/**
 * @brief �����ݴ���Socketͨ��
 * @return true:�ɹ�,false:ʧ��
 */
bool ServeGuide::ReadyRecord()
{
    if (state_ != E_SG_RunState::IDLE)
        return false;

    if (!g_ptr_chn_engine->SetConsumeTask(CONSUME_TYPE_SERVE_GUIDE))
    {
        state_ = E_SG_RunState::IDLE;
        return false;
    }
    scan_cycle_ = steady_clock::now();
    return true;
}

/**
 * @brief ��ʼ���ݼ�¼����¼ǰ����ǰ�����ݴ���Socketͨ��
 * @return  true:�ɹ�,false:ʧ��
 */
bool ServeGuide::StartRecord()
{
    if (state_ != E_SG_RunState::READY)
    {
        ScPrintf("StartRecord in error state: %d", (int)state_);
        return false;
    }
    ScPrintf("-----------ServeGuide::StartRecord()-----------");

    RstOriginPoint();

    state_ = E_SG_RunState::RECORDING;
    return true;
}

/**
 * @brief �������ݲɼ�
 */
void ServeGuide::PauseRecord()
{
    if (state_ == E_SG_RunState::RECORDING)
    {
        ScPrintf("-----------ServeGuide::PauseRecord()-----------");
        RstOriginPoint();
        state_ = E_SG_RunState::READY;
    }
}

/**
 * @brief ��λ���ݲɼ�״̬
 */
void ServeGuide::ResetRecord()
{
    ScPrintf("ServeGuide::ResetRecord");

    if (state_ != E_SG_RunState::IDLE)
        state_ = E_SG_RunState::STOPPING;
}

/**
 * @brief �������ݲɼ�״̬
 * @return true:���ݲɼ���, false:�����ڲɼ�״̬
 */
bool ServeGuide::RefreshRecording()
{
    if (state_ == E_SG_RunState::RECORDING)
        return true;

    if (state_ == E_SG_RunState::STOPPING)
    {
        if (IsEmpty())
        {
            std::cout << "RunState::IDLE" << std::endl;
            state_ = E_SG_RunState::IDLE;
        }
    }
    return false;
}

/**
 * @brief ������ʼ��
 */
void ServeGuide::RstOriginPoint()
{
    DPoint origin_point;//�ָ�ֵΪ00
    origin_point.x = 0;
    origin_point.y = 0;
    type_ptr_->SetOriginPoint(origin_point);
    origin_inited = false;
}

/**
 * @brief ������ʼ��
 * @param origin_point ��ʼ������
 */
void ServeGuide::SetOriginPoint(DPoint origin_point)
{
    origin_inited = true;
    type_ptr_->SetOriginPoint(origin_point);
}

/**
 * @brief ��ǰ�Ƿ�Ϊ����״̬
 * @return true:����,false:�ǿ���
 */
bool ServeGuide::IsIdle() const
{
    return state_ == E_SG_RunState::IDLE;
}

/**
 * @brief �Ƿ����������ݲɼ�״̬
 * @return true:���ݲɼ�,false:�����ݲɼ�
 */
bool ServeGuide::IsRecord() const
{
    return state_ == E_SG_RunState::RECORDING;
}

/**
 * @brief �Ƿ�������ͨ��׼��״̬
 * @return
 */
bool ServeGuide::IsReady() const
{
    return state_ == E_SG_RunState::READY;
}

/**
 * @brief ���òɼ�ʱ����
 * @param interval ʱ����
 * @return true:�ɹ�,false:ʧ��
 */
bool ServeGuide::SetInterval(unsigned interval)
{
    if (interval > MAX_INTERVAL || interval == 0)
        return false;
    scan_interval_ = interval;
    return true;
}

/**
 * @brief �Ƿ񵽴���ʱ��
 * @return true:����,false:δ����
 */
bool ServeGuide::IsTimeout()
{
    if (duration_cast<milliseconds>(steady_clock::now() - scan_cycle_).count() >= scan_interval_)
    {
        //std::cout << "Time interval: " << duration_cast<milliseconds>(steady_clock::now() - scan_cycle_).count() << std::endl;
        scan_cycle_ = steady_clock::now();//TODO ��ʱ��Ӧ�÷�������ȽϺ���
        return true;
    }
    return false;
}

/**
 * @brief ���������Ƿ�׼���ã�׼���òſ��Խ������ݴ���
 * @return true:׼����,false:δ׼����
 */
bool ServeGuide::IsDataReady()
{
    //����������cpu��������ʱ�����Եȷ��Ͷ����е����ݻ��۵�ĳһ����ʱ��һ������
    return true;
}


/**
 * @brief ��¼�����ڷ��Ͷ�����
 * @param ��Ҫ��¼������
 */
void ServeGuide::RecordData(const double *, const double *interp)
{
    if (!origin_inited)
    {
        DPoint origin;
        origin.x = interp[type_ptr_->axis_one_];
        origin.y = interp[type_ptr_->axis_two_];
        SetOriginPoint(origin);
    }
    SG_DATA data = type_ptr_->GenData();
    std::lock_guard<std::mutex> mut(data_mut_);
    data_.push(std::move(data));//����ѹ�뷢�Ͷ�����
}

/**
 * @brief ���ݷ���
 */
void ServeGuide::SendData()
{
    SG_DATA data;
    {
        std::lock_guard<std::mutex> mut(data_mut_);
        if (!data_.empty())
        {
            data = std::move(data_.front());
            data_.pop();
        }
        else
            return;
    }

    if(-1 == send(data_send_fd, &data, sizeof(data), MSG_NOSIGNAL)){
        ScPrintf("ServeGuide: data send error");
    }
}

/**
 * @brief ��ʼ������Socket
 */
bool ServeGuide::InitSocket()
{
    if (data_socket_ != -1)
        return true;
    int keepalive = 1;              // ����TCP KeepAlive����
    int KeepAliveProbes = 2;		// ���Դ���
    int KeepAliveInterval = 1;		// ���������ļ������λ����
    int KeepAliveTime = 1;			// ���ӽ������ÿ�ʼ��������λ����

    struct timeval tcp_sock_timeout = {5, 0};   // ���ͳ�ʱʱ��
    int reuse = 1;                              // ��ַ������

    data_socket_ = socket(AF_INET, SOCK_STREAM, 0);

    if (setsockopt(data_socket_, SOL_SOCKET, SO_RCVTIMEO, (char *)&tcp_sock_timeout, sizeof(timeval)) != 0) //���ó�ʱʱ��
        std::cout << "set SO_RCVTIMEO error" << std::endl;
    if (setsockopt(data_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) //���ÿɴ���״̬
        std::cout << "set SO_REUSEADDR error" << std::endl;
    if (setsockopt(data_socket_, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepalive, sizeof(keepalive)) != 0) //����������
        std::cout << "set SO_KEEPALIVE error" << std::endl;
    if (setsockopt(data_socket_, IPPROTO_TCP, TCP_KEEPCNT, (void *)&KeepAliveProbes, sizeof(KeepAliveProbes)) != 0) //�������������Դ���
        std::cout << "set TCP_KEEPCNT error" << std::endl;
    if (setsockopt(data_socket_, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&KeepAliveTime, sizeof(KeepAliveTime)) != 0) //��������������ʱ��
        std::cout << "set TCP_KEEPIDLE error" << std::endl;
    if (setsockopt(data_socket_, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&KeepAliveInterval, sizeof(KeepAliveInterval)) != 0) //�������������ͼ��
        std::cout << "set TCP_KEEPINTVL error" << std::endl;

    sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT_TCP_DATA);

    if (bind(data_socket_, (struct sockaddr *)&addr, sizeof(sockaddr)) == -1) {
        std::cout << "bind error: " << errno << std::endl;
        return false;
    }

    if(listen(data_socket_, 5)!= 0){
        std::cout << "listen error: " << errno << std::endl;
        close(data_socket_);
        data_socket_ = -1;
        return false;//TODO ʧЧ���������ιر�SOCKET
    }

    return true;
}

/**
 * @brief �ȴ��ͻ�������
 * @return true:���ӳɹ�,false:����ʧ��
 */
bool ServeGuide::Accept()
{
    sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT_TCP_DATA);

    socklen_t len = sizeof(sockaddr);
    data_send_fd = accept(data_socket_, (struct sockaddr *)&addr, &len);
    if(data_send_fd <= 0){//���ӳ���
        std::cout << "accept error: " << data_send_fd << std::endl;
        return false;
    }

//    int reuse = 1;
//    if (setsockopt(data_send_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) //���ÿɴ���״̬
//        std::cout << "set SO_REUSEADDR error" << std::endl;
//    if (setsockopt(data_send_fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) != 0) //���ÿɴ���״̬
//        std::cout << "set SO_REUSEPORT error" << std::endl;

    state_ = E_SG_RunState::READY;
    std::cout << "ServeGuide::Accept" << std::endl;
    return true;
}

/**
 * @brief �ر����ݴ���Socketͨ��
 * @return
 */
bool ServeGuide::Close()
{
    if(data_send_fd > 0)
    {
        auto time = steady_clock::now();
        if (close(data_send_fd) == -1)
        {
            std::cout << "close err " << std::endl;
            //return false;
        }
        data_send_fd = -1;
        sync();
        auto t = duration_cast<milliseconds>(steady_clock::now() - time).count();
        printf("close data trans socket! %lld\n", t);
    }
    return true;
}

/**
 * @brief ����ɼ��õ�����
 */
void ServeGuide::ProcessData()
{
    while(state_ != E_SG_RunState::IDLE)
    {
        if (IsDataReady())
        {
            SendData();
        }
    }
    ScPrintf("ServeGuide: process data finish");
    return;
}

/**
 * @brief ���Ͷ����Ƿ�Ϊ��
 * @return true:��,false:�ǿ�
 */
bool ServeGuide::IsEmpty() const
{
    std::lock_guard<std::mutex> mut(data_mut_);
    return data_.empty();
}

int ServeGuide::CurState() const
{
    return (int)state_;
}

/**
 * @brief �����ŷ���������,�����ڿ���״̬������
 * @param type:�μ�E_SG_Type
 * @return true:�ɹ�,false:ʧ��
 */
bool ServeGuide::SetType(SG_Type_Ptr type)
{
    SetInterval(type->interval_);
    type_ptr_ = type;
    return true;
}



//--------------------------------- type --------------------------------------

SG_Rect_Type::SG_Rect_Type(SG_Rect_Config cfg)
    : SG_Type(cfg.interval, cfg.axis_one, cfg.axis_two, E_SG_Type::SG_Rect)
{

}

void SG_Type::SetInstance(double *intp, double *feedback, double *speed)
{
    intp_pos_ = intp;
    feedback_pos_ = feedback;
    feedback_speed_ = speed;
}

/**
 * @brief ��������������
 * @param origin_point���������ֵ
 */
void SG_Type::SetOriginPoint(DPoint origin_point)
{
    origin_point_.x = origin_point.x;
    origin_point_.y = origin_point.y;
    std::cout << "OriginPoint: x------> " << origin_point.x << " y------> " << origin_point.y << std::endl;
}

bool SG_Type::Verify() const
{
    if (axis_one_ < 0)
    {
        return false;
    }
    if (axis_two_ < 0)
    {
        return false;
    }
    if (interval_ <= 0)
    {
        return false;
    }
    if (feedback_pos_ == nullptr || intp_pos_ == nullptr)
    {
        return false;
    }
    return true;
}

SG_DATA SG_Rect_Type::GenData()
{
    SG_DATA data = std::make_tuple(feedback_pos_[axis_one_] - origin_point_.x, feedback_pos_[axis_two_] - origin_point_.y, -1, -1);
    return data;
}

SG_Circle_Type::SG_Circle_Type(SG_Circle_Config cfg)
    : SG_Type(cfg.interval, cfg.axis_one, cfg.axis_two, E_SG_Type::SG_Circle)
{
    radius_ = cfg.radius;
}

bool SG_Circle_Type::Verify() const
{
    if (!SG_Type::Verify())
        return false;

    if (radius_ <= 0)
    {
        return false;
    }

    return true;
}

SG_DATA CircleDletaCalc(DPlane point, DPlane pole, double radius)
{
    //������ת��
    DPlane pos;
    pos.x = point.x - pole.x;
    pos.y = point.y - pole.y;

    //����ʵ�ʰ뾶
    double radius_real = sqrt(pos.x * pos.x + pos.y * pos.y);

    //��н�
    double radian = atan2(pos.y, pos.x);

    //��������1,2
    DPlane standard_pt;
    standard_pt.x = radius * cos(radian);//!!
    standard_pt.y = radius * sin(radian);

    //����ƫ��
    double delta = radius_real - radius;

    //�����Ӧ��ƫ��1,2
    DPlane delta_pt;
    delta_pt.x = delta*cos(radian);
    delta_pt.y = delta*sin(radian);

    SG_DATA data = std::make_tuple(standard_pt.x, standard_pt.y, delta_pt.x, delta_pt.y);
    return data;
}


void CoordTransform(SG_DATA &origin, DPlane offset)
{
    std::get<0>(origin) = std::get<0>(origin) + offset.x;
    std::get<1>(origin) = std::get<1>(origin) + offset.y;
}

SG_DATA SG_Circle_Type::GenData()
{
    //ת��������ϵ
    DPlane pole_point(-radius_, 0);
    DPlane point(feedback_pos_[axis_one_] - origin_point_.x, feedback_pos_[axis_two_] - origin_point_.y);

    SG_DATA data = CircleDletaCalc(point, pole_point, radius_);
    return data;
}

SG_RecCir_Type::SG_RecCir_Type(SG_RecCir_Config cfg)
    : SG_Type(cfg.interval, cfg.axis_one, cfg.axis_two, E_SG_Type::SG_Rect_Circle)
{
    width_ = cfg.width;
    height_ = cfg.height;
    radius_ = cfg.radius;
}

SG_DATA SG_RecCir_Type::GenData()
{
    //������
    double center_x = width_ / 2;
    double center_y = -(height_ + 2 * radius_) / 2;

    double relative_feed_pos_1 = feedback_pos_[axis_one_] - origin_point_.x;
    double relative_feed_pos_2 = feedback_pos_[axis_two_] - origin_point_.y;
    //������ת��
    double pos_1 = relative_feed_pos_1 - center_x;
    double pos_2 = relative_feed_pos_2 - center_y;
    DPlane pos(pos_1, pos_2);

    //�������꣬��ֹ�����������������޼������
    double relative_interp_pos_1 = intp_pos_[axis_one_] - origin_point_.x;
    double relative_interp_pos_2 = intp_pos_[axis_two_] - origin_point_.y;
    double interp_pos_1 = relative_interp_pos_1 - center_x;
    double interp_pos_2 = relative_interp_pos_2 - center_y;
    DPlane interp_pos(interp_pos_1, interp_pos_2);

    //�жϵ�ǰ���괦�ڵڼ�����
    int quadrant = 0;
    //��ȡ����

    double delta_x = 0;
    double delta_y = 0;

    quadrant = GetQuadrant(interp_pos);
    std::cout << "-->" << "X: " << interp_pos.x << " Y: " << interp_pos.y << " Q:" << (int)quadrant << std::endl;
    SG_DATA data = std::make_tuple(-1, -1, -1, -1);
    switch (quadrant) {
    case 1:
    {//��һ���ޣ���ֱ��
        delta_x = 0;
        delta_y = pos_2 + center_y;
        data = std::make_tuple(pos.x, (height_ + 2 * radius_) / 2, delta_x, delta_y);
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 2:
    {//�ڶ�����,1/4Բ
        //DPlane pole_point(-width_/2, height_/2);
        DPlane pole_point(width_/2, height_/2);
        data = CircleDletaCalc(pos, pole_point, radius_);
        CoordTransform(data, DPlane(width_/2, height_/2));
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 3:
    {//�������ޣ���ֱ��
        delta_x = pos_1 - (width_/2 + radius_);
        delta_y = 0;
        data = std::make_tuple(width_/2 + radius_, pos.y, delta_x, delta_y);
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 4:
    {//��������,1/4Բ
        DPlane pole_point(width_/2, -height_/2);
        data = CircleDletaCalc(pos, pole_point, radius_);
        CoordTransform(data, DPlane(width_/2, -height_/2));
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 5:
    {//�������ޣ���ֱ��
        delta_x = 0;
        delta_y = pos_2 + (height_/2 + radius_);
        data = std::make_tuple(pos.x, -(height_ + 2 * radius_)/2, delta_x, delta_y);
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 6:
    {//��������,1/4Բ
        DPlane pole_point(-width_/2, -height_/2);
        data = CircleDletaCalc(pos, pole_point, radius_);
        CoordTransform(data, DPlane(-width_/2, -height_/2));
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 7:
    {//�������ޣ���ֱ��
        delta_x = pos_1 + (radius_ + width_/2);
        delta_y = 0;
        data = std::make_tuple(-(radius_+width_/2), pos.y, delta_x, delta_y);
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 8:
    {//�ڰ�����,1/4Բ
        DPlane pole_point(-width_/2, height_/2);
        data = CircleDletaCalc(pos, pole_point, radius_);
        CoordTransform(data, DPlane(-width_/2, height_/2));
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    default:
        break;
    }
    return data;
}

//ʹ�����ļ�����
int SG_RecCir_Type::GetQuadrant(DPlane plane)
{
    //�ж������ڵڼ�����(1-8)
    //��һ����
    if (plane.x > -width_/2 && plane.x < width_/2
            && abs(plane.y - (height_/2 + radius_)) < EPSINON)
    {
        return 1;
    }

    //�ڶ�����
    if (plane.x >= width_/2 && plane.x <= width_/2 + radius_
       && plane.y >= height_/2 && plane.y <= height_/2 + radius_)
    {
        return 2;
    }

    //��������
    if (abs(plane.x - (width_/2 + radius_)) < EPSINON
        && plane.y >= -height_/2 && plane.y <= height_/2)
    {
        return 3;
    }

    //��������
    if (plane.x >= width_ / 2 && plane.x <= width_ / 2 + radius_
        && plane.y >= -height_/2 - radius_ && plane.y <= -height_/2)
    {
        return 4;
    }

    //��������
    if (plane.x > -width_/2 && plane.x < width_/2
            && abs(plane.y - (-height_/2 - radius_)) < EPSINON)
    {
        return 5;
    }

    //��������
    if (plane.x >= -width_/2 - radius_ && plane.x <= -width_/2
        && plane.y >= -height_/2 - radius_ && plane.y <= -height_/2)
    {
        return 6;
    }

    //��������
    if (abs(plane.x - (-width_/2-radius_)) < EPSINON
            && plane.y > -height_/2 && plane.y < height_/2)
    {
        return 7;
    }

    //�ڰ�����
    if ((plane.x >= -width_/2 - radius_) && (plane.x <= -width_/2)
        && (plane.y >= height_/2) && (plane.y <= height_/2 + radius_))
    {
        return 8;
    }

    return -1;
}

bool SG_RecCir_Type::Verify() const
{
    if (!SG_Type::Verify())
        return false;

    if (width_ > 0 && height_ > 0 && radius_ > 0)
        return true;
    else
        return false;
}

SG_Tapping_Type::SG_Tapping_Type(SG_Tapping_Config cfg)
    : SG_Type(cfg.interval, -1, -1, E_SG_Type::SG_Tapping)//���Թ�˿�����⴦��
{   
    for (int i = 0; i < g_ptr_parm_manager->GetSystemConfig()->axis_count; ++i)
    {
        if (g_ptr_parm_manager->GetAxisConfig()[i].axis_type == AXIS_SPINDLE)
        {
            axis_one_ = i;
            break;
        }
    }
    axis_two_ = 2;
    std::cout << "one " << (int)axis_one_ << " two " << (int)axis_two_ << std::endl;
}

bool SG_Tapping_Type::Verify() const
{
    if (!SG_Type::Verify())
        return false;
    if (feedback_speed_ == nullptr)
        return false;
    return true;
}

SG_DATA SG_Tapping_Type::GenData()
{
    ChannelControl *control = g_ptr_chn_engine->GetChnControl(g_ptr_chn_engine->GetCurChannelIndex());

    double spd_feedback = feedback_pos_[axis_one_] - origin_point_.x;
    double z_axis_feedback = feedback_pos_[axis_two_] - origin_point_.y;
    double z_axis_inp = intp_pos_[axis_two_] - origin_point_.y;

    double spd_trans = 0;
    if (control->CheckFRegState(65, 0))//RGSPP(F65.0)
    {// RGSPP �źŵ�ͨ�󣬲ſ�ʼ�ɼ���������
        double radio = control->GetSpdCtrl()->Get_TapRatio();
        if (radio != 1) radio = radio / 10000;//radio �Ǵ���MI�Ĳ�������Ҫ10000ת��

        spd_trans = spd_feedback/radio;
    }
    //ת��Ϊmmÿ����
    SG_DATA data = std::make_tuple(spd_trans, z_axis_feedback, z_axis_inp, feedback_speed_[axis_two_]/1000*60);
    return data;
}

//bool SG_Register_Type::Verify() const
//{
//    return true;
//}

//SG_DATA SG_Register_Type::GenData()
//{
//    //��ȡ��ǰ����ͨ��
//    //����
//    //��ȡ��Ӧ��ļĴ���ֵ
//    //���ؽ��
//    g_ptr_chn_engine->MonitorRegisterData(cfg_register);
//    return SG_DATA{};
//}
