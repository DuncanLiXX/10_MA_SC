#include <sys/socket.h>
#include <netinet/tcp.h>

#include "servo_guide.h"
#include "global_include.h"
#include "channel_engine.h"

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

    if (!g_ptr_chn_engine->SetConsumeTask(CONSUME_TYPE_SERVE_GUIDE_READY))
    {
        state_ = E_SG_RunState::IDLE;
        return false;
    }
    return true;
}

/**
 * @brief ��ʼ���ݼ�¼����¼ǰ����ǰ�����ݴ���Socketͨ��
 * @return  true:�ɹ�,false:ʧ��
 */
bool ServeGuide::StartRecord()
{
    if (state_ != E_SG_RunState::READY)
        return false;
    ScPrintf("-----------ServeGuide::StartRecord()-----------");
    scan_cycle_ = steady_clock::now();
    state_ = E_SG_RunState::RECORDING;

    if (!g_ptr_chn_engine->SetConsumeTask(CONSUME_TYPE_SERVE_GUIDE_RECORD))
    {
        state_ = E_SG_RunState::IDLE;
        return false;
    }
    return true;
}

/**
 * @brief �������ݲɼ�
 */
void ServeGuide::StopRecord()
{
    if (state_ == E_SG_RunState::RECORDING)
    {
        ScPrintf("-----------ServeGuide::StopRecord()-----------");
        state_ = E_SG_RunState::STOPPING;
    }
}

/**
 * @brief ��λ���ݲɼ�״̬����������ã�һ��������ò�����
 */
void ServeGuide::ResetRecord()
{
    {
        std::lock_guard<std::mutex> mut(data_mut_);
        while (!data_.empty()) data_.pop();
    }
    state_ = E_SG_RunState::IDLE;
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
    if (duration_cast<milliseconds>(steady_clock::now() - scan_cycle_).count() > scan_interval_)
    {
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
    //std::lock_guard<std::mutex> mut(data_mut_);
    //if (data_.size() > 10)
    return true;
}


/**
 * @brief ��¼�����ڷ��Ͷ�����
 * @param ��Ҫ��¼������
 */
void ServeGuide::RecordData(const double *feedback)
{
    //��ͬ�������ɲ�ͬDATA
    SG_DATA data = type_ptr_->GenData(feedback);
    std::lock_guard<std::mutex> mut(data_mut_);
    data_.push(std::move(data));
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

    if( -1 == send(data_send_fd, &data, sizeof(data), MSG_NOSIGNAL)){
        std::cout << "Data send error" << std::endl;
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
    std::cout << "ServeGuide::Accept begin" << std::endl;
    sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT_TCP_DATA);

    //TODO ��ʱ����
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
    std::cout << "ServeGuide::Accept end" << std::endl;
    return true;
}

/**
 * @brief �ر����ݴ���Socketͨ��
 * @return
 */
bool ServeGuide::Close()
{
    if(data_send_fd > 0){

        auto time = steady_clock::now();
        if (close(data_send_fd) == -1)
        {
            std::cout << "close err " << std::endl;
            //return false;
        }
        data_send_fd = -1;
        sync();
        auto t = duration_cast<milliseconds>(steady_clock::now() - time).count();
        printf("close data trans socket! %ld\n", t);
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
        if (IsDataReady())//��������
        {
            SendData();
        }
    }
    std::cout << "process data finish" << std::endl;
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
    : SG_Type(cfg.interval, cfg.axis_one, cfg.axis_two)
{

}

SG_DATA SG_Rect_Type::GenData(const double *feedback)
{
    SG_DATA data = std::make_tuple(feedback[axis_one_], feedback[axis_two_], -1, -1);
    return data;
}

SG_Circle_Type::SG_Circle_Type(SG_Circle_Config cfg)
    : SG_Type(cfg.interval, cfg.axis_one, cfg.axis_two)
{
    radius_ = cfg.raduis;
    cfg.circle_type == 0 ? circle_type_ = E_SG_CType::SG_CW
            : circle_type_ = E_SG_CType::SG_CCW;
}

bool SG_Circle_Type::Verify() const
{
    if (radius_ <= 0)
        return false;
    if (circle_type_ == E_SG_CType::SG_None)
        return false;
}

SG_DATA SG_Circle_Type::GenData(const double *feedback)
{
    double radius_1 = circle_type_ == E_SG_CType::SG_CW ? radius_ : - radius_;
    double radius_2 = 0;
    //ת��������ϵ
    double pos_1 = feedback[axis_one_] - radius_1;
    double pos_2 = feedback[axis_two_] - radius_2;
    //����ʵ�ʰ뾶
    double radius_real = sqrt(pos_1 * pos_1 + pos_2 * pos_2);
    //��н�
    double radian = atan2(pos_2, pos_1);
    //��������1,2
    double standard_x = radius_ * cos(radian);
    double standard_y = radius_ * sin(radian);
    //����ƫ��
    double delta = radius_real - radius_;
    //�����Ӧ��ƫ��1,2
    double delta_x = delta*cos(radian);
    double delta_y = delta*sin(radian);

    SG_DATA data = std::make_tuple(standard_x, standard_y, delta_x, delta_y);
    return data;
}

SG_RecCir_Type::SG_RecCir_Type(SG_RecCir_Config cfg)
    : SG_Type(cfg.interval, cfg.axis_one, cfg.axis_two)
{
    width_ = cfg.width;
    height_ = cfg.height;
    radius_ = cfg.radius;
}

SG_DATA SG_RecCir_Type::GenData(const double *feedback)
{

}

bool SG_RecCir_Type::Verify() const
{
    if (width_ > 0 && height_ > 0 && radius_ > 0)
        return true;
    else
        return false;
}

SG_Tapping_Type::SG_Tapping_Type(SG_Tapping_Config cfg)
    : SG_Type(cfg.interval, -1, -1)//���Թ�˿�����⴦��
{

}

SG_DATA SG_Tapping_Type::GenData(const double *feedback)
{

}
