#include <sys/socket.h>
#include <netinet/tcp.h>

#include "servo_guide.h"
#include "global_include.h"
#include "channel_engine.h"

using namespace std::chrono;

/**
 * @brief 数据采集功能类
 */
ServeGuide::ServeGuide()
{
}

/**
 * @brief 打开数据传输Socket通道
 * @return true:成功,false:失败
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
 * @brief 开始数据记录，记录前需提前打开数据传输Socket通道
 * @return  true:成功,false:失败
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
 * @brief 结束数据采集
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
 * @brief 复位数据采集状态（方便测试用，一般情况下用不到）
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
 * @brief 更新数据采集状态
 * @return true:数据采集中, false:不处于采集状态
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
 * @brief 当前是否为空闲状态
 * @return true:空闲,false:非空闲
 */
bool ServeGuide::IsIdle() const
{
    return state_ == E_SG_RunState::IDLE;
}

/**
 * @brief 是否正处于数据采集状态
 * @return true:数据采集,false:非数据采集
 */
bool ServeGuide::IsRecord() const
{
    return state_ == E_SG_RunState::RECORDING;
}

/**
 * @brief 是否处于数据通道准备状态
 * @return
 */
bool ServeGuide::IsReady() const
{
    return state_ == E_SG_RunState::READY;
}

/**
 * @brief 设置采集时间间隔
 * @param interval 时间间隔
 * @return true:成功,false:失败
 */
bool ServeGuide::SetInterval(unsigned interval)
{
    if (interval > MAX_INTERVAL || interval == 0)
        return false;
    scan_interval_ = interval;
    return true;
}

/**
 * @brief 是否到达间隔时间
 * @return true:到达,false:未到达
 */
bool ServeGuide::IsTimeout()
{
    if (duration_cast<milliseconds>(steady_clock::now() - scan_cycle_).count() > scan_interval_)
    {
        scan_cycle_ = steady_clock::now();//TODO 计时点应该放在哪里比较合适
        return true;
    }
    return false;
}

/**
 * @brief 发送数据是否准备好，准备好才可以进行数据传送
 * @return true:准备好,false:未准备好
 */
bool ServeGuide::IsDataReady()
{
    //std::lock_guard<std::mutex> mut(data_mut_);
    //if (data_.size() > 10)
    return true;
}


/**
 * @brief 记录数据于发送队列中
 * @param 需要记录的数据
 */
void ServeGuide::RecordData(const double *feedback)
{
    //不同类型生成不同DATA
    SG_DATA data = type_ptr_->GenData(feedback);
    std::lock_guard<std::mutex> mut(data_mut_);
    data_.push(std::move(data));
}

/**
 * @brief 数据发送
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
 * @brief 初始化传输Socket
 */
bool ServeGuide::InitSocket()
{
    if (data_socket_ != -1)
        return true;
    int keepalive = 1;              // 开启TCP KeepAlive功能
    int KeepAliveProbes = 2;		// 重试次数
    int KeepAliveInterval = 1;		// 两次心跳的间隔，单位：秒
    int KeepAliveTime = 1;			// 连接建立后多久开始心跳，单位：秒

    struct timeval tcp_sock_timeout = {5, 0};   // 发送超时时间
    int reuse = 1;                              // 地址可重用

    data_socket_ = socket(AF_INET, SOCK_STREAM, 0);

    if (setsockopt(data_socket_, SOL_SOCKET, SO_RCVTIMEO, (char *)&tcp_sock_timeout, sizeof(timeval)) != 0) //设置超时时间
        std::cout << "set SO_RCVTIMEO error" << std::endl;
    if (setsockopt(data_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) //设置可从用状态
        std::cout << "set SO_REUSEADDR error" << std::endl;
    if (setsockopt(data_socket_, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepalive, sizeof(keepalive)) != 0) //设置心跳包
        std::cout << "set SO_KEEPALIVE error" << std::endl;
    if (setsockopt(data_socket_, IPPROTO_TCP, TCP_KEEPCNT, (void *)&KeepAliveProbes, sizeof(KeepAliveProbes)) != 0) //设置心跳包重试次数
        std::cout << "set TCP_KEEPCNT error" << std::endl;
    if (setsockopt(data_socket_, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&KeepAliveTime, sizeof(KeepAliveTime)) != 0) //设置心跳包发送时间
        std::cout << "set TCP_KEEPIDLE error" << std::endl;
    if (setsockopt(data_socket_, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&KeepAliveInterval, sizeof(KeepAliveInterval)) != 0) //设置心跳包发送间隔
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
        return false;//TODO 失效的情况下如何关闭SOCKET
    }

    return true;
}

/**
 * @brief 等待客户端连接
 * @return true:连接成功,false:连接失败
 */
bool ServeGuide::Accept()
{
    std::cout << "ServeGuide::Accept begin" << std::endl;
    sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT_TCP_DATA);

    //TODO 超时处理
    socklen_t len = sizeof(sockaddr);
    data_send_fd = accept(data_socket_, (struct sockaddr *)&addr, &len);
    if(data_send_fd <= 0){//连接出错
        std::cout << "accept error: " << data_send_fd << std::endl;
        return false;
    }

//    int reuse = 1;
//    if (setsockopt(data_send_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) //设置可从用状态
//        std::cout << "set SO_REUSEADDR error" << std::endl;
//    if (setsockopt(data_send_fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) != 0) //设置可从用状态
//        std::cout << "set SO_REUSEPORT error" << std::endl;

    state_ = E_SG_RunState::READY;
    std::cout << "ServeGuide::Accept end" << std::endl;
    return true;
}

/**
 * @brief 关闭数据传输Socket通道
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
 * @brief 处理采集好的数据
 */
void ServeGuide::ProcessData()
{
    while(state_ != E_SG_RunState::IDLE)
    {
        if (IsDataReady())//发送数据
        {
            SendData();
        }
    }
    std::cout << "process data finish" << std::endl;
    return;
}

/**
 * @brief 发送队列是否为空
 * @return true:空,false:非空
 */
bool ServeGuide::IsEmpty() const
{
    std::lock_guard<std::mutex> mut(data_mut_);
    return data_.empty();
}

/**
 * @brief 设置伺服引导类型,必须在空闲状态下设置
 * @param type:参见E_SG_Type
 * @return true:成功,false:失败
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
    //转换极坐标系
    double pos_1 = feedback[axis_one_] - radius_1;
    double pos_2 = feedback[axis_two_] - radius_2;
    //计算实际半径
    double radius_real = sqrt(pos_1 * pos_1 + pos_2 * pos_2);
    //求夹角
    double radian = atan2(pos_2, pos_1);
    //理论坐标1,2
    double standard_x = radius_ * cos(radian);
    double standard_y = radius_ * sin(radian);
    //计算偏差
    double delta = radius_real - radius_;
    //计算对应轴偏差1,2
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
    : SG_Type(cfg.interval, -1, -1)//刚性攻丝需特殊处理
{

}

SG_DATA SG_Tapping_Type::GenData(const double *feedback)
{

}
