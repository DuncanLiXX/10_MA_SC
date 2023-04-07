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

    if (!g_ptr_chn_engine->SetConsumeTask(CONSUME_TYPE_SERVE_GUIDE))
    {
        state_ = E_SG_RunState::IDLE;
        return false;
    }
    scan_cycle_ = steady_clock::now();
    return true;
}

/**
 * @brief 开始数据记录，记录前需提前打开数据传输Socket通道
 * @return  true:成功,false:失败
 */
bool ServeGuide::StartRecord()
{
    if (state_ != E_SG_RunState::READY)
    {
        ScPrintf("StartRecord in error state: %d", (int)state_);
        return false;
    }
    ScPrintf("-----------ServeGuide::StartRecord()-----------");
    state_ = E_SG_RunState::RECORDING;
    return true;
}

/**
 * @brief 结束数据采集
 */
void ServeGuide::PauseRecord()
{
    if (state_ == E_SG_RunState::RECORDING)
    {
        ScPrintf("-----------ServeGuide::PauseRecord()-----------");
        state_ = E_SG_RunState::READY;
    }
}

/**
 * @brief 复位数据采集状态
 */
void ServeGuide::ResetRecord()
{
    if (state_ == E_SG_RunState::READY)
    {
        {
            std::lock_guard<std::mutex> mut(data_mut_);
            while (!data_.empty()) data_.pop();
        }
        state_ = E_SG_RunState::IDLE;
    }
    else if (state_ == E_SG_RunState::RECORDING) //需要等待所有数据上传完成后再停止
        state_ = E_SG_RunState::STOPPING;
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
void ServeGuide::RecordData(const double *feedback, const double *interp)
{
    //不同类型生成不同DATA
    SG_DATA data = type_ptr_->GenData(feedback, interp);
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
    : SG_Type(cfg.interval, cfg.axis_one, cfg.axis_two, E_SG_Type::SG_Rect)
{

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
    return true;
}

SG_DATA SG_Rect_Type::GenData(const double *feedback, const double *interp)
{
    SG_DATA data = std::make_tuple(feedback[axis_one_], feedback[axis_two_], -1, -1);
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
    //极坐标转换
    DPlane pos;
    pos.x = point.x - pole.x;
    pos.y = point.y - pole.y;

    //计算实际半径
    double radius_real = sqrt(pos.x * pos.x + pos.y * pos.y);

    //求夹角
    double radian = atan2(pos.y, pos.x);

    //理论坐标1,2
    DPlane standard_pt;
    standard_pt.x = radius * cos(radian);//!!
    standard_pt.y = radius * sin(radian);

    //计算偏差
    double delta = radius_real - radius;

    //计算对应轴偏差1,2
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

SG_DATA SG_Circle_Type::GenData(const double *feedback, const double *interp)
{
    //要加入起始点，否则起始点必须要是（0，0）
    //转换极坐标系
    DPlane pole_point(-radius_, 0);
    DPlane point(feedback[axis_one_], feedback[axis_two_]);

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

SG_DATA SG_RecCir_Type::GenData(const double *feedback, const double *interp)
{
    //极坐标
    //是否需要起始点
    double center_x = width_ / 2;
    double center_y = -(height_ + 2 * radius_) / 2;

    //极坐标转换
    double pos_1 = feedback[axis_one_] - center_x;
    double pos_2 = feedback[axis_two_] - center_y;
    DPlane pos(pos_1, pos_2);

    //理论坐标，防止坐标跳动，导致象限计算错误
    double interp_pos_1 = interp[axis_one_] - center_x;
    double interp_pos_2 = interp[axis_two_] - center_y;
    DPlane interp_pos(interp_pos_1, interp_pos_2);

    //判断当前坐标处于第几象限
    int quadrant = 0;
    //获取象限

    double delta_x = 0;
    double delta_y = 0;

    quadrant = GetQuadrant(interp_pos);
    //std::cout << "-->" << "X: " << pos.x << "Y: " << pos.y << " Q:" << (int)quadrant << std::endl;
    SG_DATA data = std::make_tuple(-1, -1, -1, -1);
    switch (quadrant) {
    case 1:
    {//第一象限，横直线
        delta_x = 0;
        delta_y = pos_2 + center_y;
        data = std::make_tuple(pos.x, (height_ + 2 * radius_) / 2, delta_x, delta_y);
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 2:
    {//第二象限,1/4圆
        //DPlane pole_point(-width_/2, height_/2);
        DPlane pole_point(width_/2, height_/2);
        data = CircleDletaCalc(pos, pole_point, radius_);
        CoordTransform(data, DPlane(width_/2, height_/2));
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 3:
    {//第三象限，竖直线
        delta_x = pos_1 - (width_/2 + radius_);
        delta_y = 0;
        data = std::make_tuple(width_/2 + radius_, pos.y, delta_x, delta_y);
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 4:
    {//第四象限,1/4圆
        DPlane pole_point(width_/2, -height_/2);
        data = CircleDletaCalc(pos, pole_point, radius_);
        CoordTransform(data, DPlane(width_/2, -height_/2));
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 5:
    {//第五象限，横直线
        delta_x = 0;
        delta_y = pos_2 + (height_/2 + radius_);
        data = std::make_tuple(pos.x, -(height_ + 2 * radius_)/2, delta_x, delta_y);
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 6:
    {//第六象限,1/4圆
        DPlane pole_point(-width_/2, -height_/2);
        data = CircleDletaCalc(pos, pole_point, radius_);
        CoordTransform(data, DPlane(-width_/2, -height_/2));
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 7:
    {//第七象限，竖直线
        delta_x = pos_1 + (radius_ + width_/2);
        delta_y = 0;
        data = std::make_tuple(-(radius_+width_/2), pos.y, delta_x, delta_y);
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    case 8:
    {//第八象限,1/4圆
        DPlane pole_point(-width_/2, height_/2);
        data = CircleDletaCalc(pos, pole_point, radius_);
        CoordTransform(data, DPlane(-width_/2, height_/2));
        //std::cout << "quan: " << quadrant << "   >> 0: " << get<0>(data) << " 1: " << get<1>(data) << " 2: " << get<2>(data) << " 3: " << get<3>(data) << std::endl;
    }
        break;
    default:
        break;
    }
    if (std::get<0>(data) == -1)
    {
        std::cout << "feedback[axis_one_] " << feedback[axis_one_] << std::endl;
        std::cout << "feedback[axis_two_] " << feedback[axis_two_] << std::endl;
    }
    return data;
}

//使用中心极坐标
int SG_RecCir_Type::GetQuadrant(DPlane plane)
{
    //判断坐标在第几象限(1-8)
    //第一象限
    if (plane.x > -width_/2 && plane.x < width_/2
            && abs(plane.y - (height_/2 + radius_)) < EPSINON)
    {
        return 1;
    }

    //第二象限
    if (plane.x >= width_/2 && plane.x <= width_/2 + radius_
       && plane.y >= height_/2 && plane.y <= height_/2 + radius_)
    {
        return 2;
    }

    //第三象限
    if (abs(plane.x - (width_/2 + radius_)) < EPSINON
        && plane.y >= -height_/2 && plane.y <= height_/2)
    {
        return 3;
    }

    //第四象限
    if (plane.x >= width_ / 2 && plane.x <= width_ / 2 + radius_
        && plane.y >= -height_/2 - radius_ && plane.y <= -height_/2)
    {
        return 4;
    }

    //第五象限
    if (plane.x > -width_/2 && plane.x < width_/2
            && abs(plane.y - (-height_/2 - radius_)) < EPSINON)
    {
        return 5;
    }

    //第六象限
    if (plane.x >= -width_/2 - radius_ && plane.x <= -width_/2
        && plane.y >= -height_/2 - radius_ && plane.y <= -height_/2)
    {
        return 6;
    }

    //第七象限
    if (abs(plane.x - (-width_/2-radius_)) < EPSINON
            && plane.y > -height_/2 && plane.y < height_/2)
    {
        return 7;
    }

    //第八象限
    if (plane.x >= -width_/2 - radius_ && plane.x <= -width_/2
        && plane.y >= height_/2 && plane.y <= height_/2 + radius_)
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
    : SG_Type(cfg.interval, cfg.axis_one, cfg.axis_two, E_SG_Type::SG_Tapping)//刚性攻丝需特殊处理
{   
}

bool SG_Tapping_Type::Verify() const
{
    if (!SG_Type::Verify())
        return false;
}

SG_DATA SG_Tapping_Type::GenData(const double *feedback, const double *interp)
{
    double delta = feedback[axis_one_] - feedback[axis_two_];
    SG_DATA data = std::make_tuple(feedback[axis_one_], feedback[axis_two_], delta, -1);//速度暂时没做
    return data;
}
