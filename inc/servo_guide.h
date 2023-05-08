#ifndef SERVO_GUIDE_H
#define SERVO_GUIDE_H

#include <chrono>
#include <queue>
#include <utility>
#include <mutex>
#include <memory>

#include "hmi_shared_data.h"
#include "geometry_data.h"

struct SG_Type;

using SG_DATA = std::tuple<double, double, double, double>; //���շ��͵��������ͣ��ֽ׶ι̶�Ϊ4��double���ͣ�����ͨѶ���ù̶����ʹ洢
using SG_Type_Ptr = std::shared_ptr<SG_Type>;               //�ŷ��������ͣ��������Ͳμ� E_SG_Type

enum class E_SG_Type {
    SG_None = -1,
    SG_Rect,            //����
    SG_Circle,          //Բ
    SG_Rect_Circle,     //Բ��
    SG_Tapping,         //���Թ�˿
};

/**
 * @brief �ŷ��������ͻ��࣬��ֱ��ʹ��
 */
struct SG_Type {
    SG_Type(int8_t interval, int8_t axis_one, int8_t axis_two, E_SG_Type type)
        : type_(type), axis_one_(axis_one), axis_two_(axis_two), interval_(interval)
            { }

    E_SG_Type type_  = E_SG_Type::SG_None;      //����
    int8_t axis_one_ = -1;                      //��Ҫ���������No.1
    int8_t axis_two_ = -1;                      //��Ҫ���������No.2
    int8_t interval_ = 8;                       //��������

    DPoint origin_point_;                       //���������ʼ��

    double *feedback_pos_   = nullptr;
    double *intp_pos_       = nullptr;
    double *feedback_speed_ = nullptr;

    void SetInstance(double *intp, double *feedback, double *speed);    //����ʵ����

    void SetOriginPoint(DPoint);    //��������������
    virtual bool Verify() const;
    virtual SG_DATA GenData() = 0;
};

/**
 * @brief ���ι켣����
 */
struct SG_Rect_Type : public SG_Type {
    SG_Rect_Type(SG_Rect_Config cfg);
    SG_DATA GenData() override;
};

/**
 * @brief Բ�͹켣����
 */
struct SG_Circle_Type : public SG_Type {
//    enum class E_SG_CType {
//        SG_None = -1,
//        SG_CW,           //˳ʱ��
//        SG_CCW,          //��ʱ��
//    };
    SG_Circle_Type(SG_Circle_Config cfg);

    bool Verify() const override;
    SG_DATA GenData() override;

    //˳Բ����Բ
    //E_SG_CType circle_type_ = E_SG_CType::SG_None;
    //�뾶
    double radius_ = 0;     //>=0
};

/**
 * @brief Բ���켣����
 */
struct SG_RecCir_Type : public SG_Type {

    SG_RecCir_Type(SG_RecCir_Config cfg);

    bool Verify() const override;
    SG_DATA GenData() override;

    int GetQuadrant(DPlane plane);// ��������

    //�����ο��(���������ǰ뾶)
    double width_ = 0;
    //�����θ߶�(���������ǰ뾶)
    double height_ = 0;
    //���ǰ뾶
    double radius_ = 0;
};

/**
 * @brief ���Թ�˿�켣����
 */
struct SG_Tapping_Type : public SG_Type {
    SG_Tapping_Type(SG_Tapping_Config cfg);

    bool Verify() const override;
    SG_DATA GenData() override;
};


/**
 * @brief �ŷ�����
 * ����ʵ��cncϵͳ�����ݲɼ������ݷ��͵ȹ���
 */
class ServeGuide {
public:

    ServeGuide();

    bool ReadyRecord();                                 // �����ݴ���ͨ��
    bool StartRecord();                                 // ��ʼ���ݼ�¼
    void PauseRecord();                                 // �������ݼ�¼
    void ResetRecord();                                 // ��λ
    bool RefreshRecording();                            // �������ݲɼ�״̬

    void RstOriginPoint();                              //������ʼ��
    void SetOriginPoint(DPoint origin_point);           //������ʼ��

    bool IsIdle() const;                                // �Ƿ��ڿ���״̬
    bool IsRecord() const;
    bool IsReady() const;
    bool IsEmpty() const;
    int  CurState() const;

    bool SetType(SG_Type_Ptr type);                     // ���òɼ�����
    bool SetInterval(unsigned interval);                // ���ò�������
    bool IsTimeout();                                   // ���ڵ���

    virtual void RecordData(const double *feedback, const double *interp);  // ��¼���� //�麯������ģ�庯��

    bool InitSocket();  // ��ʼ��Socket
    bool Accept();
    bool Close();
    void ProcessData();

    bool IsDataReady(); // �Ƿ���Է�������
    void SendData();    // ��������

private:

    enum class E_SG_RunState{
        IDLE = 0,           //����
        READY = 1,          //׼����
        RECORDING = 2,      //�ɼ���
        STOPPING = 3,       //ֹͣ��
    };

    constexpr static int MAX_INTERVAL = 10000;
    std::chrono::time_point<std::chrono::steady_clock> scan_cycle_;     // ɨ������
    int scan_interval_ = 8;                                             // ɨ�����ڼ��

    E_SG_RunState state_ = E_SG_RunState::IDLE;

    // ���ݴ��(������Ҫ�̳�)
    bool origin_inited = false;         //�Ƿ��Ѿ���¼��ʼ�㣨�����������㣩
    std::queue<SG_DATA> data_;
    mutable std::mutex data_mut_;

    int data_socket_ = -1;              //����socket
    int data_send_fd = -1;              //���ݴ���socket

    SG_Type_Ptr type_ptr_;              //�ɼ���������
};

#endif
