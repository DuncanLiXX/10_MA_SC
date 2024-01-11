#ifndef SPINDLE_CONTROL_H_
#define SPINDLE_CONTROL_H_

#include <stdint.h>

class MCCommunication;
class MICommunication;
class Variable;
class FRegBits;
class GRegBits;

struct SCAxisConfig;

/*
 *@date 2022/09/19
 *@brief:ʵ�����Ṧ���߼�
 */

namespace Spindle {
enum Polar
{
    Stop = -1,
    Positive = 0,
    Negative = 1,
};
enum Level  // ��λ
{
    Low,    // ��λ1
    Middle, // ��λ2
    High    // ��λ3
};
enum Mode
{
    Speed,
    Position
};

typedef struct
{
    bool tap_flag{false};       // ��˿���
    Polar polar{Stop};       // ���᷽��
    double F{0.0};              // ��˿����
    uint16_t S{0};              // ����ת��
    uint8_t phy_axis{0};        // �������
    uint8_t z_axis{0};          // z�����
    double R{0.0};              // Rƽ��
    int dir{1};					// G84  G74
}TapState;
}

class SpindleControl
{
public:
    SpindleControl();

    // ��������������
    void SetComponent(MICommunication *mi,
                    MCCommunication *mc,
                    FRegBits *f_reg,
                    const GRegBits *g_reg,
                    Variable *variable);
    // ���������������
    void SetSpindleParams(SCAxisConfig *spindle,
                          uint32_t da_prec,
                          uint8_t phy_axis,
                          uint8_t z_axis);

    void Reset();

    void InputSCode(uint32_t s_code);            // Sָ������
    uint32_t GetSCode();

    // ���������� ��ת�ź�SRV(G70.4),��ת�ź�SFR(G70.5) ����
    // 1.����ͣ: SRV==0&&SFR==0
    // 2.������ת: SRV==0&&SFR==1
    // 3.���ᷴת: SRV==1&&SFR==0
    void InputPolar(Spindle::Polar polar);

    // ����ģʽ�� ���Թ�˿�ź�RGTAP(G61.0) ����
    // 1.�ٶ�ģʽ: RGTAP==0
    // 2.λ��ģʽ: RGTAP==1
    void SetMode(Spindle::Mode mode);
    Spindle::Mode GetMode();             // ��ȡ�������ģʽ

    uint8_t Type();  // 0:û������  1������Ϊ������ 2������Ϊ������
    uint8_t GetPhyAxis(){return phy_axis;}

    bool isTapEnable();     // ���Թ�˿�Ƿ�ʹ��

    void SetTapFeed(double feed);       // ���ù�˿����
    void StartRigidTap(double feed);    // ����ͬ��
    void CancelRigidTap();              // ȡ��ͬ��
    void ResetTapFlag();

    void InputSSTP(bool _SSTP);     // _SSTP�ź����� ����Ч
    void InputSOR(bool SOR);        // SOR�ź�����
    void InputSOV(uint8_t SOV);     // SOV�ź�����
    void InputRI(uint16_t RI);      // RI�ź�����
    void InputSGN(bool SGN);        // SGN�ź�����
    void InputSSIN(bool SSIN);      // SSIN�ź�����
    void InputSIND(bool SIND);      // SIND�ź�����
    void InputORCMA(bool ORCMA);    // ��λ�ź��ź�����
    void InputRGTAP(bool RGTAP);    // ���Թ�˿�ź�����
    void InputRGMD(bool RGMD);      // ���Թ�˿ģʽ�л�
    void InputRTNT(bool RTNT);      // ��˿����
    void RspORCMA(bool success);    // ��λ�ź�����ظ�
    void RspCtrlMode(uint8_t axis, Spindle::Mode mode);  // ģʽ�л��ظ�
    void RspAxisEnable(uint8_t axis, bool enable);       // ��ʹ�ܻظ�
    void RspSpindleSpeed(uint8_t axis, bool success);    // 0:�ٶ�δ����  1:�ٶȵ���

    void InputFIN(bool FIN);

    int32_t GetSpindleSpeed();  // ��ȡ��ʵ����ת�٣����з�������hmi��ʾת�� ��λ:rpm

    double GetSpdAngle();  // ��ȡ����Ƕ� ��λ����
    Spindle::Polar CalPolar();    // ���ݵ�ǰ״̬��ȡ����ת��

    void EStop();

    double Get_TapRatio() const { return tap_ratio; }   //��ȡ��˿����

private:
    void UpdateParams();        // ���³��������������Ա������
    void UpdateSpindleState();  // ���ݵ�ǰ״̬����ת��
    uint32_t GetMaxSpeed();     // ��ȡ��ǰ��λ���ת��
    int32_t CalDaOutput();     // ���ݵ�ǰ״̬��ȡDA��ƽֵ

    // ���ݵ�ǰ�ٶȸ��µ�λ���
    //������ speed: ����ת�� ��λ��rpm
    //����ֵ�� 0��������  1������
    bool UpdateSpindleLevel(uint16_t speed);

    // ���ݵ�ǰ״̬��ȡĿ�굵λ
    void CalLevel(uint8_t &GR1O, uint8_t &GR2O, uint8_t &GR3O);

    // ���ݵ�ǰ״̬��������ת�ٸ�Mi
    void SendSpdSpeedToMi();

    void SendSpdSpeedToMi(int16_t speed);

    // �첽�������ᶨ���߼�
    void ProcessORCMA(bool ORCMA);
    // �첽����ģʽ�л����
    void ProcessModeChanged(Spindle::Mode mode);
    // �첽�������߼�
    void ProcessSwitchLevel();

    // ��MC���͸��Թ�˿״̬���MC���л������Թ�˿���ٶȹ滮����
    void SendMcRigidTapFlag(bool enable);

    // ����/���ع�˿״̬
    void SaveTapState();
    bool LoadTapState(Spindle::TapState &state);

    // �첽����˿����
    void ProcessRTNT();

public:
    MICommunication *mi{nullptr};
    MCCommunication *mc{nullptr};
    int chn{0};
    SCAxisConfig *spindle{nullptr}; // ����1
    uint8_t phy_axis{0};       // �����������,��0��ʼ
    uint8_t z_axis{2};         // z���������,��0��ʼ
    uint32_t da_prec{2048};    // DA����

    FRegBits *F{nullptr};            // F�Ĵ���
    const GRegBits *G{nullptr};            // G�Ĵ���

    Variable *variable;

    Spindle::Level to_level{Spindle::Low};        // Ŀ�굵λ
    Spindle::Level level{Spindle::Low};           // ��ǰ��λ
    Spindle::Polar cnc_polar{Spindle::Stop};    // ���᷽��
    uint32_t cnc_speed{0};              // S�����ת�� ��λ:rpm
    int32_t cnc_speed_virtual{0};
    double tap_feed{0.0};                    // ��˿����

    Spindle::Mode mode{Spindle::Speed};             // ����ģʽ
    bool tap_enable{false};     // ��˿״̬  false:���ڹ�˿״̬ true:���ڹ�˿״̬
    double  tap_ratio{1.0};       // ��˿����
    bool motor_enable{false};   // ���ʹ��״̬
    bool wait_sar{false};       // �ȴ��ٶȵ��� 0:���ڵȴ� 1:���ڵȴ�
    bool wait_off{false};       // �ȴ������ʹ�� 0:���ڵȴ� 1:���ڵȴ�
    bool wait_on{false};        // �ȴ������ʹ�� 0:���ڵȴ� 1:���ڵȴ�

    // �ź�
    bool _SSTP;  //����ֹͣ�ź�     G29.6  ����Ч
    bool SOR;    //����׼ͣ�ź�    G29.5
    bool SAR;    //���ᵽ���ź�    G29.4
    uint8_t SOV;    //���ᱶ��       G30
    uint16_t RI; //����ת������    G31~G32
    bool SGN;    //PMC��������᷽��    G33.5   0���� 1����
    bool SSIN;   //���᷽����CNC��������PMC����  G33.6   0��CNC 1��PMC
    bool SIND;   //�����ٶ���CNC��������PMC���� G33.7   0��CNC 1��PMC
    bool ORCMA{0};  // ���ᶨ���ź� G70.6
    bool RGTAP;  //�չ�״̬  0���˳��չ�״̬  1������չ�״̬ G61.0
    bool RGMD;   //�������ģʽ 0���ٶ�ģʽ  1��λ��ģʽ G61.1
    bool RTNT;   //��˿�����ź� G62.6

    // ����
    uint8_t GST{0};    //(1657)SOR�ź����ڣ� 0�����ᶨ�� 1�����ֻ���
    uint8_t SGB{0};    //(1658)���ֻ�����ʽ 0��A��ʽ 1��B��ʽ
    uint8_t SFA{0};    //(1659)�������ܿ��أ� 0���ر� 1����
    uint8_t ORM{0};    //(1660)���ᶨ��ʱ��ת�� 0���� 1����
    uint8_t TCW{1};    //(1661)����ת���Ƿ���M03/M04Ӱ�� 0������Ӱ�� 1����Ӱ��
    uint8_t CWM{0};    //(1662)����ת��ȡ�� 0����  1����
    uint8_t TSO{0};    //(1663)���������͸��Թ�˿ʱ�����ᱶ��  0��ǿ��100%  1����Ч

    uint16_t TM{16000};    //SF�ź������ʱ ��λ:us
    uint16_t TMF{16000};   //SFѡͨ�źŴ򿪺����ݵ������ʱ ��λ:us

    Spindle::TapState tap_state;    // ��˿���ݼ�¼
    bool running_rtnt{false};       // �Ƿ����ڹ�˿����
    double pos_zero_ang{0.0};            // 0�Ȼ�е����
    int TapDir = 1;        // ��˿����
};

#endif
