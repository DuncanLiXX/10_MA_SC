#ifndef BACKUP_INFO_H_
#define BACKUP_INFO_H_

#include <string>
#include <vector>
#include <string_view>
#include "zip.h"
#include "global_definition.h"
#include "hmi_shared_data.h"

using namespace std;

const std::string MACRO_DIR = "/cnc/config/macro_var";
const std::string BACKUP_DIR = "/cncBackup.bak";
const std::string RECOVER_FILE = "/cncRecover.bak";
const std::string RECOVER_TEMP = "/recoverTemp";
const std::string SC_DIR = "/cnc/bin/10MA_SC_Module.elf";
const std::string MI_DIR = "/cnc/bin/MI.bin";
const std::string SPARTAN6_DIR = "/cnc/bin/top.bit";
const std::string PL_DIR = "/cnc/bin/PL.bin";
const std::string MC_DIR = "/cnc/bin/module_mc.ldr";
const std::string FPGA_DIR = "/cnc/bin/SPARTAN6.bin";
const std::string SCRIPT_DIR = "/cnc/script/S99rmnologin.sh";
const std::string ScriptPath = "/etc/rc5.d/S99rmnologin.sh";
const std::string MDA_DIR = "/cnc/mda/";
const std::string COORD_DATA_FILE = string(PATH_STORE) + "coord.dat";

class Backup_Info {
public:
    Backup_Info();
    Backup_Info(int type, string path);
    virtual ~Backup_Info() {};

    virtual bool Package(struct zip_t *zip);
    virtual bool UnPackage(struct zip_t *zip, string prefix);
    string GetPath() const { return m_path; }

protected:
    int    m_type;
    string m_path;        //�豸�ݵ�·��
    string m_pack_name;

    bool mk_path(string path);
};

class Script_Backup_Info : public Backup_Info {
public:
    Script_Backup_Info(int type, string path);
    virtual ~Script_Backup_Info() {};

    bool Package(struct zip_t *zip) override;
    bool UnPackage(struct zip_t *zip, string prefix) override;
};

class Sc_Backup_Info : public Backup_Info {
public:
    Sc_Backup_Info(int type, string path);
    virtual ~Sc_Backup_Info() {};
    bool UnPackage(struct zip_t *zip, string prefix) override;
};

class AxisConfig_Backup_Info : public Backup_Info {
public:
    AxisConfig_Backup_Info(int type, string path);
    virtual ~AxisConfig_Backup_Info() {};
    bool UnPackage(struct zip_t *zip, string prefix) override;
};

class Mc_Backup_Info : public Backup_Info {
public:
    Mc_Backup_Info(int type, string path);
    virtual ~Mc_Backup_Info() {};
    bool UnPackage(struct zip_t *zip, string prefix) override;
};

class BackUp_Manager {
public:
    BackUp_Manager();
    ~BackUp_Manager();

    enum SpecialType{
        Basic_Type = 0,
        Script_Type,
        Sc_Type,
        Macro_Type,
        Esb_Type,
        NcFile_Type,
        Axis_Type,
        Mc_Type,
    };

    void Init_Pack_Info(int mask);
    void Init_UnPack_Info(int mask, zip_t *zip);

    void Clean_Info();

    int Info_Cnt() const;

    vector<Backup_Info *>::iterator begin() { return m_backupInfoVec.begin(); }
    vector<Backup_Info *>::iterator end()   { return m_backupInfoVec.end();   }
private:

    void MakeBackupInfo(Backup_Info *info);
    SpecialType GetBackupType(const std::string &backName);
    vector<Backup_Info *> m_backupInfoVec;
};

class BackUp_Info_Manager {
    // coord ���
public:
    BackUp_Info_Manager();
    void SetData(Backup_Info *data);
    void Clean_Data();

    vector<Backup_Info *>::iterator begin() { return m_infoVec.begin(); }
    vector<Backup_Info *>::iterator end()   { return m_infoVec.end();   }
private:
    void InitCoordInfo();


    // ������չ����


    //
    vector<Backup_Info *> m_infoVec;
};

/**
 * @brief The Coord_Backup_Self class
 * @note �����ȡ���ܣ�SC�ڲ����ݵĴ洢�ͻָ����Է���ķ�ʽ����
 */
class Coord_Backup_Self : public Backup_Info {
public:
    static constexpr int MaxItem = 14; ///< ����洢�������

    /**
     * @brief Coord_Backup_Self
     * @param group,���걣����
     * @param path, �˲������ã��˲���������ϵͳ�ı��ݺͻָ���
     */
    Coord_Backup_Self(BU_Info_Coord info);

    /**
     * @brief ������ϵ�����ݽ��б���
     * @param zip, �˲������ã��˲���������ϵͳ�ı��ݺͻָ���
     * @return
     */
    bool Package(struct zip_t *zip) override;

    /**
     * @brief ������ϵ�����ݽ��лָ�
     * @param zip, �˲������ã��˲���������ϵͳ�ı��ݺͻָ���
     * @param prefix, �˲������ã��˲���������ϵͳ�ı��ݺͻָ���
     * @return
     */
    bool UnPackage(struct zip_t *zip, string prefix) override;

    /**
     * @brief ���� coord.dat �ļ�
     */
    static void InitCoordDatFile();

    /**
     * @brief ��ȡ�����ȡ�б���Ϣ
     * @return �б���Ϣ
     */
    static vector<string> GetCoordInfoList();

    /**
     * @brief ��ѯ���������Ƿ�������
     * @return
     * @retval true, ������
     * @retval false, ������
     */
    static bool HasCoordData(int groupid);

    static string ReadMetaInfo(int groupid);

    /**
     * @brief ɾ��������
     */
    static void DeleteCoordData(int groupid);
private:

    static constexpr char META_STR[] = "meta.txt";
    static constexpr char COORD_STR[] = "work_coord.ini";
    static constexpr char EX_COORD_STR[] = "ex_work_coord.ini";
    int channel_id = 0;
    BU_Info_Coord coord_info;
};

#endif  /*BACKUP_INFO_H_*/
