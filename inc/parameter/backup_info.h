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
    string m_path;        //需备份的路径
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
    // coord 相关
public:
    BackUp_Info_Manager();
    void SetData(Backup_Info *data);
    void Clean_Data();

    vector<Backup_Info *>::iterator begin() { return m_infoVec.begin(); }
    vector<Backup_Info *>::iterator end()   { return m_infoVec.end();   }
private:
    void InitCoordInfo();


    // 其他扩展数据


    //
    vector<Backup_Info *> m_infoVec;
};

/**
 * @brief The Coord_Backup_Self class
 * @note 坐标存取功能，SC内部数据的存储和恢复，以分组的方式管理
 */
class Coord_Backup_Self : public Backup_Info {
public:
    static constexpr int MaxItem = 14; ///< 坐标存储最大组数

    /**
     * @brief Coord_Backup_Self
     * @param group,坐标保存组
     * @param path, 此参数弃用（此参数仅用于系统的备份和恢复）
     */
    Coord_Backup_Self(BU_Info_Coord info);

    /**
     * @brief 对坐标系的数据进行保存
     * @param zip, 此参数弃用（此参数仅用于系统的备份和恢复）
     * @return
     */
    bool Package(struct zip_t *zip) override;

    /**
     * @brief 对坐标系的数据进行恢复
     * @param zip, 此参数弃用（此参数仅用于系统的备份和恢复）
     * @param prefix, 此参数弃用（此参数仅用于系统的备份和恢复）
     * @return
     */
    bool UnPackage(struct zip_t *zip, string prefix) override;

    /**
     * @brief 创建 coord.dat 文件
     */
    static void InitCoordDatFile();

    /**
     * @brief 获取坐标存取列表信息
     * @return 列表信息
     */
    static vector<string> GetCoordInfoList();

    /**
     * @brief 查询此坐标组是否有数据
     * @return
     * @retval true, 有数据
     * @retval false, 无数据
     */
    static bool HasCoordData(int groupid);

    static string ReadMetaInfo(int groupid);

    /**
     * @brief 删除坐标组
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
