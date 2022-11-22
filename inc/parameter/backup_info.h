#ifndef BACKUP_INFO_H_
#define BACKUP_INFO_H_

#include <string>
#include <vector>
#include "zip.h"

using namespace std;

const std::string MACRO_DIR = "/cnc/config/macro_var";
const std::string BACKUP_DIR = "/cncBackup.bak";
const std::string RECOVER_FILE = "/cncRecover.bak";
const std::string RECOVER_TEMP = "/recoverTemp";
const std::string SC_DIR = "/cnc/bin/10MA_SC_Module.elf";
const std::string MI_DIR = "/cnc/bin/MI.bin";
const std::string SPARTAN6_DIR = "/cnc/bin/top.bit";
const std::string PL_DIR = "/cnc/bin/PL.bin";
const std::string MC_DIR = "/cnc/bin/MC.ldr";
const std::string FPGA_DIR = "/cnc/bin/SPARTAN6.bin";

class Backup_Info {
public:
    Backup_Info();
    Backup_Info(int type, string path);
    virtual ~Backup_Info() {};

    bool Package(struct zip_t *zip);
    bool UnPackage(struct zip_t *zip, string prefix);
    //bool Make_Package(struct zip_t *zip);     //备份操作
    //bool operator==(const Backup_Info& other);
    string GetPath() const { return m_path; }

protected:
    int    m_type;
    string m_path;        //需备份的路径

private:
    bool mk_path(string prefix);
};

//
//class Sys_Backup_Info : public Backup_Info{
//public:
//    Sys_Backup_Info();
//    ~Sys_Backup_Info() {};
//};

////
//class Tool_Backup_Info : public Backup_Info{
//public:
//    Tool_Backup_Info();
//    ~Tool_Backup_Info() {};
//};

////
//class Coord_Backup_Info : public Backup_Info {
//public:
//    Coord_Backup_Info();
//    ~Coord_Backup_Info() {};
//};

////
//class Pmc_BackUp_Info : public Backup_Info {
//public:
//    Pmc_BackUp_Info();
//    ~Pmc_BackUp_Info() {};
//};

////
//class Marco_Backup_Info : public Backup_Info {
//public:
//    Marco_Backup_Info();
//    ~Marco_Backup_Info() {};
//};

////
//class Esb_Backup_Info : public Backup_Info {
//public:
//    Esb_Backup_Info();
//    ~Esb_Backup_Info() {};
//};

////
//class Gcode_Backup_Info : public Backup_Info {
//public:
//    Gcode_Backup_Info();
//    ~Gcode_Backup_Info() {};
//};

//class Remap_Backup_Info : public Backup_Info {
//public:
//    Remap_Backup_Info();
//    ~Remap_Backup_Info() {};
//};

//class Cnc_BackUp_Info : public Backup_Info {
//public:
//    Cnc_BackUp_Info();
//    ~Cnc_BackUp_Info() {};
//};


class BackUp_Manager {
public:
    BackUp_Manager();
    ~BackUp_Manager();

    void Init_Pack_Info(int mask);
    void Init_UnPack_Info(int mask, zip_t *zip);

    void Clean_Info();

    int Info_Cnt() const;

    vector<Backup_Info *>::iterator begin() { return m_backupInfoVec.begin(); }
    vector<Backup_Info *>::iterator end()   { return m_backupInfoVec.end();   }
private:

    void MakeBackupInfo(Backup_Info *info);
    vector<Backup_Info *> m_backupInfoVec;
    //struct zip_t *zip = nullptr;
};

#endif  /*BACKUP_INFO_H_*/
