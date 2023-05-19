#include <algorithm>
#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include "backup_info.h"
#include "global_definition.h"
#include "zip.h"
#include "dirent.h"
#include "hmi_shared_data.h"
#include "inifile.h"
#include "global_include.h"
#include "channel_engine.h"
#include <unistd.h>
#include <sys/stat.h>

#ifndef ISSLASH
#define ISSLASH(C) ((C) == '/' || (C) == '\\')
#endif

void GetFileName(string path, std::vector<string> &files) {
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str())))
    {
        files.push_back(path);
        return;
    }
    while ((ptr = readdir(pDir)) != 0) {
        // strcmp是C语言里的，只导入string,然后std::strcmp都是没有的
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {

            std::string filePath = path + "/" + ptr->d_name;
            DIR *pSubDir = opendir(filePath.c_str());
            if (pSubDir)
            {
                GetFileName(filePath, files);
            }
            else
            {
                files.push_back(path + "/" + ptr->d_name);  // 可以只保留名字
            }
        }
    }
    closedir(pDir);
}

Backup_Info::Backup_Info()
{

}

Backup_Info::Backup_Info(int type, string path)
    : m_type(type), m_path(path)
{
    if (ISSLASH(m_path.front()))
    {
        m_pack_name = m_path.substr(1, m_path.size());
    }
    else
    {
        m_pack_name = m_path;
    }
}

bool Backup_Info::Package(struct zip_t *zip)
{
//   if(access(m_path.c_str(), F_OK) == 0)
//   {//文件存在
//        int ret = zip_entry_open(zip, m_pack_name.c_str());
//        ret = zip_entry_fwrite(zip, m_path.c_str());
//        zip_entry_close(zip);

//        if (ret)
//        {
//            std::cout << "Package : (zip_entry_fwrite) "<< ret << " path: " << m_path << std::endl;
//            char *const x = const_cast<char *const>(m_path.c_str());
//            char *entries[] = {x};
//            std::cout << "Package : delete" << *entries << std::endl;
//            zip_entries_delete(zip, entries, 1);
//            std::cout << "Package : Warning(zip_entry_fwrite) "<< m_path << " errCode:" << ret << std::endl;
//            if (ret <= -8)
//                return false;
//        }
//        else
//        {
//            std::cout << "Package : " << m_path << std::endl;
//        }
//   }
//   else
//   {
//        std::cout << "Package : not find -- " << m_path << std::endl;
//   }
//   std::cout << std::endl;
//   return true;

    return true;
}

bool Backup_Info::UnPackage(struct zip_t *zip, string prefix)
{
    int ret = zip_entry_open(zip, m_pack_name.c_str());
    if (ret)
    {
        std::cout << "UnPackage : Warning(zip_entry_open) " << m_pack_name.c_str() << " errCode: " << ret << std::endl;
        if (ret <= -8)//磁盘空间不足导致的解压失败
            return false;
    }

    string fullPath = prefix + m_path;
    if (mk_path(fullPath))
    {
        ret = zip_entry_fread(zip, string(prefix + m_path).c_str());
        if (ret)
        {
            std::cout << "UnPackage : Warning(zip_entry_fread) " << string(prefix + m_path).c_str() << " errCode: " << ret << std::endl;
            if (ret == -8 && ret != 17)
                return false;
        }
        else
        {
            std::cout << "UnPackage : " << m_pack_name.c_str() << std::endl;
        }
    }
    else
    {
        return false;
    }

    zip_entry_close(zip);
    return true;
}

/**
 * @brief 递归创建文件夹
 * @param path,  路径,可以是绝对路径,也可以是相对路径,也可以带文件名称
 * @return true--成功   false--失败
 */
bool Backup_Info::mk_path(string path)
{
    string dirName = path;
    uint32_t beginCmpPath = 0;
    uint32_t endCmpPath = 0;

    string fullPath = "";

    if (!ISSLASH(dirName[0]))
    {
        fullPath = getcwd(nullptr, 0);
        beginCmpPath = fullPath.size();
        fullPath = fullPath + '/' + dirName;
    }
    else
    {
        fullPath = dirName;
        beginCmpPath = 1;
    }

    endCmpPath = fullPath.size();

    for(uint32_t i = beginCmpPath; i < endCmpPath; ++ i)
    {
        if (ISSLASH(fullPath[i]))
        {
            string curPath = fullPath.substr(0, i);
            if (access(curPath.c_str(), F_OK) != 0)
            {
                if (mkdir(curPath.c_str(), S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP | S_IWOTH) == -1)
                {
                    if (errno != EEXIST) {
                        std::cout << "mk_path " <<  curPath << " errno" << errno << std::endl;
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

BackUp_Manager::BackUp_Manager()
{
}

BackUp_Manager::~BackUp_Manager()
{
    Clean_Info();
}

void BackUp_Manager::Init_Pack_Info(int mask)
{
    std::cout << "mask " << mask << std::endl;
    if (mask & BackUp_System_Param) {
        int type = BackUp_System_Param;
        m_backupInfoVec.push_back(new Backup_Info(type, SYS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, CHN_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, AXIS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, FIVE_AXIS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, FIVE_AXIS_CONFIG_FILE_V2));
        m_backupInfoVec.push_back(new Backup_Info(type, CHN_PROC_PARAM_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, AXIS_PROC_PARAM_FILE));
    }

    if (mask & Backup_Tool_Param)
    {
        int type = Backup_Tool_Param;
        m_backupInfoVec.push_back(new Backup_Info(type, TOOL_CONFIG_FILE));
    }

    if (mask & Backup_Coord_Param)
    {
        int type = Backup_Coord_Param;
        m_backupInfoVec.push_back(new Backup_Info(type, WORK_COORD_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, EX_WORK_COORD_FILE));
    }

    if (mask & Backup_Pmc_Data)
    {
        int type = Backup_Pmc_Data;
        m_backupInfoVec.push_back(new Backup_Info(type, PATH_PMC_DATA));
        m_backupInfoVec.push_back(new Backup_Info(type, PATH_PMC_LDR));
        m_backupInfoVec.push_back(new Backup_Info(type, PATH_PMC_REG));
    }

    if (mask & Backup_Macro_Param)
    {
        int type = Backup_Macro_Param;
        vector<string> files;
        GetFileName(MACRO_DIR, files);
        for(auto itr = files.begin(); itr != files.end(); ++itr)
            m_backupInfoVec.push_back(new Backup_Info(type, *itr));
    }

    if (mask & Backup_Esb_Data)
    {
        int type = Backup_Esb_Data;
        vector<string> files;
        GetFileName("/cnc/svo_esb", files);
        for(auto itr = files.begin(); itr != files.end(); ++itr)
            m_backupInfoVec.push_back(new Backup_Info(type, *itr));
    }

    if (mask & Backup_Gcode_Data)
    {
        int type = Backup_Gcode_Data;
        vector<string> files;
        GetFileName("/cnc/nc_files", files);
        for(auto itr = files.begin(); itr != files.end(); ++itr)
            m_backupInfoVec.push_back(new Backup_Info(type, *itr));
    }

    if (mask & Backup_IO_Remap)
    {
        int type = Backup_IO_Remap;
        m_backupInfoVec.push_back(new Backup_Info(type, IO_REMAP_FILE));
    }
    if (mask == Backup_All)
    {
        int type = Backup_All;
        m_backupInfoVec.push_back(new Backup_Info(type, SC_DIR));
        m_backupInfoVec.push_back(new Backup_Info(type, MI_DIR));
        m_backupInfoVec.push_back(new Backup_Info(type, MC_DIR));
        //m_backupInfoVec.push_back(new Backup_Info(type, PL_DIR));
        //m_backupInfoVec.push_back(new Backup_Info(type, FPGA_DIR));
        //m_backupInfoVec.push_back(new Backup_Info(type, SPARTAN6_DIR));

        m_backupInfoVec.push_back(new Backup_Info(type, HOST_ADDRESS_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, HANDWHEEL_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, PITCH_COMP_FILE));
        //m_backupInfoVec.push_back(new Backup_Info(type, PATH_SPARTAN6_PROGRAM_BAK));
        m_backupInfoVec.push_back(new Backup_Info(type, CHN_SCENE_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, PATH_PHY_AXIS_ENCODER));
        m_backupInfoVec.push_back(new Script_Backup_Info(type, SCRIPT_DIR));
#ifdef USES_GRIND_MACHINE
        m_backupInfoVec.push_back(new Backup_Info(type, GRIND_CONFIG_FILE));
#endif
    }
}

void BackUp_Manager::Init_UnPack_Info(int mask, zip_t *zip)
{
    std::cout << "mask " << mask << std::endl;
    if (mask == Backup_All)
    {//全盘恢复,恢复压缩包的所有内容,跟类型无关
        for(int i = 0; i < zip_entries_total(zip); ++i)
        {
            zip_entry_openbyindex(zip, i);
            string name = zip_entry_name(zip);
            zip_entry_close(zip);
            if (GetBackupType(name) == Script_Type)
            {
                m_backupInfoVec.push_back(new Script_Backup_Info(0, name));
            }
            else if (GetBackupType(name) == Sc_Type)
            {
                m_backupInfoVec.push_back(new Sc_Backup_Info(0, name));
            }
            else if (GetBackupType(name) == Axis_Type)
            {
                m_backupInfoVec.push_back(new AxisConfig_Backup_Info(0, name));
            }
            else if (GetBackupType(name) == Mc_Type)
            {
                m_backupInfoVec.push_back(new Mc_Backup_Info(0, name));
            }
            else
            {
                m_backupInfoVec.push_back(new Backup_Info(0, name));
            }
        }
        return;
    }

    if (mask & BackUp_System_Param) {
        int type = BackUp_System_Param;
        m_backupInfoVec.push_back(new Backup_Info(type, SYS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, CHN_CONFIG_FILE));
        m_backupInfoVec.push_back(new AxisConfig_Backup_Info(type, AXIS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, FIVE_AXIS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, FIVE_AXIS_CONFIG_FILE_V2));
        m_backupInfoVec.push_back(new Backup_Info(type, CHN_PROC_PARAM_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, AXIS_PROC_PARAM_FILE));
    }

    if (mask & Backup_Tool_Param)
    {
        int type = Backup_Tool_Param;
        m_backupInfoVec.push_back(new Backup_Info(type, TOOL_CONFIG_FILE));
    }

    if (mask & Backup_Coord_Param)
    {
        int type = Backup_Coord_Param;
        m_backupInfoVec.push_back(new Backup_Info(type, WORK_COORD_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, EX_WORK_COORD_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, PATH_PMC_REG));
    }

    if (mask & Backup_Pmc_Data)
    {
        int type = Backup_Pmc_Data;
        m_backupInfoVec.push_back(new Backup_Info(type, PATH_PMC_DATA));
        m_backupInfoVec.push_back(new Backup_Info(type, PATH_PMC_LDR));
    }

    if (mask & Backup_IO_Remap)
    {
        int type = Backup_IO_Remap;
        m_backupInfoVec.push_back(new Backup_Info(type, IO_REMAP_FILE));
    }


    //文件夹
    for(int i = 0; i < zip_entries_total(zip); ++i)
    {
        zip_entry_openbyindex(zip, i);
        string name = zip_entry_name(zip);
        zip_entry_close(zip);

        if (mask & Backup_Macro_Param)
        {
            if (GetBackupType(name) == Macro_Type)
            {
                m_backupInfoVec.push_back(new Backup_Info(Backup_Macro_Param, name));
            }
        }
        if (mask & Backup_Esb_Data)
        {
            if (GetBackupType(name) == Esb_Type)
            {
                m_backupInfoVec.push_back(new Backup_Info(Backup_Esb_Data, name));
            }
        }
        if (mask & Backup_Gcode_Data)
        {
            if (GetBackupType(name) == NcFile_Type)
            {
                m_backupInfoVec.push_back(new Backup_Info(Backup_Gcode_Data, name));
            }
        }
    }
}

void BackUp_Manager::Clean_Info()
{
    for (auto itr = m_backupInfoVec.begin(); itr != m_backupInfoVec.end(); ++itr)
    {
        if (*itr)
        {
            delete *itr;
            *itr = nullptr;
        }
    }
    m_backupInfoVec.clear();
}

int BackUp_Manager::Info_Cnt() const
{
    return m_backupInfoVec.size();
}

BackUp_Manager::SpecialType BackUp_Manager::GetBackupType(const string &backName)
{
    if (backName.find("S99rmnologin.sh") != string::npos)
    {
        return Script_Type;
    }
    if (backName.find("10MA_SC_Module.elf") != string::npos)
    {
        return Sc_Type;
    }
    if (backName.find("cnc/config/macro_var") != string::npos)
    {
        return Macro_Type;
    }
    if (backName.find("cnc/svo_esb") != string::npos)
    {
        return Esb_Type;
    }
    if (backName.find("cnc/nc_files") != string::npos)
    {
        return NcFile_Type;
    }
    if (backName.find("axis_config.ini") != string::npos)
    {
        return Axis_Type;
    }
    if (backName.find("cnc/bin/module_mc.ldr") != string::npos)
    {
        return Mc_Type;
    }

    return Basic_Type;
}

Script_Backup_Info::Script_Backup_Info(int type, string path)
    : Backup_Info(type, path)
{

}

bool Script_Backup_Info::Package(zip_t *zip)
{
    mk_path(m_path);
    if(access(ScriptPath.c_str(), F_OK) != 0)
    {
        std::cout << "Package : " << ScriptPath << " not find" << std::endl;
        return true;
    }
    string command = "cp " + ScriptPath + " " + m_path;
    std::cout << command << std::endl;
    system(command.c_str());
    system("sync");

    if (!Backup_Info::Package(zip))
        return false;

    return true;
}

bool Script_Backup_Info::UnPackage(zip_t *zip, string prefix)
{
    if (Backup_Info::UnPackage(zip, prefix))
    {
        string path = prefix + m_pack_name;
        string command = "chmod +x " + path;
        system(command.c_str());
        std::cout << command << std::endl;
        command = "cp -f " + path + " " + ScriptPath;
        std::cout << command << std::endl;
        system(command.c_str());
        system("sync");
    }
    return true;
}

Sc_Backup_Info::Sc_Backup_Info(int type, string path)
    : Backup_Info(type, path)
{

}

bool Sc_Backup_Info::UnPackage(zip_t *zip, string prefix)
{
    if (Backup_Info::UnPackage(zip, prefix))
    {
        string scPath = RECOVER_TEMP + SC_DIR;
        if (!access(scPath.c_str(), F_OK))
        {
            string command = "chmod +x " + scPath;
            std::cout << command << std::endl;
            system(command.c_str());
            system("sync");
        }
    }
    return true;
}

AxisConfig_Backup_Info::AxisConfig_Backup_Info(int type, string path)
    : Backup_Info(type, path)
{

}

bool AxisConfig_Backup_Info::UnPackage(zip_t *zip, string prefix)
{
    if (Backup_Info::UnPackage(zip, prefix))
    {
        //读取配置文件，恢复绝对式编码器的原点参数
        string scPath = RECOVER_TEMP + AXIS_CONFIG_FILE;
        inifile::IniFile m_ini_axis;
        if (m_ini_axis.Load(scPath))
        {
            return false;
        }

        //重置 ref_encoder 参数
        std::vector<std::string> sections;
        m_ini_axis.GetSections(&sections);
        for(auto itr = sections.begin(); itr != sections.end(); ++itr)
        {
            std::string key = "ref_encoder";
            if (m_ini_axis.HasKey(*itr, key))
            {
                std::cout << "setInt64Value: [" << *itr << "] = " << key << std::endl;
                if (m_ini_axis.SetInt64Value(*itr, key, kAxisRefNoDef))
                    return false;
            }

            key = "ref_complete";
            if (m_ini_axis.HasKey(*itr, key))
            {
                std::cout << "setIntValue: [" << *itr << "] = " << key << std::endl;
                if (m_ini_axis.SetInt64Value(*itr, key, 0))
                    return false;
            }
        }
        if (m_ini_axis.Save())
            return false;

        return true;
    }
    return false;
}

Mc_Backup_Info::Mc_Backup_Info(int type, string path)
    : Backup_Info(type, path)
{

}

bool Mc_Backup_Info::UnPackage(zip_t *zip, string prefix)
{
    if (Backup_Info::UnPackage(zip, prefix))
    {
        //升级MC模块
        if (!g_ptr_chn_engine->UpdateMcModel(RECOVER_TEMP + MC_DIR))
            return false;
    }
    return true;
}
