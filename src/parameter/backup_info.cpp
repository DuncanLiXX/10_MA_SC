#include <algorithm>
#include <iostream>
#include <chrono>
#include "backup_info.h"
#include "global_definition.h"
#include "zip.h"
#include "dirent.h"
#include "hmi_shared_data.h"
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

}

bool Backup_Info::Package(struct zip_t *zip)
{
   zip_entry_open(zip, m_path.c_str());
   int ret = zip_entry_fwrite(zip, m_path.c_str());
   zip_entry_close(zip);

   if (ret)
   {
        char *const x = const_cast<char *const>(m_path.c_str());
        char *entries[] = {x};
        zip_entries_delete(zip, entries, 1);
        std::cout << "Package : Warning(zip_entry_fwrite) "<< m_path << " errCode:" << ret << std::endl;
    }
    else
    {
        std::cout << "Package : " << m_path << std::endl;
    }
    std::cout << std::endl;
    return true;
}

bool Backup_Info::UnPackage(struct zip_t *zip, string prefix)
{
    int ret = zip_entry_open(zip, m_path.c_str());
    if (ret)
        std::cout << "UnPackage : Warning(zip_entry_open) " << m_path.c_str() << " errCode: " << ret << std::endl;

    if (mk_path(prefix))
    {
        ret = zip_entry_fread(zip, string(prefix + m_path).c_str());
        if (ret)
            std::cout << "UnPackage : Warning(zip_entry_fread) " << string(prefix + m_path).c_str() << " errCode: " << ret << std::endl;
        else
        {
            std::cout << "UnPackage : " << m_path.c_str() << std::endl;
        }
    }
    else
    {
        return false;
    }

    zip_entry_close(zip);
    return true;
}

bool Backup_Info::mk_path(string prefix)
{
    string dirName = prefix + m_path;
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
                        std::cout << "mkpath err " <<  curPath << " errno" << errno << std::endl;
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
    if (mask & BackUp_System_Param) {
        int type = BackUp_System_Param;
        m_backupInfoVec.push_back(new Backup_Info(type, SYS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, CHN_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, AXIS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, FIVE_AXIS_CONFIG_FILE));
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
        GetFileName(PATH_NC_FILE, files);
        for(auto itr = files.begin(); itr != files.end(); ++itr)
            m_backupInfoVec.push_back(new Backup_Info(type, *itr));
    }

    if (mask & Backup_IO_Remap)
    {
        int type = Backup_Gcode_Data;
        m_backupInfoVec.push_back(new Backup_Info(type, IO_REMAP_FILE));
    }
    if (mask == Backup_All)
    {
        int type = Backup_All;
        m_backupInfoVec.push_back(new Backup_Info(type, SC_DIR));
        m_backupInfoVec.push_back(new Backup_Info(type, MI_DIR));
        m_backupInfoVec.push_back(new Backup_Info(type, MC_DIR));
        m_backupInfoVec.push_back(new Backup_Info(type, PL_DIR));
        m_backupInfoVec.push_back(new Backup_Info(type, FPGA_DIR));
        m_backupInfoVec.push_back(new Backup_Info(type, SPARTAN6_DIR));

        m_backupInfoVec.push_back(new Backup_Info(type, HOST_ADDRESS_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, HANDWHEEL_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, PITCH_COMP_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, PATH_SPARTAN6_PROGRAM_BAK));
        m_backupInfoVec.push_back(new Backup_Info(type, CHN_SCENE_FILE));
#ifdef USES_GRIND_MACHINE
        m_backupInfoVec.push_back(new Backup_Info(type, GRIND_CONFIG_FILE));
#endif
    }
}

void BackUp_Manager::Init_UnPack_Info(int mask, zip_t *zip)
{
    if (mask == Backup_All)
    {//全盘恢复,恢复压缩包的所有内容,跟类型无关
        for(int i = 0; i < zip_entries_total(zip); ++i)
        {
            zip_entry_openbyindex(zip, i);
            string name = zip_entry_name(zip);
            zip_entry_close(zip);
            m_backupInfoVec.push_back(new Backup_Info(0, name));
        }
        return;
    }


    if (mask & BackUp_System_Param) {
        int type = BackUp_System_Param;
        m_backupInfoVec.push_back(new Backup_Info(type, SYS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, CHN_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, AXIS_CONFIG_FILE));
        m_backupInfoVec.push_back(new Backup_Info(type, FIVE_AXIS_CONFIG_FILE));
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
    }

    if (mask & Backup_IO_Remap)
    {
        int type = Backup_Gcode_Data;
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
            if (name.find("/cnc/config/macro_var/") != string::npos)
            {
                m_backupInfoVec.push_back(new Backup_Info(Backup_Macro_Param, name));
            }
        }
        if (mask & Backup_Esb_Data)
        {
            if (name.find("/cnc/svo_esb") != string::npos)
            {
                m_backupInfoVec.push_back(new Backup_Info(Backup_Esb_Data, name));
            }
        }
        if (mask & Backup_Gcode_Data)
        {
            if (name.find(PATH_NC_FILE) != string::npos)
            {
                m_backupInfoVec.push_back(new Backup_Info(Backup_Gcode_Data, name));
            }
        }
    }


}

//bool BackUp_Manager::Has_Info(const Backup_Info &info) const
//{
//    auto ret = find_if(m_backupInfoVec.begin(), m_backupInfoVec.end(), [info](Backup_Info *item){
//        return *item == info;
//    });
//    return ret != m_backupInfoVec.end();
//}

//void BackUp_Manager::Add_Info(Backup_Info *info)
//{
//    if (Has_Info(*info)){
//        return;
//    }
//    else {
//        m_backupInfoVec.push_back(info);
//    }
//}

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

//void BackUp_Manager::Remove_Info(const Backup_Info &info)
//{

//}

//bool BackUp_Manager::Package()
//{
//    zip = zip_open(BACKUP_DIR.c_str(), 1, 'w');
//    if (!zip) return false;

//    for(auto itr = m_backupInfoVec.begin(); itr != m_backupInfoVec.end(); ++itr)
//    {
//        //(*itr)->Make_Package(zip);

//        //package backup info
//        MakeBackupInfo(*itr);

//    }

//    //发送package信息
//    zip_close(zip);

//    return true;
//}

//void BackUp_Manager::MakeBackupInfo(Backup_Info *info)
//{
//    for(auto itr = info->begin(); itr != info->end(); ++itr)
//    {
//        //压缩单个文件
//        zip_entry_open(zip, itr->c_str());
//        int ret = zip_entry_fwrite(zip, itr->c_str());
//        zip_entry_close(zip);
//    }
//}
