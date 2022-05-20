/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file license_interface.h
 *@author gonghao
 *@date 2021/07/05
 *@brief 本源文件为授权库的接口描述
 *@version V0.1--初始化版本
 */

#ifndef LICENSE_INTERFACE_H_
#define LICENSE_INTERFACE_H_

#define LICCODECOUNT (24) //授权码长度
#define SN_COUNT     (13)  //产品序列号(SN)长度


//时间结构体
struct LitronucDate
{
	int year;
	int month;
	int day;
};

//授权信息结构体
struct LitronucLicInfo
{
	LitronucLicInfo();
	void InitData();
	char sn[SN_COUNT+1];
	LitronucDate dead_line;
	char licflag;  //许可标志位
	               //'p'--在许可期    'f'--永久许可   'n'--无lic
	               //'m'--授权即将到期    'o'--过期许可  'v'--非法许可
	char nWarn; //存储距离到期日的小时数
	char nInfo;

};

/**
 * @brief 加密数据字符串
 * @param lppDest[out] : 输出的密文字符串，由函数分配内存，使用后需自行销毁
 * @param lnDesLen[out] ：返回的密文字符串长度
 * @param lpSrc[in] : 待加密的字符串明文
 * @param lnSrcLen[in] : 明文字符串长度
 * @return -1表示失败    0表示成功
 */
int DataEncrypt(char **lppDest, long &lnDesLen, char *lpSrc, long lnSrcLen);

/**
 * @brief  解密数据字符串
 * @param lppDest[out] : 输出的明文字符串，由函数分配内存，使用后需自行销毁
 * @param lnDesLen[out] ：返回的明文字符串长度
 * @param lpSrc[in] ：待解密字符串明文
 * @param lnSrcLen[in] ：明文长度
 * @return
 */
int DataDecrypt(char **lppDest, long &lnDesLen, char *lpSrc, long lnSrcLen);  //

/**
 * @brief 授权码解密
 * @param szStr[in/out] : 授权码字符串
 * @return true--合法授权码    false--非法授权码
 */
bool DecryptLicense(char *szStr);

/**
 * @brief 读取设备SN号
 * @param sn : 13字节长度的字符数组，用于返回设备序列号
 * @return  true--成功  false--失败
 */
bool ReadDevSn(char *sn);

/**
 * @brief 读取本地时间记录
 * @param dev_sn : 13字节长度字符数组，设备序列号
 * @return 返回本地累积时长， 返回本地累积时长, -1：表示文件丢失   -2：表示文件损坏    -3：表示非本机文件
 */
long ReadLocalTime(char *dev_sn);

/**
 * @brief 初始化创建本地计时文件
 * @return true--成功   false--失败
 */
bool InitLocalTime();

/**
 * @brief 更新本地计时文件
 * @param dev_sn : 13字节长度字符数组，设备序列号
 * @param hours ：以小时计距离1970年1月1日（UTC）的时间
 * @param force : true--强制写入   false--非强制写入，当hours小于0时不写入
 * @return  true--成功   false--失败
 */
bool WriteLocalTime(char *dev_sn, long hours, bool force = false);

/**
 * @brief 检查系统时间
 * @param plic ： 系统授权信息
 * @param local_time ： 系统计时
 * @return 0--正常  -1--文件丢失    -2--文件损坏   -3--非本机文件     -4--系统时间异常
 */
int CheckLocalTime(LitronucLicInfo *plic, long &local_time);

/**
 * @brief 读取授权信息
 * @param sn[in] : 设备SN号
 * @param plic[out] : 读取的授权数据
 * @return true--成功   false--失败
 */
bool ReadLicense(char *sn, LitronucLicInfo *plic);

/**
 * @brief 用授权码注册
 * @param sn[in] : 设备序列号
 * @param licCode[in] : 授权码
 * @param plic[out] : 解析授权码得到的授权信息
 * @return 0--成功  1--授权码加密失败   2--授权文件生成失败    3--授权文件写入失败
 */
int RegisterLicWithCode(char *sn, char* licCode, LitronucLicInfo *plic);

#endif /* LICENSE_INTERFACE_H_ */
