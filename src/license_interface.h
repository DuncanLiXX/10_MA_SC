/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file license_interface.h
 *@author gonghao
 *@date 2021/07/05
 *@brief ��Դ�ļ�Ϊ��Ȩ��Ľӿ�����
 *@version V0.1--��ʼ���汾
 */

#ifndef LICENSE_INTERFACE_H_
#define LICENSE_INTERFACE_H_

#define LICCODECOUNT (24) //��Ȩ�볤��
#define SN_COUNT     (13)  //��Ʒ���к�(SN)����


//ʱ��ṹ��
struct LitronucDate
{
	int year;
	int month;
	int day;
};

//��Ȩ��Ϣ�ṹ��
struct LitronucLicInfo
{
	LitronucLicInfo();
	void InitData();
	char sn[SN_COUNT+1];
	LitronucDate dead_line;
	char licflag;  //��ɱ�־λ
	               //'p'--�������    'f'--�������   'n'--��lic
	               //'m'--��Ȩ��������    'o'--�������  'v'--�Ƿ����
	char nWarn; //�洢���뵽���յ�Сʱ��
	char nInfo;

};

/**
 * @brief ���������ַ���
 * @param lppDest[out] : ����������ַ������ɺ��������ڴ棬ʹ�ú�����������
 * @param lnDesLen[out] �����ص������ַ�������
 * @param lpSrc[in] : �����ܵ��ַ�������
 * @param lnSrcLen[in] : �����ַ�������
 * @return -1��ʾʧ��    0��ʾ�ɹ�
 */
int DataEncrypt(char **lppDest, long &lnDesLen, char *lpSrc, long lnSrcLen);

/**
 * @brief  ���������ַ���
 * @param lppDest[out] : ����������ַ������ɺ��������ڴ棬ʹ�ú�����������
 * @param lnDesLen[out] �����ص������ַ�������
 * @param lpSrc[in] ���������ַ�������
 * @param lnSrcLen[in] �����ĳ���
 * @return
 */
int DataDecrypt(char **lppDest, long &lnDesLen, char *lpSrc, long lnSrcLen);  //

/**
 * @brief ��Ȩ�����
 * @param szStr[in/out] : ��Ȩ���ַ���
 * @return true--�Ϸ���Ȩ��    false--�Ƿ���Ȩ��
 */
bool DecryptLicense(char *szStr);

/**
 * @brief ��ȡ�豸SN��
 * @param sn : 13�ֽڳ��ȵ��ַ����飬���ڷ����豸���к�
 * @return  true--�ɹ�  false--ʧ��
 */
bool ReadDevSn(char *sn);

/**
 * @brief ��ȡ����ʱ���¼
 * @param dev_sn : 13�ֽڳ����ַ����飬�豸���к�
 * @return ���ر����ۻ�ʱ���� ���ر����ۻ�ʱ��, -1����ʾ�ļ���ʧ   -2����ʾ�ļ���    -3����ʾ�Ǳ����ļ�
 */
long ReadLocalTime(char *dev_sn);

/**
 * @brief ��ʼ���������ؼ�ʱ�ļ�
 * @return true--�ɹ�   false--ʧ��
 */
bool InitLocalTime();

/**
 * @brief ���±��ؼ�ʱ�ļ�
 * @param dev_sn : 13�ֽڳ����ַ����飬�豸���к�
 * @param hours ����Сʱ�ƾ���1970��1��1�գ�UTC����ʱ��
 * @param force : true--ǿ��д��   false--��ǿ��д�룬��hoursС��0ʱ��д��
 * @return  true--�ɹ�   false--ʧ��
 */
bool WriteLocalTime(char *dev_sn, long hours, bool force = false);

/**
 * @brief ���ϵͳʱ��
 * @param plic �� ϵͳ��Ȩ��Ϣ
 * @param local_time �� ϵͳ��ʱ
 * @return 0--����  -1--�ļ���ʧ    -2--�ļ���   -3--�Ǳ����ļ�     -4--ϵͳʱ���쳣
 */
int CheckLocalTime(LitronucLicInfo *plic, long &local_time);

/**
 * @brief ��ȡ��Ȩ��Ϣ
 * @param sn[in] : �豸SN��
 * @param plic[out] : ��ȡ����Ȩ����
 * @return true--�ɹ�   false--ʧ��
 */
bool ReadLicense(char *sn, LitronucLicInfo *plic);

/**
 * @brief ����Ȩ��ע��
 * @param sn[in] : �豸���к�
 * @param licCode[in] : ��Ȩ��
 * @param plic[out] : ������Ȩ��õ�����Ȩ��Ϣ
 * @return 0--�ɹ�  1--��Ȩ�����ʧ��   2--��Ȩ�ļ�����ʧ��    3--��Ȩ�ļ�д��ʧ��
 */
int RegisterLicWithCode(char *sn, char* licCode, LitronucLicInfo *plic);

#endif /* LICENSE_INTERFACE_H_ */
