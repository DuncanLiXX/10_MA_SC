
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_structure.h
 *@author gonghao
 *@date 2020/06/15
 *@brief ��ͷ�ļ���������ȫ�����ݽṹ�ӿں���ʵ��
 *@version
 */


#include "global_structure.h"
#include "global_include.h"
#include "parm_manager.h"

//����ϵ��Ϣ
CoordInfo::CoordInfo() {
	Init();
}

void CoordInfo::Init() {
	surface_flag = g_ptr_parm_manager->GetChannelConfig(0)->default_plane;   //02��G����

	coord_id = G54_COORD;
//	for (int i = 0; i < 3; i++) {
//		surface_vector[i] = 0;
//	}
//	for (int i = 0; i < kMaxAxisChn; i++) {
//		wcs_offset_differ[i] = 0;
//	}
	Reset();
}

void CoordInfo::Reset() {
	build_coord_flag = 0;
	build_wcs_mask = 0;
	for (int i = 0; i < 3; i++) {
		build_coord_dest[i] = 0;
	}
}

bool CoordInfo::CheckMask(uint8_t index) {
	if ((build_wcs_mask & ((uint64_t) 1 << index)) == 0) {
		return false;
	} else {
		return true;
	}
}

void CoordInfo::SetMask(uint8_t index, uint8_t value) {
	if (value == 1) {
		build_wcs_mask = build_wcs_mask | ((uint64_t) 1 << index);
	} else {
		build_wcs_mask = build_wcs_mask & (~((uint64_t) 1 << index));
	}
}
