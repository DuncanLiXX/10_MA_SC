#ifndef INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_
#define INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_


struct Bar{
	uint32_t axis0;
	uint32_t axis1;
	uint32_t axis2;
	uint32_t axis3;

};

struct axis_mask{
	bool dir;
	bool active;

};

struct BarFlags{
	axis_mask axis0;
	axis_mask axis1;
	axis_mask axis2;
	axis_mask axis3;

};

#endif
