#ifndef INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_
#define INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_


/*************************************************************************
 * 	本程序是LITRAN-0MB 刀补用程序
 * 	编程时间：2022-10-15
 * 	编程人：	侯长合
 * ***********************************************************************/

#include "stdint.h"

/***************************************************************
 * 	译码地址表
 * *************************************************************/
//	       地址 //  名称   长度
/*
#define	A1	0	// 	X		4
#define	A2	4	// 	Y		4
#define	A3	8	// 	Z		4
#define	A4	12	// 	4		4
#define	A5	16	// 	5		4
#define	A6	20	// 	6		4
#define	A7	24	// 	7		4
#define	A8	28	// 	8		4
#define	I	32	// 	I-12	4
#define	J	36	// 	J-12	4
#define	K	40	// 	K-12	4
#define	E	44	// 	E-8		4
#define	F	48	// 	F-8		4
#define	P	52	//	P-8		4
#define	Q	56	//	Q-3		4
#define	R	60	// 	R-12	4
#define	D	64	//	D-3		2
#define	H	66	//	H-3		2
#define	L	68	// 	L-8		2
#define	O	70	// 	O-8		2
#define	N	72	// 	N-8		2
#define	S	74	// 	S-8		4
#define	T	78	// 	T-8		2
#define	M	80	// 	M-8		2
#define	B	82	//	B-8		2
#define	GX	84	//	GX组		2
#define	GA	86	//	GA组		2
#define	GB	88	//	GB组		2
#define	GC	90	//	GC组		2
#define	GD	92	//	GD组		2
#define	GE	94	//	GE组		2
#define	GF	96	//	GF组		2
#define	GG	98	//	GG组		2
#define	GH	100	//	GH组		2
#define	GI	102	//	GI组		2
#define	GJ	104	//	GJ组		2
#define	GK	106	//	GK组		2
#define	GL	108	//	GL组		2
#define	GM	110	//	GM组		2
#define	GN	112	//	GN组		2
#define	GO	114	//	GO组		2
#define	GP	116	//	GP组		2
#define	GQ	118	//	GQ组		2
#define	GR	120	//	GR组  	2
#define	GS	122	//	GS组		2
#define	GT	124	//	GT组		2
#define	FX	126	//	第1轴地址标志	1
#define	FY	127	//	第2轴地址标志	1
#define	FZ	128	//	第3轴地址标志	1
#define	F4	129	//	第4轴地址标志	1
#define	F5	130	//	第5轴地址标志	1
#define	F6	131	//	第6轴地址标志	1
#define	F7	132	//	第7轴地址标志	1
#define	F8	133	//	第8轴地址标志	1
#define	FI	134	//	地址I标志	1
#define	FJ	135	//	地址J标志	1
#define	FK	136	//	地址K标志	1
#define	FE	137	//	地址E标志	1
#define	FF	138	//	地址F标志	1
#define	FP	139	//	地址P标志	1
#define	FQ	140	//	地址Q标志	1
#define	FR	141	//	地址R标志	1
#define	FD	142	//	地址D标志	1
#define	FH	143	//	地址H标志	1
#define	FL	144	//	地址L标志	1
#define	FO	145	//	地址O标志	1
#define	FN	146	//	地址N标志	1
#define	FS	147	//	地址S标志	1
#define	FT	148	//	地址T标志	1
#define	M	149	//	地址M1标志	1
#define	FB	150	//	地址B标志	1
#define	FG	151	//	地址G标志	4
*/


/***********************************************
 * 	输出寄存器地址表
 * *********************************************/
#define	OUTPUT		0	// 	输出指令寄存器			1
#define	FIN			1	//	辅助功能寄存器			1
#define	SSPSV		2	// 							1
#define	FCNTL		3	//	进给控制寄存器			1
#define	ESN1		4	// 	插补运动方向				1
#define	ESN2		5	//	插补运动方向				1
#define	SQCTL		6	// 	顺序控制寄存器			1
#define	OTCTL		7	//	自动换刀ATC寄存器			1
#define	SHIFT		8	// 	规格化移位位数			1
#define	HSHIFT		9	//	螺旋线插补轴规格化移位数;	1
#define	DA1			10	// 	坐标号					1
#define	DA2			11	//	坐标号					1
#define	DAH			12	// 	螺旋线插补寄存器;			1
#define	CQUAD		13	//	圆弧插补象限寄存器		1
#define	FLAG		14	// 							1
#define	CYL			15	//							1
#define	CYL2		16	// 							1
#define	NZRN		17	//							1
#define	DEN			18	// 	插补分配完成状态寄存器	1
#define	IPLTP		19	//	插补类型寄存器			1
#define	IPLC1		20	// 	插补控制寄存器			8
#define	INOT		28	//	剩余移动量				32
#define	DWEL		60	// 	延时插补寄存器			4
#define	DREG		64	//	直线插补寄存器			32
#define	DREGA1		96	// 	圆弧插补寄存器A1			4
#define	DREGA2		100	//	圆弧插补寄存器A2			4
#define	DREGB1		104	// 	圆弧插补寄存器B1			4
#define	DREGB2		108	//	圆弧插补寄存器B2			4
#define	ACTP1		112	// 	圆弧插补起点寄存器A1		4
#define	ACTP2		116	//	圆弧插补起点寄存器A2		4
#define	ENDP1		120	// 	圆弧插补终点寄存器B1		4
#define	ENDP2		124	//	圆弧插补终点寄存器B2		4
#define	KFEED		128	// 	KFEED = 2^54/R@LONG		4
#define	KFDDR		132	//	KFDDR = KFEED			4
#define	FORG		136	// 	FORG = FCODE * KFEED	8
#define	FMAX		144	//	FMAX = KFDDR * KM/IN	8
#define	KORG		152	// 							8
#define	CNTMV		160	//							1
#define	THRED		161	// 							1
#define	FCNTD		162	// 							1
#define	MACRO		163	//							1
#define	BXIDX		164	// 							1
#define	HIRMD		165	//							1
#define	TLFC		166	// 							1
#define	OVRAP		167	//							8
#define	PCR1		175	// 	圆弧插补输出余数寄存器A	2
#define	PCR2		177	//	圆弧插补输出余数寄存器B	2
#define	PA1			179	// 	圆弧插补输出寄存器1A		2
#define	PA2			181	//	圆弧插补输出寄存器1B		2
#define	PB1			183	// 	圆弧插补输出寄存器2A		2
#define	PB2			185	//	圆弧插补输出寄存器2B		2
#define	FACC		187	// 	进给速率余数				2
#define	DACC 		189	// 	直线插补DDA余数			16
#define	DACA1		205	// 	圆弧插补DDAA余数			4
#define	DACA2		209	// 							4
#define	DACB1		213	// 	圆弧插补DDAB余数			4
#define	DACB2		217	// 							4
#define	SE			221	// 	伺服轴互锁信号			1


typedef struct WORD_ADRS_FLAG_BITS
{
	uint8_t	inp:1;			// Input flag;
	uint8_t	absd:1;			// Absolute/Increse  0/1;
	uint8_t	sign:1;			// Negtive flag;
}FAD_DEF;

struct GCODE_ADRS_STRUCT_DEF
{
	uint8_t  Grp;			// G代码组
	uint8_t  Incd;		// 内部码
};


typedef union GCODE_ADRS_UNION_DEF
{
	uint16_t All;
	struct GCODE_ADRS_STRUCT_DEF GCode;
}GCD_DEF;



/****************************************************************
 * 	地址寄存器
 * **************************************************************/
typedef struct ADDRESS_REGISTER_STRUCT_DEF
{
//--------------------------------------------------------
//	类型			地址			//  地址     	长度    地址
//--------------------------------------------------------
	uint32_t		A1;			//	第1轴地址		4	0
	uint32_t		A2;			//	第2轴地址		4	4
	uint32_t		A3;			//	第3轴地址		4	8
	uint32_t		A4;			//	第4轴地址		4	12
	uint32_t		A5;			//	第5轴地址		4	16
	uint32_t		A6;			//	第6轴地址		4	20
	uint32_t		A7;			//	第7轴地址		4	24
	uint32_t		A8;			//	第8轴地址		4	28
	uint32_t		I;			// 	I-12			4	128
	uint32_t		J;			// 	J-12			4	132
	uint32_t		K;			// 	K-12			4	136
	uint32_t		E;			// 	E-8				4	140
	uint32_t		F;			// 	F-8				4	144
	uint32_t		P;			//	P-8				4	148
	uint16_t		Q;			//	Q-3				2	152
	uint32_t		R;			// 	R-12			4	154
	uint16_t		D;			//	D-3				2	158
	uint16_t		H;			//	H-3				2	160
	uint32_t		L;			// 	L-8				4	162
	uint32_t		O;			// 	O-8				4	166
	uint32_t		N;			// 	N-8				4	170
	uint32_t		S;			// 	S-8				4	174
	uint32_t		T;			// 	T-8				4	178
	uint32_t		M1;			// 	M-8				4	182
	uint32_t		M2;			// 	M-8				4	186
	uint32_t		M3;			// 	M-8				4	190
	uint32_t		B;			//	B-8				4	194
	GCD_DEF		GX;			//	GX组				2	198
	GCD_DEF		GA;			//	GA组				2	200
	GCD_DEF		GB;			//	GB组				2	202
	GCD_DEF		GC;			//	GC组				2	204
	GCD_DEF		GD;			//	GD组				2	206
	GCD_DEF		GE;			//	GE组				2	208
	GCD_DEF		GF;			//	GF组				2	210
	GCD_DEF		GG;			//	GG组				2	212
	GCD_DEF		GH;			//	GH组				2	214
	GCD_DEF		GI;			//	GI组				2	216
	GCD_DEF		GJ;			//	GJ组				2	218
	GCD_DEF		GK;			//	GK组				2	220
	GCD_DEF		GL;			//	GL组				2	222
	GCD_DEF		GM;			//	GM组				2	224
	GCD_DEF		GN;			//	GN组				2	226
	GCD_DEF		GO;			//	GO组				2	228
	GCD_DEF		GP;			//	GP组				2	230
	GCD_DEF		GQ;			//	GQ组				2	232
	GCD_DEF		GR;			//	GR组  			2	234
	GCD_DEF		GS;			//	GS组				2	236
	GCD_DEF		GT;			//	GT组				2	238
	FAD_DEF		FA1;		//	第1轴地址标志		1	240
	FAD_DEF		FA2;		//	第2轴地址标志		1	241
	FAD_DEF		FA3;		//	第3轴地址标志		1	242
	FAD_DEF		FA4;		//	第4轴地址标志		1	243
	FAD_DEF		FA5;		//	第5轴地址标志		1	244
	FAD_DEF		FA6;		//	第6轴地址标志		1	245
	FAD_DEF		FA7;		//	第7轴地址标志		1	246
	FAD_DEF		FA8;		//	第8轴地址标志		1	247
	FAD_DEF		FI;			//	地址I标志		1	272
	FAD_DEF		FJ;			//	地址J标志		1	273
	FAD_DEF		FK;			//	地址K标志		1	274
	FAD_DEF		FE;			//	地址E标志		1	275
	FAD_DEF		FF;			//	地址F标志		1	276
	FAD_DEF		FP;			//	地址P标志		1	277
	FAD_DEF		FQ;			//	地址Q标志		1	278
	FAD_DEF		FR;			//	地址R标志		1	279
	FAD_DEF		FD;			//	地址D标志		1	280
	FAD_DEF		FH;			//	地址H标志		1	281
	FAD_DEF		FL;			//	地址L标志		1	282
	FAD_DEF		FO;			//	地址O标志		1	283
	FAD_DEF		FN;			//	地址N标志		1	284
	FAD_DEF		FS;			//	地址S标志		1	285
	FAD_DEF		FT;			//	地址T标志		1	286
	FAD_DEF		FM1;		//	地址M1标志		1	287
	FAD_DEF		FM2;		//	地址M2标志		1	288
	FAD_DEF		FM3;		//	地址M3标志		1	289
	uint8_t		FB;			//	地址B标志		1	290
	uint16_t	FG;			//	地址G标志		4	291

}ADRSREG_DEF;




/*
*******************************************************************
	G-CODE TABLE OF STANDARD CODE
		LOW BYTE --- GROUP DATA
		HIG BYTE --- INTERANL DATA
*******************************************************************
*/
uint16_t	const TGCDROM[] =
{
		0x0001,		// G00-GGA
		0x0101,		// G01-GGA
		0x0201,		// G02-GGA
		0x0301,		// G03-GGA
/*										*/
		0x0000,		// G04-GGX
		0x1600,		// G05-GGX
		0xffff,		// G06-GGX
		0x1400,		// G07-GGX
		0xffff,		// G08-GGX
		0x0f00,		// G09-GGX
		0x0100,		// G10-GGX
		0x1700,		// G17-GGX
		0x0200,		// G12-GGX
		0x0300,		// G13-GGX
/*										*/
		0x0601,		// G14-GGA
		0x0701,		// G15-GGA
		0xffff,		// G16-GG-
/*										*/
		0x0002,		// G17-GGB
		0x0802,		// G18-GGB
		0x0402,		// G19-GGB
/*										*/
		0x0006,		// G20-GGF
		0x0106,		// G21-GGF
/*										*/
		0x0104,		// G22-GGD
		0x0004,		// G23-GGD
/*										*/
		0xffff,		// G24-GG-
		0x1300,		// G25-GGM
		0xffff,		// G26-GG-
/*										*/
		0x0400,		// G27-GGX
		0x0500,		// G28-GGX
		0x0600,		// G29-GGX
		0x0700,		// G30-GGX
		0x1000,		// G31-GGX
/*										*/
		0xffff,		// G32-GG-
		0x0401,		// G33-GGA
		0xffff,		// G34-GGA
		0xffff,		// G35-GGA
		0xffff,		// G36-GGA
/*								*/
		0xffff,		// G37-GG-
		0x0800,		// G38-GGX
		0x0900,		// G39-GGX
/*								*/
		0x0007,		// G40-GGG
		0x0107,		// G41-GGG
		0x0207,		// G42-GGG
/*								*/
		0x0108,		// G43-GGH
		0x0208,		// G44-GGH
/*								*/
		0x0a00,		// G45-GGX
		0x0b00,		// G46-GGX
		0x0c00,		// G47-GGX
		0x0d00,		// G48-GGX
/*								*/
		0x0d08,		// G49-GG
/*								*/
		0x000b,		// G50-GGK
		0x010b,		// G51-GGK
/*								*/
		0x1800,		// G52-GGX
		0xffff,		// G53-GGX
/*								*/
		0x000e,		// G54-GGN
		0x010e,		// G55-GGN
		0x020e,		// G56-GGN
		0x030e,		// G57-GGN
		0x040e,		// G58-GGN
		0x050e,		// G59-GGN
/*								*/
		0x1100,		// G60-GGX
/*								*/
		0x010f,		// G61-GGO
		0x020f,		// G62-GGO
		0xffff,		// G63-GGO
		0x000f,		// G64-GGO
/*								*/
		0x1200,		// G65-GGX
/*								*/
		0x010c,		// G66-GGL
		0x000c,		// G67-GGL
/*								*/
		0x0110,		// G68-GGP
		0x0010,		// G69-GGP
		0xffff,		// G70-GG-
		0xffff,		// G71-GG-
		0xffff,		// G72-GG-
/*								*/
		0x0a09,		// G73-GGI
		0x0b09,		// G74-GGI
		0xffff,		// G75-GGI
		0x0c09,		// G76-GGI
		0xffff,		// G77-GGI
		0xffff,		// G78-GGI
		0xffff,		// G79-GGI
		0x0009,		// G80-GGI
		0x0109,		// G81-GGI
		0x0209,		// G82-GGI
		0x0309,		// G83-GGI
		0x0409,		// G84-GGI
		0x0509,		// G85-GGI
		0x0609,		// G86-GGI
		0x0709,		// G87-GGI
		0x0809,		// G88-GGI
		0x0909,		// G89-GGI
/*								*/
		0x0003,		// G90-GGC
		0x0103,		// G91-GGC
/*								*/
		0x0e00,		// G92-GGX
/*								*/
		0xffff,		// G93-GGE
		0x0005,		// G94-GGE
		0x0105,		// G95-GGE
/*								*/
		0x010d,		// G96-GGD
		0x000d,		// G97-GGD
/*								*/
		0x000a,		// G98-GGJ
		0x010a,		// G99-GGJ
};

#endif
