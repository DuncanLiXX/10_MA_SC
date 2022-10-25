#ifndef INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_
#define INC_TOOL_COMPENSATE_TOOL_COMP_DATA_H_


/*************************************************************************
 * 	��������LITRAN-0MB �����ó���
 * 	���ʱ�䣺2022-10-15
 * 	����ˣ�	���
 * ***********************************************************************/

#include "stdint.h"

/***************************************************************
 * 	�����ַ��
 * *************************************************************/
//	       ��ַ //  ����   ����
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
#define	GX	84	//	GX��		2
#define	GA	86	//	GA��		2
#define	GB	88	//	GB��		2
#define	GC	90	//	GC��		2
#define	GD	92	//	GD��		2
#define	GE	94	//	GE��		2
#define	GF	96	//	GF��		2
#define	GG	98	//	GG��		2
#define	GH	100	//	GH��		2
#define	GI	102	//	GI��		2
#define	GJ	104	//	GJ��		2
#define	GK	106	//	GK��		2
#define	GL	108	//	GL��		2
#define	GM	110	//	GM��		2
#define	GN	112	//	GN��		2
#define	GO	114	//	GO��		2
#define	GP	116	//	GP��		2
#define	GQ	118	//	GQ��		2
#define	GR	120	//	GR��  	2
#define	GS	122	//	GS��		2
#define	GT	124	//	GT��		2
#define	FX	126	//	��1���ַ��־	1
#define	FY	127	//	��2���ַ��־	1
#define	FZ	128	//	��3���ַ��־	1
#define	F4	129	//	��4���ַ��־	1
#define	F5	130	//	��5���ַ��־	1
#define	F6	131	//	��6���ַ��־	1
#define	F7	132	//	��7���ַ��־	1
#define	F8	133	//	��8���ַ��־	1
#define	FI	134	//	��ַI��־	1
#define	FJ	135	//	��ַJ��־	1
#define	FK	136	//	��ַK��־	1
#define	FE	137	//	��ַE��־	1
#define	FF	138	//	��ַF��־	1
#define	FP	139	//	��ַP��־	1
#define	FQ	140	//	��ַQ��־	1
#define	FR	141	//	��ַR��־	1
#define	FD	142	//	��ַD��־	1
#define	FH	143	//	��ַH��־	1
#define	FL	144	//	��ַL��־	1
#define	FO	145	//	��ַO��־	1
#define	FN	146	//	��ַN��־	1
#define	FS	147	//	��ַS��־	1
#define	FT	148	//	��ַT��־	1
#define	M	149	//	��ַM1��־	1
#define	FB	150	//	��ַB��־	1
#define	FG	151	//	��ַG��־	4
*/


/***********************************************
 * 	����Ĵ�����ַ��
 * *********************************************/
#define	OUTPUT		0	// 	���ָ��Ĵ���			1
#define	FIN			1	//	�������ܼĴ���			1
#define	SSPSV		2	// 							1
#define	FCNTL		3	//	�������ƼĴ���			1
#define	ESN1		4	// 	�岹�˶�����				1
#define	ESN2		5	//	�岹�˶�����				1
#define	SQCTL		6	// 	˳����ƼĴ���			1
#define	OTCTL		7	//	�Զ�����ATC�Ĵ���			1
#define	SHIFT		8	// 	�����λλ��			1
#define	HSHIFT		9	//	�����߲岹������λ��;	1
#define	DA1			10	// 	�����					1
#define	DA2			11	//	�����					1
#define	DAH			12	// 	�����߲岹�Ĵ���;			1
#define	CQUAD		13	//	Բ���岹���޼Ĵ���		1
#define	FLAG		14	// 							1
#define	CYL			15	//							1
#define	CYL2		16	// 							1
#define	NZRN		17	//							1
#define	DEN			18	// 	�岹�������״̬�Ĵ���	1
#define	IPLTP		19	//	�岹���ͼĴ���			1
#define	IPLC1		20	// 	�岹���ƼĴ���			8
#define	INOT		28	//	ʣ���ƶ���				32
#define	DWEL		60	// 	��ʱ�岹�Ĵ���			4
#define	DREG		64	//	ֱ�߲岹�Ĵ���			32
#define	DREGA1		96	// 	Բ���岹�Ĵ���A1			4
#define	DREGA2		100	//	Բ���岹�Ĵ���A2			4
#define	DREGB1		104	// 	Բ���岹�Ĵ���B1			4
#define	DREGB2		108	//	Բ���岹�Ĵ���B2			4
#define	ACTP1		112	// 	Բ���岹���Ĵ���A1		4
#define	ACTP2		116	//	Բ���岹���Ĵ���A2		4
#define	ENDP1		120	// 	Բ���岹�յ�Ĵ���B1		4
#define	ENDP2		124	//	Բ���岹�յ�Ĵ���B2		4
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
#define	PCR1		175	// 	Բ���岹��������Ĵ���A	2
#define	PCR2		177	//	Բ���岹��������Ĵ���B	2
#define	PA1			179	// 	Բ���岹����Ĵ���1A		2
#define	PA2			181	//	Բ���岹����Ĵ���1B		2
#define	PB1			183	// 	Բ���岹����Ĵ���2A		2
#define	PB2			185	//	Բ���岹����Ĵ���2B		2
#define	FACC		187	// 	������������				2
#define	DACC 		189	// 	ֱ�߲岹DDA����			16
#define	DACA1		205	// 	Բ���岹DDAA����			4
#define	DACA2		209	// 							4
#define	DACB1		213	// 	Բ���岹DDAB����			4
#define	DACB2		217	// 							4
#define	SE			221	// 	�ŷ��ụ���ź�			1


typedef struct WORD_ADRS_FLAG_BITS
{
	uint8_t	inp:1;			// Input flag;
	uint8_t	absd:1;			// Absolute/Increse  0/1;
	uint8_t	sign:1;			// Negtive flag;
}FAD_DEF;

struct GCODE_ADRS_STRUCT_DEF
{
	uint8_t  Grp;			// G������
	uint8_t  Incd;		// �ڲ���
};


typedef union GCODE_ADRS_UNION_DEF
{
	uint16_t All;
	struct GCODE_ADRS_STRUCT_DEF GCode;
}GCD_DEF;



/****************************************************************
 * 	��ַ�Ĵ���
 * **************************************************************/
typedef struct ADDRESS_REGISTER_STRUCT_DEF
{
//--------------------------------------------------------
//	����			��ַ			//  ��ַ     	����    ��ַ
//--------------------------------------------------------
	uint32_t		A1;			//	��1���ַ		4	0
	uint32_t		A2;			//	��2���ַ		4	4
	uint32_t		A3;			//	��3���ַ		4	8
	uint32_t		A4;			//	��4���ַ		4	12
	uint32_t		A5;			//	��5���ַ		4	16
	uint32_t		A6;			//	��6���ַ		4	20
	uint32_t		A7;			//	��7���ַ		4	24
	uint32_t		A8;			//	��8���ַ		4	28
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
	GCD_DEF		GX;			//	GX��				2	198
	GCD_DEF		GA;			//	GA��				2	200
	GCD_DEF		GB;			//	GB��				2	202
	GCD_DEF		GC;			//	GC��				2	204
	GCD_DEF		GD;			//	GD��				2	206
	GCD_DEF		GE;			//	GE��				2	208
	GCD_DEF		GF;			//	GF��				2	210
	GCD_DEF		GG;			//	GG��				2	212
	GCD_DEF		GH;			//	GH��				2	214
	GCD_DEF		GI;			//	GI��				2	216
	GCD_DEF		GJ;			//	GJ��				2	218
	GCD_DEF		GK;			//	GK��				2	220
	GCD_DEF		GL;			//	GL��				2	222
	GCD_DEF		GM;			//	GM��				2	224
	GCD_DEF		GN;			//	GN��				2	226
	GCD_DEF		GO;			//	GO��				2	228
	GCD_DEF		GP;			//	GP��				2	230
	GCD_DEF		GQ;			//	GQ��				2	232
	GCD_DEF		GR;			//	GR��  			2	234
	GCD_DEF		GS;			//	GS��				2	236
	GCD_DEF		GT;			//	GT��				2	238
	FAD_DEF		FA1;		//	��1���ַ��־		1	240
	FAD_DEF		FA2;		//	��2���ַ��־		1	241
	FAD_DEF		FA3;		//	��3���ַ��־		1	242
	FAD_DEF		FA4;		//	��4���ַ��־		1	243
	FAD_DEF		FA5;		//	��5���ַ��־		1	244
	FAD_DEF		FA6;		//	��6���ַ��־		1	245
	FAD_DEF		FA7;		//	��7���ַ��־		1	246
	FAD_DEF		FA8;		//	��8���ַ��־		1	247
	FAD_DEF		FI;			//	��ַI��־		1	272
	FAD_DEF		FJ;			//	��ַJ��־		1	273
	FAD_DEF		FK;			//	��ַK��־		1	274
	FAD_DEF		FE;			//	��ַE��־		1	275
	FAD_DEF		FF;			//	��ַF��־		1	276
	FAD_DEF		FP;			//	��ַP��־		1	277
	FAD_DEF		FQ;			//	��ַQ��־		1	278
	FAD_DEF		FR;			//	��ַR��־		1	279
	FAD_DEF		FD;			//	��ַD��־		1	280
	FAD_DEF		FH;			//	��ַH��־		1	281
	FAD_DEF		FL;			//	��ַL��־		1	282
	FAD_DEF		FO;			//	��ַO��־		1	283
	FAD_DEF		FN;			//	��ַN��־		1	284
	FAD_DEF		FS;			//	��ַS��־		1	285
	FAD_DEF		FT;			//	��ַT��־		1	286
	FAD_DEF		FM1;		//	��ַM1��־		1	287
	FAD_DEF		FM2;		//	��ַM2��־		1	288
	FAD_DEF		FM3;		//	��ַM3��־		1	289
	uint8_t		FB;			//	��ַB��־		1	290
	uint16_t	FG;			//	��ַG��־		4	291

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
