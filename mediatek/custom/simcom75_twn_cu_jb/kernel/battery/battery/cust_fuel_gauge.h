#include <mach/mt_typedefs.h>

#define FGAUGE_VOLTAGE_FACTOR           2.44 // mV
#define FGAUGE_CURRENT_FACTOR           6.25 // uV/Rsns
#define FGAUGE_CURRENT_OFFSET_FACTOR    1.56 // uV/Rsns
#define FGAUGE_CAR_FACTOR               6.25 // uV/Rsns
#define FGAUGE_RSNS_FACTOR              0.02 // Ohm

//#define COMPASATE_OCV                   80 // mV for evb
#define COMPASATE_OCV                   40 // mV for phone

#define BATTERY_VOLTAGE_MINIMUM         3400
#define BATTERY_VOLTAGE_MAXIMUM         4200

#if defined(ACER_Z1) || defined(ACER_Z2)
#define BATTERY_CAPACITY_MAXIMUM        1322
#else
#define BATTERY_CAPACITY_MAXIMUM        1135
#endif
#define TEMPERATURE_T0                  110
#define TEMPERATURE_T1                  0
#define TEMPERATURE_T2                  25
#define TEMPERATURE_T3                  50
#define TEMPERATURE_T                   255 // This should be fixed, never change the value

//#define BATT_CAPACITY                   1280
#if defined(ACER_Z1) || defined(ACER_Z2)
#define BATT_CAPACITY                   1299
#else
#define BATT_CAPACITY                   1135
#endif

#define ENABLE_SW_COULOMB_COUNTER       0 // 1 is enable, 0 is disable
//#define ENABLE_SW_COULOMB_COUNTER       1 // 1 is enable, 0 is disable

//#define FG_CURRENT_OFFSET_DISCHARGING 	31
#define FG_CURRENT_OFFSET_DISCHARGING 	0

#define FG_RESISTANCE 	20

#define FG_METER_RESISTANCE 	0
//#define FG_METER_RESISTANCE 	540 // current meter

//#define MAX_BOOTING_TIME_FGCURRENT	5*6 // 5 seconds, 6 points = 1s
#define MAX_BOOTING_TIME_FGCURRENT	1*10 // 10s

#if defined(CONFIG_POWER_EXT)
//#define OCV_BOARD_COMPESATE	32 //mV 
#define OCV_BOARD_COMPESATE	72 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#else
//#define OCV_BOARD_COMPESATE	0 //mV 
//#define OCV_BOARD_COMPESATE	48 //mV 
//#define OCV_BOARD_COMPESATE	25 //mV 
#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
//#define R_FG_BOARD_SLOPE	1057 //slope
//#define R_FG_BOARD_SLOPE	1075 //slope
#endif

#if defined(ACER_Z1) || defined(ACER_Z2) 
#define Q_MAX_POS_50	1322
#define Q_MAX_POS_25	1299
#define Q_MAX_POS_0		1241
#define Q_MAX_NEG_10	1163

#define Q_MAX_POS_50_H_CURRENT	1308
#define Q_MAX_POS_25_H_CURRENT	1241
#define Q_MAX_POS_0_H_CURRENT	1197
#define Q_MAX_NEG_10_H_CURRENT	1018
#else
#define Q_MAX_POS_50	1965
#define Q_MAX_POS_25	1984
#define Q_MAX_POS_0		1946
#define Q_MAX_NEG_10	1873

#define Q_MAX_POS_50_H_CURRENT	1951
#define Q_MAX_POS_25_H_CURRENT	1969
#define Q_MAX_POS_0_H_CURRENT	1913
#define Q_MAX_NEG_10_H_CURRENT	1782
#endif

#define R_FG_VALUE 				20 // mOhm, base is 20
#define CURRENT_DETECT_R_FG	100  //10mA

#define OSR_SELECT_7			0

#if defined(ACER_Z1) || defined(ACER_Z2)
#define CAR_TUNE_VALUE			99
#else
#define CAR_TUNE_VALUE			100 //1.00
#endif
/////////////////////////////////////////////////////////////////////
// <DOD, Battery_Voltage> Table
/////////////////////////////////////////////////////////////////////
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

#if defined(ACER_Z1) || defined(ACER_Z2)
// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
    {0 ,4168},
    {2 ,4148},
    {3 ,4108},
    {5 ,4081},
    {7 ,4045},
    {9 ,4010},
    {10,3990},
    {12,3977},
    {14,3966},
    {15,3958},
    {17,3952},
    {19,3948},
    {21,3944},
    {22,3938},
    {24,3930},
    {26,3921},
    {27,3913},
    {29,3902},
    {31,3891},
    {33,3879},
    {34,3868},
    {36,3856},
    {38,3846},
    {39,3837},
    {41,3829},
    {43,3822},
    {45,3817},
    {46,3812},
    {48,3807},
    {50,3803},
    {52,3800},
    {53,3797},
    {55,3793},
    {57,3791},
    {58,3790},
    {60,3788},
    {62,3788},
    {63,3787},
    {65,3787},
    {67,3786},
    {69,3784},
    {70,3783},
    {72,3781},
    {74,3778},
    {75,3773},
    {77,3768},
    {79,3760},
    {81,3751},
    {82,3740},
    {84,3727},
    {86,3718},
    {88,3714},
    {89,3710},
    {91,3707},
    {93,3704},
    {94,3697},
    {96,3684},
    {97,3659},
    {97,3632},
    {98,3608},
    {98,3584},
    {99,3565},
    {99,3546},
    {99,3531},
    {99,3516},
    {100,3505},
    {100,3495},
    {100,3486},
    {100,3478},
    {100,3472},
    {100,3400}
};      
        
// T1 0C
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{       
    {0 ,4152},
    {2 ,4125},
    {3 ,4108},
    {5 ,4096},
    {6 ,4090},
    {8 ,4082},
    {10,4056},
    {11,4023},
    {13,3999},
    {15,3984},
    {16,3973},
    {18,3965},
    {19,3959},
    {21,3956},
    {22,3952},
    {24,3945},
    {26,3936},
    {27,3928},
    {29,3920},
    {31,3912},
    {32,3903},
    {34,3894},
    {35,3884},
    {37,3872},
    {39,3860},
    {40,3848},
    {42,3837},
    {43,3828},
    {45,3821},
    {47,3815},
    {48,3809},
    {50,3806},
    {51,3801},
    {53,3797},
    {55,3794},
    {56,3792},
    {58,3788},
    {59,3787},
    {61,3785},
    {63,3784},
    {64,3784},
    {66,3785},
    {68,3784},
    {69,3783},
    {71,3783},
    {72,3782},
    {74,3779},
    {76,3777},
    {77,3771},
    {79,3765},
    {80,3756},
    {82,3745},
    {84,3732},
    {85,3718},
    {87,3711},
    {88,3708},
    {90,3704},
    {92,3702},
    {93,3700},
    {95,3691},
    {97,3645},
    {98,3560},
    {99,3497},
    {100,3453},
    {100,3419},
    {100,3396},
    {100,3379},
    {100,3368},
    {100,3358},
    {100,3351},
    {100,3351}
};      

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
    {0 ,4178},
    {2 ,4158},
    {3 ,4141},
    {5 ,4125},
    {6 ,4109},
    {8 ,4095},
    {9 ,4083},
    {11,4076},
    {12,4067},
    {14,4046},
    {15,4020},
    {17,4001},
    {18,3988},
    {20,3981},
    {21,3975},
    {23,3969},
    {25,3960},
    {26,3951},
    {28,3943},
    {29,3933},
    {31,3925},
    {32,3917},
    {34,3909},
    {35,3901},
    {37,3893},
    {38,3885},
    {40,3876},
    {41,3864},
    {43,3850},
    {45,3838},
    {46,3827},
    {48,3819},
    {49,3813},
    {51,3808},
    {52,3803},
    {54,3799},
    {55,3795},
    {57,3792},
    {58,3789},
    {60,3785},
    {61,3782},
    {63,3780},
    {65,3778},
    {66,3777},
    {68,3776},
    {69,3775},
    {71,3773},
    {72,3772},
    {74,3769},
    {75,3766},
    {77,3760},
    {78,3753},
    {80,3748},
    {81,3742},
    {83,3730},
    {85,3719},
    {86,3705},
    {88,3695},
    {89,3692},
    {91,3691},
    {92,3689},
    {94,3688},
    {95,3679},
    {97,3633},
    {98,3554},
    {100,3424},
    {100,3318},
    {100,3290},
    {100,3280},
    {100,3270},
    {100,3270}
};

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
    {0 ,4185},
    {2 ,4168},
    {3 ,4151},
    {5 ,4134},
    {6 ,4119},
    {8 ,4106},
    {9 ,4091},
    {11,4077},
    {12,4065},
    {14,4053},
    {15,4041},
    {17,4026},
    {18,4012},
    {20,4002},
    {21,3992},
    {23,3981},
    {24,3971},
    {26,3961},
    {27,3951},
    {29,3941},
    {30,3933},
    {32,3924},
    {33,3915},
    {35,3907},
    {36,3899},
    {38,3892},
    {39,3883},
    {41,3875},
    {42,3863},
    {44,3848},
    {45,3834},
    {47,3825},
    {48,3818},
    {50,3811},
    {51,3806},
    {53,3802},
    {54,3797},
    {56,3793},
    {57,3789},
    {59,3786},
    {60,3783},
    {62,3780},
    {63,3778},
    {65,3775},
    {66,3773},
    {68,3770},
    {69,3767},
    {71,3760},
    {72,3753},
    {74,3751},
    {75,3748},
    {77,3743},
    {79,3737},
    {80,3731},
    {82,3725},
    {83,3716},
    {85,3705},
    {86,3692},
    {88,3680},
    {89,3679},
    {91,3678},
    {92,3677},
    {94,3675},
    {95,3664},
    {97,3621},
    {98,3552},
    {100,3445},
    {100,3324},
    {100,3295},
    {100,3281},
    {100,3281}
};             

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
  	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};      

/////////////////////////////////////////////////////////////////////
// <Rbat, Battery_Voltage> Table
/////////////////////////////////////////////////////////////////////
typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
    {335 ,4168},
    {335 ,4148},
    {368 ,4108},
    {468 ,4081},
    {490 ,4045},
    {490 ,4010},
    {500 ,3990},
    {508 ,3977},
    {510 ,3966},
    {515 ,3958},
    {523 ,3952},
    {533 ,3948},
    {540 ,3944},
    {545 ,3938},
    {548 ,3930},
    {548 ,3921},
    {550 ,3913},
    {545 ,3902},
    {540 ,3891},
    {533 ,3879},
    {528 ,3868},
    {520 ,3856},
    {518 ,3846},
    {518 ,3837},
    {518 ,3829},
    {518 ,3822},
    {525 ,3817},
    {530 ,3812},
    {535 ,3807},
    {543 ,3803},
    {548 ,3800},
    {558 ,3797},
    {560 ,3793},
    {565 ,3791},
    {573 ,3790},
    {575 ,3788},
    {583 ,3788},
    {588 ,3787},
    {595 ,3787},
    {603 ,3786},
    {610 ,3784},
    {620 ,3783},
    {630 ,3781},
    {640 ,3778},
    {648 ,3773},
    {660 ,3768},
    {670 ,3760},
    {685 ,3751},
    {703 ,3740},
    {720 ,3727},
    {743 ,3718},
    {785 ,3714},
    {838 ,3710},
    {915 ,3707},
    {1025,3704},
    {1183,3697},
    {1215,3684},
    {1155,3659},
    {1088,3632},
    {1030,3608},
    {968 ,3584},
    {913 ,3565},
    {875 ,3546},
    {835 ,3531},
    {800 ,3516},
    {765 ,3505},
    {745 ,3495},
    {720 ,3486},
    {703 ,3478},
    {683 ,3472},
    {513 ,3400}
};      

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
    {275,4152},
    {275,4125},
    {283,4108},
    {290,4096},
    {305,4090},
    {325,4082},
    {320,4056},
    {320,4023},
    {320,3999},
    {325,3984},
    {328,3973},
    {333,3965},
    {333,3959},
    {340,3956},
    {350,3952},
    {350,3945},
    {348,3936},
    {353,3928},
    {355,3920},
    {355,3912},
    {355,3903},
    {353,3894},
    {348,3884},
    {338,3872},
    {328,3860},
    {315,3848},
    {305,3837},
    {303,3828},
    {300,3821},
    {303,3815},
    {303,3809},
    {305,3806},
    {308,3801},
    {310,3797},
    {313,3794},
    {320,3792},
    {320,3788},
    {325,3787},
    {323,3785},
    {328,3784},
    {333,3784},
    {338,3785},
    {340,3784},
    {343,3783},
    {348,3783},
    {350,3782},
    {350,3779},
    {353,3777},
    {350,3771},
    {353,3765},
    {353,3756},
    {353,3745},
    {358,3732},
    {355,3718},
    {360,3711},
    {373,3708},
    {388,3704},
    {418,3702},
    {473,3700},
    {548,3691},
    {625,3645},
    {798,3560},
    {748,3497},
    {638,3453},
    {560,3419},
    {495,3396},
    {460,3379},
    {423,3368},
    {410,3358},
    {393,3351},
    {383,3351}
};      

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
    {148,4178},
    {148,4158},
    {153,4141},
    {155,4125},
    {155,4109},
    {160,4095},
    {163,4083},
    {170,4076},
    {178,4067},
    {175,4046},
    {173,4020},
    {175,4001},
    {175,3988},
    {178,3981},
    {185,3975},
    {190,3969},
    {188,3960},
    {193,3951},
    {200,3943},
    {198,3933},
    {203,3925},
    {208,3917},
    {213,3909},
    {213,3901},
    {215,3893},
    {213,3885},
    {210,3876},
    {200,3864},
    {185,3850},
    {173,3838},
    {163,3827},
    {155,3819},
    {158,3813},
    {158,3808},
    {158,3803},
    {160,3799},
    {160,3795},
    {163,3792},
    {163,3789},
    {163,3785},
    {163,3782},
    {165,3780},
    {168,3778},
    {170,3777},
    {170,3776},
    {173,3775},
    {170,3773},
    {173,3772},
    {170,3769},
    {173,3766},
    {165,3760},
    {163,3753},
    {165,3748},
    {170,3742},
    {165,3730},
    {168,3719},
    {165,3705},
    {160,3695},
    {160,3692},
    {170,3691},
    {178,3689},
    {190,3688},
    {200,3679},
    {205,3633},
    {233,3554},
    {283,3424},
    {300,3318},
    {230,3290},
    {203,3280},
    {185,3270},
    {185,3270} 
};

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
    {123,4185},
    {123,4168},
    {125,4151},
    {128,4134},
    {130,4119},
    {135,4106},
    {135,4091},
    {135,4077},
    {140,4065},
    {140,4053},
    {145,4041},
    {145,4026},
    {143,4012},
    {148,4002},
    {150,3992},
    {150,3981},
    {153,3971},
    {155,3961},
    {155,3951},
    {155,3941},
    {163,3933},
    {165,3924},
    {168,3915},
    {170,3907},
    {175,3899},
    {183,3892},
    {180,3883},
    {183,3875},
    {170,3863},
    {153,3848},
    {138,3834},
    {133,3825},
    {130,3818},
    {130,3811},
    {130,3806},
    {133,3802},
    {135,3797},
    {135,3793},
    {138,3789},
    {140,3786},
    {143,3783},
    {145,3780},
    {148,3778},
    {148,3775},
    {150,3773},
    {148,3770},
    {148,3767},
    {135,3760},
    {128,3753},
    {135,3751},
    {140,3748},
    {138,3743},
    {135,3737},
    {138,3731},
    {138,3725},
    {138,3716},
    {138,3705},
    {138,3692},
    {128,3680},
    {130,3679},
    {140,3678},
    {148,3677},
    {155,3675},
    {158,3664},
    {178,3621},
    {203,3552},
    {248,3445},
    {315,3324},
    {240,3295},
    {208,3281},
    {208,3281}
};

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{	
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};      
#else //ACER_Z1/ACER_Z2
// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
	{0  , 4165},
	{2  , 4138},
	{3  , 4116},
	{5  , 4099},
	{6  , 4086},
	{8  , 4073},
	{10 , 4058},
	{11 , 4042},
	{13 , 4027},
	{14 , 4013},
	{16 , 4000},
	{18 , 3989},
	{19 , 3981},
	{21 , 3972},
	{22 , 3963},
	{24 , 3955},
	{26 , 3944},
	{27 , 3936},
	{29 , 3926},
	{30 , 3915},
	{32 , 3904},
	{34 , 3891},
	{35 , 3878},
	{37 , 3867},
	{38 , 3858},
	{40 , 3848},
	{42 , 3840},
	{43 , 3833},  
	{45 , 3828}, 
	{46 , 3823},
	{48 , 3816},
	{50 , 3811},  
	{51 , 3807},
	{53 , 3803},
	{54 , 3801},
	{56 , 3797},
	{58 , 3795},
	{59 , 3791},
	{61 , 3788},
	{62 , 3787},
	{64 , 3784},
	{66 , 3783},
	{67 , 3782},
	{69 , 3781},
	{70 , 3780},
	{72 , 3777},
	{74 , 3773},
	{75 , 3769},
  {77 , 3764},
  {78 , 3758},
  {80 , 3749},
  {81 , 3740},
  {83 , 3730},
  {85 , 3719},
  {86 , 3713},
  {88 , 3708},
  {89 , 3703},
  {91 , 3701},
  {93 , 3698},
  {94 , 3688},
  {96 , 3655},
  {97 , 3587},
  {98 , 3534},
  {99 , 3502},
  {99 , 3479},
  {99 , 3461},
  {100, 3446},
  {100, 3436},
  {100, 3429},
  {100, 3421},
  {100, 3417},
  {100, 3411},
  {100, 3407},
  {100, 3406},
  {100, 3403},
  {100, 3401},
  {100, 3399}
};      
        
// T1 0C
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
	{0  , 4177},
	{2  , 4157},
	{3  , 4142},
	{5  , 4125},
	{6  , 4111},
	{8  , 4097},
	{9  , 4085},
	{11 , 4073},
	{12 , 4060},
	{14 , 4046},
	{15 , 4031},
	{17 , 4018},
	{18 , 4005},
	{20 , 3994},
	{22 , 3986},
	{23 , 3978},
	{25 , 3968},
	{26 , 3958},
	{28 , 3951},
	{29 , 3941},
	{31 , 3932},
	{32 , 3922},
	{34 , 3912},
	{35 , 3902},
	{37 , 3890},
	{38 , 3876},
	{40 , 3863},
	{42 , 3853},  
	{43 , 3846}, 
	{45 , 3837},
	{46 , 3830},
	{48 , 3824},  
	{49 , 3818},
	{51 , 3813},
	{52 , 3808},
	{54 , 3805},
	{55 , 3800},
	{57 , 3798},
	{58 , 3793},
	{60 , 3791},
	{62 , 3788},
	{63 , 3787},
	{65 , 3784},
	{66 , 3783},
	{68 , 3781},
	{69 , 3780},
	{71 , 3780},
	{72 , 3778},
  {74 , 3776},
  {75 , 3772},
  {77 , 3766},
  {78 , 3761},
  {80 , 3752},
  {82 , 3744},
  {83 , 3733},
  {85 , 3723},
  {86 , 3711},
  {88 , 3704},
  {89 , 3701},
  {91 , 3699},
  {92 , 3697},
  {94 , 3693},
  {95 , 3677},
  {97 , 3623},
  {98 , 3538},
  {100, 3407},
  {100, 3362},
  {100, 3338},
  {100, 3327},
  {100, 3319},
  {100, 3316},
  {100, 3312},
  {100, 3311},
  {100, 3309},
  {100, 3309},
  {100, 3307},
  {100, 3305}
};

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
	{0  , 4189},
	{2  , 4171},
	{3  , 4157},
	{5  , 4142},
	{6  , 4127},
	{8  , 4113},
	{9  , 4100},
	{11 , 4086},
	{12 , 4073},
	{14 , 4061},
	{15 , 4049},
	{17 , 4036},
	{18 , 4024},
	{20 , 4013},
	{21 , 4003},
	{23 , 3992},
	{24 , 3980},
	{26 , 3972},
	{27 , 3962},
	{29 , 3952},
	{30 , 3943},
	{32 , 3934},
	{33 , 3926},
	{35 , 3917},
	{36 , 3908},
	{38 , 3900},
	{39 , 3887},
	{41 , 3873},  
	{42 , 3860}, 
	{44 , 3847},
	{45 , 3839},
	{47 , 3832},  
	{48 , 3826},
	{50 , 3820},
	{51 , 3813},
	{53 , 3809},
	{54 , 3804},
	{56 , 3800},
	{57 , 3797},
	{59 , 3793},
	{60 , 3789},
	{62 , 3787},
	{63 , 3784},
	{65 , 3782},
	{66 , 3778},
	{68 , 3777},
	{69 , 3774},
	{71 , 3771},
  {72 , 3767},
  {74 , 3764},
  {75 , 3758},
  {77 , 3753},
  {78 , 3747},
  {80 , 3740},
  {81 , 3734},
  {83 , 3723},
  {84 , 3713},
  {86 , 3701},
  {88 , 3690},
  {89 , 3688},
  {90 , 3686},
  {92 , 3685},
  {93 , 3680},
  {95 , 3668},
  {97 , 3626},
  {98 , 3557},
  {99 , 3454},
  {100, 3292},
  {100, 3268},
  {100, 3261},
  {100, 3257},
  {100, 3257},
  {100, 3256},
  {100, 3256},
  {100, 3256},
  {100, 3254},
  {100, 3253}
}; 

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
	{0  , 4189},
	{2  , 4174},
	{3  , 4158},
	{5  , 4144},
	{6  , 4130},
	{8  , 4115},
	{9  , 4100},
	{11 , 4087},
	{12 , 4073},
	{14 , 4060},
	{15 , 4049},
	{17 , 4035},
	{18 , 4023},
	{20 , 4012},
	{21 , 4002},
	{23 , 3991},
	{24 , 3980},
	{26 , 3970},
	{27 , 3959},
	{29 , 3949},
	{30 , 3939},
	{32 , 3932},
	{33 , 3922},
	{35 , 3915},
	{37 , 3905},
	{38 , 3898},
	{40 , 3886},
	{41 , 3872},  
	{43 , 3859}, 
	{44 , 3846},
	{46 , 3838},
	{47 , 3830},  
	{49 , 3824},
	{50 , 3817},
	{52 , 3812},
	{53 , 3808},
	{55 , 3803},
	{56 , 3797},
	{58 , 3794},
	{59 , 3791},
	{61 , 3786},
	{62 , 3784},
	{64 , 3781},
	{65 , 3778},
	{67 , 3774},
	{69 , 3772},
	{70 , 3765},
	{72 , 3755},
  {73 , 3750},
  {75 , 3745},
  {76 , 3739},
  {78 , 3733},
  {79 , 3725},
  {81 , 3721},
  {82 , 3714},
  {84 , 3702},
  {85 , 3692},
  {87 , 3678},
  {88 , 3674},
  {90 , 3672},
  {91 , 3670},
  {93 , 3668},
  {94 , 3662},
  {96 , 3636},
  {97 , 3578},
  {99 , 3491},
  {100, 3357},
  {100, 3266},
  {100, 3260},
  {100, 3257},
  {100, 3256},
  {100, 3254},
  {100, 3254},
  {100, 3254},
  {100, 3253},
  {100, 3253},
  {100, 3253}
};              

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
	  {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
	  {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}
};    

/////////////////////////////////////////////////////////////////////
// <Rbat, Battery_Voltage> Table
/////////////////////////////////////////////////////////////////////
typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
	{333    ,  4165},
	{333    ,  4138},
	{360    ,  4116},
	{373    ,  4099},
	{380    ,  4086},
	{383    ,  4073},
	{388    ,  4058},
	{393    ,  4042},
	{398    ,  4027},
	{405    ,  4013},
	{405    ,  4000},
	{410    ,  3989},
	{415    ,  3981},
	{418    ,  3972},
	{420    ,  3963},
	{425    ,  3955},
	{428    ,  3944},
	{433    ,  3936},
	{428    ,  3926},
	{428    ,  3915},
	{428    ,  3904},
	{415    ,  3891},
	{405    ,  3878},
	{398    ,  3867},
	{400    ,  3858},
	{398    ,  3848},
	{390    ,  3840},
	{393    ,  3833},  
	{395    ,  3828}, 
	{398    ,  3823},
	{398    ,  3816},
	{398    ,  3811},  
	{403    ,  3807},
	{408    ,  3803},
	{413    ,  3801},
	{415    ,  3797},
	{420    ,  3795},
	{420    ,  3791},
	{423    ,  3788},
	{425    ,  3787},
	{425    ,  3784},
	{430    ,  3783},
	{438    ,  3782},
	{440    ,  3781},
	{448    ,  3780},
	{445    ,  3777},
	{450    ,  3773},
	{450    ,  3769},
  {455    ,  3764},
  {458    ,  3758},
  {463    ,  3749},
  {465    ,  3740},
  {468    ,  3730},
  {470    ,  3719},
  {475    ,  3713},
  {490    ,  3708},
  {503    ,  3703},
  {530    ,  3701},
  {578    ,  3698},
  {638    ,  3688},
  {700    ,  3655},
  {828    ,  3587},
  {835    ,  3534},
  {760    ,  3502},
  {700    ,  3479},
  {655    ,  3461},
  {620    ,  3446},
  {595    ,  3436},
  {578    ,  3429},
  {563    ,  3421},
  {550    ,  3417},
  {533    ,  3411},
  {523    ,  3407},
  {523    ,  3406},
  {518    ,  3403},
  {505    ,  3401},
  {505    ,  3399}
};

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
	{228    ,  4177},
	{228    ,  4157},
	{238    ,  4142},
	{235    ,  4125},
	{240    ,  4111},
	{243    ,  4097},
	{243    ,  4085},
	{245    ,  4073},
	{248    ,  4060},
	{253    ,  4046},
	{253    ,  4031},
	{258    ,  4018},
	{263    ,  4005},
	{263    ,  3994},
	{270    ,  3986},
	{273    ,  3978},
	{275    ,  3968},
	{275    ,  3958},
	{285    ,  3951},
	{288    ,  3941},
	{285    ,  3932},
	{285    ,  3922},
	{285    ,  3912},
	{285    ,  3902},
	{278    ,  3890},
	{268    ,  3876},
	{255    ,  3863},
	{255    ,  3853},  
	{253    ,  3846}, 
	{250    ,  3837},
	{248    ,  3830},
	{245    ,  3824},  
	{250    ,  3818},
	{250    ,  3813},
	{248    ,  3808},
	{253    ,  3805},
	{255    ,  3800},
	{255    ,  3798},
	{255    ,  3793},
	{260    ,  3791},
	{260    ,  3788},
	{265    ,  3787},
	{263    ,  3784},
	{268    ,  3783},
	{268    ,  3781},
	{273    ,  3780},
	{273    ,  3780},
	{273    ,  3778},
  {275    ,  3776},
  {273    ,  3772},
  {268    ,  3766},
  {270    ,  3761},
  {265    ,  3752},
  {265    ,  3744},
  {265    ,  3733},
  {270    ,  3723},
  {270    ,  3711},
  {268    ,  3704},
  {275    ,  3701},
  {278    ,  3699},
  {293    ,  3697},
  {313    ,  3693},
  {325    ,  3677},
  {328    ,  3623},
  {365    ,  3538},
  {478    ,  3407},
  {408    ,  3362},
  {348    ,  3338},
  {320    ,  3327},
  {298    ,  3319},
  {293    ,  3316},
  {280    ,  3312},
  {280    ,  3311},
  {275    ,  3309},
  {280    ,  3309},
  {280    ,  3307},
  {270    ,  3305}
}; 

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
	{173    ,  4189},
	{138    ,  4171},
	{143    ,  4157},
	{140    ,  4142},
	{143    ,  4127},
	{143    ,  4113},
	{145    ,  4100},
	{145    ,  4086},
	{148    ,  4073},
	{148    ,  4061},
	{153    ,  4049},
	{153    ,  4036},
	{153    ,  4024},
	{155    ,  4013},
	{158    ,  4003},
	{158    ,  3992},
	{153    ,  3980},
	{163    ,  3972},
	{165    ,  3962},
	{165    ,  3952},
	{170    ,  3943},
	{173    ,  3934},
	{173    ,  3926},
	{178    ,  3917},
	{180    ,  3908},
	{183    ,  3900},
	{178    ,  3887},
	{170    ,  3873},  
	{160    ,  3860}, 
	{153    ,  3847},
	{150    ,  3839},
	{148    ,  3832},  
	{150    ,  3826},
	{148    ,  3820},
	{145    ,  3813},
	{150    ,  3809},
	{150    ,  3804},
	{150    ,  3800},
	{155    ,  3797},
	{155    ,  3793},
	{153    ,  3789},
	{158    ,  3787},
	{158    ,  3784},
	{160    ,  3782},
	{158    ,  3778},
	{160    ,  3777},
	{160    ,  3774},
	{158    ,  3771},
  {155    ,  3767},
  {153    ,  3764},
  {148    ,  3758},
  {150    ,  3753},
  {148    ,  3747},
  {150    ,  3740},
  {153    ,  3734},
  {150    ,  3723},
  {145    ,  3713},
  {153    ,  3701},
  {148    ,  3690},
  {148    ,  3688},
  {150    ,  3686},
  {160    ,  3685},
  {158    ,  3680},
  {158    ,  3668},
  {165    ,  3626},
  {178    ,  3557},
  {193    ,  3454},
  {233    ,  3292},
  {170    ,  3268},
  {155    ,  3261},
  {143    ,  3257},
  {143    ,  3257},
  {143    ,  3256},
  {143    ,  3256},
  {143    ,  3256},
  {145    ,  3254},
  {138    ,  3253}
}; 

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
	{170    ,  4189},
	{140    ,  4174},
	{135    ,  4158},
	{145    ,  4144},
	{145    ,  4130},
	{143    ,  4115},
	{140    ,  4100},
	{140    ,  4087},
	{140    ,  4073},
	{138    ,  4060},
	{148    ,  4049},
	{140    ,  4035},
	{140    ,  4023},
	{148    ,  4012},
	{148    ,  4002},
	{150    ,  3991},
	{153    ,  3980},
	{153    ,  3970},
	{150    ,  3959},
	{150    ,  3949},
	{150    ,  3939},
	{160    ,  3932},
	{158    ,  3922},
	{163    ,  3915},
	{163    ,  3905},
	{170    ,  3898},
	{168    ,  3886},
	{160    ,  3872},  
	{158    ,  3859}, 
	{148    ,  3846},
	{148    ,  3838},
	{145    ,  3830},  
	{148    ,  3824},
	{145    ,  3817},
	{145    ,  3812},
	{148    ,  3808},
	{150    ,  3803},
	{143    ,  3797},
	{145    ,  3794},
	{148    ,  3791},
	{148    ,  3786},
	{153    ,  3784},
	{155    ,  3781},
	{155    ,  3778},
	{158    ,  3774},
	{160    ,  3772},
	{153    ,  3765},
	{148    ,  3755},
  {150    ,  3750},
  {150    ,  3745},
  {148    ,  3739},
  {148    ,  3733},
  {140    ,  3725},
  {145    ,  3721},
  {148    ,  3714},
  {140    ,  3702},
  {145    ,  3692},
  {143    ,  3678},
  {145    ,  3674},
  {145    ,  3672},
  {148    ,  3670},
  {150    ,  3668},
  {150    ,  3662},
  {150    ,  3636},
  {158    ,  3578},
  {153    ,  3491},
  {165    ,  3357},
  {168    ,  3266},
  {153    ,  3260},
  {143    ,  3257},
  {143    ,  3256},
  {140    ,  3254},
  {140    ,  3254},
  {138    ,  3254},
  {138    ,  3253},
  {135    ,  3253},
  {138    ,  3253}
}; 

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
	  {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
	  {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}
};     
#endif

int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);