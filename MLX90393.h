#ifndef MLX90393_h
#define MLX90393_h

    #include "main.h"
    #include <stdint.h>


// val for calibration
//extern long int xmin;
//extern long int xmax;
//extern long int ymin;
//extern long int ymax;
//extern long int zmin;
//extern long int zmax;

//extern  int x_offset;
//extern int y_offset;
//extern int z_offset;

extern int lastangle;
extern int dp_last;
extern uint8_t flag_last;
extern volatile uint8_t Timer_Reset;


extern long int M1X_offset;
extern long int M2X_offset;
extern long int M3X_offset;
extern long int M4X_offset;
extern long int M5X_offset;

extern long int M1Y_offset;
extern long int M2Y_offset;
extern long int M3Y_offset;
extern long int M4Y_offset;
extern long int M5Y_offset;


extern long int x1min;
extern long int x1max;
extern long int y1min;
extern long int y1max;
extern long int z1min;
extern long int z1max;

extern long int x2min;
extern long int x2max;
extern long int y2min;
extern long int y2max;
extern long int z2min;
extern long int z2max;

extern long int x3min;
extern long int x3max;
extern long int y3min;
extern long int y3max;
extern long int z3min;
extern long int z3max;

extern long int x4min;
extern long int x4max;
extern long int y4min;
extern long int y4max;
extern long int z4min;
extern long int z4max;

extern long int x5min;
extern long int x5max;
extern long int y5min;
extern long int y5max;
extern long int z5min;
extern long int z5max;


extern float avg_delta_x1;
extern float avg_delta_y1;
extern float avg_delta_z1;
extern float avg_delta1;

extern float avg_delta_x2;
extern float avg_delta_y2;
extern float avg_delta_z2;
extern float avg_delta2;

extern float avg_delta_x3;
extern float avg_delta_y3;
extern float avg_delta_z3;
extern float avg_delta3;

extern float avg_delta_x4;
extern float avg_delta_y4;
extern float avg_delta_z4;
extern float avg_delta4;

extern float avg_delta_x5;
extern float avg_delta_y5;
extern float avg_delta_z5;
extern float avg_delta5;


extern float x1_scale;
extern float y1_scale;
extern float x2_scale;
extern float y2_scale;
extern float x3_scale;
extern float y3_scale;
extern float x4_scale;
extern float y4_scale;
extern float x5_scale;
extern float y5_scale;


extern long int T1;
extern long int T2;
extern long int T3;
extern long int T4;
extern long int T5;

///////////////////////


extern float avg_delta_x;
extern float avg_delta_y;
extern float avg_delta_z;
extern float avg_delta;

extern float x_scale;
extern float y_scale;

extern uint8_t noflash;
// selection for the function setOffset
#define X_AXIS 0x01
#define Y_AXIS 0x02
#define Z_AXIS 0x03

#define DR_STATUS 0x00

#define OUT_X_MSB 0x01
#define OUT_X_LSB 0x02
#define OUT_Y_MSB 0x03 
#define OUT_Y_LSB 0x04
#define OUT_Z_MSB 0x05
#define OUT_Z_LSB 0x06

#define WHO_AM_I 0x07   // valeur ID 0xC4 => 1100 0100
#define SYSMOD 0x08

#define OFF_X_MSB 0x09
#define OFF_X_LSB 0x0A
#define OFF_Y_MSB 0x0B
#define OFF_Y_LSB 0x0C
#define OFF_Z_MSB 0x0D
#define OFF_Z_LSB 0x0E

#define DIE_TEMP 0x0F 

#define CTRL_REG1 0x10
#define CTRL_REG2 0x11
#define DEG_PER_RAD (180.0/3.14159265358979)
#define ITERA 100


enum {
    CMD_NOP = 0x00,
    CMD_EXIT = 0x80,
    CMD_START_BURST = 0x1F,
    CMD_WAKE_ON_CHANGE = 0x2F,
    CMD_START_MEASUREMENT = 0x3F,
    CMD_READ_MEASUREMENT = 0x4F,
    CMD_READ_REGISTER = 0x50,
    CMD_WRITE_REGISTER = 0x60,
    CMD_MEMORY_RECALL = 0xd0,
    CMD_MEMORY_STORE = 0xe0,
    CMD_RESET = 0xf0
};

enum { I2C_BASE_ADDR = 0x0c };
enum { GAIN_SEL_REG = 0x0, GAIN_SEL_MASK = 0x0070, GAIN_SEL_SHIFT = 4 };
enum { HALLCONF_REG = 0x0, HALLCONF_MASK = 0x000f, HALLCONF_SHIFT = 0 };
enum { BURST_SEL_REG = 0x1, BURST_SEL_MASK = 0x03c0, BURST_SEL_SHIFT = 6};
enum { TCMP_EN_REG = 0x1, TCMP_EN_MASK = 0x0400, TCMP_EN_SHIFT = 10 };
enum { TRIG_INT_SEL_REG = 0x1, TRIG_INT_SEL_MASK = 0x8000, TRIG_INT_SEL_SHIFT = 15 };
enum { EXT_TRIG_REG = 0x1, EXT_TRIG_MASK = 0x0800, EXT_TRIG_SHIFT = 11 };
enum { OSR_REG = 0x2, OSR_MASK = 0x0003, OSR_SHIFT = 0 };
enum { OSR2_REG = 0x2, OSR2_MASK = 0x1800, OSR2_SHIFT = 11 };
enum { DIG_FLT_REG = 0x2, DIG_FLT_MASK = 0x001c, DIG_FLT_SHIFT = 2 };
enum { RES_XYZ_REG = 0x2, RES_XYZ_MASK = 0x07e0, RES_XYZ_SHIFT = 5 };
enum { SENS_TC_LT_REG = 0x3, SENS_TC_LT_MASK = 0x00FF, SENS_TC_LT_SHIFT = 0 };
enum { SENS_TC_HT_REG = 0x3, SENS_TC_HT_MASK = 0xFF00, SENS_TC_HT_SHIFT = 8 };
enum { X_OFFSET_REG = 4, Y_OFFSET_REG = 5, Z_OFFSET_REG = 6 };
enum { WOXY_THRESHOLD_REG = 7, WOZ_THRESHOLD_REG = 8, WOT_THRESHOLD_REG = 9 };
enum { BURST_MODE_BIT = 0x80, WAKE_ON_CHANGE_BIT = 0x40,
       POLLING_MODE_BIT = 0x20, ERROR_BIT = 0x10, EEC_BIT = 0x08,
       RESET_BIT = 0x04, D1_BIT = 0x02, D0_BIT = 0x01 };

enum {
    NO_OFFSET = 0x00,
    APPLY_OFFSET = 0x01,
    SAVE_OFFSET = 0x02
};

uint8_t IsAlive(void);       // OK
int16_t Get_X(void);       // OK
int16_t Get_Y(void);       // OK
int16_t Get_Z(void);       // OK
float lowpass_firfilter(long int *input);

void GetXYZT(int16_t *X, int16_t *Y, int16_t *Z, uint16_t *T, uint8_t EnableOffset);

int Zcalc(uint16_t Zval);
int Ycalc();
int Xcalc();
uint16_t Zcalib();
uint16_t Ycalib();
uint16_t Xcalib();
void switchtoactive();
void switchtostandby();
void MagSubstractOffset();
void MagSubstractFineOffset();
void MagRemoveZOFF();
int Get_Xoffset();
void active();
void standby();

float calc_angle(long int *xmax, long int *ymax, long int *xmin, long int *ymin);                        // OK
void setOffset(uint8_t axis, int offset);   // OK
//void rawData(uint8_t flag);                 // OK
void enter_cal(long int *xmax, long int *ymax, long int *zmax, long int *xmin, long int *ymin, long int *zmin);
void exit_cal(long int *xmax, long int *ymax, long int *zmax, long int *xmin, long int *ymin, long int *zmin);
void calibrate(long int *xmax, long int *ymax, long int *zmax, long int *xmin, long int *ymin, long int *zmin);
void exit_cal(long int *xmax, long int *ymax, long int *zmax, long int *xmin, long int *ymin, long int *zmin);

uint8_t Get_Temp();
void mag_zeroset();
uint8_t Single_Read_Byte(void);
void init_MLX90393(uint8_t Tint);


uint8_t ReadStatus(void);
uint8_t ReadRegister(uint8_t address, uint16_t* data);
void WriteRegister(uint8_t address, uint16_t data);

// compass calib
void enter_cal_compass(); 
void compass_calibrate();
void exit_cal_compass();

#endif