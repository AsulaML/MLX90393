#include "MLX90393.h"
#include "I2C.h"
#include "Mux.h"
#include "Screens.h"

// Adresse I2C MLX90393 avec A0=A1=0
#define MLX90393_ADDR_WRITE   0x18
#define MLX90393_ADDR_READ    0x19

// Commandes
#define CMD_START_MEAS        0x3F
#define CMD_READ_MEASUREMENT  0x4E
#define CMD_READ_REGISTER     0x50
#define CMD_WRITE_REGISTER    0x60

// Calibration constants (GAIN_SEL = 7, RES = 0)
#define CALIB_GAIN_XY         0.150f
#define CALIB_GAIN_Z          0.242f

// Mesure
uint16_t MOffsetX = 0, MOffsetY = 0, MOffsetZ = 0;
uint8_t TempsMesure = 0;

// ----------- Fonctions internes -----------

static int16_t Read16Bit() {
    int16_t val = ((int16_t)I2C2_read()) << 8;
    I2C2_ACK();
    val |= I2C2_read();
    I2C2_ACK();
    return val;
}

static void CalibrateAxis(int16_t* raw, uint16_t offset, float gain) {
    float val = (float)(*raw - (int16_t)offset) * gain;
    *raw = (int16_t)(val + 0.5f); // Avec arrondi
}

// ----------- Initialisation -----------

void init_MLX90393(uint8_t Tint) {
    uint16_t reg;

    ReadStatus();
    __delay_ms(75);

    ReadRegister(TCMP_EN_REG, &reg);
    WriteRegister(TCMP_EN_REG, 0); // Désactivation compensation offset

    ReadRegister(OSR_REG, &reg);

    if (Tint == 0) {
        TempsMesure = 35;
        WriteRegister(OSR_REG, (reg | (DIG_FLT_MASK + OSR_MASK)) & 0x16); // OSR=2, DIG_FLT=5
    } else {
        TempsMesure = 210;
        WriteRegister(OSR_REG, (reg | (DIG_FLT_MASK + OSR_MASK)) & 0x1F); // OSR=1, DIG_FLT=7
    }
}

// ----------- Mesure XYZ + Température -----------

void GetXYZT(int16_t* X, int16_t* Y, int16_t* Z, uint16_t* T, uint8_t EnableOffset) {

    if (EnableOffset == APPLY_OFFSET) {
        ReadRegister(X_OFFSET_REG, &MOffsetX);
        ReadRegister(Y_OFFSET_REG, &MOffsetY);
        ReadRegister(Z_OFFSET_REG, &MOffsetZ);
    } else {
        MOffsetX = MOffsetY = MOffsetZ = 0;
    }

    // Démarrage mesure
    I2C2_start();
    I2C2_write(MLX90393_ADDR_WRITE);
    I2C2_write(CMD_START_MEAS);
    I2C2_stop();

    __delay_ms(TempsMesure);

    // Demande de lecture
    I2C2_start();
    I2C2_write(MLX90393_ADDR_WRITE);
    I2C2_write(CMD_READ_MEASUREMENT);
    I2C2_stop();

    I2C2_start();
    I2C2_write(MLX90393_ADDR_READ);
    I2C2_read(); I2C2_ACK(); // Status, ignoré

    *T = Read16Bit();
    *X = Read16Bit();
    CalibrateAxis(X, MOffsetX, CALIB_GAIN_XY);
    MOffsetX = *X;

    *Y = Read16Bit();
    CalibrateAxis(Y, MOffsetY, CALIB_GAIN_XY);
    MOffsetY = *Y;

    *Z = Read16Bit();
    CalibrateAxis(Z, MOffsetZ, CALIB_GAIN_Z);
    MOffsetZ = *Z;

    I2C2_stop();

    if (EnableOffset == SAVE_OFFSET) {
        WriteRegister(X_OFFSET_REG, MOffsetX);
        WriteRegister(Y_OFFSET_REG, MOffsetY);
        WriteRegister(Z_OFFSET_REG, MOffsetZ);
    }
}

// ----------- Lecture / Écriture registre -----------

uint8_t WriteRegister(uint8_t address, uint16_t data) {
    uint8_t status;

    I2C2_start();
    I2C2_write(MLX90393_ADDR_WRITE);
    I2C2_write(CMD_WRITE_REGISTER);
    I2C2_write(data >> 8);
    I2C2_write(data & 0xFF);
    I2C2_write(address << 2);
    I2C2_stop();

    I2C2_start();
    I2C2_write(MLX90393_ADDR_READ);
    status = I2C2_read();
    I2C2_ACK();
    I2C2_stop();

    return status;
}

uint8_t ReadRegister(uint8_t address, uint16_t* data) {
    uint8_t status;

    I2C2_start();
    I2C2_write(MLX90393_ADDR_WRITE);
    I2C2_write(CMD_READ_REGISTER);
    I2C2_write(address << 2);
    I2C2_stop();

    I2C2_start();
    I2C2_write(MLX90393_ADDR_READ);
    status = I2C2_read();
    I2C2_ACK();
    *data = Read16Bit();
    I2C2_stop();

    return status;
}

// ----------- Vérification présence capteur -----------

uint8_t ReadStatus(void) {
    uint16_t dummy;
    return ReadRegister(GAIN_SEL_REG, &dummy);
}

uint8_t IsAlive(void) {
    return !(ReadStatus() & 0x04);
}