#include "MLX90393.h"
#include "I2C.h"
#include "Mux.h"
#include "Screens.h"

uint16_t MOffsetX, MOffsetY, MOffsetZ;
uint8_t TempsMesure = 0;
        
// MLX90393xLW-ABA-011-RE avec A0 et A1 à zero => adresse 0x0Ch
void init_MLX90393(uint8_t Tint) {
    
    uint16_t Reg0, Reg1, Reg2, Reg3;
    ReadStatus();
    
    __delay_ms(75);
    
    ReadRegister(GAIN_SEL_REG, &Reg0);
    
    // Read reg 1
    ReadRegister(TCMP_EN_REG, &Reg1);
    // Disable offset compensation
    WriteRegister(TCMP_EN_REG, 0);
    
    // Read reg 2
    ReadRegister(OSR_REG, &Reg2);
    
    // Single Magnetic axis conversion time: (2+2^DIG_FILT)*2^OSR*0.064
    // DIG_FILT = 7, OSR = 1 => 50ms  (reduced noise) => 0x1D
    // DIG_FILT = 5, OSR = 2 => 27ms  (reduced noise) => 0x16
    if (Tint == 0) {
        
        // Page 10 datasheet conv time
        TempsMesure = 35; //ms
        WriteRegister(OSR_REG, (Reg2 | (DIG_FLT_MASK+OSR_MASK)) & 0x16);
    } else {
        
        // Page 10 datasheet conv time
        TempsMesure = 210; //ms
        WriteRegister(OSR_REG, (Reg2 | (DIG_FLT_MASK+OSR_MASK)) & 0x1F);
    }
    
    // Read reg 2
    ReadRegister(OSR_REG, &Reg2);
    
    // Read reg 3
    ReadRegister(SENS_TC_LT_REG, &Reg3);
}



void GetXYZT(int16_t* X, int16_t* Y, int16_t* Z, uint16_t* T, uint8_t EnableOffset) {
    
    uint8_t status;
    float calib;
    
    // Apply Offset
    if (EnableOffset == APPLY_OFFSET) {
        // Read Offsets first
        ReadRegister(X_OFFSET_REG, &MOffsetX);
        ReadRegister(Y_OFFSET_REG, &MOffsetY);
        ReadRegister(Z_OFFSET_REG, &MOffsetZ);
        
    // NO_OFFSET
    } else {
        MOffsetX = 0; 
        MOffsetY = 0;
        MOffsetZ = 0; 
    }
        
    // Start Single Measurement Mode
    I2C2_start();
    // Address + Write
    I2C2_write(0x18);
    // Start Single Measurement Command
    I2C2_write(0x3F);
    I2C2_stop();
    
    // Read Status
    I2C2_start();
    // Address + Read
    I2C2_write(0x19);
    // read byte
    status = I2C2_read();
    I2C2_ACK();
    I2C2_stop();
    
    // Page 10 datasheet conv time
    __delay_ms(TempsMesure);
    
    // Read Measurement
    I2C2_start();
    // Address + Write
    I2C2_write(0x18);
    // Read Command
    I2C2_write(CMD_READ_MEASUREMENT);
    I2C2_stop();
    
    // Read Status
    I2C2_start();
    // Address + Read
    I2C2_write(0x19);
    // read byte
    status = I2C2_read();
    I2C2_ACK();

    // read byte
    *T = (int16_t)I2C2_read()*256;
    I2C2_ACK();

    // read byte
    *T += I2C2_read();
    I2C2_ACK();

    // read byte
    *X = (int16_t)I2C2_read()*256;
    I2C2_ACK();

    // read byte
    *X += I2C2_read();
    I2C2_ACK();
        
    // apply calibration 0.150 for GAIN_SEL = 7 and RES = 0
    // page 27 datasheet
    calib = *X - (int16_t)MOffsetX;
    calib *= 0.150;
    MOffsetX = *X;   // Save offset for the end of function
    *X = calib;

    // read byte
    *Y = (int16_t)I2C2_read()*256;
    I2C2_ACK();

    // read byte
    *Y += I2C2_read();
    I2C2_ACK();

    // apply calibration 0.150 for GAIN_SEL = 7 and RES = 0
    // page 27 datasheet
    calib = *Y - (int16_t)MOffsetY;
    calib *= 0.150;
    MOffsetY = *Y;   // Save offset for the end of function
    *Y = calib;
    
    // read byte
    *Z = (int16_t)I2C2_read()*256;
    I2C2_ACK();

    // read byte
    *Z += I2C2_read();
    I2C2_ACK();
    
    // apply calibration 0.242 for GAIN_SEL = 7 and RES = 0
    // page 27 datasheet
    calib = *Z - (int16_t)MOffsetZ;
    calib *= 0.150;
    MOffsetZ = *Z;     // Save offset for the end of function
    *Z = calib;
    
    I2C2_stop();
    
    // Save current values as offset
    if (EnableOffset == SAVE_OFFSET) {
        WriteRegister(X_OFFSET_REG, MOffsetX);
        ReadRegister(X_OFFSET_REG, &MOffsetX);
        WriteRegister(Y_OFFSET_REG, MOffsetY);
        ReadRegister(Y_OFFSET_REG, &MOffsetY);
        WriteRegister(Z_OFFSET_REG, MOffsetZ);
        ReadRegister(Z_OFFSET_REG, &MOffsetZ);
    }
}

uint8_t ReadStatus() {
    
    uint16_t data;
    return ReadRegister(GAIN_SEL_REG, &data);
}

void WriteRegister(uint8_t address, uint16_t data) {
    
    uint8_t status;
    
    I2C2_start();
    // Address + Write
    I2C2_write(0x18);
    // Write Reg
    I2C2_write(CMD_WRITE_REGISTER);  
    // MSB
    I2C2_write(data>>8);
    // LSB
    I2C2_write(data);
    // address
    I2C2_write(address<<2);
    I2C2_stop();
    
    
    // Read Status
    I2C2_start();
    // Address + Read
    I2C2_write(0x19);
    // read byte
    status = I2C2_read();
    I2C2_ACK();
    I2C2_stop();
}

// Read Register RR 5  0101 0abc {A5...A0,0,0}
uint8_t ReadRegister(uint8_t address, uint16_t* data) {
    
    uint8_t status;
    
    I2C2_start();
    // Address + Write
    I2C2_write(0x18);    
    // Address + Write
    I2C2_write(CMD_READ_REGISTER); 
    // address : 
    I2C2_write((address)<<2);    
    I2C2_stop();
    
    // Read Status
    I2C2_start();
    // Address + Read
    I2C2_write(0x19);
    // read byte
    status = I2C2_read();
    I2C2_ACK();

    // read byte
    *data = (int16_t)I2C2_read()*256;
    I2C2_ACK();

    // read byte
    *data += I2C2_read();
    I2C2_ACK();
    I2C2_stop();

    return status;
}

// Test RS bit page 19
// RS bit As soon as the first status byte is read, 
// the flag is cleared until the next reset occurs.
uint8_t IsAlive(void) {
    
    uint8_t status;
    status = ReadStatus();
    
    // if 1 not alive
    if (status & 0x04) return false; 
    else return true;
}
