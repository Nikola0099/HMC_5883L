#include <stdint.h>
#include <stm32f1xx_hal_conf.h>
#include <stm32f1xx_it.h>
#include <string.h>

uint16_t DevAddress =  0x3C; // Adresa uređaja (nadjena u dokumentaciji)
uint16_t DevAddressRead =  0x3D;    // Adresa za čitanje
I2C_HandleTypeDef hi2c1;

void sensorSetup(uint8_t NumOfSamples, uint8_t Data_Output_Rate, char Measurment_Mode, uint16_t Gain, uint8_t High_Speed_Mode, char Operational_Mode)
{
    /*      sensorSetup funkcija služi podešavanju parametra senzora:
     *      - Number of Samples, broj merenja izvršenih pre slanja korisniku
     *      - Data Output Rate, podešavanje brzine slanja merenih vrednosti korisniku
     *      - Operational mode ('n'- normal mode (ciklično merenje), 'f' - forced mode (jedno merenje potom sleep), 's' - sleep mode (nema merenja))
     *      - High Speed Mode, omogućava se high speed I2C protokol
     */

    uint8_t configA6_5, configA4_2, configA1_0, configA;
    
    switch(NumOfSamples)
    {
        case 1: configA6_5 = 0; break;
        case 2: configA6_5 = 32; break;
        case 4: configA6_5 = 64; break;
        case 8: configA6_5 = 96; break;
            default: configA6_5 = 96;
    }

    if(Data_Output_Rate <= 0.75)
        configA4_2 = 0;
        else if(Data_Output_Rate <= 1.5)
            configA4_2 = 4;
            else if(Data_Output_Rate <= 3)
                configA4_2 = 8;
                else if(Data_Output_Rate <= 7.5)
                    configA4_2 = 12;
                    else if(Data_Output_Rate <= 15) // Default
                        configA4_2 = 16;
                        else if(Data_Output_Rate <= 30)
                            configA4_2 = 20;
                            else
                                configA4_2 = 24;    // 75Hz
    
    switch(Measurment_Mode)
    {
        case 'n': configA1_0 = 0; break;
        case '+': configA1_0 = 1; break;
        case '-': configA1_0 = 2; break;
            default: configA1_0 = 0;
    }

    configA = configA6_5 | configA4_2 | configA1_0;

    uint8_t configA_Set[2] = {0x00, configA};

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, configA_Set, 2, HAL_MAX_DELAY);

    uint8_t configB;

    if(Gain >= 1370)
        configB = 0;
        else if(Gain >= 1090)
            configB = 32;
            else if(Gain >= 820)
                configB = 64;
                else if(Gain >= 660)
                    configB = 96;
                    else if(Gain >= 440) // Default
                        configB = 128;
                        else if(Gain >= 390)
                            configB = 160;
                            else if(Gain >= 330)
                                configB = 192;    // 75Hz
                                else
                                    configB = 224;    // Gain = 230

    uint8_t configB_Set[2] = {0x01, configB};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, configB_Set, 2, HAL_MAX_DELAY);

    uint8_t modeReg7, modeReg1_0, modeReg;

    switch(High_Speed_Mode)
    {
        case 0: modeReg7 = 0; break;
        case 1: modeReg7 = 128; break;
            default: modeReg7 = 0; break;
    }

    switch(Operational_Mode)
    {
        case 'n': modeReg1_0 = 0; break;
        case 'f': modeReg1_0 = 1; break;
        case 's': modeReg1_0 = 2; break;
            default: modeReg1_0 = 0;
    }

    modeReg = modeReg7 | modeReg1_0;

    uint8_t modeReg_Set[2] = {0x02, modeReg};
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, modeReg_Set, 2, HAL_MAX_DELAY);

}

void readValue(int16_t *AccelX, int16_t *AccelY, int16_t *AccelZ)
{

    uint8_t dataBuffer[6] = {0}, niz[6] = {0};

    uint8_t Accel_Xout_H = 0x03;
    uint8_t Accel_Xout_L = 0x04;

    uint8_t Accel_Zout_H = 0x05;
    uint8_t Accel_Zout_L = 0x06;

    uint8_t Accel_Yout_H = 0x07;
    uint8_t Accel_Yout_L = 0x08;

    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &Accel_Xout_H, 1, HAL_MAX_DELAY);
    HAL_Delay(50);
    HAL_I2C_Master_Receive(&hi2c1, DevAddressRead, dataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = dataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, (DevAddress), &Accel_Xout_L, 1, HAL_MAX_DELAY);
    HAL_Delay(50);
    HAL_I2C_Master_Receive(&hi2c1, (DevAddressRead), dataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = dataBuffer[0];
    *AccelX = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, (DevAddress), &Accel_Yout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, (DevAddressRead), dataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = dataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, (DevAddress), &Accel_Yout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, (DevAddressRead), dataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = dataBuffer[0];
    *AccelY = (uint16_t)niz[0] << 8 | niz[1];

    HAL_I2C_Master_Transmit(&hi2c1, (DevAddress), &Accel_Zout_H, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, (DevAddressRead), dataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = dataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, (DevAddress), &Accel_Zout_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, (DevAddressRead), dataBuffer, 1, HAL_MAX_DELAY);
    niz[1] = dataBuffer[0];
    *AccelZ = (uint16_t)niz[0] << 8 | niz[1];

}
