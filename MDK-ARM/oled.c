#include "oled.h"
#include "i2c.h"

#define OLED_ADDRESS 0X78


void OLED_SendCmd(uint8_t cmd)
{
    uint8_t sendBuffer[2];
    sendBuffer[0] = 0x00;  
    sendBuffer[1] = cmd;
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, sendBuffer, 2, HAL_MAX_DELAY);
}

void OLED_Init(){
HAL_Delay(100); 
OLED_SendCmd(0xAE); 
OLED_SendCmd(0xD5); 
OLED_SendCmd(0x80); 
OLED_SendCmd(0xA8);
OLED_SendCmd(0x3F); 
OLED_SendCmd(0xD3); 
OLED_SendCmd(0x00); 
OLED_SendCmd(0x40); 
OLED_SendCmd(0x8D); 
OLED_SendCmd(0x14); 
OLED_SendCmd(0x20); 
OLED_SendCmd(0x02); 
OLED_SendCmd(0xA1); 
OLED_SendCmd(0xC8); 
OLED_SendCmd(0xDA); 
OLED_SendCmd(0x12); 
OLED_SendCmd(0x81); 
OLED_SendCmd(0x7F); 
OLED_SendCmd(0xD9); 
OLED_SendCmd(0xF1); 
OLED_SendCmd(0xDB); 
OLED_SendCmd(0x40); 
OLED_SendCmd(0xA4); 
OLED_SendCmd(0xA6);
OLED_SendCmd(0xAF); 
}