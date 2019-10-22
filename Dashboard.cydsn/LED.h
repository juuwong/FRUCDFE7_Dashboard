/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "project.h"

#if !defined(LED_H)
#define LED_H


typedef enum color {
    WHITE,
    RED,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    MAGENTA,
    OFF
} color_t;

void LED_color(color_t color) {
    switch(color){
        case WHITE:
            RGB1_1_Write(0);
            RGB2_1_Write(0);
            RGB3_1_Write(0);
        break;
        case RED:
            RGB1_1_Write(0);
            RGB2_1_Write(1);
            RGB3_1_Write(1);
        break;
        case YELLOW:
            RGB1_1_Write(0);
            RGB2_1_Write(1);
            RGB3_1_Write(0);
        break;
        case GREEN:
            RGB1_1_Write(1);
            RGB2_1_Write(1);
            RGB3_1_Write(0);
        break;
        case CYAN:
            RGB1_1_Write(1);
            RGB2_1_Write(0);
            RGB3_1_Write(0);
        break;
        case BLUE:
            RGB1_1_Write(1);
            RGB2_1_Write(0);
            RGB3_1_Write(1);
        break;
        case MAGENTA:
            RGB1_1_Write(0);
            RGB2_1_Write(0);
            RGB3_1_Write(1); 
        break;
        case OFF:
            RGB1_1_Write(1);
            RGB2_1_Write(1);
            RGB3_1_Write(1);
        break;
    }
}

void LED_color_wheel(int ms) {
    for (int i = 0; i < 8; i++) {
        LED_color(i);
        CyDelay(ms);
    }
}

#endif
/* [] END OF FILE */
