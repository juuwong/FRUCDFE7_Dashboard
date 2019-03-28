/* ========================================
 *  Hex Displays of various readings for FRUCD FE5
 *  Evan Sousa 2018
 *
 * ========================================
*/
#include "HexDisplays.h"
#include "project.h"

volatile uint8_t Digit_1 = 0;
volatile uint8_t Digit_2 = 0;
volatile uint8_t Digit_3 = 0;
volatile uint8_t Digit_4 = 0;

void hex1Display(uint8_t value)
{
    uint8_t Digit_1 = ((value)%10);
    uint8_t Digit_2 = ((value/10)%10);
    
    Hex4Reg_Write(Digit_1);
    Hex3Reg_Write(Digit_2);
    
}


/* [] END OF FILE */
