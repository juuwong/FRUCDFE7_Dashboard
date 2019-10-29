
#include <project.h>
#include <stdio.h>
#include "T6963C.h"
#include "graphic.h"
#include "LED.h"
#include "can_manga.h"


// Uncomment THROTTLE_LIMITING to enable power reduction at high temperatures
#define THROTTLE_LIMITING

volatile double THROTTLE_MULTIPLIER = 1;

// declared external in can_manga.c
volatile uint8 PACK_TEMP = 0;
volatile uint8 BSPD_CATCH = 0;
volatile int32 CURRENT = 0;
volatile int ERROR_NODE;
volatile int ERROR_IDX;
volatile uint32_t voltage = 0;
uint8_t charge = 0;

const double THROTTLE_MAP[8] = { 95, 71, 59, 47, 35, 23, 11, 5 };

// mimiced in charger code
// so master board can send SOC back and forth
// 240 values taken from data sheet
// used to approximate state of charge
int SOC_LUT[240] =  {
    0, 5, 13, 22, 31, 39,
    48, 57, 67, 76, 86, 
    90, 106, 117, 127, 138,
    150, 162, 174, 186, 199,
    212, 226, 241, 256, 271, 
    288, 205, 324, 345, 364,
    386, 410, 436, 465, 497,
    534, 577, 629, 695, 780,
    881, 972, 1044, 1103, 1157,
    1206, 1253, 1299, 1344, 1389, 
    1434, 1479, 1527, 1576, 1628, 
    1682, 1738, 1798, 1859, 1924,
    1992, 2062, 2134, 2208, 2281, 
    2424, 2492, 2557, 2620, 2681,
    2743, 2804, 2868, 2903, 3003,
    3078, 3161, 3253, 3354, 3467, 
    3589, 3720, 3851, 3976, 4092, 
    4200, 4303, 4404, 4504, 4603,
    4700, 4792, 4878, 4958, 5032,
    5101, 5166, 5228, 5289, 5347,
    5405, 5462, 5518, 5573, 5628,
    5680, 5731, 5780, 5826, 5869,
    5911, 5951, 5988, 6024, 6059,
    6092, 6124, 6156, 6187, 6217, 
    6247, 6278, 6308, 6337, 6368,
    6398, 6428, 6459, 6491, 6523,
    6556, 6590, 6625, 6660, 6696,
    6733, 6770, 6808, 6846, 6884,
    6923, 6961, 7000, 7039, 7077,
    7115, 7153, 7191, 7228, 7266, 
    7303, 7340, 7376, 7413, 7449,
    7484, 7520, 7555, 7590, 7625,
    7659, 7694, 7728, 7762, 7796,
    7830, 7864, 7898, 7932, 7966, 
    8000, 8034, 8068, 8102, 8136,
    8170, 8204, 8238, 8272, 8306,
    8340, 8373, 8406, 8440, 8472,
    8505, 8538, 8570, 8602, 8632,
    8666, 8697, 8729, 8760, 8791,
    8822, 8853, 8884, 8915, 8945, 
    8976, 9006, 9036, 9067, 9097, 
    9127, 9157, 9186, 9216, 9245,
    9275, 9304, 9333, 9362, 9390,
    9419, 9447, 9475, 9503, 9531,
    9559, 9586, 9613, 9640, 9647, 
    9693, 9720, 9746, 9772, 9797,
    9823, 9848, 9873, 9898, 9923,
    9947, 9971, 9995, 100000    
};

uint16_t curr_voltage = 0;

int firstStart = 0;
int firstLV = 0;
int firstHV = 0;
int firstDrive = 0;

#define PWM_PULSE_WIDTH_STEP        (10u)
#define SWITCH_PRESSED              (0u)
#define PWM_MESSAGE_ID              (0x1AAu)
#define PWM_MESSAGE_IDE             (0u)    /* Standard message */
#define PWM_MESSAGE_IRQ             (0u)    /* No transmit IRQ */
#define PWM_MESSAGE_RTR             (0u)    /* No RTR */
#define CAN_RX_MAILBOX_0_SHIFT      (1u)
#define CAN_RX_MAILBOX_1_SHIFT      (2u)
#define DATA_SIZE                   (6u)
#define ONE_BYTE_OFFSET             (8u)

#define PEDAL_TIMEOUT 100 // Timeout after (PEDAL_TIMEOUT * 10)ms

/* Function prototypes */
//CY_ISR_PROTO(ISR_CAN);

/* Global variables used to store configuration and data for BASIC CAN mailbox */
//CAN_DATA_BYTES_MSG dataPWM;
//CAN_TX_MSG messagePWM;

/* Global variable used to store PWM pulse width value */
//uint8 pulseWidthValue = 0u;

/* Global variable used to store receive message mailbox number */
//volatile uint8 receiveMailboxNumber = 0xFFu;

void nodeCheckStart()
{
    Node_Timer_Start();
    isr_nodeok_Start();
}

void displayData() {
    GLCD_Clear_Frame();
    GLCD_DrawInt(0,0,PACK_TEMP,8);
    GLCD_DrawInt(120,0,charge,8);
    GLCD_Write_Frame();
}

CY_ISR(ISR_WDT){
    WDT_Timer_STATUS;
    WDT_Reset_Write(0);
    CyDelay(100);
    WDT_Reset_Write(1);
}

typedef enum 
{
	Startup,
	LV,
	Precharging,
	HV_Enabled,
	Drive,
	Fault
    
}Dash_State;

typedef enum 
{
	OK,
	fromLV,
	fromPrecharging,
	fromHV_Enabled,
	fromDrive,
    fromFault,
    fromBMS,
    nodeFailure
    
}Error_State;

/* Switch state defines -- Active High*/ 
#define SWITCH_ON         (1u)
#define SWITCH_OFF        (0u)
/* Switch debounce delay in milliseconds */
#define SWITCH_DEBOUNCE_UNIT   (1u)
/* Number of debounce units to count delay, before consider that switch is pressed */
#define SWITCH_DEBOUNCE_PERIOD (10u)
/* Function prototypes */
static uint32 ReadSwSwitch(void);

/* Global variable used to store switch state */
uint8 HVSwitch = SWITCH_OFF;
uint8 DriveSwitch = SWITCH_OFF;
//volatile Dash_State state = Startup;

// Global variables used to track status of nodes
volatile uint32_t pedalOK = 0; // for pedal node

volatile int previous_state = -1; // used for SOC writing

volatile BMS_STATUS bms_status = NO_ERROR;

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  main() performs the following functions:
*  1: Initializes a structure for the Basic CAN mailbox to send messages.
*  2: Starts the CAN and LCD components.
*  3: When received Message 1, sends the PWM pulse width and displays
*     received switch status and value of PWM pulse width on an LCD; 
*     When received Message 2, display received ADC data on an LCD.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/

int main()
{   
    EEPROM_1_Start();
    
    Dash_State state = Startup;
    Error_State error_state = OK;
    
    //Tach Meter Stuff
    uint8_t value=0; // replace the value with 
    int8_t direction=1;
    
    //precharging time counter
    volatile uint32_t PrechargingTimeCount = 0;
    uint32_t DriveTimeCount = 0;

    CyGlobalIntEnable;
    
    GLCD_Initalize();
    GLCD_Clear_Graphic();
    GLCD_Clear_Text();
    GLCD_Clear_CG();
    
    nodeCheckStart();
    
    // WatchDog Timer
    WDT_Timer_Start();
    isr_wdt_StartEx(ISR_WDT);
    
    bms_status = NO_ERROR;
    int bms_error;
    int faulted = 0;
    
    
    for(;;)
    {
        
        LED_Write(1);
        
        // Check if all nodes are OK
        if (pedalOK > PEDAL_TIMEOUT)
        {
            can_send_cmd(0, 0, 0); // setInterlock. 
            state = Fault;
            error_state = nodeFailure;
        }
        
        if(bms_status != NO_ERROR && state != Startup) {
            state = Fault;
            error_state = fromBMS;
            if(!faulted)
                bms_error = bms_status;
        }

        
        switch(state)
        {    

            // startup -- 
            case Startup:
                GLCD_Clear_Frame();
                if(firstStart == 0) {
                    GLCD_DrawString(0,0,"START",8);
                    GLCD_Write_Frame();
                } else {
                    displayData();
                }
                
                //Initialize CAN
                CAN_GlobalIntEnable();
                CAN_Init();
                CAN_Start();
                LED_color_wheel(200);
                
                can_send_status(state, error_state);

                Buzzer_Write(0);
                CyDelay(50);
                
                Buzzer_Write(1);
                
                state = LV;
                
            break;
                
            case LV:
                if(firstLV == 0) {
                    GLCD_Clear_Frame();
                    GLCD_DrawString(0,0,"LV",8);
                    GLCD_Write_Frame();
                    firstLV = 1;
                } else {
                    charge = SOC_LUT[(voltage - 93400) / 100] / 100;
                    displayData();
                }
                
                CAN_GlobalIntEnable();
                CAN_Init();
                CAN_Start();
                
                can_send_cmd(0, 0, 0);
                //nodeCheckStart();
          
                can_send_status(state, error_state);

                // calcualte SOC and display SOC
                //charge = SOC_LUT[(voltage - 93400) / 100] / 100;
                //hex2Display(charge);

                Buzzer_Write(0);
                
                
                //
                // RGB code goes here
                // pick a color
                // all on. 
                LED_color(YELLOW);
                
                if (Drive_Read())
                {
                    state = Fault;
                    error_state = fromLV;
                    break;
                }
            
                if (HV_Read())    /* Switch state changed status */
                {
                    state = Precharging;
                    break;
                } 
                
            break;
                
            case Precharging:
                GLCD_Clear_Frame();
                GLCD_DrawString(0,0,"PRCHGE",8);
                GLCD_Write_Frame();
                
                CAN_GlobalIntEnable();
                CAN_Init();
                CAN_Start();
                //nodeCheckStart();
                
                can_send_status(state, error_state);
                
                LED_color(MAGENTA);
                Buzzer_Write(0);
                
                PrechargingTimeCount = 0;
                
                while(1)
                {
                    can_send_cmd(1, 0, 0); // setInterlock
                
                    uint8_t CapacitorVolt = getCapacitorVoltage(); //can_read(data_queue, data_head, data_tail, 0x0566, 0);
                   
                    if(CapacitorVolt >= 0x16) // need to be tuned
                    {
                        state = HV_Enabled;
                        break;
                    }
                
                
                    PrechargingTimeCount++;
                
                    if (PrechargingTimeCount == 10)
                    {
                        state = Fault;
                        error_state = fromPrecharging;
                        break;
                    }
                
                    CyDelay(1000);
                }    
                
            break;
	        
            case HV_Enabled:
                if(firstHV == 0) {
                    GLCD_Clear_Frame();
                    GLCD_DrawString(0,0,"HV",8);
                    GLCD_Write_Frame();
                    firstHV = 1;
                } else {
                    charge = SOC_LUT[(voltage - 93400) / 100] / 100;
                    displayData();
                }

                CAN_GlobalIntEnable();
                CAN_Init();
                CAN_Start();
                
                //nodeCheckStart();
                
                can_send_status(state, error_state);
                
                //
                // RGB code goes here
                // Blue
                LED_color(BLUE);
                
                /*
                RGB3_2_Write(1);
                RGB2_2_Write(1);
                RGB1_2_Write(1);
                */
                //CyDelay(5000); ///for debug
                
                Buzzer_Write(0);
                
                //charge = SOC_LUT[(voltage - 93400) / 100] / 100;
                //hex1Display(charge);
                
                if (Drive_Read())
                {
                    CyDelay(1000); // wait for the brake msg to be sent
                    if(getErrorTolerance() == 0) // 100 for error tolerance /// needs to be getErrorTolerance
                    {
                        Buzzer_Write(1);
                        CyDelay(1000);
                        Buzzer_Write(0);
                        state = Drive;
                        break;
                    }
                    else
                    {
                        state = Fault;
                        error_state = fromHV_Enabled;
                        break;
                    }                     
                }
                
                // if capacitor voltage is undervoltage, change the threshold 0x16
                if(!HV_Read() | (getCapacitorVoltage() < 0x16))
                {
                    state = LV;
                    DriveTimeCount = 0;
                    break;
                }
                
            break;
                
	        case Drive:
                if(firstDrive == 0) {
                    GLCD_Clear_Frame();
                    GLCD_DrawString(0,0,"DRIVE",8);
                    GLCD_Write_Frame();
                    firstDrive = 1;
                } else {
                    // calcualte SOC
                    if(CURRENT < 2500) {
                        charge = SOC_LUT[(voltage - 93400) / 100] / 100;
                    } 
                    displayData();
                }
                
                can_send_charge(charge, 0);
                
                can_send_status(state, error_state);
                //
                // RGB code goes here
                // Green
                LED_color(GREEN);
                   
                uint8_t ACK = 0xFF;
                
                DriveTimeCount++;
                if (DriveTimeCount > 100) //EDIT: was 100!
                {
                    DriveTimeCount = 0; 
                    ACK = getAckRx();
                }
   
                uint8_t ABS_Motor_RPM = getABSMotorRPM();
                
                uint16_t Throttle_Total = 0x0;
                Throttle_Total |= getPedalHigh() << 8;
                Throttle_Total |= getPedalLow();
                Throttle_Total = (double)Throttle_Total * THROTTLE_MULTIPLIER;
                
                uint8_t Throttle_High = Throttle_Total >> 8;
                uint8_t Throttle_Low = Throttle_Total & 0x00FF;
                
                // send attenuated throttle and interlock to motor controller
                can_send_cmd(1, Throttle_High, Throttle_Low); // setInterlock 
                
                // Display pack temp and soc on display
               // displayData();
                
                // check if everything is going well
                // if exiting drive improperly also send charge
                // probably kyle just turing the car off wrong
                if (!HV_Read()) {
                    can_send_cmd(0, 0, 0);   
                    can_send_charge(charge, 1); 
                    state = LV;
                }
                // if exiting drive mode send SOC to BMS
                // likely that car is about to shut down
                if (!Drive_Read()) {
                    state = HV_Enabled;
                    can_send_cmd(1, 0, 0);
                    can_send_charge(charge, 1);
                }
                if ((ACK != 0xFF) | 
                    (!getCurtisHeartBeatCheck())) // TODO: Heart beat message is never cleared
                {
                    can_send_cmd(0, 0, 0);
                    state = Fault;
                    error_state = fromDrive;
                    DriveTimeCount = 0;
                    break;
                }
                
                previous_state = Drive;
            break;
                
	        case Fault:
                
                CAN_GlobalIntEnable();
                CAN_Init();
                CAN_Start();
                //nodeCheckStart();
                
                can_send_status(state, error_state);
                
                //
                // RGB code goes here
                // flashing red
                LED_color(RED);
                CyDelay(1000);
                LED_color(OFF);
                CyDelay(1000);
                
                Buzzer_Write(0);
                
                if(error_state == fromBMS) {}
                GLCD_Clear_Frame();
                GLCD_DrawString(0,0,"DASH",2);
                GLCD_DrawString(0,32,"FAULT:",2);
                GLCD_DrawInt(80,32,error_state,2);
                GLCD_DrawString(110, 0, "T:", 2);
                GLCD_DrawString(110, 32, "FALUT:", 2);
                char* bms_f;
                //sprintf(bms_f, "%x", bms_error);
                if(error_state == fromBMS) {
                    GLCD_DrawChar(110, 0, PACK_TEMP, 2);
                GLCD_DrawInt(180, 32, bms_error, 2);
                GLCD_DrawInt(180, 0, ERROR_NODE, 2);
                GLCD_DrawChar(184, 0, ',', 2);
                GLCD_DrawInt(188, 0, ERROR_IDX, 2);
                }
                GLCD_Write_Frame();
                
                if(error_state == fromLV)
                {
                    if(!Drive_Read())
                    {
                        state = LV;
                        error_state = OK;
                    }
                }
                else if (error_state == fromPrecharging)
                {
                    if(!Drive_Read() && !HV_Read())
                    {
                        state = LV;
                        error_state = OK;
                    }
                }
                else if (error_state == fromHV_Enabled)
                {
                    if(!Drive_Read())
                    {
                        state = HV_Enabled;
                        error_state = OK;
                    }
                }
                else if (error_state == fromDrive)
                {   
                    can_send_cmd(1, Throttle_High, Throttle_Low); // setInterlock
                    
                    CyDelay(200);
                    
                    // Curtis Come back online again without error
                    if((getCurtisHeartBeatCheck())) // EDIT: Removed !(Curtis_Fault_Check(data_queue,data_head,data_tail) & 
                    {
                        state = LV;
                        error_state = OK;
                    }
                    else if(0xFF == getAckRx()) //ACK received
                    {
                        state = HV_Enabled;
                        error_state = OK;
                    }                   
                }
                else if (error_state == nodeFailure)
                {
                    state = Fault; //HERE
                    GLCD_DrawChar(110, 0, BSPD_CATCH, 2);
                }
                else if (error_state == fromBMS)
                {
                    state = Fault;
                }
            break;
                
            default:
                RGB3_1_Write(1);
                RGB2_1_Write(1);
                RGB1_1_Write(1);
           
        }// end of switch
        

        
    } 
}

/*******************************************************************************
* Function Name: ISR_CAN
********************************************************************************
*
* Summary:
*  This ISR is executed at a Receive Message event and set receiveMailboxNumber
*  global variable with receive message mailbox number.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
CY_ISR(ISR_CAN)
{   
    /* Clear Receive Message flag */
    CAN_INT_SR_REG.byte[1u] = CAN_RX_MESSAGE_MASK;

    /* Set the isrFlag */
    //isrFlag = 1u;    

    /* Acknowledges receipt of new message */
    CAN_RX_ACK_MESSAGE(CAN_RX_MAILBOX_0);

    ///* Clear Receive Message flag */
    //CAN_INT_SR_REG.byte[1u] = CAN_RX_MESSAGE_MASK;
    /* Switch Status message received */
   // if ((CY_GET_REG16((reg16 *) &CAN_BUF_SR_REG.byte[0u]) & CAN_RX_MAILBOX_0_SHIFT) != 0u)
   // {        
   //     receiveMailboxNumber = CAN_RX_MAILBOX_switchStatus;

        /* Acknowledges receipt of new message */
   //     CAN_RX_ACK_MESSAGE(CAN_RX_MAILBOX_switchStatus);
   // }

    /* ADC data message received */
   // if ((CY_GET_REG16((reg16 *) &CAN_BUF_SR_REG.byte[0u]) & CAN_RX_MAILBOX_1_SHIFT) != 0u)
   // {
   //     receiveMailboxNumber = CAN_RX_MAILBOX_ADCdata;

        /* Acknowledges receipt of new message */
   //     CAN_RX_ACK_MESSAGE(CAN_RX_MAILBOX_ADCdata);
   // }
}



/* [] END OF FILE */
