//Name: Aseem Thapa
//ID: 1001543178

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart.h"
#include "eeprom.h"
#include "wait.h"
#include "speaker.h"
#include "rule_and_help.h"

// Defined Masks:
#define IN_MASK 4 //For Port A2 (Sensor Input)
#define OUT_MASK 8 //For Port B3 (Toggle Output)
#define GPO_Mask 32 //For port B5 (PWM for LED)
#define ON_VAL 1053 //For 38kHz wave
//Allow a maximum of 32 instructions (each instruct takes 4 blocks) (0-127):
#define max_Eeprom_addr 128


//Using Green LED as sensor when the Program starts:

// PortF masks
#define GREEN_LED_MASK 8 //Use the Green LED to show blink when the input is received
#define RED_LED_MASK 1   //Red LED used in rules

//Bitband Masks:
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

//GLOABAL VARIABLES:

//Global varaible for configuring timers/ Counter for how many timer interrupts have occured. =>
uint8_t time1 = 0; //for Timer 1
uint8_t time2 = 0; //for Timer 2

//Decode the data by calculating the difference between two 0s:
uint8_t time_zero_position = 0;
uint8_t time_diff = 0;

//Global variables for value of input/decoded values.
uint16_t valueA = 0;
uint16_t valueD = 0;

//Global variable for address to be output
uint8_t global_addr = 0;
uint8_t global_data = 0;

// GLOBAL BOOLEANS/Flags:

//Modulator is the variable that is used to control when to burst and not during address and data transmissions.
bool modulator = 0;
bool decode_ON = 0; //Decode and produce output only when supposed to
bool good_alert = 1; //Boolean for good_alert
bool bad_alert = 1; //Boolean for bad_alert

//END GLOBAL VARIABLES

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2;  //Using Timers 1 and 2
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0|SYSCTL_RCGCGPIO_R1|SYSCTL_RCGCGPIO_R5; //For Clock on Ports A,B,F
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;  //PWM0 clock
    _delay_cycles(3);

    // Configure pins

    //Port A:
    GPIO_PORTA_DIR_R &= ~IN_MASK;  // Port A2 is input
    GPIO_PORTA_DEN_R |= IN_MASK;  // enable pin A2

    //Port B: (w/ PWM management):
    GPIO_PORTB_DIR_R |= OUT_MASK | GPO_Mask;  // Port B3 and B5 are outputs
    GPIO_PORTB_DR2R_R |= OUT_MASK | GPO_Mask; // Drive strength 2mA
    GPIO_PORTB_DEN_R |= OUT_MASK | GPO_Mask;  // enable pin B3 and B5
    GPIO_PORTB_AFSEL_R |= GPO_Mask;           // Select the auxillary function (PWM output in this case)
    GPIO_PORTB_PCTL_R &= GPIO_PCTL_PB5_M;     // enable PWM
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_M0PWM3;

    //Port F:
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK; //Use Green LED as output
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK;
    GPIO_PORTF_DIR_R |= RED_LED_MASK; //Use Red LED as output
    GPIO_PORTF_DEN_R |= RED_LED_MASK;

    //Configure interrupt on Pin A2:
    GPIO_PORTA_IS_R &= ~IN_MASK; //edge sensitive interrupt triggered
    GPIO_PORTA_IBE_R &= ~IN_MASK; //Use IEV to control Interrupt event
    GPIO_PORTA_IEV_R &= ~IN_MASK; //Use falling edge interrupt trigger
    GPIO_PORTA_ICR_R |= IN_MASK; //Clear the interrupt
    NVIC_EN0_R |= 1 << (INT_GPIOA-16); // turn-on interrupt 16 (GPIOA)
    GPIO_PORTA_IM_R |= IN_MASK; //Turn on interrupt

    //Configuring the PWM0 Module:
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    // Port B5 == M0PWM3 which is in PWM0_Gen1_b
    PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                    // output 3 on PWM0, gen 1b, cmpb
    PWM0_1_LOAD_R = ON_VAL;                            // This value is approx 38 kHz
    PWM0_INVERT_R = PWM_INVERT_PWM3INV;              // Invert the output
    //For Inverted PWM signal, Duty Cycle = Compare/Load:
    PWM0_1_CMPB_R = 0;                               // No output at first
    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN;               // enable outputs
}

//This function used to select the load value for different timer interrupts:
//Since only IR Decode uses this (Timer 2):
uint32_t timeselect(uint32_t time)
{
    if (time < 3) return 90000; //corresponds to 2.25 ms
    else if (time == 3) return 150000; //corresponds to 2.25ms + 1.5ms
    else if (time == 4) return 60000; //corresponds to 1.5ms
    else if (time == 5) return 80000; //corresponds to nearly 1.5ms + 562.5/2 us
    else return 22500; //corresponds to nearly 562.5 us
}

//Checks consistency of first 9 ms + 4.5 ms (only IR Decode Uses this):
bool checkerForConsistency()
{
    int Data = GPIO_PORTA_DATA_R>>2; //the 2nd bit is the data bit
    if (time2 < 3 && Data == 0)
        {
            //putcUart0(Data + 48);
            return true; //for first 9 ms of 0
        }
    else if (time2 < 3 && Data == 1)
        {
            return false;
        }
    if (time2 < 5 && Data == 1)
        {
            //putcUart0(Data + 48);
            return true; //for the 4.5 ms of 1 after 9 ms of 0.
        }
    else if (time2 < 5 && Data == 0)
        {
            return false;
        }
    return true; //The rest of the bits are data bits
}

//Function to put the output on the Uart:
void putUartValue(bool id)
{
    //For address bits:
    uint16_t valuePos = 0;
    uint16_t valueNeg = 0;
    if (!id)
    {
        valuePos = valueA >> 8;      //Address bits
        valueNeg = valueA & 0xFF;    //Compliment of address bits
    }
    else
    {
        valuePos = valueD >> 8;      //Data bits
        valueNeg = valueD & 0xFF;    //Compliment of data bits
    }
    //Conditional to check if the bits are compliments of each other----->
    if (valuePos + valueNeg == 255)
    {
        if (!id) putsUart0("Received Address bits: ");
        else putsUart0("Received Data Bits: ");
        putint8Uart0(valuePos);
        putsUart0("\r\n");
    }
    else{
        if (!id) putsUart0("Received Address bits not consistent.\r\n");
        else putsUart0("Received Data bits not consistent.\r\n");
        //reset the values:
        valueA = 0;
        valueD = 0;
    }
}


//Port A interrupt
void portAIsr()
{
    GPIO_PORTA_ICR_R |= IN_MASK; //Clear the interrupt
    GPIO_PORTA_IM_R &= ~IN_MASK; //Turn off interrupt when triggered
    GPIO_PORTB_DATA_R ^= OUT_MASK; //Trigger output.
    time2 = 0;                                       //Re-emphasize the beginning of counter
    valueA = 0;                                      //reset the value in Address
    valueD = 0;                                      //reset the value in Data
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = timeselect(time2);               // This value corresponds to 2.25ms
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

void playComment (uint8_t address, uint8_t data)
{
    time1 = 0;                                       // Re-emphasize the beginning of counter
    modulator = 0;                                   // Re-emphasize the beginning of modulator
    global_addr = address;                           // Store the address globally
    global_data = data;                              // Store the data globally.
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 360000;                         // This value corresponds to 9 ms
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    PWM0_1_CMPB_R = ON_VAL>>1;                       // 50% duty cycle (Burst)
}


//Interrupt for Timer1 (Timer 1 was used for IR Burst Control):
void Timer1Isr()
{
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;              // clear interrupt flag
        uint8_t current_bit = 0;                        //The current bit

        //For the first 4.5ms of silence:
        if (time1 == 0)
        {
            time1++;                                         // increment the data pointer for global counter
            PWM0_1_CMPB_R = 0;                               // No output here (silence)
            TIMER1_TAILR_R = 195000;                         // This value corresponds to nearly 4.5 ms
        }

        // For the address and data bits:
        else if (time1 > 0 && time1 < 33)
        {
            // Go bit by bit:
            // time: 1-8 = address, 9-16 = ~address, 17-24 = data, 25-32 = ~data.

            if (time1 < 9) current_bit = global_addr >> (8-time1);
            else if (time1 > 8 && time1 < 17) current_bit = ~(global_addr) >> (16-time1);
            else if (time1 > 16 && time1 < 25) current_bit = global_data >> (24-time1);
            else if (time1 > 24 && time1 < 33) current_bit = ~(global_data) >> (32-time1);

            //Look at the last bit of the shifted result:
            current_bit = current_bit & 1;
            //putcUart0(current_bit + 48);                        //Debug Line

            // Now when modulator = 0 it outputs burst and when modulator = 1 it outputs silence
            if (!modulator)
            {
                //GPIO_PORTB_DEN_R |= GPO_Mask;                 // Toggle DEN pin B5 as burst
                PWM0_1_CMPB_R = ON_VAL>>1;                      // Ouput 50% duty cycle burst here
                TIMER1_TAILR_R = 22500;                         // For 562.5 us
                modulator = 1;                                  // Toggle modulator
            }

            else
            {
                // Length of silence determined by the current bit
                // If bit = 0 => 562.5 us if bit = 1 => 3 * 562.5 us
                if (current_bit == 0)
                {
                    PWM0_1_CMPB_R = 0;                              // Output silence
                    TIMER1_TAILR_R = 22500;                         // For 562.5 us
                    modulator = 0;                                  // Toggle modulator
                    time1++;                                        //increment the data pointer for global counter
                    //putcUart0(48);                                //Debug Line
                }
                else
                {
                    PWM0_1_CMPB_R = 0;                              // Output silence
                    TIMER1_TAILR_R = 67500;                         // For 3 * 562.5 us
                    modulator = 0;                                  // Toggle modulator
                    time1++;                                        //increment the data pointer for global counter
                    //putcUart0(49);                                //Debug Line
                }
            }
        }

        //Transmit an additional 1 at the end of the signal to denote end:
        else if (time1 == 33)
        {
            if (!modulator)
            {
                PWM0_1_CMPB_R = ON_VAL>>1;                      // Output burst
                TIMER1_TAILR_R = 22500;                         // For 562.5 us
                modulator = 1;                                  // Toggle modulator
            }

            else
            {
                PWM0_1_CMPB_R = 0;                              // Output silence
                TIMER1_TAILR_R = 67500;                         // For 3 * 562.5 us
                modulator = 0;                                  // Toggle modulator
                time1++;                                        //increment the data pointer for global counter
            }
        }

        //End of transmission:
        else
        {
            PWM0_1_CMPB_R = 0;                              // End output here (silence)
            TIMER1_IMR_R = 0;                               // turn-off timer interrupts
            NVIC_EN0_R |= 0 << (INT_TIMER1A-16);            // turn-off interrupt 37 (TIMER1A)
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off timer
            time1 = 0;                                       // value of index goes to 0
            modulator = 0;                                  // modulator goes to 0
            return;
        }
}

//Interrupt for Timer2 (Timer 2 was used for IR Decode):
void Timer2Isr()
{
        //Case for inconsistency of received data in the 9 ms period + 4.5 ms period + address bits
        if (!checkerForConsistency())
        {
            if(decode_ON)
            {
                putsUart0("Error! (Too many presses or Input start not recognized)");
                putcUart0('\r');
                putcUart0('\n');
                time2 = 105; //This will force the function to abort
            }
        }

        if (time2 == 5)
            {
                time_zero_position = 5; //First zero for addr bits at 5
            }


        //Check for the address bits: (Address bits start at the 5th toggle)
        if (time2 > 4 & time2 <= 53)
        {
            //Check for where the 0 is. Don't look at 53rd bit as it will be start of data which will be 0:
            if ((GPIO_PORTA_DATA_R>>2) == 0 && time2 != 5)
            {
                time_diff = time2 - time_zero_position; //Calculate the difference in time between two consecutive 0s
                time_zero_position = time2;  //Update the value of latest position for a 0

                //When there is difference of 2 its a 0 when there is difference of 4 its a 1
                if (time_diff == 2)
                    {
                        //putcUart0(48); //Debug code for 0
                        valueA = (valueA<<1);
                    }
                else if (time_diff == 4)
                    {
                        //putcUart0(49); //Debug code for 1
                        valueA = (valueA<<1) + 1;
                    }
            }
        }

        if (time2 == 53)
            {
                time_zero_position = 53; //First zero for data bits at 53
                //putsUart0("\r\n");
            }

        //Check for the data bits: (Data bits start at the 53rd toggle)
        if (time2 > 52 & time2 < 105)
        {
            //putcUart0((GPIO_PORTA_DATA_R>>2)+48);
            //Check for where the 0 is. Don't look at 53rd bit as it will be start of data which will be 0:
            if ((GPIO_PORTA_DATA_R>>2) == 0 && time2 != 53)
            {
                time_diff = time2 - time_zero_position; //Calculate the difference in time between two consecutive 0s
                time_zero_position = time2;  //Update the value of latest position for a 0

                //When there is difference of 2 its a 0 when there is difference of 4 its a 1
                if (time_diff == 2)
                    {
                        //putcUart0(48); //Debug code for 0
                        valueD = (valueD<<1);
                    }
                else if (time_diff == 4)
                    {
                        //putcUart0(49); //Debug code for 1
                        valueD = (valueD<<1) + 1;
                    }
            }
        }

        TIMER2_ICR_R = TIMER_ICR_TATOCINT;              // clear interrupt flag
        GPIO_PORTB_DATA_R ^= OUT_MASK;                  // Trigger/toggle output.

        time2++; //increment the data pointer for global counter

        //Sampling the data 104 times leads to the end of data
        if (time2 == 105)
        {
            TIMER2_IMR_R = 0;                               // turn-off timer interrupts
            NVIC_EN0_R |= 0 << (INT_TIMER2A-16);            // turn-off interrupt 37 (TIMER1A)
            TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off timer
            GPIO_PORTA_ICR_R |= IN_MASK;                    // Clear the interrupt
            GPIO_PORTA_IM_R |= IN_MASK;                     // Turn on interrupt when triggered
            GPIO_PORTB_DATA_R = 0;                          // Reset the data
            time2 = 0;                                      //reset the time counter
            time_zero_position = 0;                         //reset the first 0 position
            //putsUart0("\r\n");                            //Debug line
            //Only output when told to:
            if (decode_ON)
            {
                putUartValue(0);                                //put Address Value on Uart (0 id for Address)
                putUartValue(1);                                //put Data Value on Uart (1 id for Data)
                decode_ON = 0;
            }
            return;
        }

        TIMER2_TAILR_R = timeselect(time2);              // This value corresponds calculated by function below
}

//This function converts the name to corresponding address:
int name_to_address(char* inName)
{
       char input_name[12];
       int i = 0;
       //Initialize to all nulls
       for (i = 0; i < 12; i++)
       {
           input_name[i] = '\0';
       }
       i = 0;
       while ((inName[i] != '\0') && (i < 12))
       {
           input_name[i] = inName[i];
           i++;
       }
       if (input_name[0] == '\0')
       {
           putsUart0("Button name has to be a string.\r\n");
           return -1;
       }
       else
       {
           i = 0;
           //Only let 11 char Button names:
           input_name[11] = '\0';
           uint32_t checker = 0;
           int chk = -2;
           for (i=0; i < max_Eeprom_addr; i+=4)
           {
               //For each valid block check the name------------->
               if ((readEeprom(i+3) >> 31) == 1)
               {
                   //check the first block:
                   checker = (input_name[0]<<24) + (input_name[1]<<16) + (input_name[2]<<8) + input_name[3];
                   if (checker == readEeprom(i))
                   {
                       //check second block:
                       checker = (input_name[4]<<24) + (input_name[5]<<16) + (input_name[6]<<8) + input_name[7];
                       if (checker == readEeprom(i+1))
                       {
                           //check third block:
                           checker = (input_name[8]<<24) + (input_name[9]<<16) + (input_name[10]<<8) + input_name[11];
                           if (checker == readEeprom(i+2))
                           {
                               chk = i; ///store the address;
                               i = max_Eeprom_addr; //break out of loop
                           }
                       }
                   }
               }
           }
           return chk;
       }
}


//Alerts:
void play_Good_alert()
{
    if (good_alert)
    {
        playNote(440); //A
        waitMicrosecond(250000); //0.25ms
        playNote(660); //E
        waitMicrosecond(500000); //0.5ms
        playNote(0); //sound off
    }
}

void play_Bad_alert()
{
    if (bad_alert)
    {
        playNote(588); //D
        waitMicrosecond(500000); //0.5ms
        playNote(415); //G#
        waitMicrosecond(500000); //0.5ms
        playNote(0); //sound off
    }
}

void list_rules()
{
    int i = 0;
    int count = 0;
    while (i<8)
    {
        //Rule array will contain -1 if no address is associated with that rule
        if (rule_arr[i] != -1)
        {
            //Increase no. of rules counted
            count++;
            //Now output appropriate rule with the command NAME:
            putsUart0("Rule#");
            putint8Uart0(i+1);
            putsUart0(") ");
            int j = 0;
            int k = rule_arr[i];
            char outstr[12];
            for (j = 0; j < 12 ; j++)
            {
                if ((j%4 == 0) && (j!=0)) k++;
                outstr[j] = (readEeprom(k) >> (8*(3-(j%4)))) & 0xFF;
            }
            putsUart0(outstr);
            putsUart0(" => ");
            ruleDescription(i);
            putsUart0("\r\n");
        }
        i++;
    }
    if (count == 0) putsUart0("No rules implemented.\r\n");
}

//Initialize/Restore rules array at startup:
void initRules()
{
    //first clear the rules array:
    clearRules();
    int i = 0;
    for (i = 0; i < max_Eeprom_addr; i = i+4)
    {
        uint32_t val = readEeprom(i + 3);
        //Look at the rules for this address:
        uint8_t rules_for_val = (val>>16) & 0xFF;
        if (rules_for_val == 1) rule_arr[0] = i;
        else if (rules_for_val == 2) rule_arr[1] = i;
        else if (rules_for_val == 4) rule_arr[2] = i;
        else if (rules_for_val == 8) rule_arr[3] = i;
        else if (rules_for_val == 16) rule_arr[4] = i;
        else if (rules_for_val == 32) rule_arr[5] = i;
        else if (rules_for_val == 64) rule_arr[6] = i;
        else if (rules_for_val == 128) rule_arr[7] = i;
    }
}

//Function for playing rules:
void playRule(int rule_no)
{
    if (rule_no == 0)
    {
        //Green LED ON:
        GREEN_LED = 1;
        waitMicrosecond(500000); //0.5 second
        GREEN_LED = 0;
    }
    if (rule_no == 1)
    {
        //Red LED ON:
        RED_LED = 1;
        waitMicrosecond(500000); //0.5 second
        RED_LED = 0;
    }
    if (rule_no == 2)
    {
        //UART PRINT:
        putsUart0("Rule check fine.\r\n");
    }
    if (rule_no == 3)
    {
        //Alert Signal 1
        /*playNote(440); //A
        waitMicrosecond(250000); //0.25ms
        playNote(587); //D
        waitMicrosecond(250000); //0.25ms
        playNote(554); //C#
        waitMicrosecond(250000); //0.25ms
        playNote(440); //A
        waitMicrosecond(250000); //0.25ms
        playNote(0); //sound off*/
        playNote(440); //A
        waitMicrosecond(250000); //0.25ms
        playNote(494); //B
        waitMicrosecond(250000); //0.25ms
        playNote(523); //C
        waitMicrosecond(250000); //0.25ms
        playNote(587); //D
        waitMicrosecond(250000); //0.25ms
        playNote(494); //B
        waitMicrosecond(500000); //0.5ms
        playNote(392); //G
        waitMicrosecond(250000); //0.25ms
        playNote(440); //A
        waitMicrosecond(500000); //0.5ms
        playNote(0);

    }
    if (rule_no == 4)
    {
        //Alert Signal 2
        playNote(440); //A
        waitMicrosecond(500000); //0.5ms
        playNote(784); //G
        waitMicrosecond(500000); //0.5ms
        playNote(587); //D
        waitMicrosecond(500000); //0.5ms
        playNote(660); //E
        waitMicrosecond(500000); //0.5ms
        playNote(0); //sound off

    }
    if (rule_no == 5)
    {
        //Green LED blink 3 times
        GREEN_LED = 1;
        waitMicrosecond(250000); //0.25 second
        GREEN_LED = 0;
        waitMicrosecond(250000);
        GREEN_LED = 1;
        waitMicrosecond(250000); //0.25 second
        GREEN_LED = 0;
        waitMicrosecond(250000);
        GREEN_LED = 1;
        waitMicrosecond(250000); //0.25 second
        GREEN_LED = 0;
    }
    if (rule_no == 6)
    {
        //Green LED then Red LED
        GREEN_LED = 1;
        waitMicrosecond(250000); //0.25 second
        GREEN_LED = 0;
        waitMicrosecond(250000);
        RED_LED = 1;
        waitMicrosecond(500000); //0.5 second
        RED_LED = 0;
    }
    if (rule_no == 7)
    {
        //Alert Sound 3
        playNote(440); //A
        waitMicrosecond(500000); //0.5ms
        playNote(660); //E
        waitMicrosecond(250000); //0.25ms
        playNote(784); //G
        waitMicrosecond(500000); //0.5ms
        playNote(660); //E
        waitMicrosecond(250000); //0.25ms
        playNote(440); //A
        waitMicrosecond(500000); //0.5ms
        playNote(660); //E
        waitMicrosecond(500000); //0.5ms
        playNote(880); //A
        waitMicrosecond(500000); //0.5ms
        playNote(0); //sound off
    }
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)

{
    //Initialize hardware
    initHw();

    //Initialize Uart0 with default baud rate of 115200 Hz.
    initUart0();

    //Initialize EEPROM:
    initEeprom();

    //Initialize Speaker:
    initSpeaker();

    //Initialize the Rules array:
    initRules();

    //Device ON sequence:
    GREEN_LED = 1;
    waitMicrosecond(1000000); //1 second
    GREEN_LED = 0;

    playNote(440); //A
    waitMicrosecond(250000); //0.25ms
    playNote(660); //E
    waitMicrosecond(500000); //0.5ms
    playNote(0); //sound off

    putsUart0("DEVICE ON AND READY!!!\r\n");

    //Define variable:
    USER_DATA data;

    //Get commands from user:
    while (true)
    {
        //Read the data:
        getsUart0(&data);

        //Output the read data on terminal:
        putsUart0(data.buffer);

        //start from new line:
        putsUart0("\r\n");

        //Parse this data:
        parseFields(&data);

        //1) Decode command:
        if (isCommand(&data,"decode",0))
        {
            decode_ON = 1; //Turn on the decode flag
            putsUart0("Please press a button on the remote.\r\n");
        }

        //2) Learn command:
        else if (isCommand(&data,"learn",1))
        {
            int chk = name_to_address(getFieldString(&data,1));
            //This means input name was not string
            if(chk == -1)
            {
                putsUart0("Command name entered has to be a string. \r\n");
                play_Bad_alert();
            }
            //This means command name was found:
            else if(chk >= 0)
            {
                putsUart0("Command name already exists. \r\n");
                play_Bad_alert();
            }
            //Only decode once the name has been approved:
            else
            {
                int i = 0;
                decode_ON = 1; //Turn on the decode flag
                putsUart0("Push the button to be learned for the name:\r\n");
                //wait for the decode to finish:
                while (decode_ON);
                //This variable stores Eeprom address:
                int Eeprom_addr = -1;
                //search for an empty/invalid block (which has to be checked for every 4th block):
                for (i=0; i < max_Eeprom_addr; i+=4)
                {
                    //when the empty block is found (valid bit = 0)------------->
                    if ((readEeprom(i+3) >> 31) == 0)
                    {
                        Eeprom_addr = i; //Use this value as a flag
                        i = max_Eeprom_addr; //break out of for loop
                    }
                }

                //check if name and/or address and data values are good:
                //check if the codes have already been stored:
                for (i=0; i < max_Eeprom_addr; i+=4)
                {
                    //when the block is valid check
                    if ((readEeprom(i+3) >> 31) == 1)
                    {
                        if ((((valueA >> 8) << 8) + (valueD >> 8)) == (readEeprom(i+3) & 0xFFFF))
                        {
                            Eeprom_addr = -2; //Use this value as a flag
                            i = max_Eeprom_addr; //break out of for loop
                        }
                    }
                }

                //check if both address and values are 0
                //when data bits inconsistent this happens:
                if (valueA == 0 && valueD == 0) Eeprom_addr = -3;

                if (Eeprom_addr == -1)
                {
                    putsUart0 ("No empty block to put new command in.\r\n");
                    putsUart0 ("Use erase command to delete some blocks.\r\n");
                    play_Bad_alert();
                }
                else if (Eeprom_addr == -2)
                {
                    putsUart0 ("The command you entered has already been stored.\r\n");
                    play_Bad_alert();
                }
                else if (Eeprom_addr == -3)
                {
                    putsUart0 ("Could not learn command.\r\n");
                    play_Bad_alert();
                }
                //If everything is clear write the block:
                else
                {
                    i = 0;
                    char input_name[12];
                    //Initialize to all nulls
                    for (i = 0; i < 12; i++)
                    {
                        input_name[i] = '\0';
                    }
                    i = 0;
                    while ((getFieldString(&data,1)[i] != '\0') && (i < 12))
                    {
                        input_name[i] = getFieldString(&data,1)[i];
                        i++;
                    }
                    //Convert the string value into uint32_t to write to Eeprom:
                    uint32_t Eeprom_line_in = 0;
                    for (i=0; i<12 ; i++)
                    {
                        Eeprom_line_in = (Eeprom_line_in << 8) + input_name[i];
                        //write every 4th run of loop (4 runs = 8 * 4 bits = 32 bits):
                        if ((i+1)%4 == 0)
                        {
                            writeEeprom(Eeprom_addr, Eeprom_line_in);
                            Eeprom_addr++;
                            Eeprom_line_in = 0; //reset this value
                        }
                    }

                    //Now add the address and data values:
                    //These values contain the values and the compliment bits
                    //Thus only need the first 8 bits which are the actual values:
                    valueA = valueA >> 8;
                    valueD = valueD >> 8;

                    //1 denotes the command is valid/not deleted (in 31st bit):
                    Eeprom_line_in = (1 << 31) + (valueA << 8) + valueD;
                    writeEeprom(Eeprom_addr, Eeprom_line_in);
                    Eeprom_addr++;

                    //Display message to state command learned
                    putsUart0("learned command: ");
                    putsUart0(input_name);
                    putsUart0("\r\n");
                    play_Good_alert();
                }
            }

        }

        //3) For list command:
        else if (isCommand(&data,"list",0))
        {
            //The condition for list rules:
            char* attrb = getFieldString(&data,1);
            if ((attrb[0] == 'r') && (attrb[1] == 'u') && (attrb[2] == 'l') && (attrb[3] == 'e') && (attrb[4] == 's'))
            {
                list_rules();
            }
            //Condition for correct syntax:
            else if (attrb[0] == '\0')
            {
                int i = 0;
                int j = 0;
                int k = 0;
                uint8_t count = 0; //No of commands counted
                char outstr[12];
                //Go through each command (each command takes 4 blocks)
                for (i = 0; i < max_Eeprom_addr; i = i + 4)
                {
                    //Check if the current command is valid on the 31st bit:
                    if ((readEeprom(i + 3) >> 31) == 1)
                    {
                        count++;
                        k = i;
                        //Read the name:
                        for (j = 0; j < 12 ; j++)
                        {
                            if ((j%4 == 0) && (j!=0)) k++;
                            outstr[j] = (readEeprom(k) >> (8*(3-(j%4)))) & 0xFF;
                        }
                        //Output the name:
                        putint8Uart0(count);
                        putsUart0(") ");
                        putsUart0(outstr);
                        putsUart0("\r\n");
                    }
                }
                if (count == 0) putsUart0("No commands stored.\r\n");
            }
            else
            {
                putsUart0("Invalid syntax.\r\n");
                play_Bad_alert();
            }
        }

        //4) Erase command:
        else if (isCommand(&data,"erase",1))
        {
            int chk = name_to_address(getFieldString(&data,1)) ;
            //This means found:
            if (chk >= 0)
            {
                //First check for rules:
                int rule_no = check_for_rule(chk);
                //If address/NAME has a rule delete it:
                if (rule_no >= 0)
                {
                    rule_arr[rule_no] = -1;
                    //delete the rule in appropriate Eeprom address:
                    uint32_t val = readEeprom(chk + 3);
                    val = val - (1<<(rule_no + 16));
                    writeEeprom(chk + 3, val);
                }
                //Write everything to be null:
                writeEeprom(chk,0);
                writeEeprom(chk + 1,0);
                writeEeprom(chk + 2,0);
                writeEeprom(chk + 3,0);
                putsUart0("Erased command: ");
                putsUart0(getFieldString(&data,1));
                putsUart0("\r\n");
                play_Good_alert();
            }
            //This means input name was not string
            else if(chk == -1)
            {
                putsUart0("Command name entered has to be a string. \r\n");
                play_Bad_alert();
            }
            //This means not found
            else if(chk == -2)
            {
                putsUart0("Command not found. \r\n");
                play_Bad_alert();
            }
        }

        //5) clear command:
        else if (isCommand(&data,"clear",0))
        {
            char ch;
            putsUart0("Erase all the values from Eeprom(y/n)? => ");
            ch = getcUart0();
            if (ch == 'y' || ch == 'Y')
            {
                //Initialize all values in Eeprom to 0:
                int c = 0;
                for (c= 0; c< max_Eeprom_addr; c++)
                {
                    writeEeprom(c,0);
                }
                //clear the rules array:
                clearRules();
                putsUart0("Cleared Eeprom.\r\n");
            }
            else
            {
                putsUart0("Did not clear Eeprom.\r\n");
            }
        }

        //6) Info Command:
        else if (isCommand(&data,"info",1))
        {
            int chk = name_to_address(getFieldString(&data,1)) ;
            //This means found:
            if (chk >= 0)
            {
                putsUart0("Command found => \r\n");
                putsUart0("Address: ");
                putint8Uart0(((readEeprom(chk + 3))>>8) & 0xFF);
                putsUart0(" Data: ");
                putint8Uart0((readEeprom(chk + 3)) & 0xFF);
                putsUart0("\r\n");
                play_Good_alert();
            }
            //This means input name was not string
            else if(chk == -1)
            {
                putsUart0("Command name entered has to be a string. \r\n");
                play_Bad_alert();
            }
            //This means not found
            else if(chk == -2)
            {
                putsUart0("Command not found. \r\n");
                play_Bad_alert();
            }
        }

        //7) Play Command:
        else if (isCommand(&data,"play",1))
        {
            int chk = name_to_address(getFieldString(&data,1)) ;
            //This means found:
            if (chk >= 0)
            {
                uint8_t Addr = ((readEeprom(chk + 3))>>8) & 0xFF;
                uint8_t Data = (readEeprom(chk + 3)) & 0xFF;
                putsUart0("Do you want decode ON(y/n)? => ");
                char ch = getcUart0();
                if (ch == 'y' || ch == 'Y')
                {
                    decode_ON = 1;
                    putsUart0("Decode turned ON.\r\n");
                }
                else
                {
                    putsUart0("Decode turned OFF.\r\n");
                }
                playComment(Addr, Data);
                //check for rules if any:
                int rules = check_for_rule(chk);
                if (rules >= 0) playRule(rules);
            }
            //This means input name was not string
            else if(chk == -1)
            {
                putsUart0("Command name entered has to be a string. \r\n");
                play_Bad_alert();
            }
            //This means not found
            else if(chk == -2)
            {
                putsUart0("Command not found. \r\n");
                play_Bad_alert();
            }
        }

        //8) Alert handler:
        else if (isCommand(&data,"alert",0))
        {
            char ch;
            putsUart0("Do you want GOOD alert on(y/n)? => ");
            ch = getcUart0();
            if (ch == 'y' || ch == 'Y')
            {
                good_alert = 1;
                putsUart0("Good alert turned ON.\r\n");
            }
            else
            {
                good_alert = 0;
                putsUart0("Good alert turned OFF.\r\n");
            }
            putsUart0("Do you want BAD alert on(y/n)? => ");
            ch = getcUart0();
            if (ch == 'y' || ch == 'Y')
            {
                bad_alert = 1;
                putsUart0("Bad alert turned ON.\r\n");
            }
            else
            {
                bad_alert = 0;
                putsUart0("Bad alert turned OFF.\r\n");
            }
        }

        //9) Help command:
        else if (isCommand(&data,"help",0))
        {
            help();
        }

        //10) Rule Command:
        else if (isCommand(&data,"rule",1))
        {
            //Get address for input string:
            int address = name_to_address(getFieldString(&data,1));
            //If NAME is found:
            if (address >= 0)
            {
                char* Code = getFieldString(&data,2);
                //if no second attribute/code provided:
                //It is deletion of Rule associated with NAME:
                if (Code[0] == '\0')
                {
                    //Check if this NAME has already a rule:
                    int rule_no = check_for_rule(address);
                    //If address/NAME has a rule delete it:
                    if (rule_no >= 0)
                    {
                        rule_arr[rule_no] = -1;
                        //delete the rule in appropriate Eeprom address:
                        uint32_t val = readEeprom(address + 3);
                        val = val - (1<<(rule_no + 16));
                        writeEeprom(address + 3, val);
                        putsUart0("Rule deletion successful!\r\n");
                        play_Good_alert();
                    }
                    else
                    {
                        putsUart0("No rules associated with this entry.\r\n");
                        play_Bad_alert();
                    }
                }
                else
                {
                    //if second attribute/code provided:
                    //It means associate NAME with Rule Code:
                    int rule_no = ruleSetbyCode(address, Code);
                    //Now write this rule into Eeprom for this address:
                    if (rule_no >= 0)
                    {
                        //set the rule in appropriate Eeprom address:
                        uint32_t val = readEeprom(address + 3);
                        val = val + (1<<(rule_no + 16));
                        writeEeprom(address + 3, val);
                    }
                }
            }
            //This means input name was not string
            else if(address == -1)
            {
                putsUart0("Command name entered has to be a string. \r\n");
                play_Bad_alert();
            }
            //This means not found
            else if(address == -2)
            {
                putsUart0("Command not found. \r\n");
                play_Bad_alert();
            }
        }

        else
        {
            putsUart0("Unrecognized command.\r\n");
            play_Bad_alert();
        }

    }
}
