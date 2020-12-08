//Name: Aseem Thapa
//ID: 1001543178

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "speaker.h"

// PortB mask for D0 (used for PWM output)
#define GPO_Mask 1
//#define Load_val 45454 //For note A

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initSpeaker()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;  //PWM1 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; //Port D clk
    _delay_cycles(3);

    // Configure pins

    //Port E:
    GPIO_PORTD_DIR_R |= GPO_Mask;                       // Port D0 is output
    GPIO_PORTD_DR2R_R |= GPO_Mask;                      // Drive strength 2mA
    GPIO_PORTD_DEN_R |= GPO_Mask;                       // Enable port D0 at start
    GPIO_PORTD_AFSEL_R |= GPO_Mask;                     // Select the auxillary function (PWM output in this case)
    GPIO_PORTD_PCTL_R &= GPIO_PCTL_PD0_M;               // disable GPIO
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_M1PWM0;          // enable PWM


    //Configuring the PWM1 Module:
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state

    // Port D0 == M1PWM0 which is in PWM1_Gen0_a

    PWM1_1_CTL_R = 0;                                // turn-off PWM1 generator 1
    PWM1_0_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                     // output 3 on PWM1, gen 1b, CMPA
    PWM1_0_LOAD_R = 45454;                           // Play note A

    PWM1_INVERT_R = PWM_INVERT_PWM0INV;              // Invert the output

    //For Inverted PWM signal, Duty Cycle = Compare/Load:

    PWM1_0_CMPA_R = 0;                               // No output
    PWM1_0_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 1
    PWM1_ENABLE_R = PWM_ENABLE_PWM0EN;               // enable outputs
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

void playNote (uint16_t freq)
{
    uint32_t Load_val = 40000000/(2 * freq); //This will be load value
    PWM1_0_LOAD_R = Load_val;
    PWM1_0_CMPA_R = Load_val >> 1; //50% duty cycle
}
