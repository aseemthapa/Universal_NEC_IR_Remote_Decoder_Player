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
#include "rule_and_help.h"
#include "uart.h"


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void clearRules()
{
    //Initialize all rules to -1 (no address holds rule)
    int i = 0;
    for (i = 0; i < 8 ; i++)
    {
        rule_arr[i] = -1;
    }
}

int ruleSet(uint8_t addr, int rule_num)
{
    //first check if Address already has a rule (Unsuccessful):
    if (check_for_rule(addr) >= 0) return -1;

    //Only set a rule if no other functionality has taken the rule (Successful).
    //Also return the rule_num: (0-7)
    if (rule_arr[rule_num] == -1)
    {
        rule_arr[rule_num] = addr;
        return rule_num;
    }

    //This means that the rule is not available (Unsuccessful):
    return -2;
}

void help()
{
    putsUart0("Functionalities granted: (select number associated)\r\n");
    putsUart0("0. decode\r\n");
    putsUart0("1. learn\r\n");
    putsUart0("2. list\r\n");
    putsUart0("3. erase\r\n");
    putsUart0("4. clear\r\n");
    putsUart0("5. info\r\n");
    putsUart0("6. play\r\n");
    putsUart0("7. alert\r\n");
    putsUart0("8. rule\r\n");
    bool ch = 1;
    char choice;
    while (ch)
    {
        choice = getcUart0();
        if (choice == '0')
        {
            putsUart0("Syntax: decode\r\n");
            putsUart0("Decodes any incoming IR signal.\r\n");
        }
        else if (choice == '1')
        {
            putsUart0("Syntax: learn NAME\r\n");
            putsUart0("Saves an input signal with NAME in memory\r\n");
        }
        else if (choice == '2')
        {
            putsUart0("Syntax: list\r\n");
            putsUart0("Lists all the NAMEs of signals saved in memory.\r\n");
            putsUart0("Syntax: list rules\r\n");
            putsUart0("Lists all the rules stored.\r\n");
        }
        else if (choice == '3')
        {
            putsUart0("Syntax: erase NAME\r\n");
            putsUart0("Erases command NAME from memory.\r\n");
        }
        else if (choice == '4')
        {
            putsUart0("Syntax: clear\r\n");
            putsUart0("Clears the entire memory.\r\n");
        }
        else if (choice == '5')
        {
            putsUart0("Syntax: info NAME\r\n");
            putsUart0("Gives data and address infos for stored command NAME.\r\n");
        }
        else if (choice == '6')
        {
            putsUart0("Syntax: play NAME\r\n");
            putsUart0("Playback for saved command NAME from IR LED.\r\n");
        }
        else if (choice == '7')
        {
            putsUart0("Syntax: alert\r\n");
            putsUart0("Handler for the Alert signals.\r\n");
        }
        else if (choice == '8')
        {
            putsUart0("Syntax: rule NAME\r\n");
            putsUart0("deletes rules implemented in command NAME\r\n");
            putsUart0("Syntax: rule NAME Code\r\n");
            putsUart0("set rule of Code to command NAME.\r\n");
            putsUart0("Available rules:\r\n");
            putsUart0("1. turn Green LED ON (Code: GLO)\r\n");
            putsUart0("2. turn Red LED ON (Code: RLO)\r\n");
            putsUart0("3. Output on Uart (Code: UTN)\r\n");
            putsUart0("4. Alert signal 1 (Code: AS1)\r\n");
            putsUart0("5. Alert signal 2 (Code: AS2)\r\n");
            putsUart0("6. Blink Green LED 3 times (Code: GL3)\r\n");
            putsUart0("7. Blink: GREEN then RED (Code: GTR)\r\n");
            putsUart0("8. Alert signal 3 (Code: AS3)\r\n");
        }
        else
        {
            putsUart0("Invalid entry. Help Menu Exited\r\n");
            ch = 0; //Exit
        }
    }
}

int check_for_rule(uint8_t address)
{
    int i = 0;
    for (i = 0; i< 8; i++)
    {
        //Return rule no if address has a rule:
        if (rule_arr[i] == address) return i;
    }
    return -1; //-1 when address does not have rule
}


int ruleSetbyCode (uint8_t addr, char* code)
{
    //Set rules according to code:
    int chk = 0;
    //Green LED ON:
    if ((code[0] == 'G') && (code[1] == 'L') && (code[2] == 'O'))
    {
        chk = ruleSet(addr,0);
    }
    //Red LED ON:
    else if ((code[0] == 'R') && (code[1] == 'L') && (code[2] == 'O'))
    {
        chk = ruleSet(addr,1);
    }
    //Print Name on UART:
    else if ((code[0] == 'U') && (code[1] == 'T') && (code[2] == 'N'))
    {
        chk = ruleSet(addr,2);
    }
    //Alert Sound 1:
    else if ((code[0] == 'A') && (code[1] == 'S') && (code[2] == '1'))
    {
        chk = ruleSet(addr,3);
    }
    //Alert Sound 2:
    else if ((code[0] == 'A') && (code[1] == 'S') && (code[2] == '2'))
    {
        chk = ruleSet(addr,4);
    }
    //Green LED Blink 3 times:
    else if ((code[0] == 'G') && (code[1] == 'L') && (code[2] == '3'))
    {
        chk = ruleSet(addr,5);
    }
    //Green LED then Red LED:
    else if ((code[0] == 'G') && (code[1] == 'T') && (code[2] == 'R'))
    {
        chk = ruleSet(addr,6);
    }
    //Alert Sound 3:
    else if ((code[0] == 'A') && (code[1] == 'S') && (code[2] == '3'))
    {
        chk = ruleSet(addr,7);
    }
    //Invalid CODE:
    else
    {
        putsUart0("Invalid Code entry.\r\n");
        return -1;
    }
    if (chk == -1) putsUart0("Input NAME already has a rule.\r\n");
    else if (chk >= 0) putsUart0("Rule Set Successful!\r\n");
    else if (chk == -2) putsUart0("Rule Not Available. Another Input has it.\r\n");
    return chk;
}

void ruleDescription(int rule_no)
{
    if (rule_no == 0) putsUart0("Green LED ON");
    else if (rule_no == 1) putsUart0("Red LED ON");
    else if (rule_no == 2) putsUart0("Print on UART");
    else if (rule_no == 3) putsUart0("Alert Signal 1");
    else if (rule_no == 4) putsUart0("Alert Signal 2");
    else if (rule_no == 5) putsUart0("Green LED blink 3 times");
    else if (rule_no == 6) putsUart0("Green LED ON then Red LED ON");
    else if (rule_no == 7) putsUart0("Alert Signal 3");
}
