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

#ifndef RULE_AND_HELP_H_
#define RULE_AND_HELP_H_

//Global Variable for rules array:
int rule_arr[8];

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void clearRules();
int ruleSet(uint8_t addr, int rule_num);
int ruleSetbyCode (uint8_t addr, char* code);
void help();
int check_for_rule(uint8_t address);
void ruleDescription (int rule_no);
#endif
