// #include "headers.h"
#include "gp25workcell/CmdParser.h"




CommandParser TCPparser;

char *CommandParser::command_ptr,
     *CommandParser::string_arg;
char CommandParser::command_letter;
int CommandParser::codenum;

void CommandParser::parse(char *p){
    
    //skip spaces
    while(*p == ' ') ++p;

    // *p now points to the current command
    command_ptr = p;

    // Get the command letter
    const char letter = *p++;

    //return if letter is not assigned letter
    switch (letter)
    {
        case 'M':
        case 'R':
            break;
        default:
            return;
    }

    //skip spaces
    while(*p == ' ') ++p;

    // *p now points to numberic code
    // return if there is no numeric code
    if(!NUMERIC(*p)) return;

    // Save the command letter
    command_letter = letter;

    // Get the code number - integer only
    codenum = 0;
    do{
        codenum *= 10, codenum += *p++ - '0';
    }while(NUMERIC(*p));

    //skip spaces
    while(*p == ' ') ++p;

};