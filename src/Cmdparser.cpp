// #include "headers.h"
#include "gp25workcell/CmdParser.h"

CommandParser TCPparser;

char *CommandParser::command_ptr,
     *CommandParser::string_arg;
char CommandParser::command_letter;
int CommandParser::codenum;
double CommandParser::param[30];

void CommandParser::parse(char *p)
{    
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
        case 'm':
        case 'R':
        case 'r':
        case 'C':
        case 'c':
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

    bool decSeen = false;
    int cnt = 0;
    int i = 10;
    param[cnt] = 0;
    float test;
    bool negParam = false;
    //store the next values as parameters untill *p = null
    while(*p != '~')
    {
        //skip spaces and inc cnt and reset for next param
        if(*p == ' ')
        {
            if(negParam == true)
            {
                param[cnt] = -param[cnt];
                negParam = false;
            }
            ++cnt;
            i = 10;
            decSeen = false;
            negParam = false;
            param[cnt] = 0;
            while(*p == ' ') ++p;
        }        

        if(*p == '-')
        {
            negParam = true;
            *p++;
        }
        if(*p == '.')
        {
            decSeen = true;
            *p++;
        }            
        if(!decSeen)
        {
            param[cnt] *= 10, param[cnt] += *p - '0';
        }
        else if(decSeen)
        {
            param[cnt] = param[cnt] + (double(*p - '0') / (double) i);
            i = i * 10;
        }

        ++p;
    }

};