#ifndef PARSER_H
#define PARSER_H

#include <ros/ros.h>

#include "std_msgs/String.h"

#include <iostream>
#include "string"
#include <bits/stdc++.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define WITHIN(V,L,H) ((V) >= (L) && V <= (H))
#define NUMERIC(a) WITHIN (a, '0', '9')
#define DECIMAL(a) (NUMERIC(a) || a == '.')

class CommandParser{
    private:
        static char *value_ptr;

    public:
        static void parse(char * p);
        static char *command_ptr, *string_arg;
        static char command_letter;
        static int codenum;
};

extern CommandParser TCPparser;

#endif

