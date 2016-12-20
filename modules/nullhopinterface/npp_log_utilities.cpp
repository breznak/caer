#ifndef __NPP_LOG_UTILITIES_CPP__
#define __NPP_LOG_UTILITIES_CPP__

#define ENABLE_LOG
#define VERBOSITY_NONE

#include "npp_log_utilities.h"
#include "string.h"
#include <iostream>
#include <stdexcept>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#ifdef ENABLE_LOG




log_utilities::log_utilities() {
#ifndef LOGFILE
   log_file = fopen("npp_run.log", "w");
#else
   log_file = fopen(LOGFILE, "w");
#endif
}
//All following functions share some code. We didn't manage to find a simpler way to implement it
//If it exists, would be nice being able to forward the ellipses to a common formatter
 void log_utilities::none(std::string message,...) {
   char print_message[MAX_SIZE_LINE];
   va_list vl;
   va_start(vl, message);
   vsprintf(print_message,message.c_str(),vl);
   va_end(vl);
   print(print_message, 0);
}

 void log_utilities::low(std::string message,...) {
#if defined (VERBOSITY_LOW) || defined (VERBOSITY_MEDIUM) ||  defined (VERBOSITY_HIGH) ||  defined (VERBOSITY_FULL) || defined (VERBOSITY_DEBUG)
   char print_message[MAX_SIZE_LINE];
   va_list vl;
   va_start(vl, message);
   vsprintf(print_message,message.c_str(),vl);
   va_end(vl);
   print(print_message, 1);
#endif
}

 void log_utilities::medium(std::string message,...) {
#if defined (VERBOSITY_MEDIUM) ||  defined (VERBOSITY_HIGH) ||  defined (VERBOSITY_FULL) || defined (VERBOSITY_DEBUG)
   char print_message[MAX_SIZE_LINE];
   va_list vl;
   va_start(vl, message);
   vsprintf(print_message,message.c_str(),vl);
   va_end(vl);
   print(print_message, 2);
#endif
}

 void log_utilities::high(std::string message,...) {
#if defined (VERBOSITY_HIGH) ||  defined (VERBOSITY_FULL) ||  defined (VERBOSITY_DEBUG)
   char print_message[MAX_SIZE_LINE];
   va_list vl;
   va_start(vl, message);
   vsprintf(print_message,message.c_str(),vl);
   va_end(vl);
   print(print_message, 3);
#endif
}

 void log_utilities::full(std::string message,...) {
#if defined (VERBOSITY_FULL) ||  defined (VERBOSITY_DEBUG)
   char print_message[MAX_SIZE_LINE];
   va_list vl;
   va_start(vl, message);
   vsprintf(print_message,message.c_str(),vl);
   va_end(vl);
   print(print_message, 4);
#endif
}

 void log_utilities::debug(std::string message,...) {
#ifdef VERBOSITY_DEBUG
   char print_message[MAX_SIZE_LINE];
   va_list vl;
   va_start(vl, message);
   vsprintf(print_message,message.c_str(),vl);
   va_end(vl);
   print(print_message, 5);
#endif
}
#else

//Empty implementation if log is disabled
 void log_utilities::none(std::string message, ...) {
}

 void log_utilities::low(std::string message, ...) {
}

 void log_utilities::medium(std::string message, ...) {
}

 void log_utilities::high(std::string message, ...) {
}

 void log_utilities::full(std::string message, ...) {
}

 void log_utilities::debug(std::string message, ...) {
}

#endif
//Error messages are printed in any case
 void log_utilities::error(std::string message, ...) {
   char print_message[MAX_SIZE_LINE];
   va_list vl;
   va_start(vl, message);
   vsprintf(print_message, message.c_str(), vl);
   va_end(vl);
   print(print_message, 5, true);

}


std::string log_utilities::time_as_string() {
   time_t rawtime;
   struct tm * timeinfo;
   char buffer[80];

   time(&rawtime);
   timeinfo = localtime(&rawtime);

   strftime(buffer, 80, "%d-%m-%Y - %I:%M:%S", timeinfo);
   std::string str(buffer);
   return (str);
}

void log_utilities::print(char* message, int verbosity, bool error_stream) {
   std::string message_as_string(message);
   std::string time = time_as_string();
   std::string out_message;
   out_message.append(time);
   out_message.append(" - ");
   //out_message.append(verbosity);
   // out_message.append(" - ");
   out_message.append(message_as_string);
   out_message.append("\n");
   std::cout << out_message;
#ifdef FILE_LOG
   fprintf(log_file,out_message.c_str());
#endif

   if (error_stream == true) {
      std::cerr << out_message;
   }

}


#endif
