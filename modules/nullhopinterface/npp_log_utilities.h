#ifndef __NPP_LOG_UTILITIES_H__
#define __NPP_LOG_UTILITIES_H__


#include "string.h"
#include <iostream>

class log_utilities {
   public:
      //Functions to guarantee this class is a singleton
       static log_utilities& getInstance() {
         static log_utilities instance; // Guaranteed to be destroyed.
         return (instance);               // Instantiated on first use.

      }
       log_utilities(log_utilities const&) = delete;
       void operator=(log_utilities const&) = delete;

      static  void error(std::string message,...);

      static  void none(std::string message,...);

      static  void low(std::string message,...);

      static  void medium(std::string message,...);

      static  void high(std::string message,...);

      static  void full(std::string message,...);

      static  void debug(std::string message,...);

   private:
      FILE *log_file;
      static const int MAX_SIZE_LINE = 2048; //apparently no way to do it without this parameter (see cpp)
      log_utilities();
      static std::string time_as_string();
      static void print(char* message, int verbosity, bool error_stream = false);
};



#endif
