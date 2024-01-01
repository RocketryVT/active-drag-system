#pragma once
#include <iostream>
#include <fstream>
#include <ctime>

class Logger {

    private:
        std::fstream file;
        std::string filename;
        time_t t;
        tm* now;
        bool file_open;

        std::string infoTag = "-> [INFO]: ";
        std::string errorTag = "-> [ERROR]: ";
        std::string months[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sept", "Oct", "Nov", "Dec"};
        std::string days[7] = {"Sun", "Mon", "Tues", "Wed", "Thur", "Frid", "Sat"};

        /**
         * @brief Create formatted current-date tag 
         * 
         * @return string 
         */
        std::string getDate();

        /**
         * @brief Create formatted current-time tag 
         * 
         * @return string 
         */
        std::string getTime();
    

    public:
        
        static Logger& Get();

        bool openLog(std::string _filename);

        void closeLog();

        bool log(std::string data);

        bool logErr(std::string data);

        bool printLog();
};