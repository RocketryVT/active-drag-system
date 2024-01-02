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
        
        /**
         * @brief 
         * 
         */
        static Logger& Get();

        /**
         * @brief Open a Log File for writing
         * 
         * @param _filename Name of file to open
         * @return true Successful Open
         * @return false Unsuccessful Open
         */
        bool openLog(std::string _filename);

        /**
         * @brief Close the Log File
         * 
         */
        void closeLog();

        /**
         * @brief Write data to Log file
         * 
         * @param data Data to log
         * @return true Data Successfully Logged
         * @return false Data Logging Unsuccessful
         */
        bool log(std::string data);

        /**
         * @brief Write error data to Log file
         * 
         * @param data Error Data to log
         * @return true Data Successfully Logged
         * @return false Data Logging Unsuccessful
         */
        bool logErr(std::string data);

        /**
         * @brief Print Log Data to Terminal
         * 
         * @return true Successful Print
         * @return false Unsuccessful Print
         */
        bool printLog();
};