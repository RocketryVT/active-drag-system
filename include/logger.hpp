#pragma once
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <cstring>

enum FileTypes { TXT, CSV };

class Logger {

    private:
        std::fstream txt_file;
        std::fstream csv_file;
        std::string txt_filename;
        std::string csv_filename;
        bool txt_file_open;
        bool csv_file_open;
        
        time_t t;
        tm* now;

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
         * @brief Open a .txt and .csv log file with the given filename.
         * 
         * @param _filename Name of file to open. No Suffix. (e.g "filename". NOT "filename.txt")
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
         * @param type Specifies the file to write to (TXT or CSV)
         * @return true Data Successfully Logged
         * @return false Data Logging Unsuccessful
         */
        bool log(std::string data, FileTypes type);

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
         * @param type Specifies the file to print (TXT or CSV)
         * @return true Successful Print
         * @return false Unsuccessful Print
         */
        bool printLog(FileTypes type);
};