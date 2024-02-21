#include "../include/logger.hpp"

// Private----------------------------------------------------------------------
std::string Logger::getDate() {

    t = time(nullptr);
    now = localtime(&t);
    return "(" + days[now->tm_wday] + " " + months[now->tm_mon] + " " 
        + std::to_string(now->tm_mday) + " " + std::to_string(now->tm_year + 1900) + ")";
}

std::string Logger::getTime() {

    t = time(nullptr);
    now = localtime(&t);
    std::string hour = std::to_string(now->tm_hour);
    std::string min = std::to_string(now->tm_min);
    std::string sec = std::to_string(now->tm_sec);
    //string hour = "0" + to_string(now->tm_hour);

    if (now->tm_hour < 10) {
        hour = "0" + std::to_string(now->tm_hour);
    }

    if (now->tm_min < 10) {
        min = "0" + std::to_string(now->tm_min);
    }

    if (now->tm_sec < 10) {
        sec = "0" + std::to_string(now->tm_sec);
    }

    return hour + ":" + min + 
        ":" + sec;
}


// Public----------------------------------------------------------------------
Logger& Logger::Get() {

    static Logger loggerSingleton;
    return loggerSingleton;
}

//Logger Logger::loggerSingleton;


bool Logger::openLog(std::string _filename) {

    txt_filename = _filename + ".txt";
    csv_filename = _filename + ".csv";

    // Start each file
    txt_file_open = false;
    csv_file_open = false;
    txt_file.open(txt_filename, std::ios::in | std::ios::out | std::ios::app);
    csv_file.open(csv_filename, std::ios::in | std::ios::out | std::ios::app);
    if (!txt_file || !csv_file) {
        return false;
    }
    txt_file_open = true;
    csv_file_open = true;
    std::string date = getDate();
    std::string timestamp = getTime();
    txt_file << timestamp << infoTag << "Log Start---- " << date << std::endl;
    
    return true;
}


void Logger::closeLog() {

    std::string timestamp = getTime();
    txt_file << timestamp << infoTag << "Log End----\n\n";

    txt_file.close();
    csv_file.close();
    txt_file_open = false;
    csv_file_open = false;
}


bool Logger::log(std::string data, FileTypes type) {

    if (type == TXT) {

        if (!txt_file) {
            return false;
        }

        if (!txt_file_open) {
            return false;
        }

        std::string timestamp = getTime();
        txt_file << timestamp << infoTag << data << std::endl;
    }

    else if (type == CSV) {

        if (!csv_file) {
            return false;
        }

        if (!csv_file_open) {
            return false;
        }

        csv_file << data << std::endl;
        
    }

    return true;
}




bool Logger::logErr(std::string data) {

    if (!txt_file || !csv_file) {
        return false;
    }

    if (!txt_file_open || csv_file_open) {
        return false;
    }

    std::string timestamp = getTime();
    txt_file << timestamp << errorTag << data << std::endl;
    return true;
}


bool Logger::printLog(FileTypes type) {

    if (type == TXT) {

        if (txt_file.is_open()) {
            std::cout << "Log still open. Please close Log." << std::endl;
            return false;
        }

        txt_file.open(txt_filename, std::ios::in);

        if (!txt_file.is_open()) {
            return false;
        }

        std::string line;
        while(getline(txt_file, line)) {
            std::cout << line << std::endl;
        }

        txt_file.close();
    }

    else if (type == CSV) {

        if (csv_file.is_open()) {
            std::cout << "Log still open. Please close Log." << std::endl;
            return false;
        }

        csv_file.open(csv_filename, std::ios::in);

        if (!csv_file.is_open()) {
            return false;
        }

        std::string line;
        while(getline(csv_file, line)) {
            std::cout << line << std::endl;
        }

        csv_file.close();
    }
    
    return true;
}
