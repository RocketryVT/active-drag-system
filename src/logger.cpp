#include "logger.hpp"

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

    filename = _filename;
    
    if (file_open) {
        return false;
    }
    
    file.open(filename, std::ios::in | std::ios::out | std::ios::app);

    if (!file) {
        return false;
    }

    file_open = true;
    std::string date = getDate();
    std::string timestamp = getTime();
    file << timestamp << infoTag << "Log Start---- " << date << std::endl;

    return true;
}


void Logger::closeLog() {

    std::string timestamp = getTime();
    file << timestamp << infoTag << "Log End----\n\n";

    file.close();
    file_open = false;
}


bool Logger::log(std::string data) {

    if (!file) {
        return false;
    }

    if (!file_open) {
        return false;
    }
    std::string timestamp = getTime();
    file << timestamp << infoTag << data << std::endl;
    return true;
}

bool Logger::logErr(std::string data) {

    if (!file) {
        return false;
    }

    if (!file_open) {
        return false;
    }

    std::string timestamp = getTime();
    file << timestamp << errorTag << data << std::endl;
    return true;
}


bool Logger::printLog() {

    if (file.is_open()) {
        std::cout << "Log still open. Please close Log." << std::endl;
        return false;
    }

    file.open(filename, std::ios::in);

    if (!file.is_open()) {
        return false;
    }

    std::string line;
    while(getline(file, line)) {
        std::cout << line << std::endl;
    }

    file.close();

    return true;
}
