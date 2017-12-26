#ifndef CUSTOM_LOGGER_H
#define CUSTOM_LOGGER_H

#include <fstream>
#include <iostream>
#include <cstdarg>
#include <string>

using namespace std;

#define LOGGER logger::GetLogger()

class logger
{
public:
    void Log(const std::string& message);
    void Log(const char * format, ...);
    logger& operator << (const string& message);
    static logger* GetLogger();
    std::string CurrentDateTime();
    static std::string SetFileName();
    static std::string FileName;

private:
    logger(){}; // Default constructor
    logger(const logger&){};             // Copy constructor
    logger& operator=(const logger&){ return *this; };  // Assignment operator for the logger class
    static logger* m_pThis; // Pointer for the logger class
    static ofstream m_Logfile; // Log file stream object.

};

#endif
