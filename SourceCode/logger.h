#ifndef LOGGER_H
#define LOGGER_H

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
    logger& operator << (const string& message);
    static logger* GetLogger();

private:
    logger(){}; // Default constructor
    logger(const logger&){};             // Copy constructor
    logger& operator=(const logger&){ return *this; };  // Assignment operator for the logger class
    static const std::string m_sFileName; // File name of the generated log
    static logger* m_pThis; // Pointer for the logger class
    static ofstream m_Logfile; // Log file stream object.
};

#endif
