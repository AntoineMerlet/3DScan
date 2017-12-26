#include "logger.h"
#include <iostream>
#include <string>
#include <time.h>

// Initiliazing static members of the class
logger* logger::m_pThis = NULL;
ofstream logger::m_Logfile;
std::string logger::FileName = SetFileName();

/// @author: Marcio Rockenbach
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Funtion to set the name of the logger file
/// @return string that is the name of the file to be created
std::string logger::SetFileName()
{
    string name = "Log_";
    name.append(m_pThis->CurrentDateTime());
    name.append(".txt");
    return name;
};

/// @author: Marcio Rockenbach
/// @date: 23-12-2017
/// @version 1.0
///
/// @brief Funtion to create the instance of logger class.
/// @return pointer to a object of the logger class.
logger* logger::GetLogger()
{
    if (m_pThis == NULL){
        m_pThis = new logger();
        m_Logfile.open(logger::FileName.c_str(), ios::out | ios::app);
    }
    return m_pThis;
}

/// @author: Marcio Rockenbach
/// @date: 25-12-2017
/// @version 1.0
///
/// @brief Logs a message with a specific format
/// @param message to be included in the log; specification of the format of the message
void logger::Log(const char * format, ...)
{
    char* message = NULL;
    int length = 0;
    va_list args;
    va_start(args, format);
    //  Return the number of characters in the string referenced the list of arguments.
    // _vscprintf doesn't count terminating '\0' (that's why +1)
    length = _vscprintf(format, args) + 1;
    message = new char[length];
    vsprintf_s(message, length, format, args);
    //vsprintf(sMessage, format, args);
    m_Logfile << logger::CurrentDateTime() << ":\t";
    m_Logfile << message << "\n";
    va_end(args);
    delete [] message;
}

/// @author: Marcio Rockenbach
/// @date: 23-12-2017
/// @version 1.0
///
/// @brief Logs a message
/// @param message to be included in the log
void logger::Log(const string& message)
{
    m_Logfile <<  logger::CurrentDateTime() << ":\t";
    m_Logfile << message << "\n";
}

/// @author: Marcio Rockenbach
/// @date: 23-12-2017
/// @version 1.0
///
/// @brief Overloads the << operator to log a message
/// @param string for the message to be logged.
logger& logger::operator<<(const string& message)
{
    m_Logfile << "\n" << logger::CurrentDateTime() << ":\t";
    m_Logfile << message << "\n";
    return *this;
}

/// @author: Marcio Rockenbach
/// @date: 25-12-2017
/// @version 1.0
///
/// @brief Returns current date and time as a string
std::string logger::CurrentDateTime()
{
    time_t     now = time(NULL);
    struct tm  tstruct;
    char       buf[80];
    localtime_s(&tstruct, &now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    buf[10] = '_';
    buf[13] = '_';
    buf[16] = '_';
    return buf;
}
