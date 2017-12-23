#include "logger.h"
#include "utilities.h"

// Initiliazing static members of the class
const string logger::m_sFileName = "Log.txt";
logger* logger::m_pThis = NULL;
ofstream logger::m_Logfile;

/// @author: Marcio Rockenbach
/// @date: 23-12-2017
/// @version 1.0
///
/// @brief Funtion to create the instance of logger class.
/// @param string for the message to be logged.
/// @return pointer to a object of the logger class.
logger* logger::GetLogger()
{
    if (m_pThis == NULL){
        m_pThis = new logger();
        m_Logfile.open(m_sFileName.c_str(), ios::out | ios::app);
    }
    return m_pThis;
}

/// @author: Marcio Rockenbach
/// @date: 23-12-2017
/// @version 1.0
///
/// @brief Logs a message
/// @param message to be included in the log
void logger::Log(const string& message)
{
    m_Logfile <<  Util::CurrentDateTime() << ":\t";
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
    m_Logfile << "\n" << Util::CurrentDateTime() << ":\t";
    m_Logfile << message << "\n";
    return *this;
}
