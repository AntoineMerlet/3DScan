#include "IO/logger.h"
#include <iostream>
#include <time.h>
#include <QObject>
#include <fstream>
#include <iostream>
#include <cstdarg>
#include <string>
#include <QString>

// Initiliazing static members of the class
logger* logger::LogPointer = NULL;
ofstream logger::LogFile;
std::string logger::FileName = SetFileName();

/// @author: Marcio Rockenbach
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Funtion to set the name of the logger file
/// @return string that is the name of the file to be created
std::string logger::SetFileName(){
    string name = "Log_";
    name.append(LogPointer->CurrentDateTime());
    name[14] = '_'; // Changing special character ':' to '_' to make it possible to create a file name with that
    name[17] = '_';
    name[20] = '_';
    name.append(".txt");
    return name;
};

/// @author: Marcio Rockenbach
/// @date: 23-12-2017
/// @version 1.0
///
/// @brief Funtion to create the instance of logger class.
/// @return pointer to a object of the logger class.
logger* logger::CreateLog()
{
    if (LogPointer == NULL){
        LogPointer = new logger();
        LogFile.open(logger::FileName.c_str(), ios::out | ios::app);
    }
    return LogPointer;
}

/// @author: Marcio Rockenbach
/// @date: 04-01-2018
/// @version 2.0
///
/// @brief Logs a string message
/// @param message to be included in the log
void logger::Log(const string& message)
{
    std::string time = logger::CurrentDateTime(); // Getting current date and time to add to the log
    std::string time_message = "00:00:00";
    for (int i = 0; i <= 7; i++) // The message displayed on the GUI will only contain the time, not the date
        time_message[i] = time[i+11];
    time_message.append(": ");
    time_message.append(message);
    QString qmessage = QString::fromStdString(time_message); // Converting from std string to QString
    emit sendmessage(qmessage); // Sending signal to display the message on the GUI
    LogFile <<  time << ":\t";
    LogFile << message << "\n";
    LogFile.flush();
}

/// @author: Marcio Rockenbach
/// @date: 04-01-2018
/// @version 2.0
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
    std::string time = logger::CurrentDateTime(); // Getting current date and time to add to the log
    std::string time_message = "00:00:00";
    for (int i = 0; i <= 7; i++) // The message displayed on the GUI will only contain the time, not the date
        time_message[i] = time[i+11];
    time_message.append(": ");
    time_message.append(message);
    QString qmessage = QString::fromStdString(time_message); // Converting from std string to QString
    emit sendmessage(qmessage); // Sending signal to display the message on the GUI
    LogFile <<  time << ":\t"; // Adds the time information to the file
    LogFile << message << "\n"; // Adds the message to the file
    LogFile.flush(); // Records the log in the file
    va_end(args);
    delete [] message;
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
    return buf;
}
