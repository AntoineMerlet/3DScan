#ifndef CUSTOM_LOGGER_H
#define CUSTOM_LOGGER_H

#include <fstream>
#include <iostream>
#include <cstdarg>
#include <string>
#include <QObject>
#include <QString>

using namespace std;

#define INITLOG {logger::CreateLog();}
#define LOG(message) logger::CreateLog()->Log(message)

class logger: public QObject
{
    Q_OBJECT
public:
    static std::string FileName;
    static logger* LogPointer; // Pointer for the logger class
    logger(){} // Default constructor
    static logger *CreateLog(); // Creates instance of the logger class
    void Log(const std::string& message); // Adds a message to the log file
    void Log(const char *format, ...); // Adds a message to the log file with a specific formatting
    logger &operator << (const string &message); // Operator overload

signals:
    void sendmessage(const QString& arg);

private:
    static ofstream LogFile; // Log file stream object.
    static std::string SetFileName(); // Sets the namefile of the log with current date and time
    std::string CurrentDateTime(); // Gets the current date and time
    logger(const logger&){} // Copy constructor
    logger &operator=(const logger&){return *this;}  // Assignment operator for the logger class
    ~logger(){delete LogPointer;}
};

#endif
