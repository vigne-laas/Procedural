#ifndef PROCEDURAL_LOGGER_H
#define PROCEDURAL_LOGGER_H

#include "procedural/utils/DualStream.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <memory>
#include <iomanip>
#include <cstdio>




namespace procedural {
class Logger {
public:
    enum Level {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    Logger(Level level, const char* function) : level_(level), function_(function) {}

    Logger(Level level, const char* function, const std::string& filePath, bool alsoToTerminal = true)
            : level_(level), function_(function)
    {
        setOutput(filePath, alsoToTerminal);
    }

    ~Logger()
    {
        if (level_ < minLevel_)
            return;

        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
        std::string output = ss.str() + " ";

        switch (level_) {
            case DEBUG:
                output += "\033[36m[DEBUG]\033[0m ";
                break;
            case INFO:
                output += "\033[32m[INFO]\033[0m ";
                break;
            case WARNING:
                output += "\033[33m[WARNING]\033[0m ";
                break;
            case ERROR:
                output += "\033[31m[ERROR]\033[0m ";
                break;
        }

        output += "[" + std::string(function_) + "] " + ss_.str() + "\n";

        if (outputStream_) {
            (*outputStream_) << output;
        } else {
            std::cout << output;
        }
    }

    template<typename T>
    Logger& operator<<(const T& msg)
    {
        ss_ << msg;
        return *this;
    }

    static void setOutput(const std::string& filePath, bool alsoToTerminal = true)
    {
        fileStream_.open(filePath, std::ios::out | std::ios::app);
        if (alsoToTerminal) {
            dualStream_ = std::make_unique<DualStream>(std::cout, fileStream_);
            outputStream_ = dualStream_.get();
        } else {
            outputStream_ = &fileStream_;
        }
    }

    static void setMinLevel(Level level)
    {
        minLevel_ = level;
    }


private:
    std::stringstream ss_;
    Level level_;
    const char* function_;
    static std::ofstream fileStream_;
    static std::unique_ptr<DualStream> dualStream_;
    static std::ostream* outputStream_;
    static Level minLevel_;
};

#define LOG(level) Logger(level, __PRETTY_FUNCTION__)
#define LOG_DEBUG Logger(Logger::DEBUG, __PRETTY_FUNCTION__)
#define LOG_INFO Logger(Logger::INFO, __PRETTY_FUNCTION__)
#define LOG_WARNING Logger(Logger::WARNING, __PRETTY_FUNCTION__)
#define LOG_ERROR Logger(Logger::ERROR, __PRETTY_FUNCTION__)

#define SET_MIN_LEVEL(level) Logger::setMinLevel(level)

} // procedural
#endif //PROCEDURAL_LOGGER_H
