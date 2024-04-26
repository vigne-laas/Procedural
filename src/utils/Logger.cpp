#include "procedural/utils/Logger.h"

std::ofstream Logger::fileStream_;
std::unique_ptr<DualStream> Logger::dualStream_;
std::ostream* Logger::outputStream_ = &std::cout;
Logger::Level Logger::minLevel_ = Logger::DEBUG;