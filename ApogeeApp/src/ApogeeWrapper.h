#ifndef INC_APOGEEWRAPPER_H
#define INC_APOGEEWRAPPER_H

#include <stdint.h>

#include <vector>
#include <string>
#include <sstream>

namespace Apogee
{
	std::vector<std::string> Tokenize( const std::string& str, const std::string& separator);
    uint16_t    GetInt ( const std::string& msg, const std::string& item);
    std::string GetStr ( const std::string& msg, const std::string& item);
}

#endif
