#include "ApogeeWrapper.h"

std::vector<std::string> Apogee::Tokenize( const std::string& str, const std::string& separator)
{
    std::vector<std::string> returnVector;
    std::string::size_type start = 0;
    std::string::size_type end = 0;
    
    while ((end = str.find(separator, start)) != std::string::npos)
    {
        returnVector.push_back(str.substr(start, end - start));
        start = end + separator.size();
    }
    
    returnVector.push_back(str.substr(start));
    
    return returnVector;
}

std::string Apogee::GetStr( const std::string& msg, const std::string& item)
{
    std::vector<std::string> params = Tokenize(msg, ",");
    std::vector<std::string>::iterator iter;
    
    for (iter = params.begin(); iter != params.end(); ++iter)
    {
        if (std::string::npos != (*iter).find(item))
        {
            std::string result = Tokenize((*iter), "=").at(1);
            
            return result;
        }
    }
    
    std::string noOp;
    return noOp;
}

uint16_t Apogee::GetInt( const std::string& msg, const std::string& item)
{
    std::stringstream convert;
    uint16_t output;
    
    std::string value = Apogee::GetStr(msg, item);
    
    convert << std::hex << std::showbase << value.c_str();
    convert >> output;
    
    return output;
}
