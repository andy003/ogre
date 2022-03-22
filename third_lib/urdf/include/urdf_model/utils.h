/**
 ***********************************************************************************************************************
 *
 * @author  ZhangRan
 * @version 1.0.0
 *
 * <h2><center>&copy; COPYRIGHT 2022 </center></h2>
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace urdf
{

// Replacement for boost::split( ... , ... , boost::is_any_of(" "))
inline void split_string(std::vector<std::string>& result, const std::string& input, const std::string& isAnyOf)
{
    std::string::size_type start = 0;
    std::string::size_type end = input.find_first_of(isAnyOf, start);
    while (end != std::string::npos)
    {
        result.push_back(input.substr(start, end - start));
        start = end + 1;
        end = input.find_first_of(isAnyOf, start);
    }
    if (start < input.length())
    {
        result.push_back(input.substr(start));
    }
}

// This is a locale-safe version of string-to-double, which is suprisingly
// difficult to do correctly.  This function ensures that the C locale is used
// for parsing, as that matches up with what the XSD for double specifies.
// On success, the double is returned; on failure, a std::runtime_error is
// thrown.
static inline double strToDouble(const char* in)
{
    char* err_string;
    double out = strtod(in, &err_string);
    if (*err_string != 0)
        throw std::runtime_error("Failed converting string to double");
    return out;
}

}    // namespace urdf
