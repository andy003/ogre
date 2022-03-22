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

#include <string>
#include <stdexcept>

namespace urdf
{

class ParseError : public std::runtime_error
{
public:
    explicit ParseError(const std::string& error_msg) : std::runtime_error(error_msg){};
};

}    // namespace urdf
