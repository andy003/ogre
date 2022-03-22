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

#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

#include "utils.h"
#include "urdf_exception/exception.h"

namespace urdf
{

class Color
{
public:
    Color() : r(0.0), g(0.0), b(0.0), a(1.0)
    {
        this->clear();
    };
    float r;
    float g;
    float b;
    float a;

    void clear()
    {
        r = g = b = 0.0f;
        a = 1.0f;
    }
    bool init(const std::string& vector_str)
    {
        this->clear();
        std::vector<std::string> pieces;
        std::vector<float> rgba;
        urdf::split_string(pieces, vector_str, " ");
        for (auto& i : pieces)
        {
            if (!i.empty())
            {
                try
                {
                    double piece = strToDouble(i.c_str());
                    if ((piece < 0) || (piece > 1))
                        throw ParseError("Component [" + i + "] is outside the valid range for colors [0, 1]");
                    rgba.push_back(static_cast<float>(piece));
                }
                catch (std::runtime_error& /*e*/)
                {
                    throw ParseError("Unable to parse component [" + i + "] to a double (while parsing a color value)");
                }
            }
        }

        if (rgba.size() != 4)
        {
            return false;
        }

        this->r = rgba[0];
        this->g = rgba[1];
        this->b = rgba[2];
        this->a = rgba[3];

        return true;
    };
};

}    // namespace urdf
