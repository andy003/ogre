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
#include <sstream>
#include <vector>
#include <math.h>
#include "pose.h"

namespace urdf{


class Twist
{
public:
  Twist() { this->clear(); };

  Vector3  linear;
  // Angular velocity represented by Euler angles
  Vector3  angular;

  void clear()
  {
    this->linear.clear();
    this->angular.clear();
  };
};

}
