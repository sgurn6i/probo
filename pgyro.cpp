/* pgyro.cpp    -*- C++ -*-
   Gyro Sensor. Dummy implementation.
   2016-12-29 15:59:49 Sgurn6i
*/
#include "pgyro.hpp"
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#define LOG_TAG "pgyro"

probo::Gyro::Gyro(const std::string& name)
  : probo::Sensor(name)
{ }

int probo::Gyro::sense( double time )
{
  return EA1_OK;
}
int probo::Gyro::reset_sense( double time )
{
  return EA1_OK;
}

int probo::Gyro::get_curr_quat(double * out_x,
                               double * out_y,
                               double * out_z,
                               double * out_w)
{
  if ((out_x == NULL)
      || (out_y == NULL)
      || (out_z == NULL)
      || (out_w == NULL))
    {
      LOGE("%s: pointer was NULL", __func__);
      return EA1_EINVAL;
    }
  *out_x = 0.0;
  *out_y = 0.0;
  *out_z = 0.0;
  *out_w = 1.0;
  return EA1_OK;
}
