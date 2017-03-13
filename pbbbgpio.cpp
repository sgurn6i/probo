/* pbbbgpio.cpp    -*- C++ -*-
   BBB GPIO
   2017-03-11 18:24:32 Sgurn6i
*/
#include <unistd.h> /* usleep */ 
#include "SimpleGPIO/SimpleGPIO.h"
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#include "pbbbgpio.hpp"
#define LOG_TAG "pbbbgpio"
#define IMPL (*(this->impl))

class probo::BbbGpioin::Impl
{
public:
  bool initialized = false;
  bool done_set_dir = false;
  int set_dir();
  unsigned int gpio_num;
  int val = EA1_E_NOT_READY;
};

int probo::BbbGpioin::Impl::set_dir()
{
  int rc = EA1_E_NOT_READY;
  if(initialized)
    {
      int gpio_rc = -1;
      for (int ix = 0; ix < 100; ix ++)
        {
          gpio_rc = gpio_set_dir(gpio_num, INPUT_PIN);
          if (gpio_rc >= 0)
            {
              done_set_dir = true;
              rc = EA1_OK;
              break;
            }
          usleep(10 * 1000);  // wait for udev permissions to be changed
        }
      if (gpio_rc < 0)
        LOGE("%s: gpio error code %d", __func__, gpio_rc);
    }
  return rc;
}

probo::BbbGpioin::BbbGpioin(const std::string& name)
  : probo::Gpioin(name),
    impl(new Impl())
{
  /* todo: unexport必要か？ */
}

probo::BbbGpioin::~BbbGpioin()
{ }

int probo::BbbGpioin::init(unsigned int gpio_num)
{
  int rc = EA1_EINVAL;
  IMPL.initialized = false;
  IMPL.done_set_dir = false;
  IMPL.val = EA1_E_NOT_READY;
  int gpio_rc = gpio_export(gpio_num);
  if (gpio_rc >= 0)
    {
      IMPL.initialized = true;
      IMPL.gpio_num = gpio_num;
      rc = EA1_OK;
    }
  else  
    {
      LOGE("%s: gpio error code %d", __func__, gpio_rc);
    }

  return rc;
}

int probo::BbbGpioin::sense(double time)
{
  int rc = EA1_E_NOT_READY;
  if (IMPL.initialized)
    {
      int gpio_rc = 0;
      if (! IMPL.done_set_dir)
        {
          gpio_rc = IMPL.set_dir();
        }
      if ((gpio_rc >= 0) && IMPL.done_set_dir)
        {
          unsigned int val;
          gpio_rc = gpio_get_value(IMPL.gpio_num, &val);
          if (gpio_rc >= 0)
            {
              IMPL.val = val;
              rc = EA1_OK;
            }
          else
            {
              rc = EA1_FAIL;
              LOGE("%s: gpio read error code %d", __func__, gpio_rc);
            }
        }
    }
  if (rc < 0)
    {
      IMPL.val = rc;
    }
  return rc;
}

int probo::BbbGpioin::reset_sense(double time)
{
  IMPL.val = EA1_E_NOT_READY;
  return EA1_OK;
}

int probo::BbbGpioin::get_val()
{
  return IMPL.val;
}
