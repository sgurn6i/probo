/* probopwm.cpp    -*- C++ -*-
   probo pwm hardware interface class
   2016-07-16 17:15:59 Sgurn6i
*/
#include <map>
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#include "probopwm.hpp"
#define LOG_TAG "probopwm"

using probo::Pwmservo;

const std::map<probo::pwmservo_type_t, probo::pwmservo_params_t> probo::g_pwmservo_type_params 
= {
  /* {type, {pw_0deg, pw_90deg,  }} */
  {probo::PWM_SV_RS304MD, { 1.520, 0.920, -144.0, 144.0 }}, 
  {probo::PWM_SV_DEFAULT, { 1.500, 1.000, -90.0, 90.0 }}, 
};

/* ructors */
Pwmservo::Pwmservo( int ch, pwmservo_type_t type ) :
  m_initialized( false ),
  m_ch( ch ),
  m_type( type ),
  m_curr_deg( 0.0 )
{
  if (type != probo::PWM_SV_UNKNOWN)
    {
      init( ch, type );
    }
}

/* Pwmservo funcs */
int Pwmservo::init( int ch, const pwmservo_params_t& params )
{
  int rc = EA1_OK;
  if (( params.pw_0deg == params.pw_90deg )
      || (params.min_deg >= params.max_deg ))
    {
      LOGE( "%s: invalid parameters {%f, %f, %f %f}",
            __func__, params.pw_0deg, params.pw_90deg, params.min_deg, params.max_deg );
      return EA1_EINVAL;
    }
  m_ch = ch;
  m_type = PWM_SV_CUSTOM;
  m_params = params;
  m_initialized = true;
  return rc;
}

int Pwmservo::init( int ch, pwmservo_type_t type )
{
  int rc = EA1_STATUS1; /* initializedされたとは言えない状態。 */
  m_ch = ch;
  m_type = type;
  // probo::g_pwmservo_type_params から見つかればinit ok. そうでなければinit 解消。
  try
    {
      if (probo::g_pwmservo_type_params.find( type ) == probo::g_pwmservo_type_params.end())
        {
          LOGD( "%s: not found g_pwmservo_type_params[%d]", __func__, (int)type );
          m_initialized = false;
        }
      else
        {
          m_params = probo::g_pwmservo_type_params.at( type );
          m_initialized = true;
          rc = EA1_OK;
        }
    }
  catch( const std::out_of_range & e )
    {
      LOGD( "%s: catched std::out_of_range", __func__ );
      m_initialized = false;
      rc = EA1_STATUS1;
    }
  
  return rc;
}

double Pwmservo::get_pw( double deg )
{
  if ( is_initialized() )
    {
      double limited_deg = deg;
      if ( limited_deg < m_params.min_deg )
        limited_deg = m_params.min_deg;
      else if ( limited_deg > m_params.max_deg )
        limited_deg = m_params.max_deg;
      double pw_per_deg = ( m_params.pw_90deg - m_params.pw_0deg ) / 90.0;
      return (limited_deg * pw_per_deg) + m_params.pw_0deg;
    }
  else /* ! initialized */
    {
      /* 初期化されてなかったら負数を返す。 */
      return -1.0;
    }
}
