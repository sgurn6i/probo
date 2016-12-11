/* probopwm.hpp    -*- C++ -*-
   probo pwm hardware interface class
   2016-07-13 17:04:48 Sgurn6i
*/
#ifndef _PROBOPWM_H_
#define _PROBOPWM_H_
#include <map>
#include "probo.hpp"  /* Hwc */

namespace probo
{
  /* PWM controller abstract */
  class Pwmc : public Hwc
  {
  public:
    virtual ~Pwmc(){ };
    //virtual int init( const std::string& device, int address ) = 0; /* 初期化はそれぞれのクラスで独自に行なう。 */
    /* Hwcから継承 */
    virtual int get_ch_amt() const = 0;
    virtual bool is_initialized() const = 0;
    virtual bool is_acceptable(Hwj * hwj) = 0;
    /* Pwmc */
    /* PWM周波数 freq (Hz) を設定する。 */
    virtual int set_pwm_freq( double freq ) = 0;
    /* 指定されたチャンネル ch のパルス幅 t_ms (msec) を設定する。
     * 負数だとパルスを出さない。 */
    virtual int set_pwm_width( int ch, double t_ms ) = 0;
    /* 指定されたチャンネル ch の現状パルス幅 (msec) を返す。 */
    virtual double get_pwm_width( int ch ) = 0;
  protected:
    Pwmc(){ };
  };

  /* PWM Servo タイプ */
  typedef enum pwmservo_type_enum
  {
    PWM_SV_UNKNOWN,       /* 不明。 */
    PWM_SV_CUSTOM,       /* パラメータ手動設定。 */
    PWM_SV_RS304MD,      /* Futaba RS304MD */
    PWM_SV_MG996R,      /* Towerpro MG996R */
    PWM_SV_DEFAULT     /* デフォルト。 */
  } pwmservo_type_t;

  /* pwm Servo パラメータ */
  typedef struct pwmservo_params_struct
  {
    double pw_0deg;   /* 角度 0度のパルス幅(ms)。 */
    double pw_90deg;   /* 角度 90度のパルス幅(ms)。 */
    double min_deg;     /* 最小角度 */
    double max_deg;     /* 最大角度 */
  } pwmservo_params_t;

  /* PWM Servo タイプとパラメータ対応表。 */
  extern const std::map<pwmservo_type_t, pwmservo_params_t> g_pwmservo_type_params;
  
  class Pwmservo : public Hwj
  {
  public:
    Pwmservo( int ch = -1, pwmservo_type_t type = PWM_SV_UNKNOWN );
    virtual ~Pwmservo();
    /* 初期化 */
    virtual int init( int ch, /* コントローラ内チャンネル。 */
                      pwmservo_type_t type );  /* Servoタイプ指定。 */
    /* カスタム設定。 */
    virtual int init( int ch, const pwmservo_params_t& params );
    const pwmservo_params_t * get_params() { return &m_params; }
    /* 現在のパルス幅(ms)。 */
    double get_curr_pw(){ return get_pw( m_curr_deg ); }
    double get_pw( double deg );   /* 指定された角度のパルス幅(ms)。 */
    /* Hwjから継承 */
    virtual bool is_initialized() const { return m_initialized; }
    virtual int get_ch() const { return m_ch; } 
    virtual int set_curr_deg( double deg, Hwc * hwc );
    virtual double get_curr_deg() const { return m_curr_deg; }
  protected:
    bool m_initialized;
    int m_ch;
    int m_type;
    pwmservo_params_t m_params;
    double m_curr_deg;  /* 現在角度 */
  };
 
} /* namespace probo */

#endif /* _PROBOPWM_H_ */
