/* probopwm.hpp    -*- C++ -*-
   probo pwm hardware interface class
   2016-07-13 17:04:48 Sgurn6i
*/
#ifndef _PROBOPWM_H_
#define _PROBOPWM_H_
#include <map>
#include "pfamily.hpp"  /* pfamily::Base */

namespace probo
{
  /* PWM controller abstract */
  class Pwmc : public pfamily::Base
  {
  public:
    virtual ~Pwmc(){ };
    //virtual int init( const std::string& device, int address ) = 0; /* 初期化はそれぞれのクラスで独自に行なう。 */
    /* 割り当てられる所の最大チャンネル数(固定)を返す。 */
    virtual int get_ch_amt() const = 0;
    /* PWM周波数 freq (Hz) を設定する。 */
    virtual int set_pwm_freq( double freq ) = 0;
    /* 指定されたチャンネル ch のパルス幅 t_ms (msec) を設定する。
     * 負数だとパルスを出さない。 */
    virtual int set_pwm_width( int ch, double t_ms ) = 0;
    /* 指定されたチャンネル ch の現状パルス幅 (msec) を返す。 */
    virtual double get_pwm_width( int ch ) = 0;
    /* デバイス等の初期化が完了していれば true。 */
    virtual bool is_initialized() const = 0;
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
  
  class Pwmservo : public pfamily::Base
  {
  public:
    Pwmservo( int ch = -1, pwmservo_type_t type = PWM_SV_UNKNOWN );
    virtual ~Pwmservo();
    virtual int init( int ch, /* コントローラ内チャンネル。 */
                      pwmservo_type_t type );  /* Servoタイプ指定。 */
    virtual int init( int ch, const pwmservo_params_t& params ); /* カスタム設定。 */
    virtual bool is_initialized() const { return m_initialized; }
    const pwmservo_params_t * get_params() { return &m_params; }
    int get_ch() const { return m_ch; } 
    void set_curr_deg( double deg ){ m_curr_deg = deg; } /* 現在角度設定 (degree)。 */
    double get_curr_deg() const { return m_curr_deg; }
    double get_curr_pw(){ return get_pw( m_curr_deg ); }   /* 現在のパルス幅(ms)。 */
    double get_pw( double deg );   /* 指定された角度のパルス幅(ms)。 */
  protected:
    bool m_initialized;
    int m_ch;
    int m_type;
    pwmservo_params_t m_params;
    double m_curr_deg;  /* 現在角度 */
  };
 
} /* namespace probo */

#endif /* _PROBOPWM_H_ */
