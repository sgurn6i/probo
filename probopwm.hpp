/* probopwm.hpp    -*- C++ -*-
   probo hardware interface abstract class
   2016-07-13 17:04:48 Sgurn6i
*/
#ifndef _PROBOPWM_H_
#define _PROBOPWM_H_

namespace probo
{
  /* PWM controller abstract */
  class Pwmc
  {
  public:
    virtual ~Pwmc(){ };
    virtual int init( const std::string& device, int address ) = 0;
    virtual int get_ch_amt() const = 0;
    virtual int set_pwm_freq( double freq ) = 0;   /* PWM周波数(Hz)設定。 */
    virtual int set_pwm_width( int ch, double t_ms ) = 0; /* ch のパルス幅t_ms(msec) を指定する。 
                                                           * 負数だとパルスを出さない。 */
    virtual double get_pwm_width( int ch ) = 0;
  protected:
    Pwmc(){ };
  };
} /* namespace probo */

#endif /* _PROBOPWM_H_ */
