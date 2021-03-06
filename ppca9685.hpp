/* ppca9685.hpp    -*- C++ -*-
   Probo PCA9685 Interface
   2016-07-08 16:02:00 Sgurn6i
*/
#ifndef _PPCA9685_H_
#define _PPCA9685_H_
#include <string>
#include <inttypes.h> /* uint8_t */
#include "probopwm.hpp"

namespace probo
{
  class Pca9685 : public Pwmc
  {
  public:
    /* Pwmc class */
    virtual int get_ch_amt() const { return m_ch_amt; }
    virtual int set_pwm_freq( double freq );
    virtual int set_pwm_width( int ch, double t_ms );
    virtual double get_pwm_width( int ch );
    virtual bool is_initialized() const { return m_initialized; }
    /* ructors */
    Pca9685();
    virtual ~Pca9685();
    /*  */
    int init( const std::string& device = "/dev/i2c-2", int i2c_addr = 0x40, double pwm_freq = 60.0 );
    const std::string& get_device() const { return m_device; }
    int get_i2c_addr() const { return m_i2c_addr; }
    int cal_osc_freq( double osc_freq );    /* 公称25MHz内部オシレータ周波数設定。calibration用。 */
    uint8_t read_reg( uint8_t addr ); /* for debug */
  protected:
    void write8( uint8_t addr, uint8_t data );
    void write16( uint8_t addr, uint16_t data );
    uint8_t read8( uint8_t addr );
    uint16_t read16( uint8_t addr );
    void set_pwm_reg( int ch, uint16_t on, uint16_t off ); 
    void set_all_pwm_reg( uint16_t on, uint16_t off ); 
  private:
    bool m_initialized;
    static const int m_ch_amt = 16;
    double m_pwm_widths[m_ch_amt];
    std::string m_device;
    int m_i2c_addr;
    int m_fd; /* i2c file descriptor */
    double m_pwm_freq;  /* PWM実周波数。 */
    int m_prescale; /* m_pwm_freq から計算したprescale値。 */
    double m_osc_freq;  /* 25MHz内部osc周波数。 */
  };
  
  /* test function */
  int test_ppca9685(int argc, char *argv[]);
} /* namespace probo */


#endif /* _PPCA9685_H_ */
