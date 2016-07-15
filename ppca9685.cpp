/* ppca9685.cpp    -*- C++ -*-
   Probo PCA9685 Interface
   2016-07-09 15:52:00 Sgurn6i
*/
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h> /* read/write */
#include "ea1/ea1_benri.h"
#include "ea1/ea1_debug.h"
#include "ppca9685.hpp"
#define LOG_TAG "pca9685"

#define PCA9685_MODE1   0x00
#define PCA9685_MODE2   0x01
#define PCA9685_PRESCALE 0xfe
#define PCA9685_LED_0_ON_L  0x06
#define PCA9685_LED_0_ON_H  0x07
#define PCA9685_LED_0_OFF_L 0x08
#define PCA9685_LED_0_OFF_H 0x09
#define PCA9685_ALL_LED_ON_L  0xFA
#define PCA9685_ALL_LED_ON_H  0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD

#define PCA9685_RESTART 0x80
#define PCA9685_SLEEP   0x10
#define PCA9685_AUTOINC 0x20    /* register auto increment */
#define PCA9685_ALLCALL 0x01
#define PCA9685_OUTDRV  0x04    /* MODE2 output totempole */

#define PCA9685_OSC_WAIT_US   500  /* oscillator wait 時間 (us) */
#define PCA9685_LED_MAX_CT  4095    /* LED on/off カウント値としてのレジスタMAX値。 */
#define PCA9685_LED_FULL    4096    /* LED full ON/OFF レジスタ設定値。 */
#define PCA9685_OSC_FREQ    27296727.04 /* オシロ周波数実測値 */
#define PCA9685_DEFAULT_PWM_FREQ 60.0

/* ructors */
probo::Pca9685::Pca9685() :
  m_device( "" ),
  m_i2c_addr( -1 ),
  m_fd( -1 ),
  m_prescale( 0x65 ),
  m_osc_freq( PCA9685_OSC_FREQ )
{
  for (int ix = 0; ix < m_ch_amt; ix ++)
    {
      m_pwm_widths[ix] = -1.0;
    }
}

probo::Pca9685::~Pca9685()
{
  if (m_fd >= 0)
    close( m_fd );
}

void probo::Pca9685::write8( uint8_t addr, uint8_t data )
{
  if (( m_fd >= 0 )
      && ( ioctl( m_fd, I2C_SLAVE, m_i2c_addr ) >= 0 ))
    {
      uint8_t wds[2];
      wds[0] = addr;
      wds[1] = data;
      int w_size = write( m_fd, &wds, 2 );
      if (w_size != 2)
        THROW_E( EA1_FILE_ERROR, "%s: failed to write addr 0x%02x, data 0x%02x, rc %d",
                 __func__, addr, data, w_size );
    }
  else
    {
      THROW_E( EA1_E_NOT_READY, "%s: cannot write addr 0x%02x, data 0x%02x",
               __func__, addr, data );
    }
}

void probo::Pca9685::write16( uint8_t addr, uint16_t data )
{
  if (( m_fd >= 0 )
      && ( ioctl( m_fd, I2C_SLAVE, m_i2c_addr ) >= 0 ))
    {
      uint8_t wds[3];
      wds[0] = addr;
      wds[1] = data & 0x0ff;
      wds[2] = data >> 8;
      int w_size = write( m_fd, &wds, 3 );
      if (w_size != 3)
            THROW_E( EA1_FILE_ERROR, "%s: failed to write addr 0x%02x, data 0x%02x, rc %d",
                  __func__, addr, data, w_size );
    }
  else
    THROW_E( EA1_E_NOT_READY, "%s: cannot write addr 0x%02x, data 0x%04x", __func__, addr, data );
}

uint8_t probo::Pca9685::read8( uint8_t addr )
{
  uint8_t rd = 0xff;
  if (( m_fd >= 0 )
      && ( ioctl( m_fd, I2C_SLAVE, m_i2c_addr ) >= 0 ))
    {
      uint8_t wd = addr;
      int w_size = write( m_fd, &wd, 1 );
      int r_size = 0;
      if (w_size == 1)
        r_size = read (m_fd, &rd, 1 );
      if ((w_size != 1) && (r_size != 1))
        THROW_E( EA1_FILE_ERROR, "%s: failed to read addr 0x%02x, ws %d, rs %d",
              __func__, addr, w_size, r_size );
    }
  else
    THROW_E( EA1_E_NOT_READY, "%s: cannot read addr 0x%02x", __func__, addr );
  return rd;
}

uint16_t probo::Pca9685::read16( uint8_t addr )
{
  uint16_t rd = 0xffff;
  if (( m_fd >= 0 )
      && ( ioctl( m_fd, I2C_SLAVE, m_i2c_addr ) >= 0 ))
    {
      uint8_t wd = addr;
      int w_size = write( m_fd, &wd, 1 );
      int r_size = 0;
      if (w_size == 1)
        r_size = read (m_fd, &rd, 2 );
      if ((w_size != 1) && (r_size != 2))
        THROW_E( EA1_FILE_ERROR, "%s: failed to read addr 0x%02x, ws %d, rs %d",
              __func__, addr, w_size, r_size );
    }
  else
    THROW_E( EA1_E_NOT_READY, "%s: cannot read addr 0x%02x", __func__, addr );
  return rd;
}

int probo::Pca9685::init( const std::string& device, int i2c_addr )
{
  int rc = EA1_OK;
  try
    {
      m_device = device;
      m_i2c_addr = i2c_addr;
      m_fd = open( m_device.c_str(), O_RDWR );
      if (m_fd < 0)
        THROW_E( EA1_FILE_ERROR, "%s: failed to open %s", __func__, m_device.c_str() );
      write8( PCA9685_MODE2, PCA9685_OUTDRV );
      write8( PCA9685_MODE1, PCA9685_AUTOINC | PCA9685_ALLCALL );
      set_all_pwm_reg( 0, PCA9685_LED_FULL );  /* no pulse */
      usleep( PCA9685_OSC_WAIT_US );
      uint8_t mode1 = read8( PCA9685_MODE1 );
      mode1 &= ~PCA9685_SLEEP;
      write8( PCA9685_MODE1, mode1 );   /* wakeup */
      usleep( PCA9685_OSC_WAIT_US );
      rc = set_pwm_freq( PCA9685_DEFAULT_PWM_FREQ );
    }
  catch( ea1_status_t rc_error )
    { rc = rc_error; }
  return rc;
}

int probo::Pca9685::set_pwm_freq( double freq )
{
  ea1_status_t rc = EA1_OK;
  try
    {
      if (m_fd < 0)
        THROW_E( EA1_FILE_ERROR, "%s: not initialized", __func__ );
      /* calculate prescale */
      if( freq <= 0.0 )
        THROW_E( EA1_ERANGE, "%s: freq %f must be positive",
                 __func__, freq );
      double f_prescale = m_osc_freq / (freq * 4096);
      if( f_prescale > 256.0 )
        THROW_E( EA1_ERANGE, "%s: freq %f too small",
                 __func__, freq );
      int prescale = (int)( f_prescale + 0.5 ) - 1;
      if(( prescale < 0 ) || ( prescale >= 256 ))
        THROW_E( EA1_ERANGE, "%s: prescale %d is out of range, freq %f",
                 __func__, prescale, freq );
      /* set prescale */
      uint8_t org_mode1 = read8( PCA9685_MODE1 );
      uint8_t sleep_mode1 = ( org_mode1 & ~PCA9685_RESTART ) | PCA9685_SLEEP;
      write8( PCA9685_MODE1, sleep_mode1 ); /* go to sleep */
      write8( PCA9685_PRESCALE, prescale );
      //m_pwm_freq = freq;
      m_pwm_freq = m_osc_freq / ((prescale + 1) * 4096);
      LOGD( "%s:  %f Hz -> %f Hz prescale %d", __func__, freq, m_pwm_freq, prescale );
      write8( PCA9685_MODE1, org_mode1 );
      usleep( PCA9685_OSC_WAIT_US );
      write8( PCA9685_MODE1, org_mode1 | PCA9685_RESTART ); /* restart待ち？ */
    }
  catch( ea1_status_t rc_error )
    { rc = rc_error; }
  return rc;
}

double probo::Pca9685::get_pwm_width( int ch )
 {
   if(( ch < 0 ) || ( ch >= m_ch_amt ))
     {
        LOGE( "%s: ch %d outof range", __func__, ch );
       return 0.0; /* これでいいの？ */
     }
   return m_pwm_widths[ ch ];
 }

int probo::Pca9685::set_pwm_width( int ch, double t_ms )
{
  /* note: 小さすぎるパルスは出さず、大きすぎる場合は最大幅のパルスを出す。 */
  ea1_status_t rc = EA1_OK;
  try
    {
      if (m_fd < 0)
        THROW_E( EA1_FILE_ERROR, "%s: not initialized", __func__ );
      int count;
      if( t_ms <= 0.0 ) /* 小さすぎ */
        count = -1;
      if( t_ms * m_pwm_freq >= 1.0e3 )    /* 大きすぎ */
        count = PCA9685_LED_MAX_CT;
      else
        {
          count = (int)(PCA9685_LED_FULL * t_ms * 1.0e-3 * m_pwm_freq + 0.5);
          LOGD( "%s: ch %d, %f ms count %d", __func__, ch, t_ms, count );
          if( count > PCA9685_LED_MAX_CT )
            count = PCA9685_LED_MAX_CT;
        }
      if (count <= 0)
          /* パルス出さない。 */
          set_pwm_reg( ch, 0, PCA9685_LED_FULL );
      else
          set_pwm_reg( ch, 0, count );
    }
  catch( ea1_status_t rc_error )
    { rc = rc_error; }
  return rc;
}

uint8_t probo::Pca9685::read_reg( uint8_t addr )
{
  uint8_t data = 0;
  try
    {
      if (m_fd < 0)
        THROW_E( EA1_FILE_ERROR, "%s: not initialized", __func__ );
      data = read8( addr );
    }
  catch( ea1_status_t rc_error )
    {
      LOGE( "%s: read failed addr 0x%02x, rc %d", __func__, addr, (int)rc_error ); 
    }
  return data;
}

void probo::Pca9685::set_pwm_reg( int ch, uint16_t on, uint16_t off )
{
  if(( ch < 0 ) || ( ch >= m_ch_amt ))
    THROW_E( EA1_ERANGE, "%s: ch %d outof range", __func__, ch );
  int d_addr = 4 * ch;  /* address displacement */
  write16( PCA9685_LED_0_ON_L + d_addr, on );
  write16( PCA9685_LED_0_OFF_L + d_addr, off );
    
 }

void probo::Pca9685::set_all_pwm_reg( uint16_t on, uint16_t off )
{
  write16( PCA9685_ALL_LED_ON_L, on );
  write16( PCA9685_ALL_LED_OFF_L, off );
}

int probo::Pca9685::cal_osc_freq( double osc_freq )
{
  int rc = EA1_OK;
  try
    {
      if (m_fd < 0)
        THROW_E( EA1_FILE_ERROR, "%s: not initialized", __func__ );
      if ((osc_freq < 10.0e6) || (osc_freq > 50.0e6))
        THROW_E( EA1_ERANGE, "%s: osc_freq %f outof range", __func__, osc_freq );
      m_osc_freq = osc_freq;
      /* PWM周波数再設定 */
      rc = set_pwm_freq( m_pwm_freq );
    }
  catch( ea1_status_t rc_error )
    { rc = rc_error; }
  return rc;
}

/* test function */
int probo::test_ppca9685(int argc, char *argv[])
{
  int rc = EA1_OK;
  probo::Pca9685 * pca = NULL;
  LOGD("test ppca9685");
  try
    {
      pca = new probo::Pca9685();
      if (pca == NULL)
        THROW_E( EA1_FAIL, "failed to create pca9685" );
      pca->init();
      pca->set_pwm_freq( 60 );
      pca->set_pwm_width( 0, 0.560 );
      pca->set_pwm_width( 1, 0.560 );
      pca->set_pwm_width( 2, 0.560 );
      pca->set_pwm_width( 3, 1.500 );
      usleep ( 1500 * 1000 );
#if 1
      pca->set_pwm_width( 0, 0.920 ); // 90deg
      usleep ( 1500 * 1000 );
      pca->set_pwm_width( 0, 2.120 ); // -90deg
      usleep ( 1500 * 1000 );
#endif
      pca->set_pwm_width( 0, 2.480 );
      pca->set_pwm_width( 1, 2.480 );
      pca->set_pwm_width( 2, 2.480 );
      usleep ( 1500 * 1000 );
      pca->set_pwm_width( 0, 1.530 );
      usleep ( 1500 * 1000 );
      pca->set_pwm_width( 0, 1.520 );
      pca->set_pwm_width( 1, 1.520 );
      pca->set_pwm_width( 2, 1.520 );
      usleep ( 1500 * 1000 );
    }
  catch( ea1_status_t rc_error )
    { rc = rc_error; }
  /* 後始末 */
  EA1_SAFE_DELETE( pca );
  
  return rc;
}

#ifdef TEST_PPCA9685
/* test main */
int main(int argc, char *argv[])
{
  return test_ppca9685( argc, argv );
}
#endif // TEST_PPCA9685
