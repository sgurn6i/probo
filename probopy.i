/* probopy.i -*- C++ -*-
 * python interface SWIG定義
 * 2016-11-08 15:31:29 Sgurn6i
 */
%module probopy
%{
  #include "ea1/ea1_benri.h"
  #include "pfamily.hpp"
  #include "probo.hpp"
  #include "probopwm.hpp"
  #include "ppca9685.hpp"
%}
%include <std_string.i>
%include "ea1/ea1_benri.h"

namespace pfamily
{
  class Base
  {
  public:
    virtual ~Base(){ }
    virtual const std::string& get_name() const;
  };

  class Parent;
  class Child;

  class Child : virtual public Base
  {
    friend class Parent;
  public:
    int get_sn() const;
    Parent& get_parent() const;
  protected:
    Child(Parent& parent, int sn, const std::string& name);
    virtual ~Child();
  };

  class Parent : virtual public Base
  {
    friend Child::~Child();
  public:
    Parent(const std::string& name = "parent");
    virtual ~Parent();
    virtual Child * create_child( const std::string& name = "p child" );
    int get_children_amt();
    Child * get_child(int sn);
    bool has_child(Child * cp);
  };
} /* namespace pfamily */

namespace probo
{
  /* probo.hpp */
  class Joint;
  class Sensor;
  class Controller;
  class Hwc;
  class Hwj;

  class Body : public pfamily::Parent
  {
  public:
    Body(const std::string& name = "Body");
    virtual ~Body();
    int do_em_in(double time);
    Controller * get_controller(int n);
    virtual Controller * create_controller(const std::string& name = "ct");
    int set_tick(double tick);
    double get_tick() const;
    void reset_time();
  };

  class Controller :
    public pfamily::Parent, 
    public pfamily::Child 
  {
    friend Controller * Body::create_controller(const std::string& name);
  public:
    virtual ~Controller(){ }
    virtual int go_target_at( double percent );
    virtual void update_pos();
    Body& get_body() const;
    virtual Joint * create_joint( const std::string& name = "j" );
    /* pwm */
    int attach_hwc( Hwc& hwc );
    void detach_hwc();
    Hwc * get_hwc() const;
  protected:
    Controller( Body& body, int sn, const std::string& name );
  };

  class Joint :
    public pfamily::Child
  {
    friend Joint * Controller::create_joint( const std::string& name);
  public:
    virtual ~Joint(){}
    int target(double pos);
    double get_curr_pos() const;
    virtual int go_target_at(double percent);
    virtual void update_pos();
    /* PWM servo関係 */
    int attach_hwj( Hwj& hwj );
    void detach_hwj();
    Hwj * get_hwj() const;
  protected:
    Joint(Controller& controller, int sn, const std::string& name);
  };

  class Hwc : public pfamily::Base
  {
  public:
    virtual ~Hwc(){ };
    virtual int get_ch_amt() const = 0;
    virtual bool is_initialized() const = 0;
    virtual bool is_acceptable(Hwj * hwj) = 0;
  };
  
  class Hwj : public pfamily::Base
  {
  public:
    virtual ~Hwj(){ };
    virtual bool is_initialized() const = 0;
    virtual int get_ch() const = 0;
    virtual int set_curr_deg( double deg, Hwc * hwc) = 0;
    virtual double get_curr_deg() const = 0;
  };
    
  /* probopwm.hpp */
  class Pwmc : public Hwc
  {
  public:
    virtual ~Pwmc(){ };
    virtual int get_ch_amt() const = 0;
    virtual bool is_initialized() const = 0;
    virtual bool is_acceptable(Hwj * hwj) = 0;
    virtual int set_pwm_freq( double freq ) = 0;
    virtual int set_pwm_width( int ch, double t_ms ) = 0;
    virtual double get_pwm_width( int ch ) = 0;
  protected:
    Pwmc();
  };

  %feature("notabstract") Pca9685;
  class Pca9685 : public Pwmc
  {
  public:
    /* Pwmc class */
    virtual int get_ch_amt();
    virtual bool is_initialized() const;
    virtual bool is_acceptable(Hwj * hwj);
    virtual int set_pwm_freq( double freq );
    virtual int set_pwm_width( int ch, double t_ms );
    virtual double get_pwm_width( int ch );
    /* ructors */
    Pca9685();
    virtual ~Pca9685();
    /*  */
    int init( const std::string& device = "/dev/i2c-2", int i2c_addr = 0x40, double pwm_freq = 60.0 );
    const std::string& get_device();
    int get_i2c_addr() const;
    int cal_osc_freq( double osc_freq );
    uint8_t read_reg( uint8_t addr );
  };

  typedef enum pwmservo_type_enum
  {
    PWM_SV_UNKNOWN,       /* 不明。 */
    PWM_SV_CUSTOM,       /* パラメータ手動設定。 */
    PWM_SV_RS304MD,      /* Futaba RS304MD */
    PWM_SV_MG996R,      /* Towerpro MG996R */
    PWM_SV_DEFAULT     /* デフォルト。 */
  } pwmservo_type_t;

  typedef struct pwmservo_params_struct
  {
    double pw_0deg;   /* 角度 0度のパルス幅(ms)。 */
    double pw_90deg;   /* 角度 90度のパルス幅(ms)。 */
    double min_deg;     /* 最小角度 */
    double max_deg;     /* 最大角度 */
  } pwmservo_params_t;

  //extern const std::map<pwmservo_type_t, pwmservo_params_t> g_pwmservo_type_params;
  
  class Pwmservo : public Hwj
  {
  public:
    Pwmservo( int ch = -1, pwmservo_type_t type = PWM_SV_UNKNOWN );
    virtual ~Pwmservo();
    virtual int init( int ch, pwmservo_type_t type );
    virtual int init( int ch, const pwmservo_params_t& params );
    const pwmservo_params_t * get_params();
    double get_curr_pw();
    double get_pw( double deg );
    virtual bool is_initialized() const;
    virtual int get_ch() const;
    virtual int set_curr_deg( double deg, Hwc * hwc );
    virtual double get_curr_deg() const;
  };

} /* namespace probo */
