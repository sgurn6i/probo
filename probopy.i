/* probopy.i -*- C++ -*-
 * python interface SWIG定義
 * 2016-11-08 15:31:29 Sgurn6i
 */
%module probopy
%{
  #include "pfamily.hpp"
  #include "probo.hpp"
  #include "probopwm.hpp"
  #include "ppca9685.hpp"
%}
%include <std_string.i>

namespace pfamily
{
  class Base
  {
  public:
    virtual ~Base(){ }
  };

  class Parent;
  class Child;

  class Child : public Base
  {
    friend class Parent;
  public:
    int get_sn() const;
    const std::string& get_name();
    Parent& get_parent() const;
  protected:
    Child(Parent& parent, int sn, const std::string& name);
    virtual ~Child();
  };

  class Parent : public Base
  {
    friend Child::~Child();
  public:
    Parent(const std::string& name = "parent");
    virtual ~Parent();
    virtual Child * create_child( const std::string& name = "p child" );
    const std::string& get_p_name();
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
  class Pwmc;

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
    int attach_pwmc( Pwmc& pwmc );
    void detach_pwmc();
    Pwmc * get_pwmc() const;
  protected:
    Controller( Body& body, int sn, const std::string& name );
  };

  class Pwmservo; /* probopwm.hpp */
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
    int attach_pwmservo( Pwmservo& pwmservo );
    void detach_pwmservo();
    Pwmservo * get_pwmservo() const;
  protected:
    Joint(Controller& controller, int sn, const std::string& name);
  };

  /* probopwm.hpp */
  class Pwmc : public pfamily::Base
  {
  public:
    virtual ~Pwmc(){ };
    virtual int get_ch_amt() const = 0;
    virtual int set_pwm_freq( double freq ) = 0;
    virtual int set_pwm_width( int ch, double t_ms ) = 0;
    virtual double get_pwm_width( int ch ) = 0;
    virtual bool is_initialized() const = 0;
  protected:
    Pwmc();
  };

  %feature("notabstract") Pca9685;
  class Pca9685 : public Pwmc
  {
  public:
    /* Pwmc class */
    virtual int get_ch_amt();
    virtual int set_pwm_freq( double freq );
    virtual int set_pwm_width( int ch, double t_ms );
    virtual double get_pwm_width( int ch );
    virtual bool is_initialized() const;
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
  
  class Pwmservo : public pfamily::Base
  {
  public:
    Pwmservo( int ch = -1, pwmservo_type_t type = PWM_SV_UNKNOWN );
    virtual ~Pwmservo();
    virtual int init( int ch, pwmservo_type_t type );
    virtual int init( int ch, const pwmservo_params_t& params );
    virtual bool is_initialized() const;
    const pwmservo_params_t * get_params();
    int get_ch() const;
    void set_curr_deg( double deg );
    double get_curr_deg() const;
    double get_curr_pw();
    double get_pw( double deg );
  };

} /* namespace probo */
