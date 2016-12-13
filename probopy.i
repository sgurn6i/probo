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
    virtual ~Base();
  };

  class Named
  {
  public:
    Named(const std::string& name = "");
    virtual ~Named();
    virtual const std::string& get_name() const;
  };
  
  class Parent;
  class ChildBuilder;
  class Child : virtual public Base, virtual public Named
  {
    friend class ChildBuilder;
  public:
    int get_sn() const;
    Parent& get_parent() const;
    virtual ~Child();
  protected:
    Child(Parent& parent, int sn=-1, const std::string& name="_child");
  };

  /* child を作って渡すBuilderクラス */
  class ChildBuilder
  {
  public:
    virtual ~ChildBuilder();
    virtual Child * create_child(Parent& parent,
                                 const std::string& name = "b_child" );
  };

  class Parent : virtual public Base
  {
    friend Child::~Child();
  public:
    Parent();
    virtual ~Parent();
    virtual Child * create_child(ChildBuilder& builder,
                                 const std::string& name = "p_child" );
    int get_children_amt() const;
    Child * get_child(int sn);
    bool has_child(Child * cp);
  protected:
    virtual int add_child(Child& c);
    int remove_child(Child * cp);
  };
}

namespace probo
{
  /* probo.hpp */
  class Joint;
  class JointBuilder;
  class Controller;
  class ControllerBuilder;
  class Hwc;
  class Hwj;

  class Body : virtual public pfamily::Named, public pfamily::Parent
  {
  public:
    Body(const std::string& name = "Body");
    virtual ~Body();
    virtual pfamily::Child * create_child(pfamily::ChildBuilder& builder,
                                          const std::string& name = "bchild" );
    int do_em_in(double time);
    Controller * get_controller(int n);
    virtual Controller * create_controller(ControllerBuilder& builder,
                                           const std::string& name = "ct");
    int set_tick(double tick);
    double get_tick() const;
    void reset_time();
  };

  class ControllerBuilder : public pfamily::ChildBuilder
  {
  public:
    virtual ~ControllerBuilder();
    virtual pfamily::Child * create_child(pfamily::Parent& parent,
                                 const std::string& name = "b_ct_child" );
  };

  class Controller :
    public pfamily::Parent, 
    public pfamily::Child 
  {
    friend ControllerBuilder;
  public:
    virtual ~Controller();
    virtual pfamily::Child * create_child(pfamily::ChildBuilder& builder,
                                          const std::string& name = "cchild" );
    virtual int go_target_at( double percent );
    virtual void update_pos();
    Body& get_body() const;
    virtual Joint * create_joint(JointBuilder& builder,
                                 const std::string& name = "j" );
    int attach_hwc( Hwc& hwc );
    void detach_hwc();
    Hwc * get_hwc() const;
  protected:
    Controller( Body& body, int sn, const std::string& name );
  private:
    Hwc * m_hwc;
  };

  class JointBuilder : public pfamily::ChildBuilder
  {
  public:
    virtual ~JointBuilder();
    virtual pfamily::Child * create_child(pfamily::Parent& parent,
                                 const std::string& name = "b_jt_child" );
  };

  /* 関節ジョイント */
  class Joint :
    public pfamily::Child
  {
    friend JointBuilder;
  public:
    virtual ~Joint();
    int target(double pos);
    double get_curr_pos() const;
    virtual int go_target_at(double percent);
    virtual void update_pos();
    int attach_hwj( Hwj& hwj );
    void detach_hwj();
    Hwj * get_hwj() const;
  protected:
    Joint(Controller& controller, int sn, const std::string& name);
    Hwc * get_hwc();
  };

  class Hwc : public pfamily::Base
  {
  public:
    virtual ~Hwc();
    virtual int get_ch_amt() const = 0;
    virtual bool is_initialized() const = 0;
    virtual bool is_acceptable(Hwj * hwj) = 0;
  };
  
  class Hwj : public pfamily::Base
  {
  public:
    virtual ~Hwj();
    virtual bool is_initialized() const = 0;
    virtual int get_ch() const = 0;
    virtual int set_curr_deg( double deg, Hwc * hwc) = 0;
    virtual double get_curr_deg() const = 0;
  };
    
  /* probopwm.hpp */
  class Pwmc : public Hwc
  {
  public:
    virtual ~Pwmc();
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
