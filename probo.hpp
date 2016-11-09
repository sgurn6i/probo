/* probo.hpp    -*- C++ -*-
   probo制御
   2016-06-12 15:30:42 Sgurn6i
*/
#ifndef _PROBO_H_
#define _PROBO_H_
#include <string>
#include <vector>
#include "pfamily.hpp"

namespace probo
{
  /* 前方宣言 */
  class Joint; /* 関節ジョイント */
  class Sensor; /* センサ予定 */
  class Controller; /* コントローラー。シリアル通信他 */
  class Pwmc; /* probopwm.hpp */
  /* 親玉本体 */
  class Body : public pfamily::Parent
  {
  public:
    Body(const std::string& name = "Body");
    virtual ~Body();
    int do_em_in(double time); /* time(ms)の間に、targetまで進め */
    Controller * get_controller(int n);
    virtual Controller * create_controller(const std::string& name = "ct");
    int set_tick(double tick);
    double get_tick() const { return m_tick; }
    void reset_time();  /* 現在時刻をリセット。 */
  private:
    int make_go_target_at(double percent);
    int make_update_pos();
    double m_tick = 20.0;  /* 単位時間(ms) */
    double m_time_prev;   /* 前回の時刻(ms) */
  };

  class Controller :
    public pfamily::Parent, 
    public pfamily::Child 
  {
    friend Controller * Body::create_controller(const std::string& name);
  public:
    virtual ~Controller(){ }
    virtual int go_target_at( double percent ); /* ターゲットのpercent % まで進め */
    virtual void update_pos(); /* previous position を現在位置にアップデートせよ。 */
    Body& get_body() const { return (Body&)get_parent(); }
    virtual Joint * create_joint( const std::string& name = "j" );
    /* pwm */
    int attach_pwmc( Pwmc& pwmc ); /* init済のpwmcをattachする。 */
    void detach_pwmc(){ m_pwmc = NULL; }
    Pwmc * get_pwmc() const { return m_pwmc; }
  protected:
    Controller( Body& body, int sn, const std::string& name );
  private:
    int control_joint_hw ( Joint& joint );  /* jointに付随するハードウェアを動かす。 */
    int control_joint_pwm_hw ( Joint& joint );  /* jointに付随するPWMハードウェアを動かす。 */
    Pwmc * m_pwmc;
  };

  class Pwmservo; /* probopwm.hpp */
  class Joint :
    public pfamily::Child
  {
    friend Joint * Controller::create_joint( const std::string& name);
  public:
    virtual ~Joint(){}
    int target(double pos);   /* ターゲットposition設定。 */
    double get_curr_pos() const { return m_curr_pos; }
    /* Controllerからの指令。 */
    virtual int go_target_at(double percent);
    virtual void update_pos();
    /* PWM servo関係 */
    int attach_pwmservo( Pwmservo& pwmservo ); /* init済のpwmservoをattachする。
                                                  先にControllerにPwmcが付いているを期待する。 */
    void detach_pwmservo() { m_pwmservo = NULL; }
    Pwmservo * get_pwmservo() const { return m_pwmservo; }
  protected:
    Joint(Controller& controller, int sn, const std::string& name) :
      pfamily::Child::Child(controller,sn,name)
    { }
  private:
    double m_target_pos = 0.0;
    double m_prev_pos = -1000.0;  // 過去位置、動作開始時の位置。
    double m_curr_pos = 0.0;  // 現在位置
    Pwmservo * m_pwmservo = NULL;
  };

  /* テスト用関数 */
  int test_main(int argc, char *argv[]);
} /* namespace probo */

#endif /* _PROBO_H_ */
