/* probo.hpp    -*- C++ -*-
   probo制御
   2016-06-12 15:30:42 Sgurn6i
*/
#ifndef _PROBO_H_
#define _PROBO_H_
#include <vector>
#include <string>
#include "pfamily.hpp"

namespace probo
{
  class Joint; /* 関節ジョイント */
  class Sensor; /* センサ予定 */
  class Controller; /* コントローラー。シリアル通信他 */
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
  public:
    Controller(Body& body, int sn, const std::string& name) :
      pfamily::Parent::Parent(name),
      pfamily::Child::Child(body,sn,name){ }
    virtual ~Controller(){ }
    virtual int go_target_at(double percent); /* ターゲットのpercent % まで進め */
    virtual void update_pos(); /* previous position を現在位置にアップデートせよ。 */
    Body& get_body() const { return (Body&)get_parent(); }
    virtual Joint * create_joint(const std::string& name = "j");
  private:
  };

  class Joint :
    public pfamily::Child
  {
  public:
    Joint(Controller& controller, int sn, const std::string& name) :
      pfamily::Child::Child(controller,sn,name){ }
    virtual ~Joint(){}
    int target(double pos);   /* ターゲットposition設定。 */
    double get_curr_pos() const { return m_curr_pos; }
    /* Controllerからの指令。 */
    virtual int go_target_at(double percent);
    virtual void update_pos();
  private:
    int m_servo_n;   // サーボ番号
    double m_target_pos = 0.0;
    double m_prev_pos = 0.0;  // 過去位置、動作開始時の位置。
    double m_curr_pos = m_prev_pos;  // 現在位置
  };
  int test_main(int argc, char *argv[]);
} /* namespace probo */

#endif /* _PROBO_H_ */
