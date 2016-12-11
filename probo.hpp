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
  class Hwc; /* アタッチされるハードウェアコントローラ。 */
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
     /* previous position が現在位置になるようにアップデートせよ。 */
    virtual void update_pos();
    Body& get_body() const { return (Body&)get_parent(); }
    virtual Joint * create_joint( const std::string& name = "j" );
    /* pwm */
    int attach_hwc( Hwc& hwc ); /* init済のhwcをattachする。 */
    void detach_hwc(){ m_hwc = NULL; }
    Hwc * get_hwc() const { return m_hwc; }
  protected:
    Controller( Body& body, int sn, const std::string& name );
  private:
    Hwc * m_hwc;
  };

  class Hwj;
  class Joint :
    public pfamily::Child
  {
    friend Joint * Controller::create_joint( const std::string& name);
  public:
    virtual ~Joint(){}
    /* ターゲットposition設定。 */
    int target(double pos);
    double get_curr_pos() const { return m_curr_pos; }
    /* Controllerからの指令。 */
    virtual int go_target_at(double percent);
    virtual void update_pos();
    /* PWM servo関係 */
    /* init済のhwjをattachする。先にControllerにHwcが付いているを期待する。 */
    int attach_hwj( Hwj& hwj );
    void detach_hwj() { m_hwj = NULL; }
    Hwj * get_hwj() const { return m_hwj; }
  protected:
    Joint(Controller& controller, int sn, const std::string& name) :
      pfamily::Child::Child(controller,sn,name),
      m_hwj(NULL)
    { }
    Hwc * get_hwc();
  private:
    double m_target_pos = 0.0;
    double m_prev_pos = -100.0;  // 過去位置、動作開始時の位置。
    double m_curr_pos = 0.0;  // 現在位置
    Hwj * m_hwj;
  };

  /* ハードウェアAbstractクラス。 */
  /* ハードウェアコントローラ */
  class Hwc : public pfamily::Base
  {
  public:
    virtual ~Hwc(){ };
    /* 割り当てられる所の最大チャンネル数(固定)を返す。 */
    virtual int get_ch_amt() const = 0;
    /* デバイス等の初期化が完了していれば true。 */
    virtual bool is_initialized() const = 0;
    /* 与えられた Hwj が受け入れ可能ならtrue。 */
    virtual bool is_acceptable(Hwj * hwj) = 0;
  };
  
  /* ハードウェアジョイント */
  class Hwj : public pfamily::Base
  {
  public:
    virtual ~Hwj(){ };
    virtual bool is_initialized() const = 0;
    virtual int get_ch() const = 0;
    /* 現在角度設定 (degree)。hwc がnon-NULLなら設定は hwcにも反映させる。 */
    virtual int set_curr_deg( double deg, Hwc * hwc) = 0;
    virtual double get_curr_deg() const = 0;
    /* Todo: 脱力設定。 */
  };
    

  /* テスト用関数 */
  int test_main(int argc, char *argv[]);
} /* namespace probo */

#endif /* _PROBO_H_ */
