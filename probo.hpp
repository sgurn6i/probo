/* probo.hpp    -*- C++ -*-
   probo制御
   2016-06-12 15:30:42 Sgurn6i
*/
#ifndef _PROBO_H_
#define _PROBO_H_
#include <string>
#include <vector>
#include "ea1/ea1_composite.hpp"

namespace probo
{
  /* 前方宣言 */
  class Joint;
  class Sensor;
  class Controller;
  class Hwc;
  class Hwj;
  /* 親玉本体 */
  class Body : public ea1::Composite
  {
  public:
    Body(const std::string& name = "Body");
    virtual ~Body();
    /* Compositeから継承。 */
    virtual int add_child(ea1::Component*  c);
    /* time(ms)の間に、targetまで進め。
     * time ms だけ処理をブロックする。
     * その間、tick毎に以下を指令する。
     *   子Controllerにパーセントpos指定。
     *   子Sensorにセンス指令。     */
    int do_em_in(double time);
    /* 更新単位時間 tick(ms) 設定。*/
    int set_tick(double tick);
    double get_tick() const { return m_tick; }
    /* 前回の時刻をリセット。 */
    void reset_time();
    double get_time_prev() const { return m_time_prev; }
    /* 各子Sensorのreset_sense()を呼ぶ。 */
    int reset_sense();
  protected:
    /* Component 継承 */
    virtual int set_parent(ea1::Composite& cs);
  private:
    int make_go_target_at(double percent);
    int make_update_pos();
    double m_tick = 20.0;  /* 単位時間(ms) */
    /* 前回の時刻(ms),
     * 前回 do_em_in()を終えた時刻。 */
    double m_time_prev;
  };

  /* コントローラー。シリアル通信他 */
  class Controller : public ea1::Composite
  {
  public:
    Controller(const std::string& name = "Ct");
    virtual ~Controller(){ }
    /* Compositeから継承。 */
    virtual int add_child(ea1::Component*  c);
    /* ターゲットのpercent % まで進め */
    virtual int go_target_at( double percent );
     /* previous position が現在位置になるようにアップデートせよ。 */
    virtual void update_pos();
    /* hwc */
    int attach_hwc( Hwc& hwc ); /* init済のhwcをattachする。 */
    void detach_hwc();
    Hwc * get_hwc() const { return m_hwc; }
  protected:
    /* Component 継承 */
    virtual int set_parent(ea1::Composite& cs);
  private:
    Hwc * m_hwc = NULL;
  };

  /* 関節ジョイント */
  class Joint : public ea1::Leaf
  {
  public:
    Joint(const std::string& name = "Joint");
    virtual ~Joint(){}
    /* ターゲットposition設定。 */
    int target(double pos);
    /* 現在のpositionを返す。
     * 実測値ではなく、go_target_atで指令された値。 */
    double get_curr_pos() const { return m_curr_pos; }
    /* 現在のpositionを返す。
     * hwjが付いていればhwj実測値。そうでなければ指令値。 */
    double get_curr_hw_pos() const;
    /* Controllerからの指令。 */
    virtual int go_target_at(double percent);
    virtual void update_pos();
    /* PWM servo関係 */
    /* init済のhwjをattachする。先にControllerにHwcが付いているを期待する。 */
    int attach_hwj( Hwj& hwj );
    void detach_hwj() { m_hwj = NULL; }
    Hwj * get_hwj() const { return m_hwj; }
  protected:
    /* Component 継承 */
    virtual int set_parent(ea1::Composite& cs);
    Hwc * get_hwc();
  private:
    double m_target_pos = 0.0;
    double m_prev_pos = 0.0;  // 過去位置、動作開始時の位置。
    double m_curr_pos = 0.0;  // 現在位置
    Hwj * m_hwj = NULL;
  };

  /* abstract センサー。gyro-accel等。 */
  class Sensor : public ea1::Leaf
  {
  public:
    Sensor(const std::string& name ="Sensor");
    virtual ~Sensor(){ }
    /* sense 実行。定期的に呼ばれるを期待する。
     * 内部積分値、微分値更新もこの時に行なう。
     * parameter:
     *   time: 実行時刻 (msec)。
     * returns:
     *   ea1_status_enum 値。
     */
    virtual int sense( double time ) = 0;
    /* センサ微積分値リセット。
     * parameter:
     *   time: 実行時刻 (msec)。
     * returns:
     *   ea1_status_enum 値。
     */
    virtual int reset_sense( double time ) = 0;
  protected:
    /* Component 継承 */
    virtual int set_parent(ea1::Composite& cs);
  };

  /* ハードウェアAbstractクラス。 */
  /* ハードウェアコントローラ */
  class Hwc : public ea1::Named
  {
  public:
    Hwc(const std::string& name = "Hwc");
    virtual ~Hwc(){ };
    /* 割り当てられる所の最大チャンネル数(固定)を返す。 */
    virtual int get_ch_amt() const = 0;
    /* デバイス等の初期化が完了していれば true。 */
    virtual bool is_initialized() const = 0;
    /* 与えられた Hwj が受け入れ可能ならtrue。 */
    virtual bool is_acceptable(Hwj * hwj) = 0;
  };
  
  /* ハードウェアジョイント */
  class Hwj : public ea1::Named
  {
  public:
    Hwj(const std::string& name = "Hwj");
    virtual ~Hwj(){ };
    virtual bool is_initialized() const = 0;
    virtual int get_ch() const = 0;
    /* 現在角度設定 (degree)。hwc がnon-NULLなら設定は hwcにも反映させる。 */
    virtual int set_curr_deg( double deg, Hwc * hwc) = 0;
    /* 現在角度実測値。 */
    virtual double get_curr_deg() const = 0;
    /* Todo: 脱力設定。 */
  };
    
} /* namespace probo */

#endif /* _PROBO_H_ */
