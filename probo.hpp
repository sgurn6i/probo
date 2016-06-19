/* probo.hpp    -*- C++ -*-
   probo制御
   2016-06-12 15:30:42 Sgurn6i
*/
#ifndef _PROBO_H_
#define _PROBO_H_
#include <vector>
#include <string>
#include "pfamily.hpp"

namespace Probo
{
  class Joint; /* 関節ジョイント */
  class Sensor; /* センサ予定 */
  class Controller; /* コントローラー。シリアル通信他 */
  /* 親玉本体 */
  class Body : public Pfamily::Parent
  {
  public:
    Body();
    virtual ~Body();
    int do_em_in(double time); /* time(ms)の間に、targetまで進め */
    Joint * create_joint(Controller * ct, int number,
                         const std::string& name = "");
    int add_controller(Controller * ct);
    int remove_controller(Controller * ct);
    Controller * get_controller(int n);
    int get_controller_amt(){ return (int)m_controllers.size(); }
  private:
    std::vector <Controller *> m_controllers;
    std::vector <Joint *> m_joints;
    double m_time_prev;   /* 前回の時刻(ms) */
    double m_time_to_target;  /* m_time_prev から target に達する時間。 */
  };

  class Controller
  {
  public:
    friend class Body;
    Controller(){ /* Todo: 中身 */ }
    virtual ~Controller(){ /* Todo: 中身 */ }
    Body * get_body(){ return m_body; }
  private:
    Body * m_body = NULL;  /* 所属Body */
    int set_body(Body * body);
  };
  
  class Joint
  {
  public:
    friend class Body;
    virtual ~Joint();
    int set_name(const char * name);
    int target(double pos);   /* ターゲットposition設定。 */
    /* Bodyからの指令。 */
    int do_to(double ratio);  /* ターゲットの ratioの割合まで進め。 */
  private:
    Joint(){ } /* 使わない */
    Joint(Body * body, Controller * ct, int number,
          const std::string& name);
    Body * m_body = NULL;  /* 所属Body */
    Controller * m_controller;
    int m_num;   // サーボ番号
    std::string m_name; // 名前(オプション)
    double m_target_pos;
    double m_prev_pos;  // 過去位置、動作開始時の位置。
    double m_curr_pos;  // 現在位置
  };
} /* Probo */

#endif /* _PROBO_H_ */
