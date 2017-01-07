/* probo.cpp    -*- C++ -*-
   probo 制御
   2016-06-12 15:30:09 Sgurn6i
*/
//#include <iostream>
#include <time.h> /* nanosleep */
#include <sys/time.h> /* gettimeofday */
#include <unistd.h> /* usleep */
#include <vector>
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#include "ea1/ea1_time.h"
#include "probopwm.hpp"
#include "ppca9685.hpp"
#include "probo.hpp"
#define LOG_TAG "probo"
using probo::Body;
using probo::Controller;
using probo::Joint;
using probo::Sensor;

/* a と b が delta > 0 的に近い。 */
#define NEAR_POS(a,b,delta) \
  ((((a) - (b)) < (delta)) && (((b) - (a)) < (delta)))

Body::Body(const std::string& name) :
  ea1::Composite(name)
{
  reset_time();
}

Body::~Body()
{
}

int Body::add_child(ea1::Component* c)
{
  auto ct = dynamic_cast<Controller *>(c);
  auto ss = dynamic_cast<Sensor *>(c);
  if ((ct == NULL) && (ss == NULL))
    {
      LOGE("%s %s: unknown child", get_name().c_str(), __func__);
      return EA1_EINVAL;
    }
  return ea1::Composite::add_child(c);
}

int Body::set_tick(double tick)
{
  if (tick <= 1.0)
    {
      LOGE("%s %s: invalid tick %f", get_name().c_str(), __func__, tick);
      return EA1_EINVAL;
    }
  m_tick = tick;
  return EA1_OK;
}

void Body::reset_time()
{
  m_time_prev = ea1_gettimeofday_ms();
}

int Body::reset_sense()
{
  int rc = EA1_OK;
  int amt = get_children_amt();
  for (int ix = 0; ix < amt; ix ++)
    {
      auto sensor = dynamic_cast<Sensor *>(get_child(ix));
      if (sensor != NULL)
        {
          int sensor_rc = sensor->reset_sense(ea1_gettimeofday_ms());
          if (sensor_rc < rc)
            rc = sensor_rc;
        }
    }
  return rc;
}

int Body::do_em_in(double time)
{
  int rc = EA1_OK;
  /* reset_time() はしない。
   * できるだけユーザー設定時間に追いつくように制御する。 */
  double time_target = m_time_prev + time;
  /* 遅れを考慮して、作業するために与えられたる時間。 */
  double wt = time_target - ea1_gettimeofday_ms();
  /* 次にコントローラに指令し、動作及びsleepを終える時点と計画した所の時刻。 */
  double time_lap = m_time_prev;
  double time_sleep = 0.0;
  int count = 0;
  if (m_tick <= 0.0)
    return EA1_FAIL;
  do
    {
      time_lap = ea1_gettimeofday_ms() + m_tick;
      double lap_percent;
      /* 最終lap近くなら一気に進める。 */
      if ((time_lap >= time_target - m_tick * 0.5) || (wt <= 0.0))
        {
          lap_percent = 100.0;
          time_lap = time_target;
        }
      else
        {
          lap_percent = (time_lap - m_time_prev) / wt * 100.0;
          if (lap_percent > 100.0)
            lap_percent = 100.0;
        }
      /* issue orders to controllers. */
      rc = make_go_target_at(lap_percent);
      if (rc < 0)
        goto RET;
      /* 余り時間は寝て待つ。 */
      time_sleep = time_lap - ea1_gettimeofday_ms();
      LOGD("%s %s: %d t %.2f, dt %.2f, sleep %.2f, %.2f %%",
           get_name().c_str(), __func__, count, time_lap, time_lap - m_time_prev,
           time_sleep, lap_percent);
      if (time_sleep > 0)   // 2016-10-12 15:30:36 added
        ea1_sleep_ms(time_sleep);
      count ++;
    }
  while (time_lap < time_target);
  if (time_sleep < 0)
    {
      LOGD("%s %s: count %d wt %.2f delay %.2f",
           get_name().c_str(), __func__, count, wt, - time_sleep);
    }
 RET:
  make_update_pos();
  m_time_prev = time_target;
  return rc;
}

int Body::set_parent(ea1::Composite& cs)
{
  /* Bodyの親は無い。 */
  LOGE("%s %s: unknown parent", get_name().c_str(), __func__);
  return EA1_EINVAL;
}

int Body::make_go_target_at(double percent)
{
  if (percent < 0.0) percent = 0.0;
  if (percent > 100.0) percent = 100.0;
  int amt = get_children_amt();
  for (int ix = 0; ix < amt; ix ++)
    {
      auto ctlr = dynamic_cast<Controller *>(get_child(ix));
      auto sensor = dynamic_cast<Sensor *>(get_child(ix));
      if (ctlr != NULL)
        {
          ctlr->go_target_at(percent);
        }
      else if (sensor != NULL)
        {
          sensor->sense(ea1_gettimeofday_ms());
        }
    }
  return EA1_OK;
}
int Body::make_update_pos()
{
  int amt = get_children_amt();
  for (int ix = 0; ix < amt; ix ++)
    {
      Controller * ctlr = dynamic_cast<Controller *>(get_child(ix));
      if (ctlr != NULL)
        {
          ctlr->update_pos();
        }
    }
  return EA1_OK;
}

Controller::Controller(const std::string& name)
  : ea1::Composite(name)
{ }
int Controller::add_child(ea1::Component* c)
{
  auto jt = dynamic_cast<Joint *>(c);
  if (jt == NULL)
    {
      LOGE("%s %s: unknown child", get_name().c_str(), __func__);
      return EA1_EINVAL;
    }
  return ea1::Composite::add_child(c);
}

int Controller::go_target_at(double percent)
{
  //LOGD("%s %s: %f %%", get_name().c_str(), __func__, percent);
  int rc = EA1_OK;
  /*  Pass to joints.  */
    int amt = get_children_amt();
  for (int ix = 0; ix < amt; ix ++)
    {
      Joint * joint = dynamic_cast<Joint *>( get_child( ix ));
      if (joint != NULL)
        {
          int joint_rc = joint->go_target_at( percent );
          if (joint_rc < 0)
            {
              LOGE("%s %s: joint %s failed, rc %d",
                   get_name().c_str(), __func__, joint->get_name().c_str(), joint_rc);
            }
          if ((rc >= 0) && (joint_rc < 0))
            rc = joint_rc;
        }
    }
  return rc;
}
  
void Controller::update_pos()
{
  /*  Pass to joints.  */
  int amt = get_children_amt();
  for (int ix = 0; ix < amt; ix ++)
    {
      Joint * joint = dynamic_cast<Joint *>(get_child(ix));
      if (joint != NULL)
        {
          joint->update_pos();
        }
    }
}


int Controller::attach_hwc( probo::Hwc& hwc )
{
  int rc = EA1_OK;
  if (! hwc.is_initialized())
    {
      LOGE("%s %s: hwc is not initialized", get_name().c_str(), __func__);
      return EA1_E_NOT_READY;
    }
  detach_hwc(); /* jointの旧Hwjをdetachする。 */
  m_hwc = &hwc;
  return rc;
}

int Controller::set_parent(ea1::Composite& cs)
{
  Body * body = dynamic_cast<Body *>( &cs );
  if (body == NULL)
    {
      LOGE("%s %s: unknown parent", get_name().c_str(), __func__);
      return EA1_EINVAL;
    }
  return ea1::Composite::set_parent(cs);
}

/* hwc無ければhwj無い。 */
void Controller::detach_hwc()
{
  int amt = get_children_amt();
  for (int ix = 0; ix < amt; ix ++)
    {
      Joint * joint = dynamic_cast<Joint *>(get_child(ix));
      if (joint != NULL)
        joint->detach_hwj();
    }
  m_hwc = NULL;
}

Joint::Joint(const std::string& name)
  : ea1::Leaf(name)
{ }

int Joint::go_target_at(double percent)
{
  int rc = EA1_OK;
  double ratio = percent * 0.01;
  if (ratio < 0.0) ratio = 0.0;
  if (ratio > 1.0) ratio = 1.0;
  /* curr_pos to be updated. */
  double new_curr_pos = m_prev_pos + (m_target_pos - m_prev_pos) * ratio;
  /* 近いのは変えない。 Todo: 途中でservo起動したりすると面倒なことになりそう.. */
  if(! NEAR_POS(m_curr_pos, new_curr_pos, 1.0e-5))
    {
      m_curr_pos = new_curr_pos;
      Hwc * hwc = get_hwc();
      if (m_hwj != NULL)
        {
          rc = m_hwj->set_curr_deg( m_curr_pos, hwc );
        }
    }
  LOGD("%s %s: %f %% pos %f rc %d", get_name().c_str(), __func__, percent, m_curr_pos, rc);
  return rc;
}

void Joint::update_pos()
{
  m_prev_pos = m_curr_pos;
  if (m_hwj != NULL)
    {
      m_hwj->set_curr_deg( m_curr_pos, NULL ); /* Todo: 不要かも */
    }
 }

int Joint::target(double pos)
{
  m_target_pos = pos;
  /* todo: check ターゲット範囲。 */
  LOGD("%s %s: pos curr %f, target %f",
       get_name().c_str(), __func__, m_curr_pos, m_target_pos);  
  return EA1_OK;
}

double Joint::get_curr_hw_pos() const
{
  if (m_hwj != NULL)
    {
      return m_hwj->get_curr_deg();
    }
  else
    {
      return m_curr_pos;
    }
}

probo::Hwc * Joint::get_hwc()
{
  Controller * controller = dynamic_cast<Controller *>( get_parent() );
  Hwc * hwc = NULL;
  if (controller != NULL)
    hwc = controller->get_hwc();
  return hwc;
}

int Joint::attach_hwj( probo::Hwj& hwj )
{
  int rc = EA1_OK;
  if (! hwj.is_initialized())
    {
      LOGE("%s %s: hwj is not initialized", get_name().c_str(), __func__);
      return EA1_E_NOT_READY;
    }
  Hwc * hwc = get_hwc();
  if (hwc == NULL)
    {
      LOGE("%s %s: no hwc was attached", get_name().c_str(), __func__);
      return EA1_E_NOT_READY;
    }
  /* jointの親のcontrollerに付いてるhwcのチャンネル範囲を超えたらエラー。 */
  if ((hwj.get_ch() < 0) || (hwj.get_ch() >= hwc->get_ch_amt()))
    {
      LOGE("%s %s: hwj ch %d out of range 0 .. %d",
           get_name().c_str(), __func__, hwj.get_ch(), hwc->get_ch_amt() - 1);
      return EA1_E_RANGE;
    }
  m_hwj = &hwj;
  return rc;
}

int Joint::set_parent(ea1::Composite& cs)
{
  Controller * controller = dynamic_cast<Controller *>( &cs );
  if (controller == NULL)
    {
      LOGE("%s %s: unknown parent", get_name().c_str(), __func__);
      return EA1_EINVAL;
    }
  return ea1::Leaf::set_parent(cs);
}

Sensor::Sensor(const std::string& name)
  : ea1::Leaf(name)
{ }

int Sensor::set_parent(ea1::Composite& cs)
{
  Body * body = dynamic_cast<Body *>( &cs );
  if (body == NULL)
    {
      LOGE("%s %s: unknown parent", get_name().c_str(), __func__);
      return EA1_EINVAL;
    }
  return ea1::Leaf::set_parent(cs);
}

probo::Hwc::Hwc(const std::string& name)
  : ea1::Named(name)
{ }

probo::Hwj::Hwj(const std::string& name)
  : ea1::Named(name)
{ }

#if 0
int probo::test_main(int argc, char *argv[])
{
  /* test pfamily */
  //pfamily::test_main(argc,argv);
  //LOGI("");
  LOGI("starts");
  Body * body1 = new Body();
  ControllerBuilder cb;
  JointBuilder jb;
  const char * name1 = "ct1";
  Controller * ct1 = body1->create_controller(cb, name1);
  Controller * ct2 = body1->create_controller(cb, "ct2");
  Joint * j11 = ct1->create_joint(jb, "j11");
  Joint * j12 = ct1->create_joint(jb, "j12");
  Joint * j21 = ct2->create_joint(jb, "j21_has_a_long_name_like_this");
  /* pwm */
  probo::Pca9685 * pca9685 = new probo::Pca9685();
  pca9685->init( "/dev/i2c-2", 0x40, 100.0 );
  ct1->attach_hwc( *pca9685 );
  probo::Pwmservo * sv1 = new probo::Pwmservo( 0, PWM_SV_RS304MD );
  probo::Pwmservo * sv2 = new probo::Pwmservo( 1, PWM_SV_RS304MD );
  j11->attach_hwj( *sv1 );
  j12->attach_hwj( *sv2 );
  /* sequence */
  body1->set_tick(20.0);
  j11->target(90.0);
  j12->target(-90.0);
  j21->target(45.0);
  body1->do_em_in(300.0);
  j12->target(180.0);
  j21->target(-90.0);
  body1->do_em_in(2200.0);
  j11->target(0.0);
  j12->target(0.0);
  body1->do_em_in(200.0);
  /* destruct */
  EA1_SAFE_DELETE(body1);
  EA1_SAFE_DELETE( sv1 );
  EA1_SAFE_DELETE( sv2 );
  EA1_SAFE_DELETE( pca9685 );
  return 0;
}
#endif
