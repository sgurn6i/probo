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
#include "pfamily.hpp"
#include "probopwm.hpp"
#include "ppca9685.hpp"
#include "probo.hpp"
#define LOG_TAG "probo"
using probo::Body;
using probo::Controller;
using probo::Joint;
using probo::Sensor;
using probo::ControllerBuilder;
using probo::JointBuilder;
using probo::SensorBuilder;

/* a と b が delta > 0 的に近い。 */
#define NEAR_POS(a,b,delta) \
  ((((a) - (b)) < (delta)) && (((b) - (a)) < (delta)))

Body::Body(const std::string& name) :
  pfamily::Named::Named(name),
  pfamily::Parent::Parent()
{
  reset_time();
}
Body::~Body()
{
}

pfamily::Child * Body::create_child(pfamily::ChildBuilder& builder,
                                    const std::string& name)
{
  /* dynamic casted builder  */
  pfamily::ChildBuilder * dc_b; 
  dc_b = dynamic_cast<ControllerBuilder *>(&builder);
  if (dc_b == NULL)
    dc_b = dynamic_cast<SensorBuilder *>(&builder);
  if (dc_b == NULL)
    {
      LOGE("%s: unknown builder", __func__);
      return NULL;
    }
  return pfamily::Parent::create_child(builder, name);
}

Controller * Body::get_controller(int n)
{
  return dynamic_cast<Controller *>(get_child(n));
}

Controller * Body::create_controller(ControllerBuilder& builder,
                                     const std::string& name)
{
  return dynamic_cast<Controller *>(create_child(builder, name));
}

int Body::set_tick(double tick)
{
  if (tick <= 1.0)
    {
      LOGE("%s: invalid tick %f", __func__, tick);
      return EA1_EINVAL;
    }
  m_tick = tick;
  return EA1_OK;
}

void Body::reset_time()
{
  m_time_prev = ea1_gettimeofday_ms();
}

int Body::do_em_in(double time)
{
  int rc = EA1_OK;
  /* Todo: time reset 必要か。*/
  reset_time();
  double time_target = m_time_prev + time;
  double time_lap = m_time_prev;    /* calculated lap time. */
  if (m_tick <= 0.0)
    return EA1_FAIL;
  do
    {
      time_lap += m_tick;
      double lap_percent;
      if ((time_lap >= time_target) || (time <= 0.0))
        {
          lap_percent = 100.0;
          time_lap = time_target;
        }
      else
        {
          lap_percent = (time_lap - m_time_prev) / time * 100.0;
          if (lap_percent > 100.0)
            lap_percent = 100.0;
        }
      /* issue orders to controllers. */
      rc = make_go_target_at(lap_percent);
      if (rc < 0)
        goto RET;
      double time_sleep = time_lap - ea1_gettimeofday_ms();
      LOGD("%s: t %f, dt %f, sleep %f, %f %%",
           __func__, time_lap, time_lap - m_time_prev,
           time_sleep, lap_percent);
      if (time_sleep > 0)   // 2016-10-12 15:30:36 added
        ea1_sleep_ms(time_sleep); 
    }
  while (time_lap < time_target);
 RET:
  make_update_pos();
  m_time_prev = time_lap;
  return rc;
}

int Body::make_go_target_at(double percent)
{
  if (percent < 0.0) percent = 0.0;
  if (percent > 100.0) percent = 100.0;
  int amt = get_children_amt();
  for (int ix = 0; ix < amt; ix ++)
    {
      Controller * ctlr = dynamic_cast<Controller *>(get_child(ix));
      if (ctlr != NULL)
        {
          ctlr->go_target_at(percent);
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

pfamily::Child * ControllerBuilder::create_child(pfamily::Parent& parent,
                                                 const std::string& name)
{
  Body * bp = dynamic_cast<Body *>(&parent);
  if (bp == NULL)
    {
      LOGE("%s: unknown parent", __func__);
      return NULL;
    }
  Controller * cp = new Controller(*bp, parent.get_children_amt(), name);
  if (cp == NULL)
    {
      LOGE("%s: failed to create Controller( %s )", __func__, name.c_str());
      return NULL;
    }
  return cp;
}


Controller::Controller(Body& body, int sn, const std::string& name) :
  pfamily::Parent::Parent(),
  pfamily::Child::Child( body, sn, name ),
  m_hwc( NULL )
{ }

pfamily::Child * Controller::create_child(pfamily::ChildBuilder& builder,
                                          const std::string& name)
{
  JointBuilder * jb
    = dynamic_cast<JointBuilder *>(&builder);
  if (jb == NULL)
    {
      LOGE("%s: unknown builder", __func__);
      return NULL;
    }
  return pfamily::Parent::create_child(builder, name);
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

Joint * Controller::create_joint(JointBuilder& builder,
                                 const std::string& name)
{
  return dynamic_cast<Joint *>(create_child(builder, name));
}

int Controller::attach_hwc( probo::Hwc& hwc )
{
  int rc = EA1_OK;
  if (! hwc.is_initialized())
    {
      LOGE("%s: hwc is not initialized", __func__);
      return EA1_E_NOT_READY;
    }
  m_hwc = &hwc;
  return rc;
}

pfamily::Child * JointBuilder::create_child(pfamily::Parent& parent,
                                            const std::string& name)
{
  Controller * cp = dynamic_cast<Controller *>(&parent);
  if (cp == NULL)
    {
      LOGE("%s: unknown parent", __func__);
      return NULL;
    }
  Joint * jp
    = new Joint(*cp, parent.get_children_amt(), name);
  if (jp == NULL)
    {
      LOGE("%s: failed to create Joint( %s )", __func__, name.c_str());
      return NULL;
    }
  return jp;
}

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
  LOGD("%s: pos curr %f, target %f", get_name().c_str(), m_curr_pos, m_target_pos);  
  return EA1_OK;
}

probo::Hwc * Joint::get_hwc()
{
  Controller * controller = dynamic_cast<Controller *>( &get_parent() );
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
    }
  m_hwj = &hwj;
  return rc;
}

Body * SensorBuilder::get_body(pfamily::Parent& parent)
{
  Body * b = dynamic_cast<Body *>(&parent);
  if (b == NULL)
    {
      LOGE("%s: unknown parent", __func__);
    }
  return b;
}

Sensor::Sensor(Body& body, int sn, const std::string& name)
  : pfamily::Child::Child( body, sn, name)
{
}

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
