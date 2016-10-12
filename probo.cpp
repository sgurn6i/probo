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

/* a と b が delta > 0 的に近い。 */
#define NEAR_POS(a,b,delta) \
  ((((a) - (b)) < (delta)) && (((b) - (a)) < (delta)))

Body::Body(const std::string& name) :
  pfamily::Parent::Parent(name)
{
  reset_time();
}
Body::~Body()
{
}
Controller * Body::get_controller(int n)
{
  return dynamic_cast<Controller *>(get_child(n));
}

Controller * Body::create_controller(const std::string& name)
{
  Controller * cp = new Controller(*this, get_children_amt(), name);
  if (cp == NULL) return NULL;
  int rc = add(*cp);
  if (rc != EA1_OK)
    {
      LOGE("%s: failed to add()", __func__);
      remove_child(cp);
      return NULL;
    }
  return cp;
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

Controller::Controller(Body& body, int sn, const std::string& name) :
  pfamily::Parent::Parent( name ),
  pfamily::Child::Child( body, sn, name ),
  m_pwmc( NULL )
{ }


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
          joint->go_target_at( percent );
          int joint_rc = control_joint_hw( *joint );
          if ((rc >= 0) && (joint_rc < 0))
            rc = joint_rc;
        }
    }
  return rc;
}

int Controller::control_joint_hw( Joint& joint )
{
  int rc = EA1_OK;
  if (m_pwmc != NULL)
    {
      rc = control_joint_pwm_hw( joint );
      if (rc < 0)
        return rc;
    }
  return rc;
}

int Controller::control_joint_pwm_hw( Joint& joint )
{
  int rc = EA1_OK;
  Pwmservo * pwmservo = joint.get_pwmservo();
  if (pwmservo != NULL)
    {
      rc = m_pwmc->set_pwm_width( pwmservo->get_ch(), pwmservo->get_curr_pw() );
      if (rc < 0)
        {
          LOGE("%s %s: joint %s failed", get_name().c_str(), __func__, joint.get_name().c_str());
          return rc;
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

Joint * Controller::create_joint(const std::string& name)
{
  Joint * jp = new Joint(*this, get_children_amt(), name);
  if (jp == NULL) return NULL;
  int rc = add(*jp);
  if (rc != EA1_OK)
    {
      LOGE("%s: failed to add()", __func__);
      remove_child(jp);
      return NULL;
    }
  return jp;
}

int Controller::attach_pwmc( probo::Pwmc& pwmc )
{
  int rc = EA1_OK;
  if (! pwmc.is_initialized())
    {
      LOGE("%s: pwmc is not initialized", __func__);
      return EA1_E_NOT_READY;
    }
  m_pwmc = &pwmc;
  return rc;
}

int Joint::go_target_at(double percent)
{
  double ratio = percent * 0.01;
  if (ratio < 0.0) ratio = 0.0;
  if (ratio > 1.0) ratio = 1.0;
  /* curr_pos to be updated. */
  double new_curr_pos = m_prev_pos + (m_target_pos - m_prev_pos) * ratio;
  /* 近いのは変えない。 Todo: 途中でservo起動したりすると面倒なことになりそう.. */
  if(! NEAR_POS(m_curr_pos, new_curr_pos, 1.0e-5))
    {
      m_curr_pos = new_curr_pos;
      if (m_pwmservo != NULL)
        {
          m_pwmservo->set_curr_deg( m_curr_pos );
        }
    }
  LOGD("%s %s: %f %% pos %f", get_name().c_str(), __func__, percent, m_curr_pos);
  return EA1_OK;
}

void Joint::update_pos()
{
  m_prev_pos = m_curr_pos;
  if (m_pwmservo != NULL)
    {
      m_pwmservo->set_curr_deg( m_curr_pos );
    }
 }

int Joint::target(double pos)
{
  m_target_pos = pos;
  /* todo: check ターゲット範囲。 */
  LOGD("%s: pos curr %f, target %f", get_name().c_str(), m_curr_pos, m_target_pos);  
  return EA1_OK;
}

int Joint::attach_pwmservo( probo::Pwmservo& pwmservo )
{
  int rc = EA1_OK;
  if (! pwmservo.is_initialized())
    {
      LOGE("%s %s: pwmservo is not initialized", get_name().c_str(), __func__);
      return EA1_E_NOT_READY;
    }
  Controller * controller = dynamic_cast<Controller *>( &get_parent() );
  probo::Pwmc * pwmc = NULL;
  if (controller != NULL)
    pwmc = controller->get_pwmc();
  if (pwmc == NULL)
    {
      LOGE("%s %s: no pwmc was attached", get_name().c_str(), __func__);
      return EA1_E_NOT_READY;
    }
  /* jointの親のcontrollerに付いてるpwmcのチャンネル範囲を超えたらエラー。 */
  if ((pwmservo.get_ch() < 0) || (pwmservo.get_ch() >= pwmc->get_ch_amt()))
    {
      LOGE("%s %s: pwmservo ch %d out of range 0 .. %d",
           get_name().c_str(), __func__, pwmservo.get_ch(), pwmc->get_ch_amt() - 1);
    }
  m_pwmservo = &pwmservo;
  return rc;
}

int probo::test_main(int argc, char *argv[])
{
  /* test pfamily */
  //pfamily::test_main(argc,argv);
  //LOGI("");
  LOGI("starts");
  Body * body1 = new Body();
  const char * name1 = "ct1";
  Controller * ct1 = body1->create_controller(name1);
  Controller * ct2 = body1->create_controller("ct2");
  Joint * j11 = ct1->create_joint("j11");
  Joint * j12 = ct1->create_joint("j12");
  Joint * j21 = ct2->create_joint("j21_has_a_long_name_like_this");
  /* pwm */
  probo::Pca9685 * pca9685 = new probo::Pca9685();
  pca9685->init( "/dev/i2c-2", 0x40, 100.0 );
  ct1->attach_pwmc( *pca9685 );
  probo::Pwmservo * sv1 = new probo::Pwmservo( 0, PWM_SV_RS304MD );
  probo::Pwmservo * sv2 = new probo::Pwmservo( 1, PWM_SV_RS304MD );
  j11->attach_pwmservo( *sv1 );
  j12->attach_pwmservo( *sv2 );
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
