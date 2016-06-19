/* probo.cpp    -*- C++ -*-
   probo 制御
   2016-06-12 15:30:09 Sgurn6i
*/
//#include <iostream>
#include <vector>
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#include "pfamily.hpp"
#include "probo.hpp"
#define LOG_TAG "probo"

Probo::Body::Body() :
  m_time_prev(0.0),
  m_time_to_target(0.0)
{
}
Probo::Body::~Body()
{
  /* Todo: Joint破壊していく。 */
}
int Probo::Body::add_controller(Controller * ct)
{
  if (ct == NULL) return EA1_EINVAL;
  if ((ct->get_body() != NULL) && (ct->get_body() != this))
    {
      LOGE("This controller belongs to another body");
      return EA1_EINVAL;
    }
  /* 既に登録されてるかチェックして、あれば終了。 */
  for (auto itr = m_controllers.cbegin(); itr != m_controllers.cend(); ++itr)
    {
      if (*itr == ct)
        {
           LOGD("ct already exists, n = %d", (int)m_controllers.size());
          return EA1_OK;
        }
    }
  if (int rc = ct->set_body(this) != EA1_OK) return rc;
  m_controllers.push_back(ct);
  LOGD("added ct, n = %d", (int)m_controllers.size());
  return EA1_OK;
}

int Probo::Controller::set_body(Body * body)
{
  if (body == NULL) return EA1_EINVAL;
  if ((m_body != NULL) && (m_body != body))
    {
      LOGE("Already belongs to another body");
      return EA1_EINVAL;
    }
  m_body = body;
  return EA1_OK;
}

int main(int argc, char *argv[])
{
  LOGI("starts");
  //std::vector <int> v;
  Probo::Body * pr1 = new Probo::Body();
  Probo::Controller * ct1 = new Probo::Controller();
  pr1->add_controller(ct1);
  pr1->add_controller(ct1);
  LOGI("controller amount %d", pr1->get_controller_amt());
  /* destruct */
  EA1_SAFE_DELETE(ct1);
  EA1_SAFE_DELETE(pr1);
  /* test pfamily */
  Pfamily::test_main(argc,argv);
  return 0;
}
