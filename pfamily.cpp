/* pfamily.cpp    -*- C++ -*-
   parent, children階層
   2016-06-17 16:22:46 Sgurn6i
*/
#include "pfamily.hpp"
using Pfamily::Child;
using Pfamily::Parent;
using Pfamily::TestBody;
using Pfamily::TestController;
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#define LOG_TAG "pfamily"

Parent::~Parent()
{
  LOGD("~Parent(%s)", get_p_name().c_str());
  /* delete children */
  while (! m_children.empty())
    {
      Child * cp = m_children.front();
      remove_child(cp); /* 無限ループ回避 */
      EA1_SAFE_DELETE(cp); /* ここでも remove_child()呼ばれるけど、
                            * 既に消してるものを消さないだけ。 */ 
    }
}

int Parent::add(Child& c)
{
  m_children.push_back(&c);
  return EA1_OK;
}

int Parent::remove_child(Child * cp)
{
  if (cp == NULL) return EA1_EINVAL;
  if (&cp->get_parent() != this)
    {
      LOGE("%s: %s is not my child", __func__, cp->get_name().c_str());
      return EA1_EINVAL;
    }
  /* m_children から一致する要素を削除。 */
  int s1 =  m_children.size();
  m_children.remove(cp);
  int s2 =  m_children.size();
  LOGD("%s removed %d elements", __func__, s1 - s2);
  return EA1_OK;
}

TestController * TestBody::create_controller(const std::string& name)
{
  TestController * cp = new TestController(*this, get_next_sn(), name);
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

int Pfamily::test_main(int argc, char *argv[])
{
  TestBody * fbd1 = new TestBody("family_body1");
  TestBody * fbd2 = new TestBody("family_body2");
  TestController * fct_a = fbd1->create_controller("fct_a");
  TestController * fct_b = fbd1->create_controller("fct_baa");
  LOGI("Faily controller %s, parent %s",
       fct_a->get_name().c_str(), fct_a->get_parent().get_p_name().c_str());
  //delete fct_a;
  EA1_SAFE_DELETE(fbd1);
  EA1_SAFE_DELETE(fbd2);
  return 0;
}
