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
      //remove_child(cp); /* 戸籍抹消 */
      EA1_SAFE_DELETE(cp); /* 子殺し。 */ 
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
  /* m_children から一致する要素を全部削除。 */
  int ser = 0;
  auto itr = m_children.begin();
  while (itr != m_children.end())
    {
      if (*itr == cp)
        {
          itr = m_children.erase(itr);
            LOGD("%s removed [%d]", __func__, ser);
        }
      else
        itr ++;
      //LOGD("%s ser %d", __func__, ser);
      ser ++;
    }
  return EA1_OK;
}

Child::Child(Parent& parent, int sn, const std::string& name)
  : m_name(name), m_sn(parent.get_children_amt()), m_parent_p(&parent)
{
  LOGD("Child(%s) #%d", name.c_str(), m_sn);
}

Child::~Child(){
  LOGD("~Child %s #%d", m_name.c_str(), m_sn);
    m_parent_p->remove_child(this);
}

/* sn 番目の子を返す。無ければNULL返す。 */
Child * Parent::get_child(int sn)
{
  if (sn >= (int)m_children.size())
    return NULL;
  return m_children[sn];
}

bool Parent::has_child(Child * cp)
{
  for (auto itr = m_children.begin(); itr != m_children.end(); itr ++)
    {
      if (*itr == cp)
        return true;
    }
  return false;
}

TestController * TestBody::create_controller(const std::string& name)
{
  TestController * cp = new TestController(*this, get_children_amt(), name);
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
  TestController * fct_a = fbd1->create_controller("fct1_a");
  TestController * fct_b = fbd1->create_controller("fct1_b");
  LOGI("Faily controller %s, parent %s",
       fct_a->get_name().c_str(), fct_a->get_parent().get_p_name().c_str());
  EA1_SAFE_DELETE(fct_b);
  //delete fct_a;
  EA1_SAFE_DELETE(fbd1);
  EA1_SAFE_DELETE(fbd2);
  return 0;
}
