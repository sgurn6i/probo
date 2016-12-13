/* pfamily.cpp    -*- C++ -*-
   base, parent, children階層。child独立型。
   2016-12-12 16:16:52 Sgurn6i
*/
#include "pfamily.hpp"
using pfamily::Named;
using pfamily::Base;
using pfamily::Child;
using pfamily::ChildBuilder;
using pfamily::Parent;
using pfamily::TestBody;
using pfamily::TestController;
using pfamily::TestControllerBuilder;
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#define LOG_TAG "pfamily"

Parent::Parent()
{
  LOGD("Parent()");
}

Parent::~Parent()
{
  LOGD("~Parent()");
  /* delete children */
  while (! m_children.empty())
    {
      Child * cp = m_children.back();
      //remove_child(cp); /* 戸籍抹消 */
      EA1_SAFE_DELETE(cp); /* 子殺し。 */ 
    }
}

Child * Parent::create_child(ChildBuilder& builder,
                             const std::string& name )
{
  Child * cp = builder.create_child(*this, name);
  if (cp == NULL) return NULL;
  int rc = add_child(*cp);
  if (rc != EA1_OK)
    {
      LOGE("%s: failed to add( %s )", __func__, name.c_str());
      remove_child(cp);
      return NULL;
    }
  return cp;
}

int Parent::add_child(Child& c)
{
  if (&c.get_parent() != this)
    {
      LOGE("%s: %s not assumes me as the parent", __func__, c.get_name().c_str());
      return EA1_EINVAL;
    }
  if (! has_child(&c))
    {
      m_children.push_back(&c);
    }

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

Child * ChildBuilder::create_child(Parent& parent,
                                   const std::string& name)
{
  Child * cp = new Child(parent, parent.get_children_amt(), name);
  if (cp == NULL)
    {
      LOGE("%s: failed to create Child( %s )", __func__, name.c_str());
      return NULL;
    }
  return cp;
}

Child::Child(Parent& parent, int sn, const std::string& name)
  : Named(name),
    m_sn(parent.get_children_amt()),
    m_parent_p(&parent)
{
  LOGD("Child(%s) #%d", name.c_str(), m_sn);
}

Child::~Child()
{
  LOGD("~Child %s #%d", get_name().c_str(), m_sn);
  if(m_parent_p)
    m_parent_p->remove_child(this);
}

Child * TestBody::create_child(ChildBuilder& builder,
                               const std::string& name)
{
  TestControllerBuilder * tcb
    = dynamic_cast<TestControllerBuilder *>(&builder);
  if (tcb == NULL)
    {
      LOGE("%s: unknown builder", __func__);
      return NULL;
    }
  return Parent::create_child(builder, name);
}
TestController::TestController(TestBody& parent, int sn, const std::string& name )
  : Parent(), Child(parent, sn, name)
{ }

Child * TestControllerBuilder::create_child(Parent& parent,
                                       const std::string& name)
{
  TestBody * tbp = dynamic_cast<TestBody *>(&parent);
  if (tbp == NULL)
    {
      LOGE("%s: unknown parent", __func__);
      return NULL;
    }
  TestController * cp
    = new TestController(*tbp, parent.get_children_amt(), name);
  if (cp == NULL)
    {
      LOGE("%s: failed to create Child( %s )", __func__, name.c_str());
      return NULL;
    }
  return cp;
}

int pfamily::test_main(int argc, char *argv[])
{
  TestControllerBuilder tcb;
  //ChildBuilder cb;
  TestBody * fbd1 = new TestBody("family_body1");
  TestController * fct_a
    = dynamic_cast<TestController *>(fbd1->create_child(tcb, "fct_a"));
  if (fct_a == NULL)
    {
      LOGE("%s: failed to create fct_a", __func__);
      return EA1_FAIL;
    }
  TestController * fct_b
    = dynamic_cast<TestController *>(fbd1->create_child(tcb, "fct_b"));
  if (fct_b == NULL)
    {
      LOGE("%s: failed to create fct_b", __func__);
      return EA1_FAIL;
    }
  
  TestController * fct_c = fbd1->create_controller(tcb, "fct_c");
  if (fct_c == NULL)
    {
      LOGE("%s: failed to create fct_c", __func__);
      return EA1_FAIL;
    }
  
  EA1_SAFE_DELETE(fbd1);
  return EA1_OK;
}
