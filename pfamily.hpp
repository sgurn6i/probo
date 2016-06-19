/* pfamily.hpp    -*- C++ -*-
   parent, children階層
   2016-06-16 16:56:16 Sgurn6i
*/
#ifndef _PFAMILY_H_
#define _PFAMILY_H_
#include <string>
#include <list>
#include "ea1/ea1_debug.h"
#define LOG_TAG "pfamily"
namespace Pfamily
{
  /* 親子関係。
   * Child は Parent が作る。
   * Child は必ず1つのParentを持つ。
   * Parent は複数のChildを持てる。
   * Parentをdestructした時は傘下のChildはすべてdestructされる。
   * ChildをdestructしたらParentのリストから登録抹消される。 */
  class Child;
  class Parent
  {
  public:
    Parent(const std::string& name = "p")
      : m_p_name(name)
    {LOGD("Parent(%s)", name.c_str()); }
    virtual ~Parent();
    const std::string& get_p_name() const { return m_p_name; }
    int get_next_sn(){ return (int)m_children.size(); } 
    int remove_child(Child * cp);
    Child * get_child(int sn); /* Todo: 実装。無ければNULL返す。 */
  protected:
    virtual int add(Child& c);
  private:
    const std::string m_p_name = "";
    std::list <Child *> m_children;
  };
  class Child
  {
  public:
    virtual ~Child(){
      LOGD("~Child %s #%d", m_name.c_str(), m_sn);
      m_parent_p->remove_child(this);
    }
    int get_sn() const { return m_sn; }
    const std::string& get_name() const { return m_name; }
    Parent& get_parent() const { return *m_parent_p; }
    Child(Parent& parent, int sn, const std::string& name)
      : m_name(name), m_sn(parent.get_next_sn()), m_parent_p(&parent)
    {
      LOGD("Child(%s) #%d", name.c_str(), m_sn);
    }
  protected:
  private:
    int m_sn = -1;    /* 親から見た serial number  */
    const std::string m_name = "";
    Parent * m_parent_p;
  };

  /* for test */
  class TestController;
  class TestBody : public Parent
  {
  public:
    TestBody(const std::string& name = "fbody")
      : Parent(name){}
    virtual ~TestBody(){ }
    virtual TestController * create_controller(const std::string& name = "ct");
  };
  
  class TestController : public Parent, public Child
  {
  public:
    TestController(TestBody& parent, int sn, const std::string& name = "TestController")
      : Child(parent, sn, name), Parent(name)
    { }
  protected:
  private:
  };
  /* test function */
  int test_main(int argc, char *argv[]);
} /* namespace Pfamily */
#undef LOG_TAG
#endif /* _PFAMILY_H_ */
