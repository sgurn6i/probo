/* pfamily.hpp    -*- C++ -*-
   base, parent, children階層
   2016-06-16 16:56:16 Sgurn6i
*/
#ifndef _PFAMILY_H_
#define _PFAMILY_H_
#include <string>
//#include <list>
#include <vector>
#include "ea1/ea1_debug.h"
#define LOG_TAG "pfamily"
namespace pfamily
{
  /* 共通 base。
   * dynanmic_cast される所の class として使う。 */
  class Base
  {
  public:
    virtual ~Base(){ }
  };
  
  /* 親子関係。
   * Child は Parent が作る。
   * Child は作成時は1つのParentを持つ。
   * Parentから引き離されて孤児になる事がある。destructの都合上。
   * 再び親を持つことは無い。
   * Parent は複数のChildを持てる。
   * Parentをdestructした時は傘下のChildはすべてdestructされる。
   * ChildをdestructしたらParentのリストから登録抹消される。 */
  class Parent;
  class Child;

  class Child : public Base
  {
  public:
    Child(Parent& parent, int sn, const std::string& name);
    virtual ~Child();
    int get_sn() const { return m_sn; }
    const std::string& get_name() const { return m_name; }
    Parent& get_parent() const { return *m_parent_p; }
  protected:
  private:
    const std::string m_name;
    int m_sn = -1;    /* 親から見た serial number  */
    Parent * const m_parent_p;
  };

  class Parent : public Base
  {
    friend Child::~Child();
  public:
    Parent(const std::string& name = "parent")
      : m_p_name(name)
    {LOGD("Parent(%s)", name.c_str()); }
    virtual ~Parent();
    const std::string& get_p_name() const { return m_p_name; }
    int get_children_amt(){ return (int)m_children.size(); } 
    Child * get_child(int sn);/* sn 番目の子を返す。無ければNULL返す。 */
    bool has_child(Child * cp); /* その子はあんたの子か。 */
  protected:
    virtual int add(Child& c);
    int remove_child(Child * cp); /* リストからcp削除。 */
  private:
    const std::string m_p_name = "";
    std::vector <Child *> m_children;
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
      : Parent(name), Child(parent, sn, name)
    { }
  protected:
  private:
  };
  /* test function */
  int test_main(int argc, char *argv[]);
} /* namespace pfamily */
#undef LOG_TAG
#endif /* _PFAMILY_H_ */
