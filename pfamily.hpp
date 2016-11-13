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
    virtual const std::string& get_name() const { return m_name; }
  protected:
    virtual void set_name(const std::string& name){ m_name = name; }
  private:
    std::string m_name = "";
  };
  
  /* 親子関係。
   * Child は Parent が create_child() で作る。適宜オーバーライドして使う。
   * Child は作成時は1つのParentを持つ。
   * Parentから引き離されて孤児になる事がある。destructの都合上。
   * 再び親を持つことは無い。
   * Parent は複数のChildを持てる。
   * Parentをdestructした時は傘下のChildはすべてdestructされる。
   * ChildをdestructしたらParentのリストから登録抹消される。 */
  class Parent;
  class Child;

  class Child : virtual public Base
  {
    friend class Parent;
  public:
    int get_sn() const { return m_sn; }
    Parent& get_parent() const { return *m_parent_p; }
  protected:
    Child(Parent& parent, int sn, const std::string& name);
    virtual ~Child();
  private:
    int m_sn = -1;    /* 親から見た serial number  */
    Parent * const m_parent_p;
  };

  class Parent : virtual public Base
  {
    friend Child::~Child();
  public:
    Parent();
    Parent(const std::string& name);
    virtual ~Parent();
    virtual Child * create_child( const std::string& name = "p child" );
    int get_children_amt(){ return (int)m_children.size(); } 
    Child * get_child(int sn);/* sn 番目の子を返す。無ければNULL返す。 */
    bool has_child(Child * cp); /* その子はあんたの子か。 */
  protected:
    virtual int add(Child& c);
    int remove_child(Child * cp); /* リストからcp削除。 */
  private:
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
    virtual Child * create_child( const std::string& name  = "cct" );
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
