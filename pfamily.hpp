/* pfamily.hpp    -*- C++ -*-
   base, parent, children階層。child独立型。
   2016-12-12 15:57:07 Sgurn6i
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

  /* 名前ありの基底。 */
  class Named
  {
  public:
    Named(const std::string& name = ""){ m_name = name; }
    virtual ~Named(){ }
    virtual const std::string& get_name() const { return m_name; }
  private:
    std::string m_name = "";
  };
  
  /* 親子関係。
   * Child は Parent が create_child(builder) で作る。
   * ChildBuilderを適宜オーバーライドして使う。
   * Child は作成時は1つのParentを持つ。
   * Parentから引き離されて孤児になる事がある。destructの都合上。
   * 再び親を持つことは無い。
   * Parent は複数のChildを持てる。
   * Parentをdestructした時は傘下のChildはすべてdestructされる。
   * ChildをdestructしたらParentのリストから登録抹消される。 */
  class Parent;
  class ChildBuilder;
  class Child : virtual public Base, virtual public Named
  {
    //friend class Parent;
    friend class ChildBuilder;
  public:
    int get_sn() const { return m_sn; }
    Parent& get_parent() const { return *m_parent_p; }
    virtual ~Child();
  protected:
    Child(Parent& parent, int sn=-1, const std::string& name="_child");
  private:
    int m_sn = -1;    /* 親から見た serial number  */
    Parent * m_parent_p;
  };

  /* child を作って渡すBuilderクラス */
  class ChildBuilder
  {
  public:
    virtual ~ChildBuilder(){ }
    virtual Child * create_child(Parent& parent,
                                 const std::string& name = "b_child" );
  };

  class Parent : virtual public Base
  {
    friend Child::~Child();
  public:
    Parent();
    virtual ~Parent();
    /* ChildBuilderに子を作らせて登録する。 */
    virtual Child * create_child(ChildBuilder& builder,
                                 const std::string& name = "p_child" );
    /* 登録された子の数。 */
    int get_children_amt() const { return (int)m_children.size(); } 
    /* sn 番目の子を返す。無ければNULL返す。 */
    Child * get_child(int sn);
    /* その子はあんたの子か。 */
    bool has_child(Child * cp);
  protected:
    /* 子供追加。 */
    virtual int add_child(Child& c);
    /* リストからcp削除。 */
    int remove_child(Child * cp);
  private:
    std::vector <Child *> m_children;
  };

  /* for test */
  class TestBody;
  class TestControllerBuilder : public ChildBuilder
  {
  public:
    virtual ~TestControllerBuilder(){ }
    virtual Child * create_child(Parent& parent,
                                 const std::string& name = "b_child" );
  };
  class TestController : public Parent, public Child
  {
  public:
    TestController(TestBody& parent, int sn, const std::string& name = "TestController");
  };
  class TestBody : virtual public Named, public Parent
  {
  public:
    TestBody(const std::string& name = "fbody")
      : Named(name),Parent(){ }
    virtual ~TestBody(){ }
    /* builderの素性をチェックしてからcreateする。 */
    virtual Child * create_child(ChildBuilder& builder,
                                 const std::string& name = "tp_child" );
    /* TestController を create_childする。 */
    virtual TestController * create_controller(TestControllerBuilder& builder,
                                               const std::string& name = "tc" )
    { return dynamic_cast<TestController *>(create_child(builder,name)); }
  };
  
  /* test function */
  int test_main(int argc, char *argv[]);
} /* namespace pfamily */
#undef LOG_TAG

#endif /* _PFAMILY_H_ */
