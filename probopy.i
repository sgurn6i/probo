/* probopy.i -*- C++ -*-
 * python interface SWIG定義
 * 2016-11-08 15:31:29 Sgurn6i
 */
%module probopy
%{
  #include "ea1/ea1_benri.h"
  #include "pfamily.hpp"
  #include "probo.hpp"
  #include "probopwm.hpp"
  #include "ppca9685.hpp"
  #include "pgyro.hpp"
  #include "pmpu6050.hpp"
%}
%include <std_string.i>
%include "ea1/ea1_benri.h"

namespace pfamily
{
  class Base
  {
  public:
    virtual ~Base();
  };

  class Named
  {
  public:
    Named(const std::string& name = "");
    virtual ~Named();
    virtual const std::string& get_name() const;
  };
  
  class Parent;
  class ChildBuilder;
  class Child : virtual public Base, virtual public Named
  {
    friend class ChildBuilder;
  public:
    int get_sn() const;
    Parent& get_parent() const;
    virtual ~Child();
  protected:
    Child(Parent& parent, int sn=-1, const std::string& name="_child");
  };

  /* child を作って渡すBuilderクラス */
  class ChildBuilder
  {
  public:
    virtual ~ChildBuilder();
    virtual Child * create_child(Parent& parent,
                                 const std::string& name = "b_child" );
  };

  class Parent : virtual public Base
  {
    friend Child::~Child();
  public:
    Parent();
    virtual ~Parent();
    virtual Child * create_child(ChildBuilder& builder,
                                 const std::string& name = "p_child" );
    int get_children_amt() const;
    Child * get_child(int sn);
    bool has_child(Child * cp);
  protected:
    virtual int add_child(Child& c);
    int remove_child(Child * cp);
  };
}

%feature("notabstract") Pca9685;
//%include "pfamily.hpp"    // test_main重複
%include "probo.hpp"
%include "probopwm.hpp"
%include "ppca9685.hpp"
%include "pgyro.hpp"
%include "pmpu6050.hpp"
