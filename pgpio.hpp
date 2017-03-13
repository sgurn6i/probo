/* pgpio.hpp    -*- C++ -*-
   GPIO入出力インターフェース
   2017-03-11 17:31:01 Sgurn6i
*/
#ifndef _PGPIO_H_
#define _PGPIO_H_

#include "probo.hpp"

namespace probo
{
  /* GPIO input */
  class Gpioin : public Sensor
  {
  public:
    Gpioin(const std::string& name ="Gpioin")
      : probo::Sensor(name){ }
    virtual ~Gpioin(){ }
    /*- Sensor継承。 -*/
    virtual int sense( double time ) = 0;
    virtual int reset_sense( double time ) = 0;
    /*- Gpioin機能 -*/
     /* 値取得。
       成功すれば0または1の直前sense値を返す。
       失敗すれば負数を返す。*/
    virtual int get_val() = 0;
  };

  /* Todo: GPIO output */
  
} /* namespace probo */

#endif /* _PGPIO_H_ */
