/* pbbbgpio.hpp    -*- C++ -*-
   BeagleBone Black GPIO
   2017-03-11 17:43:15 Sgurn6i
*/
#ifndef _PBBBGPIO_H_
#define _PBBBGPIO_H_
#include <memory> /* unique_ptr */
#include "pgpio.hpp"

namespace probo
{
  /* BBB GPIO input */
  class BbbGpioin : public Gpioin
  {
  public:
    BbbGpioin(const std::string& name ="BbbGpioin");
    virtual ~BbbGpioin();
    /* GPIO初期化。
       この関数による初期化成功後にGPIOが使えるようになる。
       gpio_num : BBB GPIO 番号。
       返り値 : 成功すれば EA1_OK を返す。 */
    int init(unsigned int gpio_num);
    /*- Sensor継承 -*/
    virtual int sense( double time );
    virtual int reset_sense( double time );
    /*- Gpioin機能 -*/
    virtual int get_val();
  private:
    class Impl; // hidden implementations
    std::unique_ptr<Impl> impl; 
  };

  /* Todo: BBB GPIO output */
} /* namespace probo */

#endif /* _PBBBGPIO_H_ */
