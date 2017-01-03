/* pgyro.hpp    -*- C++ -*-
   Gyro Sensor. Dummy implementation.
   2016-12-28 16:42:10 Sgurn6i
*/
#ifndef _PGYRO_H_
#define _PGYRO_H_

#include "probo.hpp"

namespace probo
{
  /* Gyroセンサひな形。 ゼロ回転しか返さないダミーGyroが実装される。  */
  class Gyro : public Sensor
  {
  public:
    Gyro(const std::string& name ="Gyro");
    virtual ~Gyro(){ }
    /* Sensor継承。 */
    virtual int sense( double time );
    /* 現在の姿勢をしてZ軸回りゼロ回転と表されるようにリセットする。 */
    virtual int reset_sense( double time );
    /* Gyro機能。 */
    /* 重力をZ軸下方向に見た座標系において、
       最近のsense()時の姿勢が
       どれだけ回転しているかを表す quaternion.
       out_x : ベクトルx成分。
       out_y : ベクトルy成分。
       out_z : ベクトルz成分。
       out_w : スカラー成分。  
       返り値:
       取得できなかったら負数を返す。
       reset_sense()後最初に呼ばれた場合に限り、EA1_STATUS1を返す。
       通常は EA1_OKを返す。  */
    virtual int get_curr_quat(double * out_x,
                              double * out_y,
                              double * out_z,
                              double * out_w);
  };

} /* namespace probo */

#endif /* _PGYRO_H_ */
