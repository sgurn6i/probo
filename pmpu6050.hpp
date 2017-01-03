/* pmpu6050.hpp    -*- C++ -*-
   probo mpu6050 gyro
   2016-12-17 17:17:12 Sgurn6i
*/
#ifndef _PMPU6050_H_
#define _PMPU6050_H_
#include <memory>
#include "pgyro.hpp"

namespace probo
{
  class Pmpu6050 : public Gyro
  {
  public:
    Pmpu6050(const std::string& name ="Pmpu6050");
    virtual ~Pmpu6050();
    int init( const std::string& device = "/dev/i2c-2", int i2c_addr = 0x68 );
    /* Sensor継承 */
    /* sense 実行。
       DMPが初期化されていない等のエラーなら負数を返す。
       FIFO oerflowしてたら EA1_STATUS1 を返す。
       FIFO データが不十分だったら EA1_STATUS1 を返す。
       うまく sense できたら EA1_OK を返す。*/
    virtual int sense( double time );
    virtual int reset_sense( double time );
    /* PGyro継承 */
    virtual int get_curr_quat(double * out_x,
                              double * out_y,
                              double * out_z,
                              double * out_w);
    bool is_dmp_ready() const;
  private:
    class Impl; // hidden implementations
    std::unique_ptr<Impl> impl; 
  };
  
} /* namespace probo */

#endif /* _PMPU6050_H_ */
