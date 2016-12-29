/* pmpu6050.cpp    -*- C++ -*-
   probo mpu6050 gyro
   2016-12-19 18:04:43 Sgurn6i
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <kdl/frames.hpp>
#include "MPU6050-Pi/helper_3dmath.h"
#include "MPU6050-Pi/I2Cdev.h"
#include "MPU6050-Pi/MPU6050_6Axis_MotionApps20.h"
#include "ea1/ea1_debug.h"
#include "ea1/ea1_benri.h"
#include "ea1/ea1_time.h"
#include "pmpu6050.hpp"
#define LOG_TAG "pmpu6050"
#define IMPL (*(this->impl))

class probo::Pmpu6050::Impl
{
public:
  std::unique_ptr<MPU6050> mpu;
  bool dmp_ready = false;
  int16_t packet_size;
  uint8_t fifo_buffer[64];
  bool fifo_buffer_ready = false;
  /* fifo リセットされた直後の呼び出し時に、get_curr_quat()に知らせる仕組み。 */
  bool first_get_curr_quat = true;
  /* reset_sense時のRotation */
  KDL::Rotation rot_0;
};

pfamily::Child * probo::Pmpu6050Builder::create_child(pfamily::Parent& parent,
                                                      const std::string& name)
{
  Body * b = get_body(parent);
  if (b)
    return new probo::Pmpu6050(*b, parent.get_children_amt(), name);
  else
    return NULL;
}


probo::Pmpu6050::Pmpu6050(Body& body, int sn, const std::string& name)
  : probo::Gyro(body, sn, name),
    impl(new Impl())
{
}

probo::Pmpu6050::~Pmpu6050(){ }

bool probo::Pmpu6050::is_dmp_ready() const {
  return IMPL.dmp_ready;
}

int probo::Pmpu6050::init( const std::string& device, int i2c_addr )
{
  int rc = EA1_OK;
  IMPL.dmp_ready = false;
  I2Cdev::dev_path = device.c_str();
  LOGD( "%s: new MPU6050(%d) device %s", __func__, i2c_addr, device.c_str());
  IMPL.mpu.reset(new MPU6050(i2c_addr));
  IMPL.fifo_buffer_ready = false;
  uint8_t devStatus;
  
  if (IMPL.mpu == NULL)
    {
      LOGE( "%s: failed to create MPU6050", __func__);
      return EA1_OUTOF_MEMORY;
    }
  if (! IMPL.mpu->testConnection())
    {
      LOGE( "%s: MPU6050 testConnection failed", __func__);
      return EA1_FAIL;
    }
  devStatus = IMPL.mpu->dmpInitialize();
  if (devStatus != 0)
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      LOGE( "%s: MPU6050 dmpInitialize failed, (code %d)", __func__, devStatus);
      return EA1_FAIL;
    }
  LOGD("Enabling DMP...");
  IMPL.mpu->setDMPEnabled(true);
  LOGD("mpu->getIntStatus %u", IMPL.mpu->getIntStatus());
  IMPL.dmp_ready = true;
  IMPL.first_get_curr_quat = true;
  IMPL.packet_size = IMPL.mpu->dmpGetFIFOPacketSize();
  /* reset sense data */
  rc = reset_sense(ea1_gettimeofday_ms());
  
  return rc;
}

#define FIFO_COUNT_READY        42
#define FIFO_COUNT_OVERFLOW     1024

int probo::Pmpu6050::reset_sense( double time )
{
  int rc = EA1_OK;
  if (! is_dmp_ready())
    {
      LOGE("%s: MPU6050 DMP is not ready.", __func__);
      return EA1_E_NOT_READY;
    }
  /* resetFIFOでは数値はリセットされない */
  //IMPL.mpu->resetFIFO();
  /* fifo 溜まるまでsenseする。 */
  for (int ix = 0; ix < 1000; ix ++)
    {
      int rc_sense = sense(time);
      if (rc_sense < 0)
        return rc_sense;
      if ((rc_sense == EA1_OK) && IMPL.fifo_buffer_ready)
        break;
      usleep(2 * 1000);
    }
  if (! IMPL.fifo_buffer_ready)
    {
      LOGE("%s: could not read MPU6050 DMP FIFO.", __func__);
      return EA1_E_NOT_READY;
    }
  /* 現状 rotation */
  Quaternion q;
  IMPL.mpu->dmpGetQuaternion(&q, IMPL.fifo_buffer);
  KDL::Rotation rot_curr = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
  /* 現状のZ成分(yaw)回転を rot_0 にする。 */
  double roll=0, pitch=0, yaw=0;
  rot_curr.GetRPY(roll, pitch, yaw);
  IMPL.rot_0 = KDL::Rotation::RotZ(yaw);
  /* reset first */
  IMPL.first_get_curr_quat = true;
  return rc;
}

int probo::Pmpu6050::sense( double time )
{
  if (! is_dmp_ready())
    {
      LOGE("%s: MPU6050 DMP is not ready.", __func__);
      return EA1_E_NOT_READY;
    }
  int rc = EA1_OK;
  uint16_t fifo_count;
  //IMPL.mpu->resetDMP(); // kese
  /* fifo読めれば fifo_buffer_ready を trueにする。 */
  fifo_count = IMPL.mpu->getFIFOCount();
  LOGD("%s: MPU6050 FIFO count %d", __func__, fifo_count);
  if (fifo_count >= FIFO_COUNT_OVERFLOW)
    {
      LOGI("%s: MPU6050 FIFO overflow", __func__);
      IMPL.mpu->resetFIFO();
      IMPL.fifo_buffer_ready = false;
      rc = EA1_STATUS1;
    }
  else if (fifo_count < FIFO_COUNT_READY)
    {
      LOGD("%s: MPU6050 FIFO not enough count %d",
           __func__, fifo_count);
      rc = EA1_STATUS1;
    }
  else
    {
      /* FIFO が程よく減るまで取り出し続ける。 */
      for (int ix = 0; ix < FIFO_COUNT_OVERFLOW / FIFO_COUNT_READY + 1; ix ++)
        {
          IMPL.mpu->getFIFOBytes(IMPL.fifo_buffer, IMPL.packet_size);
          fifo_count = IMPL.mpu->getFIFOCount();
          if (fifo_count <= FIFO_COUNT_READY * 2)
            break;
        }
      IMPL.fifo_buffer_ready = true;
    }
  
  return rc;
}

int probo::Pmpu6050::get_curr_quat(double * out_x,
                                   double * out_y,
                                   double * out_z,
                                   double * out_w)
{
  if (IMPL.fifo_buffer_ready)
    {
      Quaternion q;
      int rc;
      IMPL.mpu->dmpGetQuaternion(&q, IMPL.fifo_buffer);
      KDL::Rotation rot_raw
        = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
      /* R0^(-1) * Rraw */
      KDL::Rotation rot_d = IMPL.rot_0.Inverse() * rot_raw;
      rot_d.GetQuaternion(*out_x, *out_y, *out_z, *out_w);
      rc = IMPL.first_get_curr_quat ? EA1_STATUS1 : EA1_OK;
      IMPL.first_get_curr_quat = false;
      return rc;
    }
  else
    {
      LOGE("%s: MPU6050 DMP FIFO buffer is not ready.", __func__);
      return EA1_E_NOT_READY;
    }
}
