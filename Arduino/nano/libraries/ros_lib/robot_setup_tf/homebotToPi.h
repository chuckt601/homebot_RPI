#ifndef _ROS_robot_setup_tf_homebotToPi_h
#define _ROS_robot_setup_tf_homebotToPi_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_setup_tf
{

  class homebotToPi : public ros::Msg
  {
    public:
      typedef uint32_t _arduino_micros_type;
      _arduino_micros_type arduino_micros;
      typedef float _integrated_X_type;
      _integrated_X_type integrated_X;
      typedef float _integrated_Y_type;
      _integrated_Y_type integrated_Y;
      typedef float _X_rate_type;
      _X_rate_type X_rate;
      typedef float _Y_rate_type;
      _Y_rate_type Y_rate;
      typedef float _yaw_IMU_type;
      _yaw_IMU_type yaw_IMU;
      typedef float _yaw_encoders_type;
      _yaw_encoders_type yaw_encoders;
      typedef float _yaw_rate_IMU_type;
      _yaw_rate_IMU_type yaw_rate_IMU;
      typedef float _yaw_rate_encoders_type;
      _yaw_rate_encoders_type yaw_rate_encoders;
      typedef float _diag1_type;
      _diag1_type diag1;
      typedef float _diag2_type;
      _diag2_type diag2;
      typedef float _diag3_type;
      _diag3_type diag3;

    homebotToPi():
      arduino_micros(0),
      integrated_X(0),
      integrated_Y(0),
      X_rate(0),
      Y_rate(0),
      yaw_IMU(0),
      yaw_encoders(0),
      yaw_rate_IMU(0),
      yaw_rate_encoders(0),
      diag1(0),
      diag2(0),
      diag3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->arduino_micros >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->arduino_micros >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->arduino_micros >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->arduino_micros >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arduino_micros);
      union {
        float real;
        uint32_t base;
      } u_integrated_X;
      u_integrated_X.real = this->integrated_X;
      *(outbuffer + offset + 0) = (u_integrated_X.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_integrated_X.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_integrated_X.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_integrated_X.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->integrated_X);
      union {
        float real;
        uint32_t base;
      } u_integrated_Y;
      u_integrated_Y.real = this->integrated_Y;
      *(outbuffer + offset + 0) = (u_integrated_Y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_integrated_Y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_integrated_Y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_integrated_Y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->integrated_Y);
      union {
        float real;
        uint32_t base;
      } u_X_rate;
      u_X_rate.real = this->X_rate;
      *(outbuffer + offset + 0) = (u_X_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_X_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_X_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_X_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->X_rate);
      union {
        float real;
        uint32_t base;
      } u_Y_rate;
      u_Y_rate.real = this->Y_rate;
      *(outbuffer + offset + 0) = (u_Y_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Y_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Y_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Y_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Y_rate);
      union {
        float real;
        uint32_t base;
      } u_yaw_IMU;
      u_yaw_IMU.real = this->yaw_IMU;
      *(outbuffer + offset + 0) = (u_yaw_IMU.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_IMU.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_IMU.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_IMU.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_IMU);
      union {
        float real;
        uint32_t base;
      } u_yaw_encoders;
      u_yaw_encoders.real = this->yaw_encoders;
      *(outbuffer + offset + 0) = (u_yaw_encoders.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_encoders.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_encoders.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_encoders.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_encoders);
      union {
        float real;
        uint32_t base;
      } u_yaw_rate_IMU;
      u_yaw_rate_IMU.real = this->yaw_rate_IMU;
      *(outbuffer + offset + 0) = (u_yaw_rate_IMU.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_rate_IMU.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_rate_IMU.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_rate_IMU.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_rate_IMU);
      union {
        float real;
        uint32_t base;
      } u_yaw_rate_encoders;
      u_yaw_rate_encoders.real = this->yaw_rate_encoders;
      *(outbuffer + offset + 0) = (u_yaw_rate_encoders.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_rate_encoders.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_rate_encoders.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_rate_encoders.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_rate_encoders);
      union {
        float real;
        uint32_t base;
      } u_diag1;
      u_diag1.real = this->diag1;
      *(outbuffer + offset + 0) = (u_diag1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_diag1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_diag1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_diag1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->diag1);
      union {
        float real;
        uint32_t base;
      } u_diag2;
      u_diag2.real = this->diag2;
      *(outbuffer + offset + 0) = (u_diag2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_diag2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_diag2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_diag2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->diag2);
      union {
        float real;
        uint32_t base;
      } u_diag3;
      u_diag3.real = this->diag3;
      *(outbuffer + offset + 0) = (u_diag3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_diag3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_diag3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_diag3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->diag3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->arduino_micros =  ((uint32_t) (*(inbuffer + offset)));
      this->arduino_micros |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arduino_micros |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->arduino_micros |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->arduino_micros);
      union {
        float real;
        uint32_t base;
      } u_integrated_X;
      u_integrated_X.base = 0;
      u_integrated_X.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_integrated_X.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_integrated_X.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_integrated_X.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->integrated_X = u_integrated_X.real;
      offset += sizeof(this->integrated_X);
      union {
        float real;
        uint32_t base;
      } u_integrated_Y;
      u_integrated_Y.base = 0;
      u_integrated_Y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_integrated_Y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_integrated_Y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_integrated_Y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->integrated_Y = u_integrated_Y.real;
      offset += sizeof(this->integrated_Y);
      union {
        float real;
        uint32_t base;
      } u_X_rate;
      u_X_rate.base = 0;
      u_X_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_X_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_X_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_X_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->X_rate = u_X_rate.real;
      offset += sizeof(this->X_rate);
      union {
        float real;
        uint32_t base;
      } u_Y_rate;
      u_Y_rate.base = 0;
      u_Y_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Y_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Y_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Y_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Y_rate = u_Y_rate.real;
      offset += sizeof(this->Y_rate);
      union {
        float real;
        uint32_t base;
      } u_yaw_IMU;
      u_yaw_IMU.base = 0;
      u_yaw_IMU.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_IMU.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_IMU.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_IMU.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_IMU = u_yaw_IMU.real;
      offset += sizeof(this->yaw_IMU);
      union {
        float real;
        uint32_t base;
      } u_yaw_encoders;
      u_yaw_encoders.base = 0;
      u_yaw_encoders.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_encoders.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_encoders.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_encoders.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_encoders = u_yaw_encoders.real;
      offset += sizeof(this->yaw_encoders);
      union {
        float real;
        uint32_t base;
      } u_yaw_rate_IMU;
      u_yaw_rate_IMU.base = 0;
      u_yaw_rate_IMU.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_rate_IMU.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_rate_IMU.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_rate_IMU.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_rate_IMU = u_yaw_rate_IMU.real;
      offset += sizeof(this->yaw_rate_IMU);
      union {
        float real;
        uint32_t base;
      } u_yaw_rate_encoders;
      u_yaw_rate_encoders.base = 0;
      u_yaw_rate_encoders.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_rate_encoders.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_rate_encoders.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_rate_encoders.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_rate_encoders = u_yaw_rate_encoders.real;
      offset += sizeof(this->yaw_rate_encoders);
      union {
        float real;
        uint32_t base;
      } u_diag1;
      u_diag1.base = 0;
      u_diag1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_diag1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_diag1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_diag1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->diag1 = u_diag1.real;
      offset += sizeof(this->diag1);
      union {
        float real;
        uint32_t base;
      } u_diag2;
      u_diag2.base = 0;
      u_diag2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_diag2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_diag2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_diag2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->diag2 = u_diag2.real;
      offset += sizeof(this->diag2);
      union {
        float real;
        uint32_t base;
      } u_diag3;
      u_diag3.base = 0;
      u_diag3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_diag3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_diag3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_diag3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->diag3 = u_diag3.real;
      offset += sizeof(this->diag3);
     return offset;
    }

    const char * getType(){ return "robot_setup_tf/homebotToPi"; };
    const char * getMD5(){ return "b5b22ee234771afe1c3cd055948ca773"; };

  };

}
#endif
