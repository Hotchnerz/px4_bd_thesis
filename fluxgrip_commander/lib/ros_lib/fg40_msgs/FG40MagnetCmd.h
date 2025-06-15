#ifndef _ROS_fg40_msgs_FG40MagnetCmd_h
#define _ROS_fg40_msgs_FG40MagnetCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fg40_msgs
{

  class FG40MagnetCmd : public ros::Msg
  {
    public:
      typedef int16_t _cmd_magnet_type;
      _cmd_magnet_type cmd_magnet;

    FG40MagnetCmd():
      cmd_magnet(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_cmd_magnet;
      u_cmd_magnet.real = this->cmd_magnet;
      *(outbuffer + offset + 0) = (u_cmd_magnet.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_magnet.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cmd_magnet);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_cmd_magnet;
      u_cmd_magnet.base = 0;
      u_cmd_magnet.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_magnet.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cmd_magnet = u_cmd_magnet.real;
      offset += sizeof(this->cmd_magnet);
     return offset;
    }

    virtual const char * getType() override { return "fg40_msgs/FG40MagnetCmd"; };
    virtual const char * getMD5() override { return "91be7fa5da69e99d147fbb330073b930"; };

  };

}
#endif
