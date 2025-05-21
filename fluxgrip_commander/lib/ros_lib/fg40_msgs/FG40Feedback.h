#ifndef _ROS_fg40_msgs_FG40Feedback_h
#define _ROS_fg40_msgs_FG40Feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fg40_msgs
{

  class FG40Feedback : public ros::Msg
  {
    public:
      typedef bool _magnetized_type;
      _magnetized_type magnetized;
      typedef int8_t _remagnetization_state_type;
      _remagnetization_state_type remagnetization_state;
      uint32_t cycles_on_off[2];

    FG40Feedback():
      magnetized(0),
      remagnetization_state(0),
      cycles_on_off()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_magnetized;
      u_magnetized.real = this->magnetized;
      *(outbuffer + offset + 0) = (u_magnetized.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->magnetized);
      union {
        int8_t real;
        uint8_t base;
      } u_remagnetization_state;
      u_remagnetization_state.real = this->remagnetization_state;
      *(outbuffer + offset + 0) = (u_remagnetization_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->remagnetization_state);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->cycles_on_off[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cycles_on_off[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cycles_on_off[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cycles_on_off[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cycles_on_off[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_magnetized;
      u_magnetized.base = 0;
      u_magnetized.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->magnetized = u_magnetized.real;
      offset += sizeof(this->magnetized);
      union {
        int8_t real;
        uint8_t base;
      } u_remagnetization_state;
      u_remagnetization_state.base = 0;
      u_remagnetization_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->remagnetization_state = u_remagnetization_state.real;
      offset += sizeof(this->remagnetization_state);
      for( uint32_t i = 0; i < 2; i++){
      this->cycles_on_off[i] =  ((uint32_t) (*(inbuffer + offset)));
      this->cycles_on_off[i] |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cycles_on_off[i] |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->cycles_on_off[i] |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->cycles_on_off[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "fg40_msgs/FG40Feedback"; };
    virtual const char * getMD5() override { return "da5e53218774a2b015da5bccc056070b"; };

  };

}
#endif
