#ifndef _ROS_std_msgs_UInt16_h
#define _ROS_std_msgs_UInt16_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace std_msgs
{

  class UInt16 : public ros::Msg
  {
    public:
      typedef uint16_t _data_type;
      _data_type data;

    UInt16():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->data =  ((uint16_t) (*(inbuffer + offset)));
      this->data |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->data);
     return offset;
    }

    virtual const char * getType(){ return PSTR( "std_msgs/UInt16" ); };
    virtual const char * getMD5(){ return PSTR( "1df79edf208b629fe6b81923a544552d" ); };

  };

}
#endif
