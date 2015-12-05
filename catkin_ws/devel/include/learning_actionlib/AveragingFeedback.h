/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/yzheng/catkin_ws/devel/share/learning_actionlib/msg/AveragingFeedback.msg
 *
 */


#ifndef LEARNING_ACTIONLIB_MESSAGE_AVERAGINGFEEDBACK_H
#define LEARNING_ACTIONLIB_MESSAGE_AVERAGINGFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace learning_actionlib
{
template <class ContainerAllocator>
struct AveragingFeedback_
{
  typedef AveragingFeedback_<ContainerAllocator> Type;

  AveragingFeedback_()
    : sample(0)
    , data(0.0)
    , mean(0.0)
    , std_dev(0.0)  {
    }
  AveragingFeedback_(const ContainerAllocator& _alloc)
    : sample(0)
    , data(0.0)
    , mean(0.0)
    , std_dev(0.0)  {
    }



   typedef int32_t _sample_type;
  _sample_type sample;

   typedef float _data_type;
  _data_type data;

   typedef float _mean_type;
  _mean_type mean;

   typedef float _std_dev_type;
  _std_dev_type std_dev;




  typedef boost::shared_ptr< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct AveragingFeedback_

typedef ::learning_actionlib::AveragingFeedback_<std::allocator<void> > AveragingFeedback;

typedef boost::shared_ptr< ::learning_actionlib::AveragingFeedback > AveragingFeedbackPtr;
typedef boost::shared_ptr< ::learning_actionlib::AveragingFeedback const> AveragingFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::learning_actionlib::AveragingFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace learning_actionlib

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'actionlib_msgs': ['/opt/ros/hydro/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg'], 'learning_actionlib': ['/home/yzheng/catkin_ws/devel/share/learning_actionlib/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9e8dfc53c2f2a032ca33fa80ec46fd4f";
  }

  static const char* value(const ::learning_actionlib::AveragingFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9e8dfc53c2f2a032ULL;
  static const uint64_t static_value2 = 0xca33fa80ec46fd4fULL;
};

template<class ContainerAllocator>
struct DataType< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "learning_actionlib/AveragingFeedback";
  }

  static const char* value(const ::learning_actionlib::AveragingFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback\n\
int32 sample\n\
float32 data\n\
float32 mean\n\
float32 std_dev\n\
\n\
";
  }

  static const char* value(const ::learning_actionlib::AveragingFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sample);
      stream.next(m.data);
      stream.next(m.mean);
      stream.next(m.std_dev);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct AveragingFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::learning_actionlib::AveragingFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::learning_actionlib::AveragingFeedback_<ContainerAllocator>& v)
  {
    s << indent << "sample: ";
    Printer<int32_t>::stream(s, indent + "  ", v.sample);
    s << indent << "data: ";
    Printer<float>::stream(s, indent + "  ", v.data);
    s << indent << "mean: ";
    Printer<float>::stream(s, indent + "  ", v.mean);
    s << indent << "std_dev: ";
    Printer<float>::stream(s, indent + "  ", v.std_dev);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LEARNING_ACTIONLIB_MESSAGE_AVERAGINGFEEDBACK_H