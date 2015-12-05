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
 * Auto-generated by genmsg_cpp from file /home/yzheng/catkin_ws/src/rbx2/rbx2_utils/srv/KillProcess.srv
 *
 */


#ifndef RBX2_UTILS_MESSAGE_KILLPROCESSREQUEST_H
#define RBX2_UTILS_MESSAGE_KILLPROCESSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rbx2_utils
{
template <class ContainerAllocator>
struct KillProcessRequest_
{
  typedef KillProcessRequest_<ContainerAllocator> Type;

  KillProcessRequest_()
    : process_id()  {
    }
  KillProcessRequest_(const ContainerAllocator& _alloc)
    : process_id(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _process_id_type;
  _process_id_type process_id;




  typedef boost::shared_ptr< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct KillProcessRequest_

typedef ::rbx2_utils::KillProcessRequest_<std::allocator<void> > KillProcessRequest;

typedef boost::shared_ptr< ::rbx2_utils::KillProcessRequest > KillProcessRequestPtr;
typedef boost::shared_ptr< ::rbx2_utils::KillProcessRequest const> KillProcessRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rbx2_utils::KillProcessRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rbx2_utils

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a19d2ed2d6b55934bac7cd67cfed8983";
  }

  static const char* value(const ::rbx2_utils::KillProcessRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa19d2ed2d6b55934ULL;
  static const uint64_t static_value2 = 0xbac7cd67cfed8983ULL;
};

template<class ContainerAllocator>
struct DataType< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rbx2_utils/KillProcessRequest";
  }

  static const char* value(const ::rbx2_utils::KillProcessRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string process_id\n\
";
  }

  static const char* value(const ::rbx2_utils::KillProcessRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.process_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct KillProcessRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rbx2_utils::KillProcessRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rbx2_utils::KillProcessRequest_<ContainerAllocator>& v)
  {
    s << indent << "process_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.process_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RBX2_UTILS_MESSAGE_KILLPROCESSREQUEST_H