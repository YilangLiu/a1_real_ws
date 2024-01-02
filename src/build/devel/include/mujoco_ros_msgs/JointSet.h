// Generated by gencpp from file mujoco_ros_msgs/JointSet.msg
// DO NOT EDIT!


#ifndef MUJOCO_ROS_MSGS_MESSAGE_JOINTSET_H
#define MUJOCO_ROS_MSGS_MESSAGE_JOINTSET_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace mujoco_ros_msgs
{
template <class ContainerAllocator>
struct JointSet_
{
  typedef JointSet_<ContainerAllocator> Type;

  JointSet_()
    : header()
    , time(0.0)
    , MODE(0)
    , position()
    , torque()  {
    }
  JointSet_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time(0.0)
    , MODE(0)
    , position(_alloc)
    , torque(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _time_type;
  _time_type time;

   typedef int32_t _MODE_type;
  _MODE_type MODE;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _position_type;
  _position_type position;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _torque_type;
  _torque_type torque;





  typedef boost::shared_ptr< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> const> ConstPtr;

}; // struct JointSet_

typedef ::mujoco_ros_msgs::JointSet_<std::allocator<void> > JointSet;

typedef boost::shared_ptr< ::mujoco_ros_msgs::JointSet > JointSetPtr;
typedef boost::shared_ptr< ::mujoco_ros_msgs::JointSet const> JointSetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mujoco_ros_msgs::JointSet_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mujoco_ros_msgs::JointSet_<ContainerAllocator1> & lhs, const ::mujoco_ros_msgs::JointSet_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.time == rhs.time &&
    lhs.MODE == rhs.MODE &&
    lhs.position == rhs.position &&
    lhs.torque == rhs.torque;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mujoco_ros_msgs::JointSet_<ContainerAllocator1> & lhs, const ::mujoco_ros_msgs::JointSet_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mujoco_ros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e22f2239d2031a20304e67345beffc60";
  }

  static const char* value(const ::mujoco_ros_msgs::JointSet_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe22f2239d2031a20ULL;
  static const uint64_t static_value2 = 0x304e67345beffc60ULL;
};

template<class ContainerAllocator>
struct DataType< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mujoco_ros_msgs/JointSet";
  }

  static const char* value(const ::mujoco_ros_msgs::JointSet_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float64 time\n"
"\n"
"\n"
"int32 MODE\n"
"float64[] position\n"
"float64[] torque\n"
"\n"
"# MODE 0 = POSITION\n"
"# MODE 1 = TORQUE\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::mujoco_ros_msgs::JointSet_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time);
      stream.next(m.MODE);
      stream.next(m.position);
      stream.next(m.torque);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointSet_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mujoco_ros_msgs::JointSet_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mujoco_ros_msgs::JointSet_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "MODE: ";
    Printer<int32_t>::stream(s, indent + "  ", v.MODE);
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "torque[]" << std::endl;
    for (size_t i = 0; i < v.torque.size(); ++i)
    {
      s << indent << "  torque[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.torque[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MUJOCO_ROS_MSGS_MESSAGE_JOINTSET_H