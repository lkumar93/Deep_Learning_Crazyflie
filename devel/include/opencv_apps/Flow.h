// Generated by gencpp from file opencv_apps/Flow.msg
// DO NOT EDIT!


#ifndef OPENCV_APPS_MESSAGE_FLOW_H
#define OPENCV_APPS_MESSAGE_FLOW_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <opencv_apps/Point2D.h>
#include <opencv_apps/Point2D.h>

namespace opencv_apps
{
template <class ContainerAllocator>
struct Flow_
{
  typedef Flow_<ContainerAllocator> Type;

  Flow_()
    : point()
    , velocity()  {
    }
  Flow_(const ContainerAllocator& _alloc)
    : point(_alloc)
    , velocity(_alloc)  {
  (void)_alloc;
    }



   typedef  ::opencv_apps::Point2D_<ContainerAllocator>  _point_type;
  _point_type point;

   typedef  ::opencv_apps::Point2D_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;




  typedef boost::shared_ptr< ::opencv_apps::Flow_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opencv_apps::Flow_<ContainerAllocator> const> ConstPtr;

}; // struct Flow_

typedef ::opencv_apps::Flow_<std::allocator<void> > Flow;

typedef boost::shared_ptr< ::opencv_apps::Flow > FlowPtr;
typedef boost::shared_ptr< ::opencv_apps::Flow const> FlowConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::opencv_apps::Flow_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::opencv_apps::Flow_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace opencv_apps

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'opencv_apps': ['/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::opencv_apps::Flow_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::opencv_apps::Flow_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opencv_apps::Flow_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opencv_apps::Flow_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opencv_apps::Flow_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opencv_apps::Flow_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::opencv_apps::Flow_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dd9a9efd88ba39035e78af697593d751";
  }

  static const char* value(const ::opencv_apps::Flow_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdd9a9efd88ba3903ULL;
  static const uint64_t static_value2 = 0x5e78af697593d751ULL;
};

template<class ContainerAllocator>
struct DataType< ::opencv_apps::Flow_<ContainerAllocator> >
{
  static const char* value()
  {
    return "opencv_apps/Flow";
  }

  static const char* value(const ::opencv_apps::Flow_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::opencv_apps::Flow_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Point2D point\n\
Point2D velocity\n\
\n\
================================================================================\n\
MSG: opencv_apps/Point2D\n\
float64 x\n\
float64 y\n\
\n\
";
  }

  static const char* value(const ::opencv_apps::Flow_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::opencv_apps::Flow_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Flow_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opencv_apps::Flow_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::opencv_apps::Flow_<ContainerAllocator>& v)
  {
    s << indent << "point: ";
    s << std::endl;
    Printer< ::opencv_apps::Point2D_<ContainerAllocator> >::stream(s, indent + "  ", v.point);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::opencv_apps::Point2D_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENCV_APPS_MESSAGE_FLOW_H
