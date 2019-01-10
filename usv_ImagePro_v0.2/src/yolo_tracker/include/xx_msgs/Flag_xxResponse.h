// Generated by gencpp from file xx_msgs/Flag_xxResponse.msg
// DO NOT EDIT!


#ifndef XX_MSGS_MESSAGE_FLAG_XXRESPONSE_H
#define XX_MSGS_MESSAGE_FLAG_XXRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace xx_msgs
{
template <class ContainerAllocator>
struct Flag_xxResponse_
{
  typedef Flag_xxResponse_<ContainerAllocator> Type;

  Flag_xxResponse_()
    : result()  {
    }
  Flag_xxResponse_(const ContainerAllocator& _alloc)
    : result(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> const> ConstPtr;

}; // struct Flag_xxResponse_

typedef ::xx_msgs::Flag_xxResponse_<std::allocator<void> > Flag_xxResponse;

typedef boost::shared_ptr< ::xx_msgs::Flag_xxResponse > Flag_xxResponsePtr;
typedef boost::shared_ptr< ::xx_msgs::Flag_xxResponse const> Flag_xxResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xx_msgs::Flag_xxResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace xx_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'xx_msgs': ['/home/xx/andyoyo/usv_ImagePro/src/xx_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c22f2a1ed8654a0b365f1bb3f7ff2c0f";
  }

  static const char* value(const ::xx_msgs::Flag_xxResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc22f2a1ed8654a0bULL;
  static const uint64_t static_value2 = 0x365f1bb3f7ff2c0fULL;
};

template<class ContainerAllocator>
struct DataType< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xx_msgs/Flag_xxResponse";
  }

  static const char* value(const ::xx_msgs::Flag_xxResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string result\n\
";
  }

  static const char* value(const ::xx_msgs::Flag_xxResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Flag_xxResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xx_msgs::Flag_xxResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xx_msgs::Flag_xxResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XX_MSGS_MESSAGE_FLAG_XXRESPONSE_H
