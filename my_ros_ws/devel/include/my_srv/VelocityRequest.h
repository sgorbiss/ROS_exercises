// Generated by gencpp from file my_srv/VelocityRequest.msg
// DO NOT EDIT!


#ifndef MY_SRV_MESSAGE_VELOCITYREQUEST_H
#define MY_SRV_MESSAGE_VELOCITYREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace my_srv
{
template <class ContainerAllocator>
struct VelocityRequest_
{
  typedef VelocityRequest_<ContainerAllocator> Type;

  VelocityRequest_()
    : min(0.0)
    , max(0.0)  {
    }
  VelocityRequest_(const ContainerAllocator& _alloc)
    : min(0.0)
    , max(0.0)  {
  (void)_alloc;
    }



   typedef float _min_type;
  _min_type min;

   typedef float _max_type;
  _max_type max;





  typedef boost::shared_ptr< ::my_srv::VelocityRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_srv::VelocityRequest_<ContainerAllocator> const> ConstPtr;

}; // struct VelocityRequest_

typedef ::my_srv::VelocityRequest_<std::allocator<void> > VelocityRequest;

typedef boost::shared_ptr< ::my_srv::VelocityRequest > VelocityRequestPtr;
typedef boost::shared_ptr< ::my_srv::VelocityRequest const> VelocityRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_srv::VelocityRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_srv::VelocityRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::my_srv::VelocityRequest_<ContainerAllocator1> & lhs, const ::my_srv::VelocityRequest_<ContainerAllocator2> & rhs)
{
  return lhs.min == rhs.min &&
    lhs.max == rhs.max;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::my_srv::VelocityRequest_<ContainerAllocator1> & lhs, const ::my_srv::VelocityRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace my_srv

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::my_srv::VelocityRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_srv::VelocityRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_srv::VelocityRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_srv::VelocityRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_srv::VelocityRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_srv::VelocityRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_srv::VelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b3ee9ed25575b46bb47c7673ad202faa";
  }

  static const char* value(const ::my_srv::VelocityRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb3ee9ed25575b46bULL;
  static const uint64_t static_value2 = 0xb47c7673ad202faaULL;
};

template<class ContainerAllocator>
struct DataType< ::my_srv::VelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_srv/VelocityRequest";
  }

  static const char* value(const ::my_srv::VelocityRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_srv::VelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 min\n"
"float32 max\n"
;
  }

  static const char* value(const ::my_srv::VelocityRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_srv::VelocityRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.min);
      stream.next(m.max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VelocityRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_srv::VelocityRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_srv::VelocityRequest_<ContainerAllocator>& v)
  {
    s << indent << "min: ";
    Printer<float>::stream(s, indent + "  ", v.min);
    s << indent << "max: ";
    Printer<float>::stream(s, indent + "  ", v.max);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_SRV_MESSAGE_VELOCITYREQUEST_H