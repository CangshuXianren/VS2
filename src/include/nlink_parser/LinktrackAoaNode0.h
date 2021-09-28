// Generated by gencpp from file nlink_parser/LinktrackAoaNode0.msg
// DO NOT EDIT!


#ifndef NLINK_PARSER_MESSAGE_LINKTRACKAOANODE0_H
#define NLINK_PARSER_MESSAGE_LINKTRACKAOANODE0_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace nlink_parser
{
template <class ContainerAllocator>
struct LinktrackAoaNode0_
{
  typedef LinktrackAoaNode0_<ContainerAllocator> Type;

  LinktrackAoaNode0_()
    : role(0)
    , id(0)
    , dis(0.0)
    , angle(0.0)
    , fp_rssi(0.0)
    , rx_rssi(0.0)  {
    }
  LinktrackAoaNode0_(const ContainerAllocator& _alloc)
    : role(0)
    , id(0)
    , dis(0.0)
    , angle(0.0)
    , fp_rssi(0.0)
    , rx_rssi(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _role_type;
  _role_type role;

   typedef uint8_t _id_type;
  _id_type id;

   typedef float _dis_type;
  _dis_type dis;

   typedef float _angle_type;
  _angle_type angle;

   typedef float _fp_rssi_type;
  _fp_rssi_type fp_rssi;

   typedef float _rx_rssi_type;
  _rx_rssi_type rx_rssi;





  typedef boost::shared_ptr< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> const> ConstPtr;

}; // struct LinktrackAoaNode0_

typedef ::nlink_parser::LinktrackAoaNode0_<std::allocator<void> > LinktrackAoaNode0;

typedef boost::shared_ptr< ::nlink_parser::LinktrackAoaNode0 > LinktrackAoaNode0Ptr;
typedef boost::shared_ptr< ::nlink_parser::LinktrackAoaNode0 const> LinktrackAoaNode0ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator1> & lhs, const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator2> & rhs)
{
  return lhs.role == rhs.role &&
    lhs.id == rhs.id &&
    lhs.dis == rhs.dis &&
    lhs.angle == rhs.angle &&
    lhs.fp_rssi == rhs.fp_rssi &&
    lhs.rx_rssi == rhs.rx_rssi;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator1> & lhs, const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace nlink_parser

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >
{
  static const char* value()
  {
    return "14ecc209e569ea4b0a93a8b73bd0cd13";
  }

  static const char* value(const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x14ecc209e569ea4bULL;
  static const uint64_t static_value2 = 0x0a93a8b73bd0cd13ULL;
};

template<class ContainerAllocator>
struct DataType< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nlink_parser/LinktrackAoaNode0";
  }

  static const char* value(const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 role\n"
"uint8 id\n"
"float32 dis\n"
"float32 angle\n"
"float32 fp_rssi\n"
"float32 rx_rssi\n"
;
  }

  static const char* value(const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.role);
      stream.next(m.id);
      stream.next(m.dis);
      stream.next(m.angle);
      stream.next(m.fp_rssi);
      stream.next(m.rx_rssi);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LinktrackAoaNode0_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nlink_parser::LinktrackAoaNode0_<ContainerAllocator>& v)
  {
    s << indent << "role: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.role);
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
    s << indent << "dis: ";
    Printer<float>::stream(s, indent + "  ", v.dis);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
    s << indent << "fp_rssi: ";
    Printer<float>::stream(s, indent + "  ", v.fp_rssi);
    s << indent << "rx_rssi: ";
    Printer<float>::stream(s, indent + "  ", v.rx_rssi);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NLINK_PARSER_MESSAGE_LINKTRACKAOANODE0_H
