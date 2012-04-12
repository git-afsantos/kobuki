#ifndef KOBUKI_DOCK_IR_DATA_HPP__
#define KOBUKI_DOCK_IR_DATA_HPP__

#include <ecl/containers.hpp>
#include "../packet_handler/payload_base.hpp"

namespace kobuki
{

class DockIRData : public packet_handler::payloadBase
{
public:
  // container
  struct Data {
    Data() : docking(3) {}
    uint8_t header_id;
    std::vector<uint8_t> docking;
  } data;

  DockIRData()
  {
  }

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_dock_ir: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(data.header_id, byteStream);
    buildBytes(data.docking[0], byteStream);
    buildBytes(data.docking[1], byteStream);
    buildBytes(data.docking[2], byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_dock_ir: deserialise failed. empty byte stream.");
      return false;
    }

    buildVariable(data.header_id, byteStream);
    buildVariable(data.docking[0], byteStream);
    buildVariable(data.docking[1], byteStream);
    buildVariable(data.docking[2], byteStream);

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, docking[2], docking[1], docking[0] );
  }
};

} // namespace kobuki

#endif /* KOBUKI_IR_DATA_HPP__ */