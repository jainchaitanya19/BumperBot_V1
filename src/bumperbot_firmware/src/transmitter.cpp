#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
using std::placeholder::_1

using namespace std;
using namespace rclcpp;





class SimpleSerialTransmiter : public Node
{
    SimpleSerialTransmiter() :Node("simple_serial_transmitter")
    {
        declare_parameter<string>("port","dev/ttyUSB0");
        port_= get_parameter("port").as_string;
        sub_= create_subscription()
    }

};