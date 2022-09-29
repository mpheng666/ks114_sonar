#include <ros/ros.h>
#include "ks114/ks114.hpp"
#include <unistd.h>

unsigned int microseconds = 50000;

const std::string ROS_NODE = "ks114_utility";
uint8_t new_index;
uint8_t old_address = 0;

bool set_flag = false;

uint8_t sensor_address[20] = {0XD0, 0XD2, 0XD4, 0XD6, 0XD8, 0XDA, 0XDC, 0XDE, 0XE0, 0XE2, 0XE4, 0XE6, 0XE8, 0XEA, 0XEC, 0XEE, 0XF8, 0XFA, 0XFC, 0XFE};

serial::Serial ser;
std::string serial_port = "/dev/ttyUSB0";
int baud_rate = 115200;
const double send_interval_s = 1.0;

bool openSerial()
{
    try
    {
        ser.setPort(serial_port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        ROS_INFO_STREAM("Serial Port initialized");
        return true;
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return false;
    }
}

uint8_t checkSensorInfo()
{
    std::vector<uint8_t> byte_received;

    for (int i = 0; i < 20; i++)
    {
        uint8_t info_cmd_single[3] = {sensor_address[i], 0x02, 0x99};
        ser.write(info_cmd_single, 3);
        ROS_INFO("Checking sensor %d (%d)", i, sensor_address[i]);
        usleep(microseconds);

        ser.flush();
        ser.flushOutput();
        byte_received.clear();

        if (ser.available())
        {
            ser.read(byte_received, 22);
            ROS_INFO("Current connected sonar %d", i);
            old_address = byte_received[5];
            return byte_received[5];
        }
    }
    return 255;
}

uint8_t userInput()
{

    ROS_INFO("Counterclockwise: Front(0,1), Side_left(2,3), Back(4,5), Side_right(6,7)");
    ROS_INFO("Please insert new number! (0~7)");
    std::cin >> new_index;

    return sensor_address[new_index - 48];
}

void writeNewAddress(uint8_t old_adress, uint8_t new_address)
{
    uint8_t address_cmd_A[3] = {old_adress, 0x02, 0x9a};
    uint8_t address_cmd_B[3] = {old_adress, 0x02, 0x92};
    uint8_t address_cmd_C[3] = {old_adress, 0x02, 0x9e};
    uint8_t address_cmd_D[3] = {old_adress, 0x02, new_address};

    ROS_INFO("Old address %d to new address %d", old_address, new_address);

    ser.write(address_cmd_A, 3);
    usleep(microseconds);
    ser.write(address_cmd_B, 3);
    usleep(microseconds);
    ser.write(address_cmd_C, 3);
    usleep(microseconds);
    ser.write(address_cmd_D, 3);
    usleep(microseconds);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_NODE);

    if (openSerial())
    {
        if (checkSensorInfo() < 255)
        {
            ser.flushInput();
            usleep(microseconds);
            writeNewAddress(old_address, userInput());
            ser.flushInput();
            usleep(microseconds);
            checkSensorInfo();
            ros::shutdown();
            return 0;
        }
        else
            ROS_ERROR("NONE OF THE SENSORS ARE CONNECTED!");
        ros::shutdown();
        return 0;
    }
    ros::spin();
    return 0;
}