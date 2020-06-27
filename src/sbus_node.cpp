#include <cstdio>
#include "ros/ros.h"
#include "SBUS.h"
#include "ros_sbus/SbusPacket.h"
#include "MessageHelper.h"

ros::Publisher sbusPub;

SBUS sbus;

void onPacket(sbus_packet_t packet)
{
    ros_sbus::SbusPacket packetMsg;
    SbusConvertPacketMessage(packet, packetMsg);
    packetMsg.header.stamp = ros::Time::now();

    bool passthrough = false;
    if (ros::param::get("~passthrough", passthrough) && passthrough)
    {
        sbus.write(packet);
    }

    sbusPub.publish(packetMsg);
}

void sbusInCallback(const ros_sbus::SbusPacket::ConstPtr &msg)
{
    static uint16_t channels[16];
    static sbus_packet_t packet = {
            .channels = channels
    };
    SbusConvertPacketMessage(*msg, packet);

    if (sbus.write(packet) != SBUS_OK)
    {
        ROS_ERROR("SBUS write failed");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sbus_node");
    ros::NodeHandle nh;

    sbusPub = nh.advertise<ros_sbus::SbusPacket>("sbus_out", 32);
    ros::Subscriber sbusSub = nh.subscribe("sbus_in", 32, sbusInCallback);

    sbus_err_t ret = sbus.install("/dev/ttyUSB0");
    if (ret != SBUS_OK)
    {
        ROS_FATAL("SBUS install error: %d\n", ret);
        return ret;
    }

    sbus.onPacket(onPacket);

    ROS_INFO("SBUS node started");

    while (ros::ok())
    {
        if (sbus.read() != SBUS_OK)
        {
            ROS_ERROR("SBUS read failed");
        }

        ros::spinOnce();
    }

    return 0;
}