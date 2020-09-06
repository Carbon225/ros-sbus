#include <cstdio>
#include "ros/ros.h"
#include "sbus_node.h"

static std::shared_ptr<SBUSNode> sbusNode;

void sbusCallback(sbus_packet_t p)
{
    if (sbusNode)
        sbusNode->sbusCallback(p);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sbus_node");
    ros::NodeHandle nh;

    sbusNode = std::make_shared<SBUSNode>(&nh);
    sbusNode->setCallback(sbusCallback);
    sbusNode->start("/dev/ttyAMA0");

    ROS_INFO("SBUS node started");

    ros::spin();

    ROS_WARN("SBUS node terminating");
    sbusNode->stop();

    return 0;
}