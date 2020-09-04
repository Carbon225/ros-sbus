#include "MessageHelper.h"
#include "sbus_node.h"

SBUSNode::SBUSNode(ros::NodeHandle *nodeHandle)
    : nh(nodeHandle)
{

}

SBUSNode::~SBUSNode()
{
    stop();
}

int SBUSNode::start(const char *ttyPath)
{
    _pub = nh->advertise<ros_sbus::SbusPacket>("sbus_out", 1);
    _sub = nh->subscribe("sbus_in", 1, &SBUSNode::inCallback, this);
    getPassthrough();

    _terminateThread = false;
    _sbusThread = new std::thread(&SBUSNode::sbusTask, this);

    return _sbus.install(ttyPath, true);
}

int SBUSNode::stop()
{
    if (_sbusThread)
    {
        _terminateThread = true;
        pthread_kill(_sbusThread->native_handle(), SIGINT);
        if (_sbusThread->joinable())
            _sbusThread->join();

        delete _sbusThread;
        _sbusThread = nullptr;
    }
    return 0;
}

void SBUSNode::setCallback(sbus_packet_cb cb)
{
    _sbus.onPacket(cb);
}

void SBUSNode::sbusCallback(sbus_packet_t packet)
{
    ros_sbus::SbusPacket packetMsg;
    SbusConvertPacketMessage(packet, packetMsg);
    packetMsg.header.stamp = ros::Time::now();

    if (getPassthrough())
    {
        _sbus.write(packet);
    }

    _pub.publish(packetMsg);
}

void SBUSNode::inCallback(const ros_sbus::SbusPacket::ConstPtr &msg)
{
    uint16_t channels[16];
    sbus_packet_t packet = {
            .channels = channels
    };
    SbusConvertPacketMessage(*msg, packet);

    if (_sbus.write(packet) != SBUS_OK)
    {
        ROS_ERROR("SBUS write failed");
    }
}

void SBUSNode::sbusTask()
{
    siginterrupt(SIGINT, true);
    while (!_terminateThread)
    {
        sbus_err_t ret = _sbus.read();
        if (ret == SBUS_ERR_DESYNC)
            ROS_WARN("SBUS desync");
        else if (ret != SBUS_OK)
            ROS_ERROR("SBUS error: %d", ret);
    }
}

bool SBUSNode::getPassthrough()
{
    bool passthrough = false;
    ros::param::get("~passthrough", passthrough);
    return passthrough;
}
