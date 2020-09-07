#ifndef SRC_SBUS_NODE_H
#define SRC_SBUS_NODE_H

#include "ros/ros.h"
#include "SBUS.h"
#include "ros_sbus/SbusPacket.h"
#include <thread>
#include <csignal>

class SBUSNode
{
public:
    SBUSNode(ros::NodeHandle *nh);
    ~SBUSNode();

    int start(const char *ttyPath);
    int stop();
    void setCallback(sbus_packet_cb);
    void sbusCallback(sbus_packet_t);

private:
    ros::NodeHandle *const nh;
    SBUS _sbus;
    ros::Publisher _pub;
    ros::Subscriber _sub;

    ros_sbus::SbusPacket::ConstPtr _lastMsg = nullptr;

    std::thread *_sbusThread = nullptr;
    volatile std::sig_atomic_t _terminateThread = false;
    void sbusTask();

    static bool getPassthrough();

    void inCallback(const ros_sbus::SbusPacket::ConstPtr &msg);
};

#endif //SRC_SBUS_NODE_H
