#ifndef NODE_ABSTRACT_CLASSES_H
#define NODE_ABSTRACT_CLASSES_H

#include <ros/ros.h>

#include "node_statemachine.h"

class AbstractStateNode
{
private:
protected:
    std::shared_ptr<ros::NodeHandle> nh;
    std::shared_ptr<NodeStateMachine> node_state_machine;
public:
    AbstractStateNode(std::shared_ptr<ros::NodeHandle> nh): 
        nh(nh),
        node_state_machine(new NodeStateMachine())
    {}
};

#endif