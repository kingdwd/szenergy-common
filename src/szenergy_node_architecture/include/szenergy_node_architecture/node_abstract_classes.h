#ifndef NODE_ABSTRACT_CLASSES_H
#define NODE_ABSTRACT_CLASSES_H

#include <vector>

#include <ros/ros.h>

#include "node_statemachine.h"

enum class CONTROLSTATE_STATES {WAITING, SYNCHRONIZED, DANGER, DISABLED };

namespace szenergy {

	class AbstractNodeState
	{
	protected:
		CONTROLSTATE_STATES state;
		// Relation with state machines
		std::shared_ptr<NodeStateMachine> synchronizing_hierarch_sm;
		std::vector<std::shared_ptr<PortStateMachine> > synchronizing_port_sm;
	public:
		AbstractNodeState(): state(CONTROLSTATE_STATES::WAITING)
		{
		}

		void setSyncParentStateMachine(std::shared_ptr<NodeStateMachine> sm)
		{
			synchronizing_hierarch_sm = sm;
		}

		void addSyncPortStateMachine(std::shared_ptr<PortStateMachine> sm)
		{
			synchronizing_port_sm.push_back(sm);
		}

		bool synchronized();

		bool transitSynchronizing();

		bool transitDanger();

		bool transitWaiting();

		bool transitDangerHandled();

		bool transitDisabled();

		bool transitDangerUnhandled();

		void transform();

		virtual void transformFunc() = 0;
		virtual void handlingDanger() = 0;
	};

	class AbstractStateNode
	{
	private:
	protected:
		std::shared_ptr<ros::NodeHandle> nh;
		std::shared_ptr<ros::NodeHandle> private_nh;
		std::shared_ptr<NodeStateMachine> node_state_machine;
		std::vector<std::shared_ptr<PortStateMachine> > sub_port_state_machines;
		std::vector<std::shared_ptr<PortStateMachine> > pub_port_state_machines;

		bool setAllSubPortStateMachineRunning();

		inline bool processingAllowedOnPort(std::shared_ptr<PortStateMachine> p_sm)
		{
			return node_state_machine->isRunning() && (p_sm->isWaiting() || p_sm->isRunning());
		}
	public:
		AbstractStateNode(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<ros::NodeHandle> private_nh):
			nh(nh),
			private_nh(private_nh),
			node_state_machine(new NodeStateMachine())
		{}
	};

}

#endif
