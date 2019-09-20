/**
 * @file multiple_uavs_send_command_node.cpp
 * @brief Agent definition, for multiple UAVs collision avoidance 
 *
 * Author: Matteo Tadiello 
 * Data: 08/2018
 */


#ifndef AGENT_H_
#define AGENT_H_

#include <cstddef>
#include <utility>

// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3.h>
//#include <nav_msgs/Odometry.h>

#include "Vector3.h"
#include "Definitions.h"
#include "KdTree.h"

#include <ignition/math/Vector3.hh>

namespace ORCA{
	/**
	 * \brief   Defines an Agent.
	 */
	class Agent
 	{
	 	public:
	 	
	 		//void position_cb(ignition::math::Vector3d current_position);
		 	//void vel_cb(ignition::math::Vector3d current_velocity);
		 	//void setId(int id);
		 	//bool IsColliding(const ORCA::Agent* b );
		 	explicit Agent();
		 	
		 	///////////////////////////////////////////////////////////// for retriving information from UAV
		 	
			//////////////

			//ignition::math::Vector3d current_position;
      		//ignition::math::Vector3d current_velocity;
			Vector3 position_;
			Vector3 velocity_;
			Vector3 newVelocity_;
			Vector3 prefVelocity_;
			size_t id_;
			size_t maxNeighbors_;
			float maxSpeed_;
			float neighborDist_;
			float radius_;
			float timeHorizon_;
			std::vector<std::pair<float, const Agent*> > agentNeighbors_;
			std::vector<Plane> orcaPlanes_;
			KdTree *tree_;

			
		 	/**
			 * \brief   Computes the neighbors of this agent.
			 */
			void computeNeighbors(); //The tree where the agents have to be put it

			/**
			 * \brief   Computes the new velocity of this agent.
			 */
			void computeNewVelocity();
		 	

			/**
			 * \brief   Inserts an agent neighbor into the set of neighbors of this agent.
			 * \param   agent    A pointer to the agent to be inserted.
			 * \param   rangeSq  The squared range around this agent.
			 */
			void insertAgentNeighbor(const Agent *agent, float &rangeSq);

			/**
			 * \brief   Updates the three-dimensional position and three-dimensional velocity of this agent.
			 */
			void update();


			

			friend class KdTree_;

 	};
}

#endif /* AGENT_H_ */