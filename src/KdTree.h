/*
 * Author: Matteo Tadiello
 * Data: 08/2018
 */

/**
 * \file    KdTree.h
 * \brief   Contains the KdTree class.
 */

#ifndef KD_TREE_H_
#define KD_TREE_H_

#include <cstddef>
#include <vector>

#include "Vector3.h"

namespace ORCA{
	class Agent;
	//class RVOSIMULATOR???

	class KdTree{
	public:
		void setAgents(std::vector<Agent *> x);

		/**
		 * \brief   Constructs a <i>k</i>d-tree instance.
		 * \param   sim  The simulator instance.
		 */
		explicit KdTree();

		/**
		 * \brief   Builds an agent <i>k</i>d-tree.
		 */
		void buildAgentTree();
	private:
		/**
		 * \brief   Defines an agent <i>k</i>d-tree node.
		 */
		class AgentTreeNode {
		public:
			/**
			 * \brief   The beginning node number.
			 */
			size_t begin;

			/**
			 * \brief   The ending node number.
			 */
			size_t end;

			/**
			 * \brief   The left node number.
			 */
			size_t left;

			/**
			 * \brief   The right node number.
			 */
			size_t right;

			/**
			 * \brief   The maximum coordinates.
			 */
			Vector3 maxCoord;

			/**
			 * \brief   The minimum coordinates.
			 */
			Vector3 minCoord;
		};

		

		

		void buildAgentTreeRecursive(size_t begin, size_t end, size_t node);

		/**
		 * \brief   Computes the agent neighbors of the specified agent.
		 * \param   agent    A pointer to the agent for which agent neighbors are to be computed.
		 * \param   rangeSq  The squared range around the agent.
		 */
		void computeAgentNeighbors(Agent *agent, float rangeSq) const;

		void queryAgentTreeRecursive(Agent *agent, float &rangeSq, size_t node) const;

		std::vector<Agent *> agents_;
		std::vector<AgentTreeNode> agentTree_;
		//RVOSimulator *sim_;

		friend class Agent;
		//friend class RVOSimulator;
	};

}

#endif