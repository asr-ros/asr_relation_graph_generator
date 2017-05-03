/**

Copyright (c) 2016, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "trainer/generator/heuristic/HeuristicalTreeGenerator.h"
#include <ros/ros.h>

namespace SceneModel {
 
  HeuristicalTreeGenerator::HeuristicalTreeGenerator()
  : AbstractGraphGenerator()
  {
  }
  
  HeuristicalTreeGenerator::~HeuristicalTreeGenerator()
  {
  }
  
  void HeuristicalTreeGenerator::buildTree(ObjectSetList pObjectSets, boost::shared_ptr<TreeNode>& pRoot)
  {
    deleteEmptyObjectSets(pObjectSets);
    
    // This is a helper datastructure for building the tree. We do agglomerative clustering,
    // which means we expand clusters until we have a tree of them. We're building bottom up,
    // thats because we have to store the tree as a list of clusters during building.
    std::vector<boost::shared_ptr<TreeNode> > clusters;

    // Initialize the helper data structure.
    BOOST_FOREACH(boost::shared_ptr<ISM::ObjectSet> set, pObjectSets.mObjectSets)
      clusters.push_back(boost::shared_ptr<TreeNode>(new TreeNode(set)));
    
    // Use the heuristics and agglomerative clustering to create the tree.
      // Search the best cluster of trajectories.
      // Remove it from the trajectory list.
      // Build the next layer of the tree using the cluster.
    while(clusters.size() > 1)
    {
      // Evaluate the heuristics for the remaining trajectories.
      evaluateHeuristics(clusters);
      
      // Get the best cluster from the best scoring heuristic.
      boost::shared_ptr<TreeNode> cluster = mHeuristics[0]->getBestCluster();
      
      // Remove the object set used by the cluster's root from our 'clusters' helper data structure.
      for(unsigned int i = 0; i < clusters.size(); i++)
      {
    if(clusters[i]->mObjectSet->mIdentifier == cluster->mObjectSet->mIdentifier)
	{
	  clusters.erase(clusters.begin() + i);
	  i--;
	}
      }
      
      // Remove the object sets used by the child nodes of the cluster's root from our 'clusters' helper data structure.
      BOOST_FOREACH(boost::shared_ptr<TreeNode> child, cluster->mChildren)
      {
	for(unsigned int i = 0; i < clusters.size(); i++)
	{
      if(clusters[i]->mObjectSet->mIdentifier == child->mObjectSet->mIdentifier)
	  {
	    clusters.erase(clusters.begin() + i);
	    i--;
	  }
	}
      }
      
      // Add the cluster to the list of candidates.
      clusters.push_back(cluster);
    }
    
    // There's only one node left. This will be the root node of the tree.
    pRoot = clusters[0];
  }
  
  void HeuristicalTreeGenerator::buildTree(ObjectSetList pTrajectories,
					   boost::shared_ptr<TreeNode>& pRoot,
					   std::string pType)
  {
    deleteEmptyObjectSets(pTrajectories);
    
    // This is a helper datastructure for building the tree. It will contain all unassigned nodes.
    std::vector<boost::shared_ptr<TreeNode> > clusters;
    
    // Initialize the helper data structure.
    BOOST_FOREACH(boost::shared_ptr<ISM::ObjectSet> set, pTrajectories.mObjectSets)
      clusters.push_back(boost::shared_ptr<TreeNode>(new TreeNode(set)));
      
    // Add the object with the given type as root node.
    for(std::vector<boost::shared_ptr<TreeNode> >::iterator it = clusters.begin() ; it != clusters.end(); ++it)
    {
      boost::shared_ptr<ISM::ObjectSet> objectSet = (*it)->mObjectSet;
      
      // Check, if we found our object set.
      if(objectSet->objects[0]->type.compare(pType) == 0)
      {
	// Initialize the root node with the found object set.
	pRoot.reset(new TreeNode(objectSet));
	
	// Delete the object set from the list.
	clusters.erase(it);
	
	// Stop the loop.
	break;
      }
    }
    
    // While there are other nodes in the helper data structure, continue searching for the best pairs.
    // Add the best pair found to the tree.
    while(clusters.size() > 0)
    {
      // This is a list of all nodes currentry assigned to the tree.
      std::vector<boost::shared_ptr<TreeNode> > assignedNodes;
      
      // Iterate over the tree and fill the list of assigned nodes.
      std::queue<boost::shared_ptr<TreeNode> > nodesToVisit;
      nodesToVisit.push(pRoot);
      
      while(nodesToVisit.size() > 0)
      {
	boost::shared_ptr<TreeNode> node = nodesToVisit.front();
	nodesToVisit.pop();
	
	BOOST_FOREACH(boost::shared_ptr<TreeNode> child, node->mChildren)
	  nodesToVisit.push(child);
	  
	assignedNodes.push_back(node);
      }
      
      // Pick an arbitrary and not yet assigned node and remove it from the list of unassigned nodes.
      boost::shared_ptr<TreeNode> nodeToAssign = *clusters.begin();
      clusters.erase(clusters.begin());
      
      // Evaluate the heuristics for the remaining trajectories.
      evaluateHeuristics(assignedNodes, nodeToAssign);
      
      // Get the best cluster from the best scoring heuristic.
      boost::shared_ptr<TreeNode> best = mHeuristics[0]->getBestParentNode();
      
      // Found a parent node or no good candidate was given by the heuristic.
      if(best)
      {
	// Add our current unassigned node to the parent node determined by the heuristic.
	std::queue<boost::shared_ptr<TreeNode> > nodesToVisitAgain;
	nodesToVisitAgain.push(pRoot);
	
	while(nodesToVisitAgain.size() > 0)
	{
	  boost::shared_ptr<TreeNode> node = nodesToVisitAgain.front();
	  nodesToVisitAgain.pop();
	  
	  if(node == best)
	  {
	    node->addChild(nodeToAssign);
	    break;
	  }
	  
	  BOOST_FOREACH(boost::shared_ptr<TreeNode> child, node->mChildren)
	    nodesToVisitAgain.push(child);
	}
      } else {
	pRoot->addChild(nodeToAssign);
	ROS_INFO("Scene tree: No matching candidate found. Appended to root node.");
      }
    }
  }
  
  void HeuristicalTreeGenerator::addHeuristic(boost::shared_ptr<AbstractHeuristic> pHeuristic)
  {
    mHeuristics.push_back(pHeuristic);
  }
  
  void HeuristicalTreeGenerator::evaluateHeuristics(std::vector<boost::shared_ptr<TreeNode> > pClusters)
  {
    // Evaluate every single trajectory based on the given object sets. Sort them by score.
    BOOST_FOREACH(boost::shared_ptr<AbstractHeuristic> heuristic, mHeuristics)
    {
      // Reset the heuristic.
      heuristic->reset();
      
      // Apply the heuristic.
      heuristic->apply(pClusters);
    }
    
    // Sort the heuristics based on their score.
    std::sort(mHeuristics.begin(), mHeuristics.end());
  }
  
  void HeuristicalTreeGenerator::evaluateHeuristics(std::vector<boost::shared_ptr<TreeNode> > pClusters,
						    boost::shared_ptr<TreeNode> pChild)
  {
    // Evaluate every single trajectory based on the given object sets. Sort them by score.
    BOOST_FOREACH(boost::shared_ptr<AbstractHeuristic> heuristic, mHeuristics)
    {
      // Reset the heuristic.
      heuristic->reset();
      
      // Apply the heuristic.
      heuristic->apply(pClusters, pChild);
    }
    
    // Sort the heuristics based on their score.
    std::sort(mHeuristics.begin(), mHeuristics.end());
  }
  
  void HeuristicalTreeGenerator::deleteEmptyObjectSets(ObjectSetList& pObjectSets)
  {
    // Iterate over the list of trajectories and delete empty object sets.
    for(unsigned int i = 0; i < pObjectSets.mObjectSets.size(); i++)
    {
      if (pObjectSets.mObjectSets[i]->objects.size() == 0 )
      {
	  pObjectSets.mObjectSets.erase(pObjectSets.mObjectSets.begin() + i);
	  i--;
      }
    }
  }
  
}
