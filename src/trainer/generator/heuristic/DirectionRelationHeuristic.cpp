/**

Copyright (c) 2016, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "trainer/generator/heuristic/DirectionRelationHeuristic.h"

namespace SceneModel {
 
  DirectionRelationHeuristic::DirectionRelationHeuristic(const double pStaticBreakRatio, const double pTogetherRatio, const double pMaxAngleDeviation)
  : AbstractHeuristic("DirectionRelationHeuristic")
  , mStaticBreakRatio(pStaticBreakRatio)
  , mTogetherRatio(pTogetherRatio)
  , mMaxAngleDeviation(pMaxAngleDeviation)
  {
  }
  
  DirectionRelationHeuristic::~DirectionRelationHeuristic()
  {
  }
  
  void DirectionRelationHeuristic::apply(std::vector<boost::shared_ptr<TreeNode> > pNodes)
  {
    // The closest distance between two objects.
    double bestDistance = std::numeric_limits<double>::max();
    
    // Iterate over all object sets.
    BOOST_FOREACH(boost::shared_ptr<TreeNode> first, pNodes)
    {
      // The currently best trajectory.
      boost::shared_ptr<TreeNode> currentBest;
      
      double currentClosestDistance = std::numeric_limits<double>::max();
      int currentBestBreaks = 1;
      int currentBestCommonPositions = 1;

      // Iterate over all object sets in an inner loop to go over all possible combinations.
      BOOST_FOREACH(boost::shared_ptr<TreeNode> second, pNodes)
      {
	// Prevent trivial comparison.
	if (first == second) {
	    continue;
	}

	/*
	  * What we do:
	  * Calculate a direction Vector from first to second for every Frame.
	  * Check in every frame the angle between the reference vector (first vector) and the current vector.
	  * If the misalignment is more than mMaxAngleDeviation degrees, increase staticBreaks and
	  * recalculate the reference vote.
	  * Also calculate the average distance between first and second.
	  *
	  * At the end, if the staticBreaks are below mStaticBreakRatio of the sample range,
	  * and they appear together in more than mTogetherRatio of the frames,
	  * and the second is closer to first than the current closest track,
	  * replace the current closest track with second.
	  *
	  * At the end, choose the first<->second combination with the lowest rate of static breaks.
	  *
	  * This will always create a cluster of two tracks.
	  */

	int commonPositions = 0;
	double averageDistance = 0;

	// Go over all objects in the first object set.
	for(unsigned int i = 0; i < first->mObjectSet->mObjects.size(); i++)
	{
	  boost::shared_ptr<Object> firstObject = first->mObjectSet->mObjects[i];
	  boost::shared_ptr<Object> secondObject = second->mObjectSet->mObjects[i];
	  
	  // Both objects not empty?
	  if (!firstObject || !secondObject)
	  {
	      continue;
	  }
	  
	  // Calculate average distance.
	  averageDistance += MathHelper::getDistanceBetweenPoints(*firstObject->mPosition, *secondObject->mPosition);
	  
	  // +1 for common position couter.
	  commonPositions++;
	}
	
	// Not enough common positions over all frames? Kick kombination!
	if(commonPositions < (double) first->mObjectSet->mObjects.size() * mTogetherRatio)
	{
	  continue;
	}

	// Calculate the average distance.
	averageDistance /= (double) commonPositions;

	int staticBreaks = 0;
	bool firstRun = true;
	Eigen::Vector3d directionVector;

	// Go over all objects in the first object set again
	for(unsigned int i = 0; i < first->mObjectSet->mObjects.size(); i++)
	{
	  boost::shared_ptr<Object> firstObject = first->mObjectSet->mObjects[i];
	  boost::shared_ptr<Object> secondObject = second->mObjectSet->mObjects[i];
	  
	  // Both objects not empty?
	  if (!firstObject || !secondObject)
	  {
	      continue;
	  }

	  // This is first run? Set reference direction vector.
	  if (firstRun) {
	      directionVector = this->getDirectionVector(firstObject, secondObject);
	      firstRun = false;
	      continue;
	  }

	  // Calculate deviation between reference and current direction vector.
	  Eigen::Vector3d currentDirection = this->getDirectionVector(firstObject, secondObject);
	  double deviation = MathHelper::rad2deg(acos(directionVector.dot(currentDirection)));

	  // It deviation bigger than threshold, note a break in continuity.
	  if (deviation > mMaxAngleDeviation)
	  {
	      staticBreaks++;
	      directionVector = currentDirection;
	  }
	}

	// Check, if the current association of tracks is valid.
	if (
	  ((double) staticBreaks < ((double) commonPositions) * mStaticBreakRatio) &&
	  (!currentBest || (currentClosestDistance > averageDistance))
	  )
	{
	  currentBest = second;
	  currentClosestDistance = averageDistance;
	  currentBestBreaks = staticBreaks;
	  currentBestCommonPositions = commonPositions;
	}
      }

      if(currentBest) {
	double conf = 1 - (double) currentBestBreaks / (double) currentBestCommonPositions;
	if(candidates.empty()
	      || (conf > this->score
	      || (conf == this->score && bestDistance > currentClosestDistance)))
	{
	  // Delete old candidates and add the currently two best ones to the list.
	  candidates.clear();
	  candidates.push_back(first);
	  candidates.push_back(currentBest);
	  
	  // Save the score of the best cluster...
	  this->score = conf;
	  
	  // ... and the best distance.
	  bestDistance = currentClosestDistance;
	}
      }
    }
  }
  
  void DirectionRelationHeuristic::apply(std::vector<boost::shared_ptr<TreeNode> > pNodes, boost::shared_ptr<TreeNode> pChild)
  {
    // The closest distance between two objects.
    double bestDistance = std::numeric_limits<double>::max();
    
    // Iterate over all object sets.
    BOOST_FOREACH(boost::shared_ptr<TreeNode> first, pNodes)
    {
      // The currently best trajectory.
      boost::shared_ptr<TreeNode> currentBest;
      
      double currentClosestDistance = std::numeric_limits<double>::max();
      int currentBestBreaks = 1;
      int currentBestCommonPositions = 1;
      
      /*
	* What we do:
	* Calculate a direction Vector from first to second for every Frame.
	* Check in every frame the angle between the reference vector (first vector) and the current vector.
	* If the misalignment is more than mMaxAngleDeviation degrees, increase staticBreaks and
	* recalculate the reference vote.
	* Also calculate the average distance between first and second.
	*
	* At the end, if the staticBreaks are below mStaticBreakRatio of the sample range,
	* and they appear together in more than mTogetherRatio of the frames,
	* and the second is closer to first than the current closest track,
	* replace the current closest track with second.
	*
	* At the end, choose the first<->second combination with the lowest rate of static breaks.
	*
	* This will always create a cluster of two tracks.
	*/
      
      int commonPositions = 0;
      double averageDistance = 0;
      
      // Go over all objects in the first object set.
      for(unsigned int i = 0; i < first->mObjectSet->mObjects.size(); i++)
      {
	boost::shared_ptr<Object> firstObject = first->mObjectSet->mObjects[i];
	boost::shared_ptr<Object> secondObject = pChild->mObjectSet->mObjects[i];
	
	// Both objects not empty?
	if (!firstObject || !secondObject)
	  continue;
	
	// Calculate average distance.
	averageDistance += MathHelper::getDistanceBetweenPoints(*firstObject->mPosition, *secondObject->mPosition);
	
	// +1 for common position couter.
	commonPositions++;
      }
      
      // Not enough common positions over all frames? Kick kombination!
      if(commonPositions < (double) first->mObjectSet->mObjects.size() * mTogetherRatio)
	continue;
      
      // Calculate the average distance.
      averageDistance /= (double) commonPositions;

      int staticBreaks = 0;
      bool firstRun = true;
      Eigen::Vector3d directionVector;

      // Go over all objects in the first object set again
      for(unsigned int i = 0; i < first->mObjectSet->mObjects.size(); i++)
      {
	boost::shared_ptr<Object> firstObject = first->mObjectSet->mObjects[i];
	boost::shared_ptr<Object> secondObject = pChild->mObjectSet->mObjects[i];
	
	// Both objects not empty?
	if (!firstObject || !secondObject)
	{
	    continue;
	}

	// This is first run? Set reference direction vector.
	if (firstRun) {
	    directionVector = this->getDirectionVector(firstObject, secondObject);
	    firstRun = false;
	    continue;
	}

	// Calculate deviation between reference and current direction vector.
	Eigen::Vector3d currentDirection = this->getDirectionVector(firstObject, secondObject);
	double deviation = MathHelper::rad2deg(acos(directionVector.dot(currentDirection)));

	// It deviation bigger than threshold, note a break in continuity.
	if (deviation > mMaxAngleDeviation)
	{
	    staticBreaks++;
	    directionVector = currentDirection;
	}
      }
      
      // Check, if the current association of tracks is valid.
      if (
	((double) staticBreaks < ((double) commonPositions) * mStaticBreakRatio) &&
	(!currentBest || (currentClosestDistance > averageDistance))
	)
      {
	currentBest = pChild;
	currentClosestDistance = averageDistance;
	currentBestBreaks = staticBreaks;
	currentBestCommonPositions = commonPositions;
      }
      
      if (currentBest) {
	double conf = 1 - (double) currentBestBreaks / (double) currentBestCommonPositions;
	if (candidates.empty()
	      || (conf > this->score
	      || (conf == this->score && bestDistance > currentClosestDistance)))
	{
	  // Delete old candidates and add the currently two best ones to the list.
	  candidates.clear();
	  candidates.push_back(first);
	  candidates.push_back(currentBest);
	  
	  // Save the score of the best cluster...
	  this->score = conf;
	  
	  // ... and the best distance.
	  bestDistance = currentClosestDistance;
	}
      }
    }
  }
  
  boost::shared_ptr<TreeNode> DirectionRelationHeuristic::getBestCluster()
  {    
    // The new root object of this subtree.
    // It's the object that either appears most frequent or the one that moves at least.
    boost::shared_ptr<TreeNode> root;
    
    // That means how often we've seen the most frequent object (in percent).
    double bestViewRatio = 0;
    
    // The sum of movements of the at least moving object.
    double bestMovement = 0;
    
    BOOST_FOREACH(boost::shared_ptr<TreeNode> candidate, candidates)
    {
      // Counter for the view ratio.
      int views = 0;
      
      // Temporary variable for storing the movement.
      double movement = 0;
      
      // The last object.
      boost::shared_ptr<Object> lastObject;
      
      // Iterate over all object in the track and determine the number of views and the sum of movements.
      BOOST_FOREACH(boost::shared_ptr<Object> object, candidate->mObjectSet->mObjects)
      {
	if(object)
	{
	  views++;
	  
	  if (lastObject) {
	    movement += MathHelper::getDistanceBetweenPoints(*object->mPosition, *lastObject->mPosition);
	  }
	  lastObject = object;
	}
      }
      
      // Calculate the numbers specified above.
      double ratio = (double) views / (double) candidate->mObjectSet->mObjects.size();
      if (ratio > bestViewRatio || (ratio == bestViewRatio && movement < bestMovement)) {
	bestViewRatio = ratio;
	bestMovement = movement;
	root = candidate;
      }
    }
    
    // Add all other nodes as children to the root node.
    BOOST_FOREACH(boost::shared_ptr<TreeNode> candidate, candidates)
      if(candidate != root)
	root->addChild(candidate);
    
    // Return a new cluster.
    return root;
  }
  
  boost::shared_ptr<TreeNode> DirectionRelationHeuristic::getBestParentNode()
  {
    return (!candidates.empty()) ? candidates[0] : boost::shared_ptr<TreeNode>();
  }
  
  Eigen::Vector3d DirectionRelationHeuristic::getDirectionVector(const boost::shared_ptr<Object> first,
								   const boost::shared_ptr<Object> second)
  {
    // Calculate the spatial difference between both positions.
    Eigen::Vector3d firstToSecond = *second->mPosition - *first->mPosition;
    
    // Get orientation of first object.
    Eigen::Quaternion<double> firstRotation = *first->mOrientation;
    
    // ???
    return firstRotation.inverse()._transformVector(firstToSecond).normalized();
  }
  
}
