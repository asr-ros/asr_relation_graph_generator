/**

Copyright (c) 2016, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global includes
#include <limits>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// Local includes
#include "helper/MathHelper.h"

#include "trainer/TreeNode.h"

#include "trainer/source/Object.h"
#include "trainer/source/ObjectSet.h"
#include "trainer/source/ObjectSetList.h"

#include "trainer/generator/heuristic/AbstractHeuristic.h"

namespace SceneModel {
  
  /**
   * Implementation of the abstract relation scoring heuristic using a directord relation approach. Calculate direction vectors between simultaneously recorded object estimations in a pair of trajectories. Calculate angles between consecutive direction vectors. Calculate direction continuity as described in Meißner et al. 2013 in Sec. V. A.. Calculate average distance between object estimations (recorded at the same time) over the trajectory pair as well.  
   *
   * @author Reno Reckling, Pascal Meissner, Joachim Gehrung
   * @version See SVN
   */
  class DirectionRelationHeuristic : public AbstractHeuristic
  {
  public:

    /**
     * Constructor.
     * 
     * @param pStaticBreakRatio Maximal percentage of breaks in object relation.
     * @param pTogetherRatio Minimal percentage of the frames objects are related relation.
     * @param pMaxAngleDeviation Maximal allowed angular distance between two objects.
     */
    DirectionRelationHeuristic(const double pStaticBreakRatio, const double pTogetherRatio, const double pMaxAngleDeviation);
    
    /**
     * Destructor.
     */
    ~DirectionRelationHeuristic();

    /**
     * Applies the heuristic to the trajectory set.
     * 
     * @param pNodes The set of nodes containing the trajectories that this heuristic should be applied to.
     */
    void apply(std::vector<boost::shared_ptr<TreeNode> > pNodes);
    
    /**
     * Applies the heuristic to the trajectory set, specifies one of the elements of the resulting cluster.
     * 
     * @param pNodes The set of nodes containing the trajectories that this heuristic should be applied to.
     * @param pChild The node that should act as child node.
     */
    void apply(std::vector<boost::shared_ptr<TreeNode> > pNodes, boost::shared_ptr<TreeNode> pChild);
    
    /**
     * Returns the best cluster found by the heuristic.
     * 
     * @return The best cluster found.
     */
    boost::shared_ptr<TreeNode> getBestCluster();
    
    /**
     * Returns the best parent node calculated for the node specified before.
     * 
     * @return The best parent node found
     */
    boost::shared_ptr<TreeNode> getBestParentNode();
    
  private:
    
    /**
     * Returns the direction vector between two objects.
     * 
     * @param first The first object.
     * @param second The second object.
     */
    Eigen::Vector3d getDirectionVector(const boost::shared_ptr<Object> first,
					const boost::shared_ptr<Object> second);

    /**
     * Maximal percentage of breaks in object relation.
     * If there are more breaks between the objects this candidate relation is ruled out.
     */
    double mStaticBreakRatio;
    
    /**
     * Minimal percentage of the time objects are related relation.
     * Objects must be at least related for this percent of all frames.
     */    
    double mTogetherRatio;
    
    /**
     * Maximal allowed angular distance between two objects.
     * It the distance is greater, we have a break in the relation between the two objects.
     */
    double mMaxAngleDeviation;
  };
}
