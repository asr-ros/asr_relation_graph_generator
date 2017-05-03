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
#include <queue>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// Local includes
#include "trainer/TreeNode.h"

#include "trainer/source/Object.h"
#include "trainer/source/ObjectSet.h"
#include "trainer/source/ObjectSetList.h"

#include "trainer/generator/AbstractGraphGenerator.h"
#include "trainer/generator/heuristic/AbstractHeuristic.h"
#include "trainer/generator/heuristic/DirectionRelationHeuristic.h"
#include "../../../../../lib_ism/libism/ISM/common_type/ObjectSet.hpp"


namespace SceneModel {
  
  /**
   * Realization of the abstract tree generator class. It creates the tree by evaluating predefined relations and clustering the trajectories with the best score.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class HeuristicalTreeGenerator : public AbstractGraphGenerator {
  public:

    /**
     * Constructor.
     */
    HeuristicalTreeGenerator();
    
    /**
     * Destructor.
     */
    ~HeuristicalTreeGenerator();
    
    /**
     * Builds the tree using agglomerative clustering.
     * 
     * @param pObjectSets The list of trajectories to build the tree from.
     * @param pRoot The root node of the tree.
     */
    void buildTree(ObjectSetList pObjectSets, boost::shared_ptr<TreeNode>& pRoot);
    
    /**
     * Builds the tree, forces the object with the given type as root node and appends a new node to what ever node it is appropriate
     * 
     * @param pTrajectories The list of trajectories to build the tree from.
     * @param pRoot The root node of the tree.
     * @param pType Type of the object that should be forced as root node.
     */
    void buildTree(ObjectSetList pTrajectories, boost::shared_ptr<TreeNode>& pRoot, std::string pType);
    
    /**
     * Adds a heuristic.
     * 
     * @param pHeuristic The heuristic to add.
     */
    void addHeuristic(boost::shared_ptr<AbstractHeuristic> pHeuristic);
    
    /**
     * Find the heuristic that generates the highest score.
     * 
     * @param pClusters The list of trajectories to build the tree from.
     */
    void evaluateHeuristics(std::vector<boost::shared_ptr<TreeNode> > pClusters);
    
    /**
     * Find the heuristic that generates the highest score.
     * 
     * @param pClusters The list of trajectories to build the tree from.
     * @param pChild The node that must be the child node.
     */
    void evaluateHeuristics(std::vector<boost::shared_ptr<TreeNode> > pClusters,
			    boost::shared_ptr<TreeNode> pChild);
  private:
    
    /**
     * Deletes empty object sets.
     * 
     * @param pObjectSets The list of trajectories to build the tree from.
     */
    void deleteEmptyObjectSets(ObjectSetList& pObjectSets);
    
    /**
     * A list of heuristics used for clustering.
     */
    std::vector<boost::shared_ptr<AbstractHeuristic> > mHeuristics;
  };
}
