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
#include <boost/shared_ptr.hpp>

// Local includes
#include "trainer/TreeNode.h"

#include "trainer/source/ObjectSetList.h"

namespace SceneModel {
  
  /**
   * An abstract interface for a clustering heuristic used to create clusters for building the tree.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class AbstractHeuristic {
  public:

    /**
     * Constructor.
     * 
     * @param pDescription Description of the heuristic.
     */
    AbstractHeuristic(std::string pDescription);
    
    /**
     * Destructor.
     */
    virtual ~AbstractHeuristic();

    /**
     * Applies the heuristic to the trajectory set.
     * 
     * @param pNodes The set of nodes containing the trajectories that this heuristic should be applied to.
     */
    virtual void apply(std::vector<boost::shared_ptr<TreeNode> > pNodes);
    
    /**
     * Applies the heuristic to the trajectory set, specifies the child node.
     * 
     * @param pNodes The set of nodes containing the trajectories that this heuristic should be applied to.
     * @param pChild The node that should act as child node.
     */
    virtual void apply(std::vector<boost::shared_ptr<TreeNode> > pNodes, boost::shared_ptr<TreeNode> pChild);
    
    /**
     * Returns the best cluster found by the heuristic.
     * 
     * @return The best cluster found.
     */
    virtual boost::shared_ptr<TreeNode> getBestCluster();
    
    /**
     * Returns the best parent node calculated for the node specified before.
     * 
     * @return The best parent node found
     */
    virtual boost::shared_ptr<TreeNode> getBestParentNode();
    
    /**
     * Resets the heuristic.
     */
    void reset();
    
    /**
     * Operator overwrite for heuristic comparison.
     */
    bool operator < (const AbstractHeuristic& heuristic) const;
    
  private:
    
    /**
     * Description of the heuristic.
     */
    std::string mDescription;
    
  protected:
    /**
     * The score for the best cluster.
     */
    double score;
    
    /**
     * A list of candidates for the best cluster.
     */
    std::vector<boost::shared_ptr<TreeNode> > candidates;
  };
}
