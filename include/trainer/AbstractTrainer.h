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
#include <string>
#include <vector>

// Package includes
#include <boost/shared_ptr.hpp>

// Local includes
#include "trainer/TreeNode.h"

#include "trainer/source/AbstractSource.h"
#include "trainer/source/ObjectSetList.h"

#include "trainer/generator/AbstractGraphGenerator.h"

namespace SceneModel {

  /**
   * This abstract class is the prototype of a tree learner. It takes object trajectories and converts them to a tree symbolizing the hierarchical relations between objects.
   * 
   * @author Joachim Gehrung
   * @version See SVN
   */
  class AbstractTrainer
  {
  public:
    /**
     * Constructor.
     */
    AbstractTrainer();
    
    /**
     * Destructor.
     */
    ~AbstractTrainer();
    
    /**
     * Loads the trajectories and builds the tree.
     */
    void loadTrajectoriesAndBuildTree();
    
    /**
     * Loads the trajectories and builds the tree, using the object with the specified type as root node.
     * 
     * @param pType Type of the object that should be forced as root node.
     */
    void loadTrajectoriesAndBuildTree(std::string pType);
    
    /**
     * Returns the tree.
     */
    boost::shared_ptr<TreeNode> getTree();
    
  protected:
    
    /**
     * The root element of the tree of hierarchical relations.
     */
    boost::shared_ptr<TreeNode> root;
    
    /**
     * The source for the trajectories the generator converts into a hierarachical tree.
     */
    boost::shared_ptr<AbstractSource> source;
    
    /**
     * Converts the trajectories delivered by the source into a tree representing the hierarchical object relationships.
     */
    boost::shared_ptr<AbstractGraphGenerator> generator;
  };
}
