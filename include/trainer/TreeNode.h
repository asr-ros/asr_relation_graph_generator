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
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

// Local includes
#include "trainer/source/ObjectSet.h"

namespace SceneModel {
  
  /**
   * The data structure representing the object relation tree. The nodes represented by this class are objects with a given type and instance id. Edges from this node (represented by a list of nodes) are references to child-objects one level deeper down the tree. 
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class TreeNode : public boost::enable_shared_from_this<TreeNode>  {
  public:

    /**
     * Constructor.
     * 
     * @param pObjectSet An object set containing all observations of a single object over time.
     */
    TreeNode(boost::shared_ptr<ObjectSet> pObjectSet);
    
    /**
     * Copy Constructor.
     * 
     * @param pRoot The root node of the tree to copy.
     */
    TreeNode(boost::shared_ptr<TreeNode> pRoot);
    
    /**
     * Destructor.
     */
    ~TreeNode();
    
    /**
     * Rebuilds the tree so that the node with the given type is the new root node.
     * 
     * @param pType The type of the new node.
     * @return The new root node.
     */
    boost::shared_ptr<TreeNode> setNewRootNodeByType(std::string pType);
    
    /**
     * Returns the object set associated with the node.
     * 
     * @return The object set holding the observations of the object associated with the tree.
     */
    boost::shared_ptr<ObjectSet> getObjectSet();
    
    /**
     * Returns the children of this node.
     * 
     * @return The child nodes.
     */
    std::vector<boost::shared_ptr<TreeNode> > getChildren();
    
    /**
     * Adds a child to the node.
     * 
     * @param child The node to add as a child.
     */
    void addChild(boost::shared_ptr<TreeNode> pChild);
    
    /**
     * Returns the number of nodes in the tree.
     * 
     * @return The number of nodes in the tree.
     */
    unsigned int getNumberOfNodes();
    
    /**
     * Returns a boost shared pointer from this.
     * 
     * @return A boost shared pointer from this.
     */
    boost::shared_ptr<TreeNode> f();
    
    /**
     * Prints the generated tree to console.
     * 
     * @param node The root of tree that should be printed.
     * @param space The number of leading spaces.
     */
    void printTreeToConsole(unsigned int space);
    
  public:
    
    /**
     * An object trajectory containing all observations of a single object over time.
     */
    boost::shared_ptr<ObjectSet> mObjectSet;
    
    /**
     * A reference to the parent node.
     * Required for changing the structure of the tree later.
     */
    boost::shared_ptr<TreeNode> mParent;
    
    /**
     * A list of all child nodes.
     */
    std::vector<boost::shared_ptr<TreeNode> > mChildren;
    
  private:
    /**
     * Recursively reassigns a new parent node.
     * 
     * @param pParent the new parent node.
     */
    void reassignNewParentNode(boost::shared_ptr<TreeNode> pParent);
  };
}
