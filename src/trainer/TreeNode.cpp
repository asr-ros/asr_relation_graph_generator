/**

Copyright (c) 2016, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "trainer/TreeNode.h"

namespace SceneModel {
 
  TreeNode::TreeNode(boost::shared_ptr<ObjectSet> pObjectSet)
  : mObjectSet(pObjectSet)
  {
  }
  
  TreeNode::TreeNode(boost::shared_ptr<TreeNode> pRoot)
  {
    mObjectSet = pRoot->mObjectSet;
    mParent = boost::shared_ptr<TreeNode>();
    
    BOOST_FOREACH(boost::shared_ptr<TreeNode> child, pRoot->mChildren)
      addChild(boost::shared_ptr<TreeNode>(child));
  }
  
  TreeNode::~TreeNode()
  {
  }
  
  boost::shared_ptr<TreeNode> TreeNode::setNewRootNodeByType(std::string pType)
  {      
    boost::shared_ptr<SceneModel::TreeNode> newRoot;
    
    // Search the new root node.
    std::queue<boost::shared_ptr<TreeNode> > nodesToVisit;
    nodesToVisit.push(f());
    
    while(nodesToVisit.size() > 0)
    {
        boost::shared_ptr<TreeNode> node = nodesToVisit.front();
        nodesToVisit.pop();

        if (!(node->mIsReference))  // A reference node can not be root, has no children, and its parent node must appear as parent node of a non-reference node elsewhere.
        {

            // New root node found?
            if(node->mObjectSet->mObjects[0]->mType.compare(pType) == 0)
            {
                newRoot = node;
                break;
            }

            // Add the child nodes to queue.
            BOOST_FOREACH(boost::shared_ptr<TreeNode> child, node->mChildren)
                    nodesToVisit.push(child);

            // Add parent node to queue, if it exists.
            if(node->mParent)
                nodesToVisit.push(node->mParent);
        }
    }

    // Reorganize the tree from the root node.
    if(newRoot->mParent)
    {
      newRoot->mParent->reassignNewParentNode(newRoot);
      newRoot->mParent = boost::shared_ptr<TreeNode>();
    }

    std::vector<boost::shared_ptr<TreeNode>> reversedReferences = updateReferences(newRoot);//(newRoot->mID);
    for (boost::shared_ptr<TreeNode> reversedReference: reversedReferences) newRoot->addChild(reversedReference);

    // Assign unique IDs to each node, counted in order of depth-first search.
    newRoot->setIDs();
    
    // Return the new root node.
    return newRoot;
  }
  
  boost::shared_ptr<ObjectSet> TreeNode::getObjectSet()
  {
    return mObjectSet;
  }
  
  std::vector<boost::shared_ptr<TreeNode> > TreeNode::getChildren()
  {
    return mChildren;
  }
  
  void TreeNode::addChild(boost::shared_ptr<TreeNode> pChild)
  {
    pChild->mParent = f();
    mChildren.push_back(pChild);
  }
  
  unsigned int TreeNode::getNumberOfNodes()
  {
    unsigned int result = 1;
    
    BOOST_FOREACH(boost::shared_ptr<TreeNode> child, mChildren)
      result += child->getNumberOfNodes();
    
    return result;
  }
  
  boost::shared_ptr<TreeNode> TreeNode::f()
  {
    return shared_from_this();
  }
  
  void TreeNode::printTreeToConsole(unsigned int pSpace)
  {
    // Generate leading spaces.
    for(unsigned int i = 0; i < pSpace; i++)
      std::cout << " ";
    std::cout << "-";
    
    // Print types and number of children and observations, as well as ID and ID of referenced (own if not reference): ID mID -> referencedID
    unsigned int referencing = mID;
    if (mIsReference) referencing = mReferenceTo->mID;
    std::cout << mObjectSet->mObjects[0]->mType << "(" << mChildren.size() << "/" << mObjectSet->mObjects.size() << ")" << " ID " << mID << " -> " << referencing << std::endl;
    
    // Print children.
    if (!mIsReference)  // References have no children
        for (boost::shared_ptr<SceneModel::TreeNode> node: mChildren)
            node->printTreeToConsole(pSpace + 1);
  }
  
  void TreeNode::reassignNewParentNode(boost::shared_ptr<TreeNode> pParent)
  {
    if(mParent)
      mParent->reassignNewParentNode(f());
    
    // Overwrite parent pointer and add as child to parent.
    mParent = pParent;
    pParent->mChildren.push_back(f());
    
    // Remove new parent from own child list.
    for(unsigned int i = 0; i < mChildren.size(); i++)
    {
      if(mChildren[i] == pParent)
      {
	mChildren.erase(mChildren.begin() + i);
	break;
      }
    }
  }

  void TreeNode::setIDs()
  {
      unsigned int id = 0;
      this->mID = id;
      for (boost::shared_ptr<TreeNode> child: mChildren) child->updateIDs(id, false);
      for (boost::shared_ptr<TreeNode> child: mChildren) child->updateIDs(id, true);
  }

  void TreeNode::updateIDs(unsigned int& pID, bool pUpdateReferenceIDs)
  {
      if (!mIsReference ^ pUpdateReferenceIDs)
      {
          pID++;
          this->mID = pID;
      }
      if (!mIsReference) // References dont't have children
          for (boost::shared_ptr<TreeNode> child: mChildren) child->updateIDs(pID, pUpdateReferenceIDs);
  }

std::vector<boost::shared_ptr<TreeNode>> TreeNode::updateReferences(boost::shared_ptr<TreeNode> pRoot)
{
    std::vector<boost::shared_ptr<TreeNode>> reversedReferences;
    std::vector<boost::shared_ptr<TreeNode>> toDelete;
    if (mIsReference) return std::vector<boost::shared_ptr<TreeNode>>();    // references have no children, so there is nothing to reverse
    for (boost::shared_ptr<TreeNode> child: this->mChildren)
    {
        if (child->mIsReference)
        {
            if (child->mReferenceTo == pRoot)
            {
                boost::shared_ptr<TreeNode> reversedReference(new TreeNode(this->mObjectSet));
                reversedReference->mReferenceTo = boost::shared_ptr<TreeNode>(f());
                reversedReference->mIsReference = true;
                // references have no mChildren:
                reversedReference->mChildren = std::vector<boost::shared_ptr<TreeNode>>();
                reversedReferences.push_back(reversedReference);
                toDelete.push_back(child);
            }
        }
        else    // references don't have children, so it is unneccessary to do this on a reference
        {
            std::vector<boost::shared_ptr<TreeNode>> childReversedReferences = child->updateReferences(pRoot);
            for (boost::shared_ptr<TreeNode> childReversedReference: childReversedReferences) reversedReferences.push_back(childReversedReference);
        }
    }
    for (boost::shared_ptr<TreeNode> childToDelete: toDelete)
    {
        for(unsigned int i = 0; i < mChildren.size(); i++)
        {
            if(mChildren[i] == childToDelete)
            {
                mChildren.erase(mChildren.begin() + i);
                break;
            }
        }
    }
    return reversedReferences;
}

}
