/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "trainer/generator/topology_tree/TopologyTreeGenerator.h"
#include <ros/ros.h>

namespace SceneModel {

TopologyTreeGenerator::TopologyTreeGenerator(): AbstractGraphGenerator() {}

TopologyTreeGenerator::~TopologyTreeGenerator() {}

void TopologyTreeGenerator::buildTree(ObjectSetList pObjectSets, boost::shared_ptr<TreeNode>& pRoot)
{
    buildTree(pObjectSets, pRoot, mRelations[0]->getObjectTypeA());    // Builds tree with first object found as node.
}

void TopologyTreeGenerator::buildTree(ObjectSetList pTrajectories, boost::shared_ptr<TreeNode>& pRoot, std::string pType)
{
    std::cout << "Creating tree from topology." << std::endl;
    // Create nodes for each type and map them to their type. bool indicates whether the node was already added to the tree.
    std::map<std::string, std::pair<boost::shared_ptr<TreeNode>, bool>> nodesByType;
    for (boost::shared_ptr<ISM::ObjectSet> trajectory: pTrajectories.mObjectSets)
    {
        boost::shared_ptr<TreeNode> newNode(new TreeNode(trajectory));
        nodesByType[trajectory->objects[0]->type] = std::pair<boost::shared_ptr<TreeNode>, bool>(newNode, false);
    }
    std::vector<std::string> typesToVisit;
    typesToVisit.push_back(pType);
    std::vector<boost::shared_ptr<Relation>> remainingRelations = mRelations;
    while (!remainingRelations.empty())
    {
        std::cout << "Number of relations left is " << remainingRelations.size() << ". ";
        std::string currentType = typesToVisit[0];
        typesToVisit.erase(typesToVisit.begin());
        std::cout << "Current type is " << currentType << std::endl;
        boost::shared_ptr<TreeNode> currentNode = nodesByType[currentType].first;
        std::vector<boost::shared_ptr<Relation>> newRemainingRelations;
        // Iterate over all relations and build tree. Relation graph is assumed to be connected, so, every type will be visited.
        for (boost::shared_ptr<Relation> relation: remainingRelations)
        {
            if (relation->containsObject(currentType))
            {
                std::string otherType = relation->getOtherType(currentType);
                std::cout << "Found relation to " << otherType << ". ";
                if (!nodesByType[otherType].second)   // type's node was not yet added to tree
                {
                    std::cout << "Node not yet in tree. ";
                    currentNode->addChild(nodesByType[otherType].first);  // add to tree
                    nodesByType[otherType].second = true;           // mark as added
                    typesToVisit.push_back(otherType);
                    std::cout << "Added node." << std::endl;
                }
                else                                    // type's node has already been added to tree: add reference to it instead
                {
                    std::cout << "Node already in tree. ";
                    boost::shared_ptr<TreeNode> reference(new TreeNode(nodesByType[otherType].first->mObjectSet));
                    // references have no mChildren:
                    reference->mChildren = std::vector<boost::shared_ptr<TreeNode>>();
                    reference->mReferenceTo = nodesByType[otherType].first;
                    reference->mIsReference = true;
                    currentNode->addChild(reference);
                    std::cout << "Added reference." << std::endl;
                }
            }
            else newRemainingRelations.push_back(relation);
        }
        remainingRelations = newRemainingRelations;
    }
    std::cout << "All relations visited." << std::endl;
    pRoot = nodesByType[pType].first;
    pRoot->setIDs();
}

void TopologyTreeGenerator::setRelations(std::vector<boost::shared_ptr<Relation>> pRelations)
{
    mRelations = std::vector<boost::shared_ptr<Relation>>();
    // Only keep one relation from duplicates (the last one found):
    std::cout << "Removing duplicate relations. " << pRelations.size(); // Old number of relations was " << pRelations.size() << std::endl;
    for (unsigned int i = 0; i < pRelations.size(); i++)
    {
        bool unique = true;
        for (unsigned int j = i + 1; j < pRelations.size(); j++)
        {
            if (pRelations[i]->containsObject(pRelations[j]->getObjectTypeA()) && pRelations[i]->containsObject(pRelations[j]->getObjectTypeB()))
                unique = false;
        }
        if (unique) mRelations.push_back(pRelations[i]);
    }
    std::cout << " -> " << mRelations.size() << " relations." << std::endl;
}

}
