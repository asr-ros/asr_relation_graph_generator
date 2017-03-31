/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "trainer/generator/fully_meshed/FullyMeshedGenerator.h"
#include <ros/ros.h>

namespace SceneModel {

FullyMeshedGenerator::FullyMeshedGenerator()
: AbstractGraphGenerator()
{
}

FullyMeshedGenerator::~FullyMeshedGenerator()
{
}

void FullyMeshedGenerator::buildTree(ObjectSetList pObjectSets, boost::shared_ptr<TreeNode>& pRoot)
{
    buildTree(pObjectSets, pRoot, pObjectSets.mObjectSets.at(0)->mObjects.at(0)->mType);
}

void FullyMeshedGenerator::buildTree(ObjectSetList pTrajectories, boost::shared_ptr<TreeNode>& pRoot, std::string pType)
{
    std::vector<boost::shared_ptr<TreeNode>> nodes;
    std::vector<std::string> types;
    for (boost::shared_ptr<ObjectSet> set: pTrajectories.mObjectSets)
    {
        nodes.push_back(boost::shared_ptr<TreeNode>(new TreeNode(set)));
        types.push_back(set->mObjects.at(0)->mType);
    }
    if (!nodes.empty())
    {
        unsigned int root_index = 0;
        for (unsigned int i = 0; i < types.size(); i++)
        {
            if (types.at(i) == pType)
            {
                root_index = i;
                break;
            }
        }
        pRoot = nodes.at(root_index);
        for (unsigned int i = 0; i < nodes.size(); i++)
        {
            if (i != root_index) {
                nodes.at(i)->mParent = pRoot;
                for (unsigned int j = i + 1; j < nodes.size(); j++)
                {
                    boost::shared_ptr<TreeNode> reference(new TreeNode(nodes.at(j)->mObjectSet));
                    reference->mParent = nodes.at(i);
                    // references have no mChildren:
                    reference->mChildren = std::vector<boost::shared_ptr<TreeNode>>();
                    reference->mReferenceTo = nodes.at(j);
                    reference->mIsReference = true;
                    nodes.at(i)->mChildren.push_back(reference);
                }
                pRoot->mChildren.push_back(nodes.at(i));
            }
        }
    }
    pRoot->setIDs();
}

}
