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
