#pragma once

// Package includes
#include <boost/shared_ptr.hpp>

#include <asr_msgs/AsrSceneGraph.h>

// Local includes
#include "trainer/TreeNode.h"
#include "trainer/AbstractTrainer.h"

#include "trainer/source/AbstractSource.h"
#include "trainer/source/PbdSceneGraphSource.h"

#include "trainer/generator/AbstractGraphGenerator.h"

namespace SceneModel {

class FullyMeshedGenerator: public AbstractGraphGenerator
{
public:
    /**
     * Constructor.
     */
    FullyMeshedGenerator();

    /**
     * Destructor.
     */
    ~FullyMeshedGenerator();

    /**
     * Builds the tree.
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
};

}
