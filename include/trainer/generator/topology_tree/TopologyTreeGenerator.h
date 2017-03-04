#pragma once

#include "topology_generator/Relation.h"
#include "trainer/TreeNode.h"
#include "trainer/generator/AbstractGraphGenerator.h"
#include "trainer/source/ObjectSetList.h"

namespace SceneModel {

class TopologyTreeGenerator: public AbstractGraphGenerator
{

public:
    /**
     * Constructor.
     */
    TopologyTreeGenerator();

    /**
     * Destructor.
     */
    ~TopologyTreeGenerator();

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

    void setRelations(std::vector<boost::shared_ptr<Relation>> pRelations);

private:
    std::vector<boost::shared_ptr<Relation>> mRelations;

};

}
