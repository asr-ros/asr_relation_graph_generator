#include <iostream>
#include <boost/lexical_cast.hpp>

#include "topology_generator/Topology.h"
#include "topology_generator/TopologyGenerator.h"
#include "trainer/generator/topology_tree/TopologyTreeGenerator.h"
#include "trainer/TreeNode.h"
#include "trainer/source/ObjectSetList.h"

namespace SceneModel
{
/**
 * Transforms a given topology into a tree and prints it to console
 * @param pTopologyType     Type of the topology, for output.
 * @param pTopology         Topology to be transformed.
 * @param pObjectSetList    List of trajectories from which the original topology was built.
 */
void printTree(const std::string& pTopologyType, const boost::shared_ptr<Topology>& pTopology, const ObjectSetList& pObjectSetList)
{
    std::cout << "Transforming " << pTopologyType << " topology " << pTopology->mIdentifier << " into tree." << std::endl;
    TopologyTreeGenerator ttgen;
    ttgen.setRelations(pTopology->mRelations);
    boost::shared_ptr<TreeNode> root;
    ttgen.buildTree(pObjectSetList, root);
    std::cout << "Tree of " << pTopologyType << " topology:" << std::endl;
    root->printTreeToConsole(0);
    std::cout << "===================================================" << std::endl;
}

}

/**
 * Generates all topologies for a given set of object types and prints out the results.
 */
int main(int agrc, char** argv)
{
    unsigned int numAdditionalObjects = 3;
    std::vector<std::string> objectTypes;

    objectTypes.push_back("Smacks");
    for (unsigned int i = 0; i < numAdditionalObjects; i++)
    {
        objectTypes.push_back("VitalisSchoko-" + boost::lexical_cast<std::string>(i));
    }

    bool allNeighbours = false; // activates old creation of all connected topologies.
    std::cout << "Object types are: ";
    for (std::string type: objectTypes) std::cout << type << ", ";
    std::cout << std::endl;

    unsigned int numObjectTypes = objectTypes.size();
    unsigned int maxNeighbours = numObjectTypes * numObjectTypes +1;    // higher than possible maximum
    SceneModel::TopologyGenerator topgen(objectTypes, maxNeighbours);

    // Create dummies of the object sets:
    std::vector<boost::shared_ptr<SceneModel::ObjectSet>> objectSets;
    for (std::string type: objectTypes)
    {
        std::vector<boost::shared_ptr<SceneModel::Object>> object;
        asr_msgs::AsrObservations observation;
        observation.type = type;
        boost::shared_ptr<SceneModel::Object> objectDummy(new SceneModel::Object(observation));
        object.push_back(objectDummy);  // each contains only a single object
        boost::shared_ptr<SceneModel::ObjectSet> objectSet(new SceneModel::ObjectSet(type + "_dummy"));
        objectSet->mObjects = object;
        objectSets.push_back(objectSet);
    }
    SceneModel::ObjectSetList objectSetList;
    objectSetList.mObjectSets = objectSets;

    std::cout << "Generating star topologies:";
    std::vector<boost::shared_ptr<SceneModel::Topology>> starTopologies = topgen.generateStarTopologies();
    for (boost::shared_ptr<SceneModel::Topology> star:starTopologies)
        std::cout << " " << star->mIdentifier;
    std::cout << std::endl;

    std::cout << "Transforming star topologies into trees." << std::endl;

    for (unsigned int i = 0; i < starTopologies.size(); i++)
    {
        std::cout << "---------------------------------------------------" << std::endl;
        boost::shared_ptr<SceneModel::Topology> star = starTopologies[i];

        SceneModel::printTree("star " + boost::lexical_cast<std::string>(i), star, objectSetList);
    }
    std::cout << "===================================================" << std::endl;

    std::cout << "Generating random topology." << std::endl;
    boost::shared_ptr<SceneModel::Topology> random = topgen.generateRandomTopology();
    SceneModel::printTree("random", random, objectSetList);

    std::cout << "Generating fully meshed topology." << std::endl;
    boost::shared_ptr<SceneModel::Topology> fullyMeshed = topgen.generateFullyMeshedTopology();
    SceneModel::printTree("fully meshed", fullyMeshed, objectSetList);
    std::cout << "===================================================" << std::endl;

    if (allNeighbours)  // old method
    {
        std::cout << "Trying to reach all possible topologies from the fully meshed one." << std::endl;
        std::vector<boost::shared_ptr<SceneModel::Topology>> neighboursToVisit;
        neighboursToVisit.push_back(fullyMeshed);
        std::vector<std::string> seenNeighbourIDs;
        seenNeighbourIDs.push_back(fullyMeshed->mIdentifier);
        unsigned int visitedCounter = 0;
        while(!neighboursToVisit.empty())
        {
            boost::shared_ptr<SceneModel::Topology> from = neighboursToVisit[0];
            neighboursToVisit.erase(neighboursToVisit.begin());

            if (!from->mRelations.empty())
            {
                SceneModel::printTree("current", from, objectSetList);
            }
            else
            {
                std::cout << "Transforming current topology " << from->mIdentifier << " into tree." << std::endl;
                std::cout << "No relations: Empty tree." << std::endl;
                std::cout << "===================================================" << std::endl;
            }

            std::cout << "Generating neighbours:";
            std::vector<boost::shared_ptr<SceneModel::Topology>> newNeighbours = topgen.generateNeighbours(from);
            unsigned int uniqueCounter = 0;
            for (boost::shared_ptr<SceneModel::Topology> newNeighbour: newNeighbours)
            {
                std::cout << " " << newNeighbour->mIdentifier;
                if (std::find(seenNeighbourIDs.begin(), seenNeighbourIDs.end(), newNeighbour->mIdentifier) == seenNeighbourIDs.end())   // not yet visited
                {
                    std::cout << "*";   // Mark unvisited.
                    neighboursToVisit.push_back(newNeighbour);
                    seenNeighbourIDs.push_back(newNeighbour->mIdentifier);
                    uniqueCounter++;
                }
            }
            std::cout << std::endl << newNeighbours.size() << " neighbours found. " << uniqueCounter << " not yet visited." << std::endl;
            std::cout << "===================================================" << std::endl;
            visitedCounter++;
        }
        std::cout << "Done." << std::endl;
        std::cout << "Visited " << visitedCounter << " topologies in total." << std::endl;
        std::cout << "===================================================" << std::endl;
    }
    else    // new method
    {
        std::cout << "Generating all connected topologies." << std::endl;
        std::vector<boost::shared_ptr<SceneModel::Topology>> allConnectedTopologies = topgen.generateAllConnectedTopologies();
        std::cout << "===================================================" << std::endl;
        for (boost::shared_ptr<SceneModel::Topology> top: allConnectedTopologies)
        {
            SceneModel::printTree("current", top, objectSetList);
        }
    }
}
