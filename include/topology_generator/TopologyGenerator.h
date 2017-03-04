#pragma once

#include <boost/shared_ptr.hpp>
#include "map"

#include "topology_generator/Relation.h"

#include "topology_generator/ConnectivityChecker.h"
#include "topology_generator/Topology.h"

namespace SceneModel {

//using boost::filesystem::path;

class TopologyGenerator
{
    public:
        TopologyGenerator(const std::vector<std::string>& pAllObjectTypes, unsigned int pMaxNeighbourCount);

        std::vector<boost::shared_ptr<Topology>> generateNeighbours(boost::shared_ptr<Topology> pFrom);
        std::vector<boost::shared_ptr<Topology>> generateStarTopologies();

        boost::shared_ptr<Topology> generateFullyMeshedTopology();
        boost::shared_ptr<Topology> generateRandomTopology();

        std::vector<boost::shared_ptr<Topology>> generateAllConnectedTopologies();

    private:
        boost::shared_ptr<ConnectivityChecker> mConnectivityChecker;

        std::vector<std::string> mAllObjectTypes;

        std::vector<std::vector<bool>> selectRandomNeighbours(std::vector<std::vector<bool>>& pNeighbours);

        /**
         * Convert bitvector representing the relations into topology and set topology's identifier
         * @param pBitvector    representing the relations
         * @return  Topology representing the same relations
         */
        boost::shared_ptr<Topology> convertBitvectorToTopology(const std::vector<bool> & pBitvector);

        std::vector<bool> convertTopologyToBitvector(boost::shared_ptr<Topology> pTopology);

        /*std::vector<std::vector<bool>> filterBitvectors(std::vector<std::vector<bool>> pBitvectors);
        std::vector<boost::shared_ptr<Topology>> convertBitvectors(std::vector<std::vector<bool>> pBitvectors);

        void logNeighbourGeneration(const std::vector<boost::shared_ptr<Topology>> & pNeighbours,
                boost::shared_ptr<Topology> pFrom, const unsigned int pTotalNumber);

        unsigned int mNumRelations;
        unsigned int mUpperRelationLimit;*/

        unsigned int mMaxNeighbourCount;

        std::vector<std::vector<bool>> calculateNeighbours(std::vector<bool> pFrom);

        bool mRemoveRelations;
        bool mSwapRelations;

};
}
