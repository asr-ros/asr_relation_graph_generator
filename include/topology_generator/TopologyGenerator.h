/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include "map"

#include "topology_generator/Relation.h"

#include "topology_generator/ConnectivityChecker.h"
#include "topology_generator/Topology.h"

namespace SceneModel {

/**
 * Generates different topologies of object graphs.
 */
class TopologyGenerator
{
public:
    /**
     * Constructor.
     * @param pAllObjectTypes       all object types to appear in the graphs.
     * @param pMaxNeighbourCount    the maximum count of neighbours to be created.
     * @param pRemoveRelations      Whether to use the removal of relations as an operation when generating neighbours.
     * @param pSwapRelations        Whether to use the swapping of relations as an operation when generating neighbours.
     */
    TopologyGenerator(const std::vector<std::string>& pAllObjectTypes, unsigned int pMaxNeighbourCount, bool pRemoveRelations, bool pSwapRelations);

    /**
     * Generates a set of neighbouring topologies from the given one using up to three operations depending on the parameters passed in constructor.
     * Operations are adding, removing and swapping relations.
     * @param pFrom the topology to generate the neighbours for.
     * @return  the set of neighbours of this topology.
     */
    std::vector<boost::shared_ptr<Topology>> generateNeighbours(boost::shared_ptr<Topology> pFrom);

    /**
     * Generates all possible star topologies containing all the object types passed in constructor.
     * A star topology is one where all objects are connected only to a sigle reference object.
     * @return the set of all possible star topologies for the given object types.
     */
    std::vector<boost::shared_ptr<Topology>> generateStarTopologies();

    /**
     * Generates the fully meshed topolgy containing all the object types from constructor.
     * @return the fully meshed topology.
     */
    boost::shared_ptr<Topology> generateFullyMeshedTopology();

    /**
     * Generates a random topology containing the object types.
     * @return a random topology.
     */
    boost::shared_ptr<Topology> generateRandomTopology();

    /**
     * Generates all connected topologies containing the object types from constructor.
     * @return all connected topologies.
     */
    std::vector<boost::shared_ptr<Topology>> generateAllConnectedTopologies();

private:
    /**
     * The ConnectivityChecker used to check whether topologies are connected.
     */
    boost::shared_ptr<ConnectivityChecker> mConnectivityChecker;

    /**
     * The object types the topologies connect through relations.
     */
    std::vector<std::string> mAllObjectTypes;

    /**
     * Randomly selects mMaxNeighbourCount neighbours from the given set.
     * @param pNeighbours   The set of neighbours to randomly pick from.
     * @return the neighbours picked randomly.
     */
    std::vector<std::vector<bool>> selectRandomNeighbours(std::vector<std::vector<bool>>& pNeighbours);

    /**
     * Convert bitvector representing the relations into topology and set topology's identifier
     * @param pBitvector    representing the relations
     * @return  Topology representing the same relations
     */
    boost::shared_ptr<Topology> convertBitvectorToTopology(const std::vector<bool> & pBitvector);

    /**
     * Convert topology into bitvector reporesenting its realtions.
     * @param pTopology the topology to convert
     * @return the bitvector representing the relations of the topology.
     */
    std::vector<bool> convertTopologyToBitvector(boost::shared_ptr<Topology> pTopology);

    /**
     * Maximum amount of neighbours to return.
     */
    unsigned int mMaxNeighbourCount;

    /**
     * Calculate neighbours of topology represented as bitvector.
     * @param pFrom topology represented as bitvector for which to create neighbours
     * @return the neighbours created.
     */
    std::vector<std::vector<bool>> calculateNeighbours(std::vector<bool> pFrom);

    /**
     * Whether to use the removal of relations as an operation when generating neighbours.
     */
    bool mRemoveRelations;

    /**
     * Whether to use the swapping of relations as an operation when generating neighbours.
     */
    bool mSwapRelations;

};
}
