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

#include "topology_creator/Topology.h"

namespace SceneModel {

/**
 * Abstract Interface to TopologyCreator, which generates different topologies of object graphs.
 */
class AbstractTopologyCreator
{
public:

    /**
     * Generates a set of neighbouring topologies from the given one using up to three operations depending on the parameters passed in constructor.
     * Operations are adding, removing and swapping relations.
     * @param pFrom the topology to generate the neighbours for.
     * @return  the set of neighbours of this topology.
     */
    virtual std::vector<boost::shared_ptr<Topology>> generateNeighbours(boost::shared_ptr<Topology> pFrom) = 0;

    /**
     * Generates all possible star topologies containing all the object types passed in constructor.
     * A star topology is one where all objects are connected only to a sigle reference object.
     * @return the set of all possible star topologies for the given object types.
     */
    virtual std::vector<boost::shared_ptr<Topology>> generateStarTopologies() = 0;

    /**
     * Generates the fully meshed topolgy containing all the object types from constructor.
     * @return the fully meshed topology.
     */
    virtual boost::shared_ptr<Topology> generateFullyMeshedTopology() = 0;

    /**
     * Generates a random topology containing the object types.
     * @return a random topology.
     */
    virtual boost::shared_ptr<Topology> generateRandomTopology() = 0;

    /**
     * Generates all connected topologies containing the object types from constructor.
     * @return all connected topologies.
     */
    virtual std::vector<boost::shared_ptr<Topology>> generateAllConnectedTopologies() = 0;

};
}

