/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "topology_creator/TopologyCreator.h"

namespace SceneModel {

TopologyCreator::TopologyCreator(const std::vector<std::string>& pAllObjectTypes, unsigned int pMaxNeighbourCount, bool pRemoveRelations, bool pSwapRelations):
    mAllObjectTypes(pAllObjectTypes), mMaxNeighbourCount(pMaxNeighbourCount), mRemoveRelations(pRemoveRelations), mSwapRelations(pSwapRelations)
{
    mConnectivityChecker = boost::shared_ptr<ConnectivityChecker>(new ConnectivityChecker(pAllObjectTypes.size()));
}

TopologyCreator::~TopologyCreator()
{ }

std::vector<boost::shared_ptr<Topology>> TopologyCreator::generateNeighbours(boost::shared_ptr<Topology> pFrom)
{
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Generating neighbours (c indicates connectedness): ";

    std::vector<bool> bitvector = convertTopologyToBitvector(pFrom);
    std::vector<std::vector<bool>> neighbours = calculateNeighbours(bitvector);
    std::vector<std::vector<bool>> selectedNeighbours;

    for (std::vector<bool> i: neighbours)
    {
        for (bool bit: i)
        {
            if (bit) std::cout << "1";
            else std::cout << "0";
        }
        if (mConnectivityChecker->isConnected(i))
        {
            std::cout << "c";
            selectedNeighbours.push_back(i);
        }
        std::cout << " ";
    }
    std::cout << std::endl;

    if (mMaxNeighbourCount < selectedNeighbours.size())
    {
        std::cout << "Found " << selectedNeighbours.size() << " neighbours, maximum is " << mMaxNeighbourCount << ". Selecting random neighbours." << std::endl;
        selectedNeighbours = selectRandomNeighbours(selectedNeighbours);
        if (selectedNeighbours.size() != mMaxNeighbourCount)
            throw std::runtime_error("In TopologyCreator: number of randomly selected neighbours (" + boost::lexical_cast<std::string>(selectedNeighbours.size()) + ") was not equal to maximum.");
    }
    std::vector<boost::shared_ptr<Topology>> result;
    std::cout << "Selected neighbours: ";
    for (std::vector<bool> i: selectedNeighbours)
    {
        for (bool bit: i)
        {
            if (bit) std::cout << "1";
            else std::cout << "0";
        }
        std::cout << " ";
        result.push_back(convertBitvectorToTopology(i));
    }
    std::cout << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;

    return result;
}

std::vector<boost::shared_ptr<Topology>> TopologyCreator::generateStarTopologies()
{
    std::vector<boost::shared_ptr<Topology>> result;
    // from lib_ism:
    std::vector<std::vector<bool>> starBitVectors;
    unsigned int numObjects = mAllObjectTypes.size();
    unsigned int numAllRelations = (numObjects - 1) * numObjects / 2;

    for (unsigned int i = 0; i < numObjects; ++i)
    {
        std::vector<bool> star(numAllRelations, 0);
        unsigned int pos = 0;
        for (unsigned int j = 0; j < i; ++j) {
            star[pos + i - (j + 1)] = 1;
            pos += numObjects - (j + 1);
        }
        for (unsigned int k = pos; k < pos + numObjects - 1 - i; ++k) {
            star[k] = 1;
        }
        starBitVectors.push_back(star);
    }

    for (std::vector<bool> starBitVector: starBitVectors)
        result.push_back(convertBitvectorToTopology(starBitVector));
    return result;
}


boost::shared_ptr<Topology> TopologyCreator::generateFullyMeshedTopology()
{
    unsigned int numObjects = mAllObjectTypes.size();
    std::vector<bool> bitvector((numObjects - 1) * numObjects / 2, true);   // contains all possible relations.
    boost::shared_ptr<Topology> result = convertBitvectorToTopology(bitvector);
    return result;
}

boost::shared_ptr<Topology> TopologyCreator::generateRandomTopology()
{
    unsigned int numAllObjects = mAllObjectTypes.size();
    unsigned int numAllRelations =  (numAllObjects - 1) * numAllObjects / 2;
    // from lib_ism:
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<unsigned int> dist(0, 1);

    std::vector<bool> relations(numAllRelations, 0);
    do
    {
        for (unsigned int i = 0; i < numAllRelations; i++) relations[i] = dist(eng);
    }
    while (!mConnectivityChecker->isConnected(relations));
    boost::shared_ptr<Topology> result = convertBitvectorToTopology(relations);
    return result;
}

std::vector<boost::shared_ptr<Topology>> TopologyCreator::generateAllConnectedTopologies()
{
    std::vector<boost::shared_ptr<Topology>> result;
    boost::shared_ptr<SceneModel::Topology> fullyMeshed = generateFullyMeshedTopology();
    result.push_back(fullyMeshed);
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

        std::cout << "Generating neighbours of topology " << from->mIdentifier << ":";
        std::vector<boost::shared_ptr<SceneModel::Topology>> newNeighbours = generateNeighbours(from);
        unsigned int uniqueCounter = 0;
        for (boost::shared_ptr<SceneModel::Topology> newNeighbour: newNeighbours)
        {
            std::cout << " " << newNeighbour->mIdentifier;
            if (std::find(seenNeighbourIDs.begin(), seenNeighbourIDs.end(), newNeighbour->mIdentifier) == seenNeighbourIDs.end())   // not yet visited
            {
                std::cout << "*";   // Mark unvisited.
                neighboursToVisit.push_back(newNeighbour);
                seenNeighbourIDs.push_back(newNeighbour->mIdentifier);
                result.push_back(newNeighbour);
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

    return result;
}

std::vector<std::vector<bool>> TopologyCreator::selectRandomNeighbours(std::vector<std::vector<bool>>& pNeighbours)
{
    if (pNeighbours.size() <= mMaxNeighbourCount) return pNeighbours;   // if amount of neighbours is already smaller than maximum amount: return neighbours
    //from lib_ism: Sort from by #relations.
    struct compare {
        bool operator() (const std::vector<bool> & first, const std::vector<bool> & second) {
            return first.size() > second.size();
        }
    };

    std::sort(pNeighbours.begin(), pNeighbours.end(), compare());

    std::random_device rd;
    std::mt19937 eng(rd());
    std::normal_distribution<double> dist(0, pNeighbours.size() / 2);

    std::vector<std::vector<bool>> selectedNeighbours;

    for (unsigned int i = 0; i < mMaxNeighbourCount; i++)
    {
        unsigned int randIndex;
        do {
            randIndex = std::abs(dist(eng));
        } while (randIndex >= pNeighbours.size());

    selectedNeighbours.push_back(pNeighbours[randIndex]);
    pNeighbours.erase(pNeighbours.begin() + randIndex);

    }

    return selectedNeighbours;
}

boost::shared_ptr<Topology> TopologyCreator::convertBitvectorToTopology(const std::vector<bool> & pBitvector)
{
    unsigned int numObjectTypes = mAllObjectTypes.size();
    std::vector<boost::shared_ptr<Relation>> relations;
    unsigned int bitvectorindex = 0;
    for (unsigned int i = 0; i < numObjectTypes - 1; i++)
    {
        std::string typeA = mAllObjectTypes[i];
        for (unsigned int j = i + 1; j < numObjectTypes; j++)
        {
            std::string typeB = mAllObjectTypes[j];
            if (pBitvector[bitvectorindex])
            {
                boost::shared_ptr<Relation> newRelation(new Relation(typeA, typeB));
                relations.push_back(newRelation);
            }
            bitvectorindex++;
        }
    }
    boost::shared_ptr<Topology> result(new Topology());
    result->mRelations = relations;
    // set result's identifier:
    std::string identifier = "";
    for (bool bit: pBitvector)
    {
        if (bit) identifier += "1";
        else identifier += "0";
    }
    result->mIdentifier = identifier;
    return result;
}

std::vector<bool> TopologyCreator::convertTopologyToBitvector(boost::shared_ptr<Topology> pTopology)
{
    unsigned int numObjectTypes = mAllObjectTypes.size();
    unsigned int numAllRelations = (numObjectTypes - 1) * numObjectTypes / 2;
    std::vector<bool> result = std::vector<bool>(numAllRelations, false);
    std::map<std::string, unsigned int> indicesByTypes;
    for (unsigned int i = 0; i < numObjectTypes; i++)
        indicesByTypes[mAllObjectTypes[i]] = i;

    std::vector<boost::shared_ptr<Relation>> relations = pTopology->mRelations;
    for (boost::shared_ptr<Relation> relation: relations)
    {
        std::string typeA = relation->getObjectTypeA();
        std::string typeB = relation->getObjectTypeB();
        unsigned int indexA = indicesByTypes[typeA];
        unsigned int indexB = indicesByTypes[typeB];
        unsigned int indexMin = std::min(indexA, indexB);
        unsigned int indexMax = std::max(indexA, indexB);
        unsigned int bitvectorindex = indexMin * numObjectTypes - indexMin * (indexMin + 1) / 2 + (indexMax - indexMin) - 1;
        result[bitvectorindex] = true;
    }
    return result;
}

std::vector<std::vector<bool>> TopologyCreator::calculateNeighbours(std::vector<bool> pFrom)
{
    // from TopologyCreatorPaper in lib_ism:
    std::vector<std::vector<bool>> neighbours;
    std::vector<unsigned int> ones;
    std::vector<unsigned int> zeros;

    for (unsigned int i = 0; i < pFrom.size(); ++i)
    {
        if (pFrom[i])
        {
            ones.push_back(i);
        } else {
            zeros.push_back(i);
        }
    }

    std::vector<bool> neighbour;
    if (mRemoveRelations)
    {
        //Remove one relation
        for (unsigned int i = 0; i < ones.size(); ++i)
        {
            neighbour = pFrom;
            neighbour[ones[i]] = 0;
            neighbours.push_back(neighbour);
        }
    }

    for (unsigned int i = 0; i < zeros.size(); ++i)
    {
        //Add one relation
        neighbour = pFrom;
        neighbour[zeros[i]] = 1;
        neighbours.push_back(neighbour);

        if (mSwapRelations)
        {
            //Swap one relation
            for (unsigned int j = 0; j < ones.size(); ++j)
            {
                neighbour = pFrom;
                neighbour[zeros[i]] = 1;
                neighbour[ones[j]] = 0;
                neighbours.push_back(neighbour);
            }
        }
    }

    return neighbours;
}

}
