/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "topology_creator/ConnectivityChecker.h"

namespace SceneModel {

    ConnectivityChecker::ConnectivityChecker(unsigned int pNumObjects): mNumObjects(pNumObjects)
    {}

    ConnectivityChecker::~ConnectivityChecker()
    { }

    bool ConnectivityChecker::isConnected(std::vector<bool> pBitvector)
    {
        if (pBitvector.size() != (mNumObjects - 1) * mNumObjects / 2)
        {
            std::cout << "Bitvector does not represent the relations properly." << std::endl;
            return false;
        }

        // from lib_ism:
        unsigned int numOnesInVector = 0;
        for (bool bit : pBitvector) numOnesInVector += bit;
        if (numOnesInVector < mNumObjects - 1) return false;    //A topology needs at least #objects - 1 relations to be connected.
        if (numOnesInVector == pBitvector.size()) return true;        // Fully meshed topology.

        // Do BFS to see whether each node can be reached
        // For starting relation: find first one (exists because of check above)
        /*
        unsigned int firstOneIndex = 0;
        while (!pBitvector[firstOneIndex]) firstOneIndex++;
        */

        unsigned int firstObject = 0;
        while (firstObject < mNumObjects - 1)
        {
            bool found = false;
            for (unsigned int j = firstObject + 1; j < mNumObjects; j++)
            {
                if (pBitvector[firstObject * mNumObjects - firstObject * (firstObject + 1) / 2 + j])
                {
                    found = true;
                    break;
                }
            }
            if (found) break;
            else firstObject++;
        }
        //firstOneIndex = firstObject;

        std::vector<unsigned int> nodesToVisit;
        nodesToVisit.push_back(firstObject);
        std::vector<unsigned int> seenNodes;
        seenNodes.push_back(firstObject);
        while (!nodesToVisit.empty())
        {
            unsigned int currentNode = nodesToVisit[0];
            nodesToVisit.erase(nodesToVisit.begin());
            for (unsigned int i = 0; i < mNumObjects; i++)
                if (currentNode != i)
                {
                    unsigned int indexMin = std::min(currentNode, i);
                    unsigned int indexMax = std::max(currentNode, i);
                    unsigned int bitvectorindex = indexMin * mNumObjects - indexMin * (indexMin + 1) / 2 + (indexMax - indexMin) - 1;
                    // if a relation exists from currentNode to i && i was not already found:
                    if (pBitvector[bitvectorindex] && (std::find(seenNodes.begin(), seenNodes.end(), i) == seenNodes.end()))
                    {
                        nodesToVisit.push_back(i);
                        seenNodes.push_back(i);
                    }
                }
        }
        return (seenNodes.size() == mNumObjects);  // Whether all nodes were found
    }
}
