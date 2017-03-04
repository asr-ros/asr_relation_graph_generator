#include "topology_generator/ConnectivityChecker.h"

namespace SceneModel {

    ConnectivityChecker::ConnectivityChecker(unsigned int pNumObjects): mNumObjects(pNumObjects)
    {}

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
