#pragma once

#include <boost/shared_ptr.hpp>
#include <iostream>

namespace SceneModel {

class ConnectivityChecker
{
public:
    /**
     * Constructor.
     * @param pNumObjects   The number of objects in the topologies to be checked.
     */
    ConnectivityChecker(unsigned int pNumObjects);

    /**
     * Checks whether a topology represented by a bitvector is connected.
     * @param pBitvector    a bitvector representing the relations in the topology. true indicates the relation exists.
     * @return  whether the topology represented by the bitvector is connected.
     */
    bool isConnected(std::vector<bool> pBitvector);

private:
    /**
     * The number of objects in the topologies to be checked.
     */
    unsigned int mNumObjects;
};
}
