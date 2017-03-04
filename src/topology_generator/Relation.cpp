#include "topology_generator/Relation.h"

namespace SceneModel {

Relation::Relation(std::string pObjectTypeA, std::string pObjectTypeB):
    mObjectTypeA(pObjectTypeA), mObjectTypeB(pObjectTypeB)
{
}

std::string Relation::getObjectTypeA() const
{
    return mObjectTypeA;
}
std::string Relation::getObjectTypeB() const
{
    return mObjectTypeB;
}

bool Relation::containsObject(const std::string& pType) const
{
    return (mObjectTypeA == pType || mObjectTypeB == pType);
}

std::string Relation::getOtherType(const std::string& pFirstType) const
{
    if (pFirstType == mObjectTypeA) return mObjectTypeB;
    else if (pFirstType == mObjectTypeB) return mObjectTypeA;
    else return "";
}

}
