#pragma once

// Local includes
#include "trainer/source/ObjectSet.h"

namespace SceneModel {

class Relation
{

public:
    Relation(std::string pObjectTypeA, std::string pObjectTypeB);
    std::string getObjectTypeA() const;
    std::string getObjectTypeB() const;
    bool containsObject(const std::string& pType) const;
    std::string getOtherType(const std::string& pFirstType) const;

private:
    std::string mObjectTypeA;
    std::string mObjectTypeB;
};

}
