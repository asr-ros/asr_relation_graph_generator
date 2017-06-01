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

// Local includes
#include "trainer/source/ObjectSet.h"

namespace SceneModel {

/**
 * This class represents a relation between to objects represented by their types.
 */
class Relation
{

public:
    /**
     * Constructor
     * @param pObjectTypeA  type of the first object
     * @param pObjectTypeB  type of the second object
     */
    Relation(std::string pObjectTypeA, std::string pObjectTypeB);

    /**
     * Get first object type
     * @return first object type
     */
    std::string getObjectTypeA() const;

    /**
     * Get second object type
     * @return second object type
     */
    std::string getObjectTypeB() const;

    /**
     * Returns whether object of given type participates in the relation
     * @param pType to check
     * @return  whether an object of this type participates in the relation
     */
    bool containsObject(const std::string& pType) const;

    /**
     * Returns the type of the other object participating in this relation given one
     * @param pFirstType    the given type
     * @return  the other type
     */
    std::string getOtherType(const std::string& pFirstType) const;

private:
    /**
     * First object type
     */
    std::string mObjectTypeA;

    /**
     * Second object type
     */
    std::string mObjectTypeB;
};

}
