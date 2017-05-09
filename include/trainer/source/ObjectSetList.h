/**

Copyright (c) 2016, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global includes
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// Local includes
#include "trainer/source/ObjectSet.h"


#include <ISM/common_type/ObjectSet.hpp>

namespace SceneModel {
  
  /**
   * This class represents a list of object sets. An object set the observations of a single object over time.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ObjectSetList  {
  public:

    /**
     * Constructor.
     */
    ObjectSetList();
    
    /**
     * Destructor.
     */
    ~ObjectSetList();
    
    /**
     * Returns the set with the given identifier, if there is any.
     * 
     * @param identifier Name if the set identifier.
     * @return Set with the given identifier or null pointer, if there's no such set.
     */
    boost::shared_ptr<ISM::ObjectSet> getSetIfExists(std::string identifier);
    
  public:
    
    /**
     * A list of objects represented by the trajectory.
     */
    std::vector<boost::shared_ptr<ISM::ObjectSet> > mObjectSets;
  };
}
