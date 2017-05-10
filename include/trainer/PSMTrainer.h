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

// Package includes
#include <boost/shared_ptr.hpp>

// Local includes
#include "trainer/TreeNode.h"
#include "trainer/AbstractTrainer.h"

#include "trainer/source/AbstractSource.h"
#include "trainer/source/PbdSceneGraphSource.h"

#include "trainer/generator/AbstractGraphGenerator.h"
#include "trainer/generator/heuristic/HeuristicalTreeGenerator.h"
#include <ISM/common_type/ObjectSet.hpp>

namespace SceneModel {

  /**
   * This class is the trainer used by the Probabilistic Scene Model.
   * 
   * @author Joachim Gehrung
   * @version See SVN
   */
  class PSMTrainer : public AbstractTrainer
  {
  public:
    
    /**
     * Constructor.
     * 
     * @param pStaticBreakRatio Maximal percentage of breaks in object relation.
     * @param pTogetherRatio Minimal percentage of the frames objects are related relation.
     * @param pMaxAngleDeviation Maximal allowed angular distance between two objects.
     */
    PSMTrainer(const double pStaticBreakRatio, const double pTogetherRatio, const double pMaxAngleDeviation);
    
    /**
     * Destructor.
     */
    ~PSMTrainer();
 
    /**
     * Adds a ISM::ObjectSet for conversion into a trajectory.
     * 
     * @param pMessage The messages to add to the source.
     */
    void addSceneGraphMessages(std::vector<ISM::ObjectSetPtr> pMessages);
    
  private:
    
    /**
     * A source for collecting and converting ISM::ObjectSets.
     */
    boost::shared_ptr<PbdSceneGraphSource> pbdSource;
  };
}
