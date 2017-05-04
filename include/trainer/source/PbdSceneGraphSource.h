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
#include <ros/ros.h>
// Package includes
#include <boost/shared_ptr.hpp>

#include <asr_msgs/AsrSceneGraph.h>

// Local includes
#include "trainer/source/ObjectSetList.h"
#include "trainer/source/AbstractSource.h"


#include <ISM/common_type/ObjectSet.hpp>


namespace SceneModel {
  
  /**
   * An abstract interface for loading trajectories.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class PbdSceneGraphSource : public AbstractSource  {
  public:

    /**
     * Constructor.
     */
    PbdSceneGraphSource();
    
    /**
     * Destructor.
     */
    ~PbdSceneGraphSource();
    
    /**
     * Adds a AsrSceneGraph message for conversion into a trajectory.
     * 
     * @param pMessage The message to add to the source.
     */
    void addSceneGraphMessage(std::vector<ISM::ObjectSetPtr> pMessage);
    
  };
}
