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

// Package includes
#include <boost/shared_ptr.hpp>

#include <asr_msgs/AsrSceneGraph.h>

// Local includes
#include "trainer/TreeNode.h"
#include "trainer/AbstractTrainer.h"

#include "trainer/source/AbstractSource.h"
#include "trainer/source/PbdSceneGraphSource.h"

#include "trainer/generator/AbstractGraphGenerator.h"
#include "trainer/generator/topology_tree/TopologyTreeGenerator.h"

namespace SceneModel {

/**
 * Trainer returning a tree with references representing an object relation graph topology.
 */
class TopologyTreeTrainer: public AbstractTrainer
{
public:

  /**
   * Constructor.
   * @param pRelations  the relations representing the topology to be transformed into a tree.
   */
  TopologyTreeTrainer(std::vector<boost::shared_ptr<Relation>> pRelations);

  /**
   * Destructor.
   */
  ~TopologyTreeTrainer();

  /**
   * Adds a AsrSceneGraph message for conversion into a trajectory.
   *
   * @param pMessage The messages to add to the source.
   */
  void addSceneGraphMessages(std::vector<ISM::ObjectSetPtr> pMessages);

private:

  /**
   * A source for collecting and converting AsrSceneGraph messages.
   */
  boost::shared_ptr<PbdSceneGraphSource> pbdSource;
};

}
