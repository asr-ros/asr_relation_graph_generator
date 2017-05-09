/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "trainer/TopologyTreeTrainer.h"

namespace SceneModel {

  TopologyTreeTrainer::TopologyTreeTrainer(std::vector<boost::shared_ptr<Relation>> pRelations)
  : AbstractTrainer()
  {
    std::cout << "Building tree from topology." << std::endl;

    // Initilaize a source that translates AsrSceneGraph messages.
    pbdSource = boost::shared_ptr<PbdSceneGraphSource>(new PbdSceneGraphSource());
    source = pbdSource;

    // Initialize the generator.
    boost::shared_ptr<TopologyTreeGenerator> gen(new TopologyTreeGenerator());
    gen->setRelations(pRelations);
    generator = gen;

    std::cout << "Topology tree trainer initialized." << std::endl;
  }

  TopologyTreeTrainer::~TopologyTreeTrainer()
  {
  }

  /*void TopologyTreeTrainer::addSceneGraphMessages(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph> > pMessages)
  {
      for(boost::shared_ptr<const asr_msgs::AsrSceneGraph> pMessage: pMessages)
          pbdSource->addSceneGraphMessage(*pMessage);
  }*/
  void TopologyTreeTrainer::addSceneGraphMessages(std::vector<ISM::ObjectSetPtr> pMessages)
  {
    //BOOST_FOREACH(boost::shared_ptr<const ISM::ObjectSetPtr> pMessage, pMessages)
      pbdSource->addSceneGraphMessage(pMessages);
  }

}
