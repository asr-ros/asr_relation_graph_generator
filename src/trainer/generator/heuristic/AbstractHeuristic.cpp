/**

Copyright (c) 2016, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "trainer/generator/heuristic/AbstractHeuristic.h"

namespace SceneModel {
 
  AbstractHeuristic::AbstractHeuristic(std::string pDescription)
  : mDescription(pDescription)
  , score(0)
  {
  }
  
  AbstractHeuristic::~AbstractHeuristic()
  {
  }
  

  void AbstractHeuristic::apply(std::vector<boost::shared_ptr<TreeNode> > pNodes)
  {
    // This method is abstract, so no implementation here.
  }
  
  void AbstractHeuristic::apply(std::vector<boost::shared_ptr<TreeNode> > pNodes, boost::shared_ptr<TreeNode> pChild)
  {
    // This method is abstract, so no implementation here.
  }
  
  boost::shared_ptr<TreeNode> AbstractHeuristic::getBestCluster()
  {
    // This method is abstract, so no implementation here.
    return boost::shared_ptr<TreeNode>();
  }
  
  boost::shared_ptr<TreeNode> AbstractHeuristic::getBestParentNode()
  {
    // This method is abstract, so no implementation here.
    return boost::shared_ptr<TreeNode>();
  }

  void AbstractHeuristic::reset()
  {
    candidates.clear();
    score = 0;
  }
  
  bool AbstractHeuristic::operator < (const AbstractHeuristic& heuristic) const
  {
    return (score < heuristic.score);
  }
  
}
