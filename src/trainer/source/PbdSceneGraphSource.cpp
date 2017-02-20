/**

Copyright (c) 2016, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "trainer/source/PbdSceneGraphSource.h"

namespace SceneModel {
 
  PbdSceneGraphSource::PbdSceneGraphSource()
  {
  }
  
  PbdSceneGraphSource::~PbdSceneGraphSource()
  {
  }
  
  void PbdSceneGraphSource::addSceneGraphMessage(asr_msgs::AsrSceneGraph pMessage)
  {
    // TODO The merging mechanism of two trajectories with the same name does require testing.
    
    // Iterator pointing to one of the AsrNodes in AsrSceneGraph.
    std::vector<asr_msgs::AsrNode>::const_iterator sceneElementsIterator;
    
    // Iterator pointing to one of the messages containing vision features for the AsrNode currently taken into account.
    std::vector<asr_msgs::AsrObservations>::const_iterator observationIterator;
    
    // Iterate over all AsrNode messages (= all object trajectories in the scene)
    for(sceneElementsIterator = pMessage.scene_elements.begin(); sceneElementsIterator != pMessage.scene_elements.end(); sceneElementsIterator++)
    {
      // The set of objects wrapped by the AsrSceneGraph (in form of AsrNodes)
      boost::shared_ptr<ObjectSet> set = mObjectSetList.getSetIfExists(pMessage.identifier);
      
      // If it doesn't exist, create it and add it to the list of sets.
      if(!set)
      {
	set.reset(new ObjectSet(pMessage.identifier));
	mObjectSetList.mObjectSets.push_back(set);
      }
      
      // Go through the whole trajectory of vision measurements for the current object (= all object observations part of the current object trajectory).
      for(observationIterator = sceneElementsIterator->track.begin(); observationIterator != sceneElementsIterator->track.end(); observationIterator++)
      {
	// Convert iterator into an object.
	boost::shared_ptr<Object> object(new Object(*observationIterator));
	
	// Add a new observed object to the object set.
	set->mObjects.push_back(object);
      }
    }
  }
  
}
