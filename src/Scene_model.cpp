/**

Copyright (c) 2016, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Scene_model.hpp"

//public
Scene_model::Scene_model(std::string database)
{
  this->trainer = new ISM::Trainer(database);
  ISM::DataCollector::setCollect(true);
  this->trainer->staticBreakRatio = 0.01;
  this->trainer->togetherRatio = 0.90;
  this->trainer->maxAngleDeviation = 45;
  std::cout<<"Begin with training"<<std::endl;
  this->trainer->trainPattern();
  std::cout<<"Fetching data"<<std::endl;
  this->data = ISM::DataCollector::getData();
  std::cout<<"Converting data to vector representation"<<std::endl;
  this->dataVector = dataToVector();
}
  
std::vector<std::map<std::string, std::string> > Scene_model::dataToVector()
{
  std::vector<std::map<std::string, std::string> > dataVector;
  std::map<std::string, std::string> topLevelReference;
  topLevelReference.insert(std::pair<std::string, std::string> ("objectType", 
								data->tracksWithRef.at(0).refTrack->type));    
  topLevelReference.insert(std::pair<std::string, std::string> ("objectId", 
								data->tracksWithRef.at(0).refTrack->observedId));
  topLevelReference.insert(std::pair<std::string, std::string> ("reference", "None"));
  
  std::string referencesReference = "None";
  for (ISM::TracksWithRef referenceIt : data->tracksWithRef)
    {
      std::string reference = referenceIt.refTrack->type;
      for (ISM::TrackPtr object : referenceIt.tracks->tracks)
	{	  
	  std::map<std::string, std::string> dataMap;
	  std::string objectType = object->type;
	  std::string objectId = object->observedId;
	  dataMap.insert(std::pair<std::string, std::string> ("reference", reference));
	  dataMap.insert(std::pair<std::string, std::string> ("objectType", objectType));
	  dataMap.insert(std::pair<std::string, std::string> ("objectId", objectId));
	  dataVector.push_back(dataMap);
	}
    }
  return dataVector;
}

void Scene_model::printData()
{
  //std::vector<TracksWithReference> referenceIt = data->tracksWithReference.begin();
  for (ISM::TracksWithRef referenceIt : data->tracksWithRef )
    {
      std::cout<<"Reference is: "<<referenceIt.refTrack->type<<std::endl;
      for (ISM::TrackPtr track : referenceIt.tracks->tracks)
	{
	  std::cout<<"Object's type is: "<<track->type<<" Observed Id is: "<<track->observedId<<std::endl;
	}
    }
}

void Scene_model::printDataVector()
{
  for (std::map<std::string, std::string> refIt : dataVector)
    { 
      std::cout<<"Reference is: "<<refIt.find("reference")->second<<std::endl;
      std::cout<<"Object's type is: "<<refIt.find("objectType")->second<<" Observed Id is: "
	       <<refIt.find("objectId")->second<<std::endl;
    }
}
  
ISM::CollectedDataPtr Scene_model::getDataCollector()
{
  return data;
}

std::vector<std::map<std::string, std::string> > Scene_model::getDataVector()
 {
   return dataVector;
 }



