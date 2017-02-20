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
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

// Package includes
#include <asr_msgs/AsrObservations.h>

namespace SceneModel
{
  
  /**
   * This class represents a single object. It can also be understood of the observation of an object, which is part of a trajectory.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class Object  {
  public:

    /**
     * Constructor.
     * 
     * @param pObject The AsrObject that should be converted into this object.
     */
    Object(asr_msgs::AsrObservations pObject);
    
    /**
     * Destructor.
     */
    ~Object();
    
  public:
    
    /**
     * Type of the object.
     */
    std::string mType;
    
    /**
     * Position of the object.
     */
    boost::shared_ptr<Eigen::Vector3d> mPosition;
    
    /**
     * Orientation of the object.
     */
    boost::shared_ptr<Eigen::Quaternion<double> > mOrientation;
  };
}
