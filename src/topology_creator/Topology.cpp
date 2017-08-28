/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "topology_creator/Topology.h"

namespace SceneModel
{

Topology::Topology(): mUsedInOptimization(false), mEvaluated(false), mCostValid(false)
{ }

Topology::~Topology() { }

void Topology::setEvaluationResult(double pAverageRecognitionRuntime, double pFalsePositives, double pFalseNegatives)
{
    mAverageRecognitionRuntime = pAverageRecognitionRuntime;
    mFalsePositives = pFalsePositives;
    mFalseNegatives = pFalseNegatives;
    mEvaluated = true;
}

double Topology::getAverageRecognitionRuntime() const
{
    if (!mEvaluated)
        throw std::runtime_error("In Topology: trying to access average recogntition runtime without having evaluated the topology first.");
    return mAverageRecognitionRuntime;
}

double Topology::getFalsePositives() const
{
    if (!mEvaluated)
        throw std::runtime_error("In Topology: trying to access number of false postives without having evaluated the topology first.");
    return mFalsePositives;
}

double Topology::getFalseNegatives() const
{
    if (!mEvaluated)
        throw std::runtime_error("In Topology: trying to access number of false negatives without having evaluated the topology first.");
    return mFalseNegatives;
}

bool Topology::isEvaluated() const
{
    return mEvaluated;
}

void Topology::setCost(double pCost)
{
    mCost = pCost;
    mCostValid = true;
}

double Topology::getCost() const
{
    if (!mCostValid)
        throw std::runtime_error("In Topology: trying to access cost without having calculated it first.");
    return mCost;
}

bool Topology::isCostValid() const
{
    return mCostValid;
}

void Topology::setTree(boost::shared_ptr<TreeNode> pTree)
{
    mTree = pTree;
}

boost::shared_ptr<TreeNode> Topology::getTree()
{
    if (!mTree)
        throw std::runtime_error("In Topology: trying to access tree without having set it first.");

    // after rearrangement in learning, the pointer points to an inner node (through parent pointers, the whole tree is still intact)
    while(mTree->mParent)
        mTree = mTree->mParent;

    return mTree;
}

bool Topology::isTreeValid() const
{
    if (!mTree) return false;
    else return true;
}

}

