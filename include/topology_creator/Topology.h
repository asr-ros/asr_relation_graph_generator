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

#include <boost/shared_ptr.hpp>

#include "topology_creator/Relation.h"
#include "trainer/TreeNode.h"

namespace SceneModel
{

// similar to lib_ism:
/**
 * Container class for a topology.
 */
class Topology
{
public:
    /**
     * Constructor.
     */
    Topology();

    /**
     * Destructor.
     */
    ~Topology();

    /**
     * The relations of which this topology is made of.
     */
    std::vector<boost::shared_ptr<Relation>> mRelations;

    /**
     * A unique identifier
     */
    std::string mIdentifier;

    /**
     * Whether this topology has already been visited during optimization.
     */
    bool mUsedInOptimization;

    /**
     * Set the evaluation result of this topology.
     * @param pAverageRecognitionRuntime    The average recognition runtime of test sets during evaluation.
     * @param pFalsePositives               The number of false positives encountered during evaluation
     * @param pFalseNegatives               The number of false negatives encountered during evaluation.
     */
    void setEvaluationResult(double pAverageRecognitionRuntime, double pFalsePositives, double pFalseNegatives);

    /**
     * Get the average recognition runtime of test sets during evaluation.
     * @return The average recognition runtime of test sets during evaluation.
     */
    double getAverageRecognitionRuntime() const;

    /**
     * Get the number of false positives encountered during evaluation.
     * @return The number of false positives encountered during evaluation.
     */
    double getFalsePositives() const;

    /**
     * Get the number of false negatives encountered during evaluation.
     * @return The number of false negatives encountered during evaluation.
     */
    double getFalseNegatives() const;

    /**
     * Returns whether the topology has been evaluated.
     * @return whether the topology has been evaluated.
     */
    bool isEvaluated() const;

    /**
     * Set the cost calculated for the tree generated from the relations of this topology.
     * @param pCost the cost calculated for the tree generated from the relations of this topology.
     */
    void setCost(double pCost);

    /**
     * Get the cost calculated for the tree generated from the relations of this topology.
     * @return the cost calculated for the tree generated from the relations of this topology.
     */
    double getCost() const;

    /**
     * Returns whether the cost is valid.
     * @return whether the cost is valid.
     */
    bool isCostValid() const;

    /**
     * Set the tree that was generated for the relations of this topology.
     * @param pTree the tree that was generated for the relations of this topology.
     */
    void setTree(boost::shared_ptr<TreeNode> pTree);

    /**
     * Get the tree that was generated for the relations of this topology.
     * @return the tree that was generated for the relations of this topology.
     */
    boost::shared_ptr<TreeNode> getTree();

    /**
     * Returns whether the tree is valid.
     * @return whether the tree is valid.
     */
    bool isTreeValid() const;

private:
    /**
     * Whether the result of the evaluation of this topology is valid.
     */
    bool mEvaluated;

    /**
     * The number of false positives encountered during evaluation
     */
    unsigned int mFalsePositives;

    /**
     * The average recognition runtime of test sets during evaluation.
     */
    double mAverageRecognitionRuntime;

    /**
     * The number of false negatives encountered during evaluation.
     */
    unsigned int mFalseNegatives;

    /**
     * Whether the cost calculated for the tree generated from the relations of this topology is valid.
     */
    bool mCostValid;

    /**
     * The cost calculated for the tree generated from the relations of this topology.
     */
    double mCost;

    /**
     * The tree that was generated for the relations of this topology.
     * Check for nullpointer when checking validity.
     */
    boost::shared_ptr<TreeNode> mTree;

};

}
