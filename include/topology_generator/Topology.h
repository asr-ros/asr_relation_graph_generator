#pragma once

#include <boost/shared_ptr.hpp>

#include "topology_generator/Relation.h"
//#include "EvaluationResult.hpp"

namespace SceneModel
{
// from lib_ism:
/**
 * Container class for a topology.
 */
class Topology
{
    public:
    /**
         * The relations of which this topology is made of.
         */
        std::vector<boost::shared_ptr<Relation>> mRelations;

        /**
         * The result of the evaluation of this topology
         */
        //EvaluationResult evaluationResult;

        /**
         * A unique identifier
         */
        std::string mIdentifier;

        /**
         * If the tree that was generated for the relations of this topology is valid
         */
        //bool isValid;

        /**
         * The cost calculated for the tree generated from the relations od this topology
         */
        //double cost;

        /**
         * A unique index for the topology
         */
        //unsigned int index;

};

}
