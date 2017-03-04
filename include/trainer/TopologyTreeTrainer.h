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
  void addSceneGraphMessages(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph> > pMessages);

private:

  /**
   * A source for collecting and converting AsrSceneGraph messages.
   */
  boost::shared_ptr<PbdSceneGraphSource> pbdSource;
};

}
