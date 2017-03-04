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
#include "trainer/generator/fully_meshed/FullyMeshedGenerator.h"

namespace SceneModel {

class FullyMeshedTrainer: public AbstractTrainer
{
public:

  /**
   * Constructor.
   *
   * @param pStaticBreakRatio Maximal percentage of breaks in object relation.
   * @param pTogetherRatio Minimal percentage of the frames objects are related relation.
   * @param pMaxAngleDeviation Maximal allowed angular distance between two objects.
   */
  FullyMeshedTrainer();

  /**
   * Destructor.
   */
  ~FullyMeshedTrainer();

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
