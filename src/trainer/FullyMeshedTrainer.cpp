#include "trainer/FullyMeshedTrainer.h"

namespace SceneModel {

  FullyMeshedTrainer::FullyMeshedTrainer()
  : AbstractTrainer()
  {
    std::cout << "Building fully meshed tree." << std::endl;

    // Initilaize a source that translates AsrSceneGraph messages.
    pbdSource = boost::shared_ptr<PbdSceneGraphSource>(new PbdSceneGraphSource());
    source = pbdSource;

    // Initialize the generator for building the tree using agglomerative clustering.
    // The heuristic used here is searching for stable relativ poses.
    boost::shared_ptr<FullyMeshedGenerator> gen(new FullyMeshedGenerator());
    generator = gen;

    std::cout << "Fully meshed tree built." << std::endl;
  }

  FullyMeshedTrainer::~FullyMeshedTrainer()
  {
  }

  void FullyMeshedTrainer::addSceneGraphMessages(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph> > pMessages)
  {
    BOOST_FOREACH(boost::shared_ptr<const asr_msgs::AsrSceneGraph> pMessage, pMessages)
      pbdSource->addSceneGraphMessage(*pMessage);
  }

}
