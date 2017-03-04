#include "trainer/TopologyTreeTrainer.h"

namespace SceneModel {

  TopologyTreeTrainer::TopologyTreeTrainer(std::vector<boost::shared_ptr<Relation>> pRelations)
  : AbstractTrainer()
  {
    std::cout << "Building tree from topology." << std::endl;

    // Initilaize a source that translates AsrSceneGraph messages.
    pbdSource = boost::shared_ptr<PbdSceneGraphSource>(new PbdSceneGraphSource());
    source = pbdSource;

    // Initialize the generator.
    boost::shared_ptr<TopologyTreeGenerator> gen(new TopologyTreeGenerator());
    gen->setRelations(pRelations);
    generator = gen;

    //std::cout << "Tree from topology built." << std::endl;
    std::cout << "Topology tree trainer initialized." << std::endl;
  }

  TopologyTreeTrainer::~TopologyTreeTrainer()
  {
  }

  void TopologyTreeTrainer::addSceneGraphMessages(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph> > pMessages)
  {
    BOOST_FOREACH(boost::shared_ptr<const asr_msgs::AsrSceneGraph> pMessage, pMessages)
      pbdSource->addSceneGraphMessage(*pMessage);
  }

}
