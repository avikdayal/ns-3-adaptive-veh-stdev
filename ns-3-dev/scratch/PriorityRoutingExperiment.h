#ifndef PRIORITY_ROUTING_EXPERIMENT_H
#define PRIORITY_ROUTING_EXPERIMENT_H

using namespace ns3;
using namespace dsr;

class VanetRoutingExperiment;
/**
 * \ingroup wave
 * \brief The VanetRoutingExperiment class implements a wifi app that
 * allows VANET routing experiments to be simulated
 */
class PriorityRoutingExperiment : public VanetRoutingExperiment {
public:
  /**
   * \brief Constructor
   * \return none
   */
  PriorityRoutingExperiment();
  virtual void OverrideConfigs();
  virtual void SetDefaultAttributeValues();
  virtual ~PriorityRoutingExperiment();
  virtual void ProcessOutputs();

protected:
private:
};
#endif
