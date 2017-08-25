
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "dpcommon/dp_type.h"

namespace Dp {

  class Object;
  class Link;
  class GaitGenerator;
  class RppIKSolver;

  typedef struct {
    double angle[4][3];
  } QuadrupedAngle;

  typedef std::vector<QuadrupedAngle> QuadrupedAngles;

  class QuadrupedWalker {
    public:
      static const std::string kFLLinkName[3];
      static const std::string kFRLinkName[3];
      static const std::string kBLLinkName[3];
      static const std::string kBRLinkName[3];

    public:
      /* robot should be initialized before calling this function */
      QuadrupedWalker(std::shared_ptr<Object> robot);

      virtual ~QuadrupedWalker();

    public:
      QuadrupedAngles& Walk(Vector3d dstep = {0.05, 0.00, 0.00}, size_t num_of_gaits = 10);

    private:
      std::shared_ptr<Object> robot_;
      std::shared_ptr<GaitGenerator> ggen_;
      std::shared_ptr<RppIKSolver> ik_[4];

      std::shared_ptr<Link> base_lnk_;
      std::shared_ptr<Link> fl_roll_lnk_;
      std::shared_ptr<Link> fr_roll_lnk_;
      std::shared_ptr<Link> bl_roll_lnk_;
      std::shared_ptr<Link> br_roll_lnk_;

      QuadrupedAngles angs_;
  };

}

