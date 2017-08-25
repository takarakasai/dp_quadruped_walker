
#include "dp_quadruped_walker/dp_quadruped_walker.h"

#include <string>
#include <cstdio>

#include "dproki/dproki.h"
#include "dp_gait_generator/dp_gait_generator.h"
#include "rpp_ik_solver/rpp_ik_solver.h"

#if 0
static std::string kFRTipLnkName = "FR/KNEE_PITCH";
static std::string kFLTipLnkName = "FL/KNEE_PITCH";
static std::string kBRTipLnkName = "BR/KNEE_PITCH";
static std::string kBLTipLnkName = "BL/KNEE_PITCH";
#endif

namespace Dp {

  const std::string QuadrupedWalker::kFLLinkName[3] = {"FL/HIP_ROLL", "FL/HIP_PITCH", "FL/KNEE_PITCH"};
  const std::string QuadrupedWalker::kFRLinkName[3] = {"FR/HIP_ROLL", "FR/HIP_PITCH", "FR/KNEE_PITCH"};
  const std::string QuadrupedWalker::kBLLinkName[3] = {"BL/HIP_ROLL", "BL/HIP_PITCH", "BL/KNEE_PITCH"};
  const std::string QuadrupedWalker::kBRLinkName[3] = {"BR/HIP_ROLL", "BR/HIP_PITCH", "BR/KNEE_PITCH"};

  QuadrupedWalker::QuadrupedWalker(std::shared_ptr<Object> robot) {
    robot_ = robot;

    base_lnk_ = robot->FindLink("Base");
    auto fr_lnk = robot->FindLink(kFRLinkName[2]);
    auto fl_lnk = robot->FindLink(kFLLinkName[2]);
    auto br_lnk = robot->FindLink(kBRLinkName[2]);
    auto bl_lnk = robot->FindLink(kBLLinkName[2]);
    fl_roll_lnk_ = robot->FindLink(kFLLinkName[0]);
    fr_roll_lnk_ = robot->FindLink(kFRLinkName[0]);
    bl_roll_lnk_ = robot->FindLink(kBLLinkName[0]);
    br_roll_lnk_ = robot->FindLink(kBRLinkName[0]);

    auto bpos   = base_lnk_->WPos();
    auto fr_pos = fr_lnk->GetWTipPos();
    auto fl_pos = fl_lnk->GetWTipPos();
    auto br_pos = br_lnk->GetWTipPos();
    auto bl_pos = bl_lnk->GetWTipPos();

    fprintf(stdout, " WCPOS (%lf, %lf, %lf)\n", bpos[0], bpos[1], bpos[2]);
    fprintf(stdout, " FL (%lf, %lf, %lf), FR (%lf, %lf, %lf)\n",
        fl_pos[0], fl_pos[1], fl_pos[2], fr_pos[0], fr_pos[1], fr_pos[2]);
    fprintf(stdout, " FL (%lf, %lf, %lf), FR (%lf, %lf, %lf)\n",
        bl_pos[0], bl_pos[1], bl_pos[2], br_pos[0], br_pos[1], br_pos[2]);

    Gait gait = {
      fl_pos, fr_pos,
      bl_pos, br_pos,
      bpos,
      NONE,
    };
    ggen_ = std::make_shared<GaitGenerator>(gait);

    auto l3_lnk = fl_lnk;
    auto l2_lnk = fl_lnk->GetParent();
    auto l1_lnk = fl_lnk->GetParent()->GetParent();

    auto l3 = l3_lnk->LTipPos();
    auto l2 = l2_lnk->LTipPos();
    auto l1 = l1_lnk->LTipPos();

    fprintf(stdout, "l1 %s : %lf, %lf, %lf\n", l1_lnk->GetName().c_str(), l1[0], l1[1], l1[2]) ;
    fprintf(stdout, "l2 %s : %lf, %lf, %lf\n", l2_lnk->GetName().c_str(), l2[0], l2[1], l2[2]) ;
    fprintf(stdout, "l3 %s : %lf, %lf, %lf\n", l3_lnk->GetName().c_str(), l3[0], l3[1], l3[2]) ;

    auto boffset = base_lnk_->WPos();
    auto floffset = fl_roll_lnk_->WPos() - boffset;
    auto froffset = fr_roll_lnk_->WPos() - boffset;
    auto bloffset = bl_roll_lnk_->WPos() - boffset;
    auto broffset = br_roll_lnk_->WPos() - boffset;

    fprintf(stdout, " FL : %lf %lf %lf (0.172, 0.08 0.00 ?)\n", floffset[0], floffset[1], floffset[2]);
    fprintf(stdout, " FR : %lf %lf %lf\n", froffset[0], froffset[1], froffset[2]);
    fprintf(stdout, " BL : %lf %lf %lf\n", bloffset[0], bloffset[1], bloffset[2]);
    fprintf(stdout, " BR : %lf %lf %lf\n", broffset[0], broffset[1], broffset[2]);

    // TODO: -0.10?
    // TODO: use link parameter
    //ik_ = std::make_shared<RppIKSolver>(0.0, -0.10, -0.10);
    ik_[0] = std::make_shared<RppIKSolver>(0.0, 0.10, 0.10, (double[3]){floffset[0], floffset[1], floffset[2]});
    ik_[1] = std::make_shared<RppIKSolver>(0.0, 0.10, 0.10, (double[3]){froffset[0], froffset[1], froffset[2]});
    ik_[2] = std::make_shared<RppIKSolver>(0.0, 0.10, 0.10, (double[3]){bloffset[0], bloffset[1], bloffset[2]});
    ik_[3] = std::make_shared<RppIKSolver>(0.0, 0.10, 0.10, (double[3]){broffset[0], broffset[1], broffset[2]});

    fprintf(stdout, "------------------------------------------------------------ constructor finished\n");

    return;
  }

  QuadrupedWalker::~QuadrupedWalker() {
    return;
  }

  QuadrupedAngles& QuadrupedWalker::Walk(Vector3d dstep, size_t nog /* num of gaits */) {
    const Gaits& gaits = ggen_->GenerateWalk(dstep/*dstep*/, nog/*nos*/);

    //DebugDump(gaits);

    fprintf(stdout, "=================================================================\n");
    fprintf(stdout, "=================================================================\n");
    fprintf(stdout, " (%lf, %lf, %lf)\n", dstep[0], dstep[1], dstep[2]);
    for (auto gait : gaits) {
      fprintf(stdout, "------------------------------------------------------------>>>\n");

      DebugDump(gait);

      Gait lgait = gait.LocalGait();

      double x, y, z;
      RppIKSolutions sol[4];

      x = lgait.fl[0]; y = lgait.fl[1]; z  = lgait.fl[2]; ik_[0]->Solve(x, y, z); sol[0] = ik_[0]->solutions_;
      fprintf(stdout, " tgt FL : %lf %lf %lf\n", x, y, z);
      x = lgait.fr[0]; y = lgait.fr[1]; z  = lgait.fr[2]; ik_[1]->Solve(x, y, z); sol[1] = ik_[1]->solutions_;
      fprintf(stdout, " tgt FR : %lf %lf %lf\n", x, y, z);
      x = lgait.bl[0]; y = lgait.bl[1]; z  = lgait.bl[2]; ik_[2]->Solve(x, y, z); sol[2] = ik_[2]->solutions_;
      fprintf(stdout, " tgt BL : %lf %lf %lf\n", x, y, z);
      x = lgait.br[0]; y = lgait.br[1]; z  = lgait.br[2]; ik_[3]->Solve(x, y, z); sol[3] = ik_[3]->solutions_;
      fprintf(stdout, " tgt BR : %lf %lf %lf\n", x, y, z);

      fprintf(stdout, " %zd %zd %zd %zd\n", sol[0].size(), sol[1].size(), sol[2].size(), sol[3].size());
      fprintf(stdout, " tgt (%lf, %lf, %lf) --> (%lf, %lf, %lf) (%lf, %lf, %lf) (%lf, %lf, %lf) (%lf, %lf, %lf)\n",
          x, y, z,
          Math::rad2deg(sol[0][0][0]), Math::rad2deg(sol[0][0][1]), Math::rad2deg(sol[0][0][2]),
          Math::rad2deg(sol[1][0][0]), Math::rad2deg(sol[1][0][1]), Math::rad2deg(sol[1][0][2]),
          Math::rad2deg(sol[2][0][0]), Math::rad2deg(sol[2][0][1]), Math::rad2deg(sol[2][0][2]),
          Math::rad2deg(sol[3][0][0]), Math::rad2deg(sol[3][0][1]), Math::rad2deg(sol[3][0][2])
          );


      QuadrupedAngle ang = {
          {
            {sol[0][0][0], sol[0][0][1], sol[0][0][2]},
            {sol[1][0][0], sol[1][0][1], sol[1][0][2]},
            {sol[2][0][0], sol[2][0][1], sol[2][0][2]},
            {sol[3][0][0], sol[3][0][1], sol[3][0][2]}
          }
      };
      angs_.push_back(ang);
    }

    return angs_;
  }

}

