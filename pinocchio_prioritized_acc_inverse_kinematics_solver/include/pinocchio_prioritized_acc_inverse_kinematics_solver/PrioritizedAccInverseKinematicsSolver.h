#ifndef PINOCCHIO_PRIORITIZED_ACC_INVERSE_KINEMATICS_SOLVER_H
#define PINOCCHIO_PRIORITIZED_ACC_INVERSE_KINEMATICS_SOLVER_H

#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio_aik_constraint/IKConstraint.h>
#include <prioritized_qp_base/PrioritizedQPBaseSolver.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>

namespace prioritized_acc_inverse_kinematics_solver {
  /*
    事前に、変数のddqを0としてpinocchio::forwardKinematics(model,data,q,v)を行うこと.
    jointControllable: 動かして良いjoint (free jointは6DOF扱い)
    ikc_list: タスクたち. vectorの前の要素の方が高優先度. 0番目の要素は必ず満たすと仮定しQPを解かない
    prevTasks: 前回のtasksを入れる. 自動的に更新される.
   */
  class IKParam {
  public:
    std::vector<double> ddqWeight; // ddqWeight.size() == dimの場合、探索変数の各要素について、wn+weをddqWeight倍する.
    double wn = 1e0;
    std::vector<double> wnVec; // wnVec.size() == ikc_list.size()の場合、wnの代わりにこっちを使う
    double we = 1e-6;
    std::vector<double> weVec; // weVec.size() == ikc_list.size()の場合、weの代わりにこっちを使う
    int debugLevel = 0;
    std::vector<bool> jointControllable; // jointControllable.size() = model.nv. ルートリンクを含む自由度数．その自由度を探索変数に入れるかどうか。
  };
  bool solveAIK (const pinocchio::Model& model,
                 pinocchio::Data& data,
                 Eigen::VectorXd& ddqResult,
                 const std::vector<std::vector<std::shared_ptr<aik_constraint::IKConstraint> > >& ikc_list,
                 std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
                 const IKParam& param = IKParam(),
                 std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc = [](std::shared_ptr<prioritized_qp_base::Task>& task, int debugLevel){
                   std::shared_ptr<prioritized_qp_osqp::Task> taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
                   if(!taskOSQP){
                     task = std::make_shared<prioritized_qp_osqp::Task>();
                     taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
                   }
                   taskOSQP->settings().verbose = debugLevel;
                   taskOSQP->settings().max_iter = 4000;
                   taskOSQP->settings().eps_abs = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
                   taskOSQP->settings().eps_rel = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
                   taskOSQP->settings().scaled_termination = true;// avoid too severe termination check
                 }
                 );

}

#endif
