#include <aik_constraint/PositionConstraint.h>
#include <aik_constraint/Jacobian.h>

namespace aik_constraint{

  void PositionConstraint::update (const pinocchio::Model& mode, pinocchio::Data& data, const std::vector<bool>& jointControllable) {
    const pinocchio::SE3 parent_pose = (this->link_) ? this->link_->T() : pinocchio::SE3::Identity(); // world frame
    const pinocchio::SE3& pos = parent_pose * this->localpos_; // world frame
    Eigen::Vector6d act_vel = Eigen::Vector6d::Zero(); // world frame
    if(this->act_link_){
      act_vel.head<3>() += this->act_link_->v();
      act_vel.head<3>() += this->act_link_->w().cross(act_parent_pose.linear() * this->act_localpos_.translation());
      act_vel.tail<3>() += this->act_link_->w();
    }
    act_vel.head<3>() += act_parent_pose.linear() * this->act_localvel_.head<3>();
    act_vel.tail<3>() += act_parent_pose.linear() * this->act_localvel_.tail<3>();
    Eigen::Vector6d act_acc = Eigen::Vector6d::Zero(); // world frame
    if(this->act_link_){
      act_acc.head<3>() += this->act_link_->dv();
      act_acc.head<3>() += this->act_link_->dw().cross(act_parent_pose.linear() * this->act_localpos_.translation()) + this->act_link_->w().cross(act_parent_pose.linear() * this->act_localvel_.head<3>());
      act_acc.tail<3>() += this->act_link_->dw();
    }

    Eigen::Vector6d pos_error; // world frame. A - B
    {
      cnoid::AngleAxis angleAxis = cnoid::AngleAxis(A_pos.linear() * B_pos.linear().transpose());
      pos_error << A_pos.translation() - B_pos.translation() , angleAxis.angle()*angleAxis.axis();
    }
    Eigen::Vector6d vel_error = A_vel - B_vel; // world frame. A - B

    cnoid::Matrix3d eval_R = (this->eval_link_) ? this->eval_link_->R() * this->eval_localR_ : this->eval_localR_;
    Eigen::Vector6d pos_error_eval; // eval frame. A - B
    pos_error_eval.head<3>() = (eval_R.transpose() * pos_error.head<3>()).eval();
    pos_error_eval.tail<3>() = (eval_R.transpose() * pos_error.tail<3>()).eval();
    Eigen::Vector6d vel_error_eval; // eval frame. A - B
    vel_error_eval.head<3>() = (eval_R.transpose() * vel_error.head<3>()).eval();
    vel_error_eval.tail<3>() = (eval_R.transpose() * vel_error.tail<3>()).eval();
    Eigen::Vector6d target_acc = Eigen::Vector6d::Zero(); // eval frame. A - B
    target_acc += this->ref_acc_;
    target_acc -= this->clamp(Eigen::Vector6d(this->pgain_.cwiseProduct(pos_error_eval)), this->maxAccByPosError_);
    target_acc -= this->clamp(Eigen::Vector6d(this->dgain_.cwiseProduct(vel_error_eval)), this->maxAccByVelError_);
    target_acc -= A_acc - B_acc;
    target_acc = this->clamp(target_acc, this->maxAcc_);

    {
      // A-Bの目標加速度を計算し、this->eq_に入れる
      if(this->eq_.rows()!=(this->weight_.array() > 0.0).count()) this->eq_ = Eigen::VectorXd((this->weight_.array() > 0.0).count());
      for(size_t i=0, idx=0; i<6; i++){
        if(this->weight_[i]>0.0) {
          this->eq_[idx] = target_acc[i] * this->weight_[i];
          idx++;
        }
      }
    }

    {
      // 行列の初期化. 前回とcol形状が変わっていないなら再利用
      if(!this->isJointsSame(joints,this->jacobian_joints_)
         || this->A_link_ != this->jacobian_A_link_
         || this->B_link_ != this->jacobian_B_link_){
        this->jacobian_joints_ = joints;
        this->jacobian_A_link_ = this->A_link_;
        this->jacobian_B_link_ = this->B_link_;

        aik_constraint::calc6DofJacobianShape(this->jacobian_joints_,//input
                                              this->jacobian_A_link_,//input
                                              this->jacobian_B_link_,//input
                                              true,//input
                                              this->jacobian_full_,
                                              this->jacobianColMap_,
                                              this->path_A_joints_,
                                              this->path_B_joints_,
                                              this->path_BA_joints_,
                                              this->path_BA_joints_numUpwardConnections_
                                              );
      }

      aik_constraint::calc6DofJacobianCoef(this->jacobian_joints_,//input
                                           this->jacobian_A_link_,//input
                                           this->A_localpos_,//input
                                           this->jacobian_B_link_,//input
                                           this->B_localpos_,//input
                                           this->jacobianColMap_,//input
                                           this->path_A_joints_,//input
                                           this->path_B_joints_,//input
                                           this->path_BA_joints_,//input
                                           this->path_BA_joints_numUpwardConnections_,//input
                                           true,//input
                                           this->jacobian_full_
                                           );

      cnoid::Matrix3d eval_R_dense = (this->eval_link_) ? this->eval_link_->R() * this->eval_localR_ : this->eval_localR_;
      Eigen::SparseMatrix<double,Eigen::RowMajor> eval_R(3,3);
      for(int i=0;i<3;i++) for(int j=0;j<3;j++) eval_R.insert(i,j) = eval_R_dense(i,j);
      this->jacobian_full_local_.resize(this->jacobian_full_.rows(), this->jacobian_full_.cols());
      this->jacobian_full_local_.topRows<3>() = eval_R.transpose() * this->jacobian_full_.topRows<3>();
      this->jacobian_full_local_.bottomRows<3>() = eval_R.transpose() * this->jacobian_full_.bottomRows<3>();

      this->jacobian_.resize((this->weight_.array() > 0.0).count(),this->jacobian_full_local_.cols());
      for(size_t i=0, idx=0;i<6;i++){
        if(this->weight_[i]>0.0) {
          this->jacobian_.row(idx) = this->weight_[i] * this->jacobian_full_local_.row(i);
          idx++;
        }
      }
    }

    if(this->debugLevel_>=1){
      std::cerr << "PositionConstraint" << std::endl;
      std::cerr << "A_pos" << std::endl;
      std::cerr << A_pos.translation().transpose() << std::endl;
      std::cerr << A_pos.linear() << std::endl;
      std::cerr << "A_vel" << std::endl;
      std::cerr << A_vel.transpose() << std::endl;
      std::cerr << "B_pos" << std::endl;
      std::cerr << B_pos.translation().transpose() << std::endl;
      std::cerr << B_pos.linear() << std::endl;
      std::cerr << "B_vel" << std::endl;
      std::cerr << B_vel.transpose() << std::endl;
      std::cerr << "target_acc" << std::endl;
      std::cerr << target_acc.transpose() << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }
}
