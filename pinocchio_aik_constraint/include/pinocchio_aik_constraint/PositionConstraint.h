#ifndef PINOCCHIO_AIK_CONSTRAINT_POSITIONCONSTRAINT_H
#define PINOCCHIO_AIK_CONSTRAINT_POSITIONCONSTRAINT_H

#include <pinocchio_aik_constraint/IKConstraint.h>
#include <iostream>

namespace aik_constraint{
  class PositionConstraint : public IKConstraint
  {
  public:
    // act_parent_link中のact_localposの部位とtarget_localposの部位を一致させる.
    //  pgain: evel座標系
    //  dgain: evel座標系
    //  ref_acc: eval_frame. feedforward目標加速度. ref_acc + pgain * error + dgain * derrorが目標加速度になる
    //  maxAcc: 目標加速度の頭打ち eval座標系. 目標加速度をmaxAccで頭打ちしてからweight倍したものがgetEq()で返る
    //  maxAccByPosError: 目標加速度の頭打ち eval座標系.
    //  maxAccByVelError: 目標加速度の頭打ち eval座標系.
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやerrorに含まれない. eval座標系
    //  link: parent link. nullptrならworld座標系を意味する
    //  localpos: parent link frame
    //  localvel: parent link frame. endeffector origin

    // kinematics configuration
    const pinocchio::JointIndex& parent_link_id() const { return parent_link_id_;}
    pinocchio::JointIndex& parent_link_id() { return parent_link_id_;}
    const pinocchio::SE3& localpos() const { return localpos_;}
    pinocchio::SE3& localpos() { return localpos_;}

    // target
    const pinocchio::SE3& target_localpos() const { return target_localpos_;}
    pinocchio::SE3& target_localpos() { return target_localpos_;}
    const pinocchio::SE3& target_localvel() const { return target_localvel_;}
    pinocchio::SE3& target_localvel() { return target_localvel_;}
    const Eigen::Vector6d& ref_acc() const { return ref_acc_;}
    Eigen::Vector6d& ref_acc() { return ref_acc_;}

    const Eigen::Vector6d& pgain() const { return pgain_;}
    Eigen::Vector6d& pgain() { return pgain_;}
    const Eigen::Vector6d& dgain() const { return dgain_;}
    Eigen::Vector6d& dgain() { return dgain_;}
    const Eigen::Vector6d& maxAcc() const { return maxAcc_;}
    Eigen::Vector6d& maxAcc() { return maxAcc_;}
    const Eigen::Vector6d& maxAccByPosError() const { return maxAccByPosError_;}
    Eigen::Vector6d& maxAccByPosError() { return maxAccByPosError_;}
    const Eigen::Vector6d& maxAccByVelError() const { return maxAccByVelError_;}
    Eigen::Vector6d& maxAccByVelError() { return maxAccByVelError_;}
    const Eigen::Vector6d& weight() const { return weight_;}
    Eigen::Vector6d& weight() { return weight_;}
    const pinocchio::JointIndex& eval_link_id() const { return eval_link_id_;}
    pinocchio::JointIndex& eval_link_id() { return eval_link_id_;}
    const Eigen::Matrix3d& eval_localR() const { return eval_localR_;}
    Eigen::Matrix3d& eval_localR() { return eval_localR_;}

    // 内部状態更新
    void update (const pinocchio::Model& mode, pinocchio::Data& data, const std::vector<bool>& jointControllable) override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    pinocchio::JointIndex parent_link_id_ = 0;
    pinocchio::SE3 parent_localpos_ = pinocchio::SE3::Identity();
    pinocchio::SE3 target_localpos_ = pinocchio::SE3::Identity();
    Eigen::Vector6d target_localvel_ = Eigen::Vector6d::Zero();
    Eigen::Vector6d ref_acc_ = Eigen::Vector6d::Zero();
    Eigen::Vector6d pgain_ = 400 * Eigen::Vector6d::Ones();
    Eigen::Vector6d dgain_ = 50 * Eigen::Vector6d::Ones();
    Eigen::Vector6d maxAcc_ = 15 * Eigen::Vector6d::Ones(); // 歩行では10は出る. 15もたまに出る
    Eigen::Vector6d maxAccByPosError_ = 5 * Eigen::Vector6d::Ones();
    Eigen::Vector6d maxAccByVelError_ = 10 * Eigen::Vector6d::Ones();
    Eigen::Vector6d weight_ = Eigen::Vector6d::Ones();
    pinocchio::JointIndex eval_link_id_ = 0;
    Eigen::Matrix3d eval_localR_ = Eigen::Matrix3d::Identity();
  };
}

#endif
