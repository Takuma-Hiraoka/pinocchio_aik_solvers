#ifndef AIK_CONSTAINT_JACOBIAN_H
#define AIK_CONSTAINT_JACOBIAN_H

#include <vector>
#include <unordered_map>

#include <Eigen/Eigen>
#include <Eigen/Sparse>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace aik_constraint {
  // world座標系で見たヤコビアン. 
  // jacobianに使うTripletを用意する。
  // joint_nvからjointControllableなものが抜き出され、更にその中からmodel.parentsをたどってヤコビアンに使用するjointのidが計算されている．
  // この結果のjacobianをcalcRotation?6:3 * jointControllable 行列でsetFromTripletsすればよい．
  void calc6DofJacobian(const std::vector<pinocchio::JointIndex>& use_joints, // input
                        const pinocchio::Data& data, // input
			const pinocchio::JointIndex& target_link_id, //input
			const pinocchio::SE3& target_localpos, // input
                        const bool& calcRotation, // input. falseならtranslationのみ
                        std::vector<Eigen::Triplet<double> >& o_Jacobian, //output
			);

  // world座標系で見た、A - B のヤコビアン. robotがnullptrの場合、world座標を意味する.
  //   jacobianを新たにコンストラクトし、非ゼロ要素に1を入れる.
  void calcCMJacobianShape(const std::vector<cnoid::LinkPtr>& joints,//input
                                  const cnoid::BodyPtr& A_robot,//input
                                  const cnoid::BodyPtr& B_robot,//input
                                  Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian,//output
                                  std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap //output
                                  );
  // world座標系で見た、A - B のヤコビアン. robotがnullptrの場合、world座標を意味する.
  //   jacobianの形状は上の関数で既に整えられている前提.
  void calcCMJacobianCoef(const std::vector<cnoid::LinkPtr>& joints,//input
                          const cnoid::BodyPtr& A_robot,//input
                          const cnoid::BodyPtr& B_robot,//input
                          const Eigen::MatrixXd& A_CMJ, //[joint root]の順 input
                          const Eigen::MatrixXd& B_CMJ, //[joint root]の順 input
                          std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //input
                          Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian//output
                          );

}

#endif
