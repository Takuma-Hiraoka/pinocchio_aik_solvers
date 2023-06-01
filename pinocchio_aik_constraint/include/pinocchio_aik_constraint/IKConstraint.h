#ifndef PINOCCHIO_AIK_CONSTRAINT_IKCONSTRAINT_H
#define PINOCCHIO_AIK_CONSTRAINT_IKCONSTRAINT_H

#include "pinocchio/algorithm/kinematics.hpp"
#include <Eigen/Sparse>
#include <unordered_map>

namespace aik_constraint{
  class IKConstraint
  {
  public:

    // 内部状態更新
    virtual void update (const pinocchio::Model& mode, pinocchio::Data& data, const std::vector<bool>& jointControllable) { return; }

    // getEq = getJacobian * ddq
    // 等式制約のエラーを返す.
    const Eigen::VectorXd& getEq () const { return this->eq_; }
    // 等式制約のヤコビアンを返す. 各jointsの加速度が変数
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobian () const { return this->jacobian_; }

    // getMinIneq <= getJacobianIneq * ddq <= getMaxIneq()
    // 不等式制約のmin値を返す
    const Eigen::VectorXd& getMinIneq () const { return this->minIneq_; }
    // 不等式制約のmax値を返す
    const Eigen::VectorXd& getMaxIneq () const { return this->maxIneq_; }
    // 不等式制約のヤコビアンを返す. 各jointsの加速度が変数
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobianIneq () const { return this->jacobianIneq_; }

    const int& debugLevel() const { return debugLevel_;}
    int& debugLevel() { return debugLevel_;}

    template<typename Derived>
    static typename Derived::PlainObject clamp(const Eigen::MatrixBase<Derived>& value, const Eigen::MatrixBase<Derived>& limit_value) {
      return value.array().max(-limit_value.array()).min(limit_value.array());
    }
    static Eigen::SparseMatrix<double,Eigen::RowMajor> cross(const Eigen::Vector3d v){
      Eigen::SparseMatrix<double,Eigen::RowMajor> m(3,3);
      m.insert(0,1) = -v[2]; m.insert(0,2) = v[1];
      m.insert(1,0) = -v[2]; m.insert(1,2) = -v[0];
      m.insert(2,0) = -v[1]; m.insert(2,1) = v[0];
      return m;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:

    int debugLevel_ = 0;

    Eigen::VectorXd eq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
    Eigen::VectorXd minIneq_;
    Eigen::VectorXd maxIneq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianIneq_;
  };
}

#endif
