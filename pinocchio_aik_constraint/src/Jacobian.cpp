#include <pinocchio_aik_constraint/Jacobian.h>

namespace aik_constraint {
  void calc6DofJacobian(const std::vector<pinocchio::JointIndex>& use_joints, // input
                        const pinocchio::Data& data, // input
			const pinocchio::JointIndex& target_link_id, //input
			const pinocchio::SE3& target_localpos, // input
                        const bool& calcRotation, // input. falseならtranslationのみ
                        std::vector<Eigen::Triplet<double> >& o_Jacobian, //output
			){

    std::vector<Eigen::Triplet<double> > jacobian;
    tripletList.reserve(100);//適当

    const pinocchio::SE3 target_position = data.oMi[target_link_id] * target_localpos;
    const Eigen::Vector3d target_p = target_position.translation();

    for (int i=0;i<use_joints.size();i++){
      
    }

    jacobian = Eigen::SparseMatrix<double,Eigen::RowMajor>(calcRotation?6:3,num_variables);
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());
  };
}
