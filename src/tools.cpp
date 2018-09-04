#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// make sure the estimation 
	if(estimations.size() == 0){
		std::cout << "ERROR: Estimation vector is zero" << std::endl;
		return rmse;
	}

	if(estimations.size() != ground_truth.size()){
		std::cout << "ERROR: Estimation vector is not the same size as the ground truth vector" << std::endl;
		return rmse;
	}

	for(unsigned int i=0; i < estimations.size(); ++i){
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse = rmse + residual;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}