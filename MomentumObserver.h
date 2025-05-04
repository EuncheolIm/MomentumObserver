/**
 * @file    momentum_observer.h
 * @brief   Momentum Observer class for estimating robot momentum.
 *
 * @author  [Euncheol Im]
 * @date    [2025-05-01]
 * @version 1.0
 *
 * @copyright
 * This file is released under the MIT License.
 * See the LICENSE file in the project root for full license information.
 */

#ifndef MOMENTUM_OBSERVER_H
#define MOMENTUM_OBSERVER_H

#include <Eigen/Dense>

/**
 * @brief Computes estimated momentum using observer.
 * @param qdot Joint velocities
 * @param M Inertia matrix
 * @param Cqdot Coriolis matrix times qdot
 * @param g Gravity compensation vector
 * @param torque Applied joint torques
 * @param K Observer gain scalar
 * @param dt Sampling time
 * @return Estimated momentum vector
 */

class MomentumObserver {
public:
    MomentumObserver();
    
    Eigen::VectorXd compute(const Eigen::VectorXd& qdot,
                            const Eigen::MatrixXd& M,
                            const Eigen::VectorXd& Cqdot,
                            const Eigen::VectorXd& g,
                            const Eigen::VectorXd& torque,
                            const double K,
                            double dt);

private:
    Eigen::VectorXd _p_hat;
    Eigen::VectorXd _p0;
    Eigen::MatrixXd _M_prev;
    bool _is_initialized;

    Eigen::MatrixXd _K0;
    Eigen::VectorXd p;      // p = M x v : Momentum
};

#endif  // MOMENTUM_OBSERVER_H