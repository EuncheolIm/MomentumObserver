#include "MomentumObserver.h"

MomentumObserver::MomentumObserver() {
    _p_hat = Eigen::VectorXd::Zero(7);
    _p0 = Eigen::VectorXd::Zero(7);
    p = Eigen::VectorXd::Zero(7);
    _M_prev = Eigen::MatrixXd::Zero(7, 7);
    _is_initialized = false;
    _K0 = 1* Eigen::MatrixXd::Identity(7, 7);
}

Eigen::VectorXd MomentumObserver::compute(const Eigen::VectorXd& qdot,
                                          const Eigen::MatrixXd& M,
                                          const Eigen::VectorXd& Cqdot,
                                          const Eigen::VectorXd& g,
                                          const Eigen::VectorXd& torque,
                                          const double K,
                                          double dt) {
    Eigen::MatrixXd M_dot = (M - _M_prev) / dt;
    _M_prev = M;

    Eigen::VectorXd C_T_qdot = M_dot * qdot - Cqdot;
    Eigen::VectorXd Beta = g - C_T_qdot;

    p = M * qdot;
    if (!_is_initialized) {
        _p0 = p;
        _is_initialized = true;
    }

    _K0 = K * _K0;

    Eigen::VectorXd r = _K0 * (p - _p0 - _p_hat);
    Eigen::VectorXd pdot_hat = torque - Beta + r;
    _p_hat += pdot_hat * dt;

    return r;
}