# 🧭 Momentum Observer

A lightweight momentum observer for robotic systems, implemented in C++ using the Eigen library.

This class estimates joint-space momentum using discrete-time dynamics and can be easily integrated into any robot control or estimation system.

---

## 👤 Author

Euncheol Im
Date: 2025-05-01

---
## 📦 Features

- Estimates joint momentum \(p̂ \) in real-time
- Based on classical rigid-body dynamics:
  \[
  \dot{p} = τ - C(q, q̇)q̇ - g(q)
  \]
- Uses inertia matrix `M`, Coriolis vector `Cqdot`, and gravity vector `g`
- Supports tunable scalar observer gain `K`
- Written in modern C++ with Eigen for matrix operations

---

## 📁 File Structure
├── MomentumObserver.cpp \
├── MomentumObserver.h \
└── README.md

---

## 🔧 Dependencies

- [Eigen 3](https://eigen.tuxfamily.org/)  
  Make sure Eigen is installed and properly included in your project.
~~~
  sudo apt update
  sudo apt install libeigen3-dev
~~~
---

## 🚀 Usage Example

```cpp
#include "momentum_observer.h"

// Initialize
MomentumObserver observer;

// Compute estimated momentum
Eigen::VectorXd p_hat = observer.compute(
    qdot,     // Joint velocity vector
    M,        // Inertia matrix
    Cqdot,    // Coriolis & Centrifugal matrix * qdot
    g,        // Gravity vector
    torque,   // Applied torque
    K,        // Observer gain (e.g., 10.0)
    dt        // Time step
);


