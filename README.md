# Flatland XR

<div align="center">
 <img width="150" src="https://github.com/user-attachments/assets/948be9e2-f2f2-43d5-b114-468c9c3752e9"/>
</div>

<p align="center">
Welcome to the Spatial Computing revolution... for A Square.
</p>

<div align="center">
  <img src="https://github.com/brenocq/flatland-xr/actions/workflows/linux.yml/badge.svg" alt="🐧 Linux"/>
  <img src="https://github.com/brenocq/flatland-xr/actions/workflows/macos.yml/badge.svg" alt="🍎 MacOS"/>
  <img src="https://github.com/brenocq/flatland-xr/actions/workflows/windows.yml/badge.svg" alt="🪟 Windows"/>
</div>

## 📋 Table of Contents

- [📖 Introduction](#-introduction)
- [🚀 Project Overview](#-project-overview)
- [🏁 Getting Started](#-getting-started)
- [📐 The Math of Flatland](#-the-math-of-flatland)
  - [📍 State Representation](#-state-representation)
  - [📡 Sensor Models](#-sensor-models)
    - [📷 1D Line-Scan Camera](#-1d-line-scan-camera)
    - [🧭 2D IMU](#-2d-imu)
- [🤝 Contributing](#-contributing)
- [📜 License](#-license)

## 📖 Introduction

Have you ever asked yourself: **"How would a tech giant build XR glasses for the inhabitants of Edwin Abbott's Flatland?"**

Me too.

It raises some fascinating questions. How do you SLAM your way through a universe that only has 3 degrees of freedom? How do you perform robust Visual Odometry when your "image" is nothing more than a 1D strip of pixels?

**Flatland XR** is a serious interactive tutorial disguised as a geometry joke. It implements a production-grade **Visual-Inertial SLAM (VI-SLAM)** stack, but mathematically projected down to a 2D world.

By stripping away the Z-axis - and the accompanying headaches of 3D rotation groups, quaternions, and gimbal locks - we can explore the core algorithms of modern **VI-SLAM** (**MSCKF**, **Bundle Adjustment**, and **Factor Graphs**) in their purest, most understandable form.

## 🚀 Project Overview

This repository contains a full-stack perception pipeline:
 - **Simulation**: A configurable "World Generator" that creates 2D environments and simulates the glasses moving through them. It generates noisy sensor data (1D Line-Scan Cameras & 2D IMU).
 - **Frontend**: 1D Optical Flow tracking and RANSAC-based geometric verification to reject moving outliers.
 - **Backend**: A choice of state-of-the-art estimators (MSCKF, Factor Graph) to fuse visual and inertial data.
 - **Visualization**: A real-time dashboard using ImGui and ImPlot to analyze residuals, covariance ellipses, and trajectory drift.

## 🏁 Getting Started

```
cmake -B build && cmake --build build
./build/flatland-xr
```

## 📐 The Math of Flatland

### 📍 State Representation

In Flatland, the pose of a rigid body is defined as $\mathbf{T} \in SE(2)$, consisting of position $(x, y) \in \mathbb{R}^2$ and orientation $\theta \in (-\pi, \pi]$. The rotation matrix is given by:

$$
\mathbf{R}(\theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
\in SO(2)
$$

### 📡 Sensor Models

#### 📷 1D Line-Scan Camera

The 1D line-scan camera performs a projective transformation mapping 2D points in the Euclidean plane $\mathbb{R}^2$ to the 1D projective line $\mathbb{P}^1$.

**Coordinate Frame Convention:**

Let $\{W\}$ be the fixed World frame and $\{C\}$ be the Camera frame attached to the sensor center.
The basis vectors of $\{C\}$ are defined as:
- $\mathbf{x}_C$: Forward (optical axis).
- $\mathbf{y}_C$: Left (parallel to the sensor array).

**World-to-Camera Transformation:**

Let ${}\_{W}\mathbf{l} \in \mathbb{R}^2$ denote a landmark expressed in the World frame.
Let the camera pose be defined by the rotation ${}\_{W}\mathbf{R}\_{C} \in SO(2)$ and the position ${}\_{W}\mathbf{p}\_{C} \in \mathbb{R}^2$.

The landmark is transformed into the Camera frame via the rigid body transformation:

$$
{}_{C}\mathbf{l} = {}_{W}\mathbf{R}_{C}^\top \left( {}_{W}\mathbf{l} - {}_{W}\mathbf{p}_{C} \right) =
\begin{bmatrix}
  {}_C l_x \\
  {}_C l_y
\end{bmatrix}
$$

**Projective Model:**

 The projection from the Euclidean camera frame to the image line is a two-step process involving normalization and intrinsic scaling.

 1.  **Projection to Normalized Coordinates:**

     First, the point ${}\_{C}\mathbf{l} = [{}\_C l\_x, {}\_C l\_y]^\top$ is projected onto the normalized image plane (at $x=1$) to obtain the homogeneous normalized coordinate $\hat{\mathbf{x}}$:

 $$
 \hat{\mathbf{x}} = \boldsymbol{\pi}({}_{C}\mathbf{l}) =
 \begin{bmatrix}
  \frac{{}_C l_y}{{}_C l_x} \\
  1
 \end{bmatrix}
 $$

 2.  **Intrinsic Scaling:**

   We map the normalized coordinate to the pixel frame using the Intrinsic Matrix $\mathbf{K} \in \mathbb{R}^{2 \times 2}$:

   $$
   \tilde{\mathbf{u}} = \mathbf{K} \cdot \hat{\mathbf{x}}
   $$

   where $\mathbf{K}$ is defined as:

   $$
   \mathbf{K} =
   \begin{bmatrix}
     f_x & c_x \\
     0 & 1
   \end{bmatrix}
   $$

   - $f_x$: Focal length (pixels)
   - $c_x$: Principal point (pixels)

   Carrying out the multiplication yields the homogeneous pixel coordinate:

   $$
   \begin{aligned}
   \tilde{\mathbf{u}} &=
   \begin{bmatrix}
     f_x & c_x \\
     0 & 1
   \end{bmatrix}
   \begin{bmatrix}
     \frac{{}_C l_y}{{}_C l_x} \\
     1
   \end{bmatrix} \\
   &= \begin{bmatrix}
     f_x \frac{{}_C l_y}{{}_C l_x} + c_x \\
     1
   \end{bmatrix} \\
   &= \begin{bmatrix}
     u \\
     1
   \end{bmatrix}
   \end{aligned}
   $$

   Thus recovering the scalar pixel coordinate $u$.

**Visibility Constraints:**

A landmark is considered a valid measurement candidate if and only if it lies within the sensor's field of view (FOV) and in front of the optical plane:

$$
{}_C l_x > 0 \quad \land \quad \left| \arctan\left(\frac{{}_C l_y}{{}_C l_x}\right) \right| \leq \frac{\text{FOV}}{2}
$$

Alternatively, defined by the image width $W_{\text{px}}$:

$$
0 \leq u \leq W_{\text{px}}
$$

**Measurement Model:**
The observed pixel coordinate $\tilde{u}$ is modeled as the true projection corrupted by additive white Gaussian noise:

$$
\tilde{u} = u + \eta_u, \quad \eta_u \sim \mathcal{N}(0, \sigma_u^2)
$$

#### 🧭 2D IMU

The Inertial Measurement Unit (IMU) consists of a 2-axis accelerometer and a 1-axis gyroscope. It measures specific force and angular velocity expressed in the Body frame $\{B\}$.

**Definitions:**
- ${}\_{W}\mathbf{g} = [0, -g]^\top$: Gravity vector in the World frame (pointing "South").
- ${}\_{W}\mathbf{a}$: True kinematic linear acceleration of the body in the World frame.
- $\omega$: True angular velocity of the body ($\dot{\theta}$).
- ${}\_{W}\mathbf{R}\_{B}$: Rotation of the Body frame with respect to the World frame.

**Accelerometer Model:**
The accelerometer measures the **specific force** (proper acceleration), which is the kinematic acceleration minus the gravitational acceleration, projected into the Body frame.

$$
{}_{B}\tilde{\mathbf{a}} = {}_{W}\mathbf{R}_{B}^\top \left( {}_{W}\mathbf{a} - {}_{W}\mathbf{g} \right) + \mathbf{b}_a + \boldsymbol{\eta}_a
$$

Where:
- $\mathbf{b}_a \in \mathbb{R}^2$: Accelerometer bias (modeled as a random walk).
- $\boldsymbol{\eta}_a \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_a)$: Additive Gaussian white noise.

**Gyroscope Model:**
The gyroscope measures the angular rate of the Body frame relative to the World frame. In 2D, the axis of rotation is always orthogonal to the plane (the Z-axis).

$$
\tilde{\omega} = \omega + b_\omega + \eta_\omega
$$

Where:
- $b_\omega \in \mathbb{R}$: Gyroscope bias (modeled as a random walk).
- $\eta_\omega \sim \mathcal{N}(0, \sigma_\omega^2)$: Additive Gaussian white noise.

**IMU Mechanization (State Propagation):**
We assume the acceleration and angular velocity remain constant between discrete time steps $t\_k$ and $t\_{k+1}$ (zero-order hold), with $\Delta t = t\_{k+1} - t\_k$.

Let the state be defined as orientation $\theta\_k$, velocity ${}\_{W}\mathbf{v}\_k$, and position ${}\_{W}\mathbf{p}\_k$. The propagation equations using the raw measurements $(\tilde{\omega}\_k, {}\_{B}\tilde{\mathbf{a}}\_k)$ are:

1.  **Orientation Update:**

  $$
  \hat{\theta}_{k+1} = \hat{\theta}_k + (\tilde{\omega}_k - \hat{b}_{\omega, k}) \Delta t
  $$

2.  **Velocity Update:**
  First, we reconstruct the world-frame acceleration from the measurement:

  $$
  \begin{aligned}
  {}_{W}\hat{\mathbf{a}}_k &= {}_{W}\hat{\mathbf{R}}_{B}(\hat{\theta}_k) \left( {}_{B}\tilde{\mathbf{a}}_k - \hat{\mathbf{b}}_{a, k} \right) + {}_{W}\mathbf{g} \\
  {}_{W}\hat{\mathbf{v}}_{k+1} &= {}_{W}\hat{\mathbf{v}}_k + {}_{W}\hat{\mathbf{a}}_k \Delta t
  \end{aligned}
  $$

3.  **Position Update:**

  $$
  {}_{W}\hat{\mathbf{p}}_{k+1} = {}_{W}\hat{\mathbf{p}}_k + {}_{W}\hat{\mathbf{v}}_k \Delta t + \frac{1}{2} {}_{W}\hat{\mathbf{a}}_k \Delta t^2
  $$

## 🤝 Contributing

Found a bug in the 2D Jacobian derivation? Want to add a "Fish-eye" 1D lens model? PRs are welcome!

## 📜 License

This project is licensed under the [MIT LICENSE](LICENSE). Feel free to use this to build your own 2D Metaverse.
