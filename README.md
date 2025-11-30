# Flatland XR

<div align="center">
 <img width="150" src="https://github.com/user-attachments/assets/948be9e2-f2f2-43d5-b114-468c9c3752e9"/>
</div>

<div align="center">
  <img src="https://github.com/brenocq/flatland-xr/actions/workflows/linux.yml/badge.svg" alt="🐧 Linux"/>
  <img src="https://github.com/brenocq/flatland-xr/actions/workflows/macos.yml/badge.svg" alt="🍎 MacOS"/>
  <img src="https://github.com/brenocq/flatland-xr/actions/workflows/windows.yml/badge.svg" alt="🪟 Windows"/>
</div>

## 📖 Introduction

**Welcome to the Spatial Computing revolution... for A. Square.**

Imagine if the engineers at a tech giant were tasked with building XR glasses for the inhabitants of Edwin Abbott's novella, Flatland. How do you perform 6-DoF tracking when the universe only has 3 degrees of freedom? How do you do Visual Odometry when your "image" is just a 1D strip of pixels?

**Flatland XR** is a serious educational sandbox disguised as a geometry joke. It implements a production-grade Visual-Inertial Odometry (VIO) stack, but mathematically projected down to a 2D world.

By stripping away the Z-axis (and the headaches of 3D rotation groups, quaternions, and gimbal locks), we can explore the core algorithms of modern **SLAM**, **MSCKF**, **Factor Graphs**, and **Bundle Adjustment** in their purest, most understandable form.

## 🚀 Project Overview

This repository contains a full-stack perception pipeline:
 - **Simulation**: A configurable "World Generator" that creates 2D environments and simulates the glasses moving through them. It generates noisy sensor data (1D Line-Scan Cameras & 2D IMU).
 - **Frontend**: 1D Optical Flow tracking and RANSAC-based geometric verification to reject moving outliers.
 - **Backend**: A choice of state-of-the-art estimators (MSCKF, Factor Graph) to fuse visual and inertial data.
 - **Visualization**: A real-time dashboard using ImGui and ImPlot to analyze residuals, covariance ellipses, and trajectory drift.

## 📐 The Math of Flatland

### State Representation

In Flatland, the state of a rigid body is described by its **pose** $\mathbf{T} \in SE(2)$, consisting of position $(x, y) \in \mathbb{R}^2$ and orientation $\theta \in (-\pi, \pi]$. The rotation matrix is given by:

$$
\mathbf{R}(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \in SO(2)
$$

### Sensor Models

#### 1D Line-Scan Camera

The 1D line-scan camera projects 2D world points onto a 1-dimensional image manifold, the projective line $\mathbb{P}^1$.

**Coordinate Frame Convention:**

The camera frame $\{C\}$ is defined with:
- $+X_C$: forward (optical axis / principal direction)
- $+Y_C$: left (perpendicular to optical axis, parellel to the image line)

**World-to-Camera Transformation:**

Let $\mathbf{p}_w \in \mathbb{R}^2$ denote a landmark in the world frame and $\mathbf{T}_{wc} = (\mathbf{R}_{wc}, \mathbf{t}_{wc}) \in SE(2)$ the camera-to-world transformation (i.e., pose of the camera in world coordinates). The landmark expressed in the camera frame is:

$$
\mathbf{p}_c = \mathbf{R}_{wc}^\top (\mathbf{p}_w - \mathbf{t}_{wc}) = \begin{bmatrix} p_c^x \\ p_c^y \end{bmatrix}
$$

**Projective Model:**

The projection from $\mathbb{R}^2$ to the projective line $\mathbb{P}^1$ is performed using homogeneous coordinates. A point $\mathbf{p}_c = (p_c^x, p_c^y)^\top$ in the camera frame is first represented in homogeneous coordinates as $\tilde{\mathbf{p}}_c = (p_c^x, p_c^y, 1)^\top \in \mathbb{R}^3$. The projection onto the image line is given by:

$$
\tilde{u} = \mathbf{K} \, \boldsymbol{\pi}(\mathbf{p}_c)
$$

where $\boldsymbol{\pi}: \mathbb{R}^2 \to \mathbb{P}^1$ is the perspective projection:

$$
\boldsymbol{\pi}(\mathbf{p}_c) = \frac{p_c^y}{p_c^x}
$$

and $\mathbf{K}$ is the $1 \times 2$ intrinsic matrix (mapping normalized coordinates to pixel coordinates):

$$
\mathbf{K} = \begin{bmatrix} c_f & c_c \end{bmatrix}
$$

where:
- $c_f$: focal length (pixels)
- $c_c$: principal point (pixels), typically $w/2$ for a centered sensor

The complete projection yields the pixel coordinate:

$$
u = c_f \cdot \frac{p_c^y}{p_c^x} + c_c
$$

**Visibility Constraints:**

A landmark is visible if and only if it lies in front of the camera and within the field of view:

$$
p_c^x > 0 \quad \land \quad |u - c_c| \leq \frac{w}{2}
$$

where $w$ is the image width in pixels. The field of view half-angle is $\alpha = \arctan\left(\frac{w}{2 c_f}\right)$.

**Measurement Model with Noise:**

The noisy measurement is modeled as:

$$
\tilde{u} = u + \eta_u, \quad \eta_u \sim \mathcal{N}(0, \sigma_u^2)
$$

where $\sigma_u$ is the pixel noise standard deviation.

#### 2D Inertial Measurement Unit (IMU)

The IMU provides measurements of linear acceleration and angular velocity in the body frame.

**Accelerometer Model:**

The accelerometer measures the specific force (proper acceleration minus gravity) in the body frame:

$$
\tilde{\mathbf{a}} = \mathbf{R}^\top (\mathbf{a}_w - \mathbf{g}_w) + \mathbf{b}_a + \boldsymbol{\eta}_a
$$

where:
- $\mathbf{a}_w \in \mathbb{R}^2$ is the true acceleration in the world frame
- $\mathbf{g}_w = [0, -g]^\top$ is the gravity vector in flatland (pointing "south")
- $\mathbf{b}_a \in \mathbb{R}^2$ is the accelerometer bias
- $\boldsymbol{\eta}_a \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_a)$ is additive Gaussian noise with $\boldsymbol{\Sigma}_a = \text{diag}(\sigma_{a_x}^2, \sigma_{a_y}^2)$

**Gyroscope Model:**

The gyroscope measures the angular velocity about the axis perpendicular to the 2D plane:

$$
\tilde{\omega} = \omega + b_\omega + \eta_\omega, \quad \eta_\omega \sim \mathcal{N}(0, \sigma_\omega^2)
$$

where:
- $\omega = \dot{\theta}$ is the true angular velocity
- $b_\omega$ is the gyroscope bias
- $\sigma_\omega$ is the gyroscope noise standard deviation

**IMU Integration (Dead Reckoning):**

Given discrete IMU measurements at time $t_k$ with time step $\Delta t = t_{k+1} - t_k$, the state propagation follows:

$$
\begin{aligned}
\theta_{k+1} &= \theta_k + (\tilde{\omega}_k - b_\omega) \Delta t \\
\mathbf{v}_{k+1} &= \mathbf{v}_k + \mathbf{R}(\theta_k)(\tilde{\mathbf{a}}_k - \mathbf{b}_a) \Delta t + \mathbf{g}_w \Delta t \\
\mathbf{p}_{k+1} &= \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2}\mathbf{R}(\theta_k)(\tilde{\mathbf{a}}_k - \mathbf{b}_a) \Delta t^2 + \frac{1}{2}\mathbf{g}_w \Delta t^2
\end{aligned}
$$

## 🛠️ Usage

```
cmake -B build && cmake --build build
./build/flatland-xr
```

## 🤝 Contributing

Found a bug in the 2D Jacobian derivation? Want to add a "Fish-eye" 1D lens model? PRs are welcome!

## 📜 License

This project is licensed under the [MIT LICENSE](LICENSE). Feel free to use this to build your own 2D Metaverse.
