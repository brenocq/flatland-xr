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

TODO

## 🛠️ Usage

```
cmake -B build && cmake --build build
./build/flatland-xr
```

## 🤝 Contributing

Found a bug in the 2D Jacobian derivation? Want to add a "Fish-eye" 1D lens model? PRs are welcome!

## 📜 License

This project is licensed under the [MIT LICENSE](LICENSE). Feel free to use this to build your own 2D Metaverse.
