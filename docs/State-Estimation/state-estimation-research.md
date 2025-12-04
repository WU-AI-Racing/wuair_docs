---

sidebar_position: 1

---


# State Estimation Research 

This is where we will compile new possible methods and algorithms for the state estimation team. Feel free to add sources here. When you do please leave a short description of the method and how we can use it for WUAIR.

:::tip[Adding New References]
Please add any research you come across! Use the same formatting as below and please leave a short description of the source to make it easy to navigate
:::

## General Overview
The Image below shows our place in the greater system diagram. Our job is to take in the positions of the cones and output a map of the track and the vehicles current position and trajectory. This first requires estimating our current velocity, which we will do by combining the data from our sensors using a Kalman Filtering. We will then use that velocity estimation in our SLAM implementation. 
![System Diagram with SLAM Highlighted](/WU-AI-Racing/wuair_docs/static/img/SLAMHighlight_Sytem.png "System Overview")

## References

### 1. Extended Kalman Filtering
*[https://automaticaddison.com/extended-kalman-filter-ekf-with-python-code-example/](https://automaticaddison.com/extended-kalman-filter-ekf-with-python-code-example/)
*[https://automaticaddison.com/how-to-derive-the-observation-model-for-a-mobile-robot/](https://automaticaddison.com/how-to-derive-the-observation-model-for-a-mobile-robot/
*[https://automaticaddison.com/how-to-derive-the-state-space-model-for-a-mobile-robot/](https://automaticaddison.com/how-to-derive-the-state-space-model-for-a-mobile-robot/))
* These are some good articles to read to learn about Extended Kalman filters, which we may end up implementing

### 2. Recurrent Kalman Network
**Becker, P., Pandya, H., Gebhardt, G., Zhao, C., Taylor, J., & Neumann, G. (2019).**
*Recurrent Kalman Networks: Factorized Inference in High-Dimensional Deep Feature Spaces. arXiv preprint arXiv:1905.07357.  [https://arxiv.org/pdf/1905.07357](arXiv:1905.07357v1 [cs.LG]) 
* **Overview:** This paper introduces Recurrent Kalman Networks (RKN), a deep-learning–based state-estimation framework that blends neural network feature extraction with Kalman-filter–style uncertainty tracking. The method learns a latent state space directly from high-dimensional, noisy observations (such as images) and performs factorized Kalman updates in that learned space. This hybrid design addresses limitations of traditional Kalman filters (which require known linear models) and recurrent neural networks (which lack principled uncertainty estimates), enabling robust sequential inference, prediction, and reconstruction in dynamic environments.

### 3. Advanced Mapping Techniques
**Liu, C., Zhang, G., Rong, Y., Shao, W., Meng, J., Li, G., & Huang, Y. (2023).** *Hybrid metric-feature mapping based on camera and Lidar sensor fusion.* Measurement, 207, 112411. [https://doi.org/10.1016/j.measurement.2022.112411](https://doi.org/10.1016/j.measurement.2022.112411)

* **Overview:** This paper proposes a mapping method that combines "metric" information (precise distances from LiDAR) with "feature" information (visual descriptors from cameras). This hybrid approach aims to solve the shortcomings of using either sensor individually for SLAM (Simultaneous Localization and Mapping).

### 4. Fast LiDAR Odometry (F-LOAM)
**Wang, H., Wang, C., & Xie, L. (2021).** *F-LOAM: Fast LiDAR Odometry and Mapping.* 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). [https://ieeexplore.ieee.org/document/9636655](https://ieeexplore.ieee.org/document/9636655)

* **Overview:** This paper presents a general solution for LiDAR-based SLAM that prioritizes computational efficiency (running at >10Hz). It uses a non-iterative two-stage distortion compensation method. This is highly relevant for WUAIR to ensure our localization doesn't lag behind the physical car at high speeds.

### 5. Graph-SLAM in Racing
**Alvarez, A., Denner, N., Feng, Z., et al. (2022).** *The Software Stack That Won the Formula Student Driverless Competition.* ArXiv. [https://arxiv.org/pdf/2210.10933](https://arxiv.org/pdf/2210.10933)

* **Overview:** This report describes the software stack used by a winning Formula Student Driverless team. It specifically details how they used **GraphSLAM** to map track cones with a root-mean-square error of less than 15 cm while driving at speeds over 70 kph. This is a primary reference for implementing our cone-mapping algorithms.

### 6. FastSLAM Implementation
**Wu, Y. (2023).** *FastSLAM: Grid-based FastSLAM1.0 and FastSLAM2.0 algorithms.* GitHub Repository. [https://github.com/yingkunwu/FastSLAM](https://github.com/yingkunwu/FastSLAM)

* **Overview:** A clean Python implementation of the grid-based FastSLAM algorithms. While we will likely use C++ for the production car, this repository is excellent for simulating and understanding the underlying particle-filter logic used in packages like `gmapping`.

