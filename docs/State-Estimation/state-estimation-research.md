---

sidebar_position: 1

---


# Stat Estimation Research 

This is where we will compile new possible methods and algorithms for the state estimation team. Feel free to add sources here. When you do please leave a short description of the method and how we can use it for WUAIR.

:::tip[Adding New References]
Please add any research you come across! Use the same formatting as below and please leave a short description of the source to make it easy to navigate
:::

## References


## 1. Advanced Mapping Techniques
**Liu, C., Zhang, G., Rong, Y., Shao, W., Meng, J., Li, G., & Huang, Y. (2023).** *Hybrid metric-feature mapping based on camera and Lidar sensor fusion.* Measurement, 207, 112411. [https://doi.org/10.1016/j.measurement.2022.112411](https://doi.org/10.1016/j.measurement.2022.112411)

* **Overview:** This paper proposes a mapping method that combines "metric" information (precise distances from LiDAR) with "feature" information (visual descriptors from cameras). This hybrid approach aims to solve the shortcomings of using either sensor individually for SLAM (Simultaneous Localization and Mapping).

### 2. Fast LiDAR Odometry (F-LOAM)
**Wang, H., Wang, C., & Xie, L. (2021).** *F-LOAM: Fast LiDAR Odometry and Mapping.* 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). [https://ieeexplore.ieee.org/document/9636655](https://ieeexplore.ieee.org/document/9636655)

* **Overview:** This paper presents a general solution for LiDAR-based SLAM that prioritizes computational efficiency (running at >10Hz). It uses a non-iterative two-stage distortion compensation method. This is highly relevant for WUAIR to ensure our localization doesn't lag behind the physical car at high speeds.

### 3. Graph-SLAM in Racing
**Alvarez, A., Denner, N., Feng, Z., et al. (2022).** *The Software Stack That Won the Formula Student Driverless Competition.* ArXiv. [https://arxiv.org/pdf/2210.10933](https://arxiv.org/pdf/2210.10933)

* **Overview:** This report describes the software stack used by a winning Formula Student Driverless team. It specifically details how they used **GraphSLAM** to map track cones with a root-mean-square error of less than 15 cm while driving at speeds over 70 kph. This is a primary reference for implementing our cone-mapping algorithms.

### 4. FastSLAM Implementation
**Wu, Y. (2023).** *FastSLAM: Grid-based FastSLAM1.0 and FastSLAM2.0 algorithms.* GitHub Repository. [https://github.com/yingkunwu/FastSLAM](https://github.com/yingkunwu/FastSLAM)

* **Overview:** A clean Python implementation of the grid-based FastSLAM algorithms. While we will likely use C++ for the production car, this repository is excellent for simulating and understanding the underlying particle-filter logic used in packages like `gmapping`.