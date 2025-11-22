---

sidebar_position: 1

---


# Perception Research 

This is where we will compile new possible methods and algorithms for the perception team. Feel free to add sources here. When you do please leave a short description of the method and how we can use it for WUAIR.

:::tip[Adding New References]
Please add any research you come across! Use the same formatting as below and please leave a short description of the source to make it easy to navigate
:::

## References


### 1. The Reference Architecture
**Kabzan, J., Valls, M. I., Reijgwart, V. J. F., Hendrikx, H. F. C., Ehmke, C., Prajapat, M., Bühler, A., Gosala, N., Gupta, M., Sivanesan, R., Dhall, A., Chisari, E., Karnchanachari, N., Brits, S., Dangel, M., Sa, I., Dubé, R., Gawel, A., Pfeiffer, M., & Liniger, A. (2020).** *AMZ Driverless: The full autonomous racing system.* Journal of Field Robotics. [https://doi.org/10.1002/rob.21977](https://doi.org/10.1002/rob.21977)

* **Overview:** This paper details the complete software and hardware architecture of "gotthard," the autonomous race car built by the AMZ Driverless team (ETH Zurich). It covers their specific approaches to perception (LiDAR/Camera fusion), estimation (SLAM), path planning, and control (Model Predictive Control).

### 2. Benchmarking Perception
**Caesar, H., Bankiti, V., Lang, A. H., Vora, S., Liong, V. E., Xu, Q., Krishnan, A., Pan, Y., Baldan, G., & Beijbom, O. (2020).** *nuScenes: A Multimodal Dataset for Autonomous Driving.* 2020 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR). [https://doi.org/10.1109/cvpr42600.2020.01164](https://doi.org/10.1109/cvpr42600.2020.01164)

* **Overview:** This paper introduces `nuScenes`, a massive public dataset for autonomous driving that includes data from the full sensor suite (Camera, LiDAR, Radar) along with 3D bounding boxes. It establishes benchmarks for detection and tracking algorithms.


### 3. Efficient RGB-D Processing
**Xu, Z., Zhan, X., Chen, B., Xiu, Y., Yang, C., & Shimada, K. (2023).** *A real-time dynamic obstacle tracking and mapping system for UAV navigation and collision avoidance with an RGB-D camera.* ArXiv.org. [https://doi.org/10.1109/ICRA48891.2023.10161194](https://doi.org/10.1109/ICRA48891.2023.10161194)

* **Overview:** This study presents a system for Unmanned Aerial Vehicles (UAVs) that performs real-time mapping and tracking using RGB-D (Depth) cameras. It emphasizes computational efficiency to run on onboard hardware while avoiding collisions.


### 4. Sensor Hardware & Fusion Theory
**Yeong, D. J., Velasco-Hernandez, G., Barry, J., & Walsh, J. (2021).** *Sensor and Sensor Fusion Technology in Autonomous Vehicles: a Review.* Sensors, 21(6), 2140. [https://doi.org/10.3390/s21062140](https://doi.org/10.3390/s21062140)

* **Overview:** A comprehensive review of the current state of sensor technology (LiDAR, Radar, Vision, Ultrasonic) and the mathematical techniques used to fuse their data (such as Kalman Filters and Bayesian networks).


### 5. Advanced Mapping Techniques
**Liu, C., Zhang, G., Rong, Y., Shao, W., Meng, J., Li, G., & Huang, Y. (2023).** *Hybrid metric-feature mapping based on camera and Lidar sensor fusion.* Measurement, 207, 112411. [https://doi.org/10.1016/j.measurement.2022.112411](https://doi.org/10.1016/j.measurement.2022.112411)

* **Overview:** This paper proposes a mapping method that combines "metric" information (precise distances from LiDAR) with "feature" information (visual descriptors from cameras). This hybrid approach aims to solve the shortcomings of using either sensor individually for SLAM (Simultaneous Localization and Mapping).
