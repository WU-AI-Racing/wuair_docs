---

sidebar_position: 1

---


# Control Research 

This is where we will compile new possible methods and algorithms for the control team. Feel free to add sources here. When you do please leave a short description of the method and how we can use it for WUAIR.

:::tip[Adding New References]
Please add any research you come across! Use the same formatting as below and please leave a short description of the source to make it easy to navigate
:::

## References

### 1. F1TENTH Control (MPC)

**F1TENTH Foundation.** Lecture 13: Model Predictive Control. F1TENTH Course Kit. https://f1tenth-coursekit.readthedocs.io/en/stable/lectures/ModuleD/lecture13.html

* **Overview:** This lecture from the official F1TENTH curriculum covers the implementation of Model Predictive Control (MPC). Unlike simpler controllers, MPC optimizes control inputs over a finite time horizon while satisfying vehicle constraints. This is the industry standard for high-performance racing and is the ultimate goal for the WUAIR control stack.

### 2. Pure Pursuit Overview

Fermi, T. Algorithms for Automated Driving: Pure Pursuit. https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html

* **Overview:** A concise, mathematical explanation of the Pure Pursuit path tracking algorithm. It details the geometric derivation of the steering angle based on a "lookahead point" on the reference path. This is the baseline control algorithm we should implement and master before attempting more complex methods like MPC.