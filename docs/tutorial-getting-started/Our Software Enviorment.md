---

sidebar_position: 4

---

# Our Software Environment

## How we use Docker and Git to develop code

Now that you definitely read the [Git](/wuair_docs/docs/tutorial-getting-started/what-is-git), [Docker](/wuair_docs/docs/tutorial-getting-started/what-is-docker) and ROS tutorials :eyes: can show you how they are used within our environment. 


To understand Docker, we first need to understand **containers** and their predecessor, **Virtual Machines** (VMs). Virtual Machines are simulated computers, using some mapping between the hardware on our real computer and the simulated hardware on the VM, to trick the VM into thinking it really is a real computer. This works, but it is computationally expensive to simulate an extra computer and OS  to run some simple commands. 


Instead, containers strip a lot of unnecessary bloat from the computer, down to exactly what it needs to run. Then, we add back in what we need and can use it like an extremely quick simulated computer. Turning this process into a streamlined series of configuration files to tune and giving users the ability to save these specialized containers as templates is what Docker does. 

:::tip[Why Simulate Computers at all?]

In our case, we want to replicate what will happen on the car as close to real life as possible, including the OS (operating system) that will be on the car. Since we all have a different OS, by emulating the car's OS using Docker we can all work under the same conditions, reducing possibility of version mismatches and other OS related troubles.

:::

For our use case, you likely will not need to know more than this and the commands listed in your OS-specific setup guide. If Docker related issues arise and solutions start necessitating `sudo` or editing docker related files, **don't hesitate to contact a team exec first**.

