Tutorial on Inverse Kinematics
==============================

[![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/from-referrer)

# Tutorial
With this code example, we aim to guide you through the theory and the practical
implementation of those methods that are traditionally employed to solve the
Inverse Kinematics (IK) problem.

In particular, we explore the pros and cons of the following algorithms for
the Differential IK applied to the **2-links serial manipulator** shown in the
animation below:
- **Jacobian Transpose**
- **Jacobian (Pseudo-)inverse**
- **Damped Least Squares**

![robot](/assets/robot.gif)

Finally, we will also see the advantages of relying on **trajectory planning**.

### How to interact with the controller
Once you launched the application in [**app/scripts**](/app/scripts), establish a RPC communication with the controller by doing:
```sh
$ yarp rpc /tutorial_inverse-kinematics-controller/cmd:rpc
```
Then, you can operate through the commands listed below.

#### Changing target position
```sh
>> target x y
```
where `x` and `y` are the new Cartesian coordinates of the target in the range **[-250,250]**.

#### Changing IK algorithm
```sh
>> mode m
```
where `m` is a string specifying the new mode among the following options:
- `t` for the Jacobian Transpose
- `inv` for the Jacobian (Pseudo-)inverse
- `dls` for the Damped Least Squares
- `idle` for putting the controller in idle.

#### Changing planner mode
```sh
>> planner `m`
```
where `m` can be `on` or `off`.

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
