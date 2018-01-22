Tutorial on Inverse Kinematics
==============================

# Tutorial
With this code example, we aim to guide you through the theory and the practical
implementation of those methods that are traditionally employed to solve the
Inverse Kinematics (IK) problem.

In particular, we explore the pros and cons of the following algorithms for
the Differential IK applied to the **2-links serial manipulator** shown in the
figure below:
- **Jacobian Transpose**
- **Jacobian (Pseudo-)inverse**
- **Damped Least Squares**

![robot](/misc/robot.gif)

### Interacting with the controller
Establish a RPC communication with the controller by doing:
```sh
$ yarp rpc /tutorial_inverse-kinematics-controller/cmd:rpc
```

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

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
