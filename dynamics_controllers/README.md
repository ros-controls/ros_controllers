## Dynamics Controllers

Package that implements controllers that add a "dynamic control layer" to
existing effort controllers.

The controllers contained in this package allow to take into account the
dynamics of a kinematic chain/tree and to perform Computed Torque Control.
Controllers are meant to operate in conjunction with existing controllers that
operate with `hardware_interface::EffortJointInterface`, which act as
"*sub-controllers*".
The command from the sub-controllers - which would normally be an effort - is
instead interpreted as an acceleration and injected in the Inverse Dynamic Model
(IDM) of the mechanism, allowing to better compensate for the nonlinearity of
robots' Dynamic Models.

Currently, there exist two controllers: `KdlChainController` and
`KdlTreeController`.
They are both based on KDL's implementation of the Recursive Newton-Euler
algorithm for the inverse dynamics.



### Gravity Compensation vs Full Inverse Dynamics

By default, the controllers perform full Inverse Dynamics computations.
However, you can ask them to perform gravity compensation only.
In this case, the command evaluated by the sub-controllers is added to the
gravitational efforts - this corresponds to assuming the Generalized Inertia
Matrix to be the Identity and Coriolis/Centrifugal efforts to be null.

To enable gravity compensation only, you can specify the parameter
`gravity_compensation_only: true` in the controller configuration.



### Resource Handling

Since the dynamic model of a generic robot operates on multiple joints at the
same time, sub-controllers must claim all joints exactly once.
As an example, if a dynamic controller is loaded for a chain with three joints,
*e.g.*, `joint1`, `joint2` and `joint3`, the sub-controllers must be loaded so
that they handle all of them without conflicts.
For instance, a valid option is to have a single sub-controller that claims all
the joints.
However, a single controller claiming only `joint1` and `joint2` would cause
failure upon initialization (since `joint3` would remain unclaimed).
On the other hand, having one sub-controller for `joint1` and `joint3` plus
another controller handling `joint2` would be legit.
Note that if the second controller in this last example was also managing
`joint1`, then initialization would fail since both the first and second
controllers try to claim the same resource.
Finally, failure will also happen if any sub-controller tries to claim any
joint not being part of the chain/tree (such as `joint4` in the example).



### Example of Controller Configuration

Assuming that you have a simple kinematic chain with two joints `joint1` and
`joint2`, you might have some controllers configured as:

```yaml
joint1_position_controller:
  type: effort_controllers/JointPositionController
  joint: joint1
  pid: {p: 10.0, i: 0.0, d: 5.0}

joint2_position_controller:
  type: effort_controllers/JointPositionController
  joint: joint2
  pid: {p: 10.0, i: 0.0, d: 5.0}

velocity_controller:
  type: effort_controllers/JointGroupVelocityController
  joints:
    - joint1
    - joint2
  joint1/pid: {p: 10.0, i: 0.0, d: 5.0}
  joint2/pid: {p: 10.0, i: 0.0, d: 5.0}
```

To use the new controllers, you can change it, *e.g.*, as follows:
```yaml
gravity: [0, 0, -9.81] # this should be projected on the base frame

position_controller:
  type: dynamics_controllers/KdlChainController
  sub_controller:
    # the sub-controller is basically the set of controllers you had before!
    joint1:
      type: effort_controllers/JointPositionController
      joint: joint1
      pid: {p: 10.0, i: 0.0, d: 5.0}
    joint2:
      type: effort_controllers/JointPositionController
      joint: joint2
      pid: {p: 10.0, i: 0.0, d: 5.0}

velocity_controller:
  type: dynamics_controllers/KdlTreeController
  sub_controller:
    # the subcontroller is basically the controller you had before!
    type: effort_controllers/JointGroupVelocityController
    joints:
      - joint1
      - joint2
    joint1/pid: {p: 10.0, i: 0.0, d: 5.0}
    joint2/pid: {p: 10.0, i: 0.0, d: 5.0}
```

Note that you will likely have to tune again the gains of the sub-controllers to
achieve good performances.

#### Detailed Controllers Configuration

Below, parameters relative to each controller are listed.
Those marked as "searched for" can live in any parent namespace since the
`NodeHandle::searchParam` method is used to locate them.
This facilitates "sharing" common parameters such as the gravity vector, which
should not depend on a specific controller (at least in principle!).

##### KdlChainController

- `sub_controller`: should contain the configuration of a set of controllers
  of type compatible with `EffortJointInterface`. It cen either be a single
  "unnamed" controller directly living in the `sub_controller` namespace or a
  set of "named" controllers.
- `robot_description`: should contain the URDF of the robot. Searched for.
- `gravity`: list with three elements, representing the gravity vector in the
  base frame of the chain. Searched for. Default: `[0,0,0]`.
- `chain_base`: base of the chain. Searched for. Default: name of the root link
  from the robot description.
- `chain_tip`: tip of the chain. Searched for. Default: if a single "branch"
  is rooted at `base_link`, `chain_tip` will correspond to the last link of the
  chain. If at any point multiple children are found, loading will fail.
- `gravity_compensation_only`: as discussed above. Default: `false`.

##### KdlTreeController

- `sub_controller`: should contain the configuration of a set of controllers
  of type compatible with `EffortJointInterface`. It cen either be a single
  "unnamed" controller directly living in the `sub_controller` namespace or a
  set of "named" controllers.
- `robot_description`: should contain the URDF of the robot. Searched for.
- `gravity`: list with three elements, representing the gravity vector in the
  base frame of the chain. Searched for. Default: `[0,0,0]`.
- `tree_root`: base of the tree. Searched for. Default: name of the root link
  from the robot description.
- `gravity_compensation_only`: as discussed above. Default: `false`.



### Limitations

The following is a list of "limitations" in the sense that if you need one or
more of the following features, you will likely have to write new controllers
by yourself or to find proper workarounds.

- They all assume a static base. The main implication is that you cannot
  "serially join" controllers for different parts of the robot.
- They require proper identification of the Dynamic Parameters.
- The parameters must be provided in URDF format. This is a limitation since
  the identification will often return a smaller set of regrouped parameters.
- The sub-controllers must control all joints of the internal `Chain`/`Tree`.



### Possible Improvements

- [ ] Compensation of external payloads, mainly in two ways:
    - By adding a subscriber that can update the `wrenches_` members
    - Allowing to dynamically modify the `Chain`/`Tree` instances
- [ ] Support for moving bases. This likely requires to re-implement the
      controllers using another library, such as pinocchio, or to expand KDL.
      A possible workaround is to add a "floating joint" (in the form of a
      3T3R chain) as the parent of the base.
- [ ] Additional parameters such as joint inertia and viscous friction.
      Note that some of these parameters are already inside KDL,
      even though they are still not used in many solvers (eg, the damping).
- [x] Allow multiple sub-controllers to manage different joints.


### Personal todo list:

- [x] How should the license be pasted? Some headers contain only WillowGarage,
  some many institutions. Should I put myself as well?
- [x] Check CMakeLists.txt (install targets etc).
- [x] Tell about the KDL version that is needed!
- [x] Complete `KdlTreeController` so that proper sub-tree extraction is possible.
- [ ] Enforce effort limits
