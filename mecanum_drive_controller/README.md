--- MECANUM DRIVE CONTROLLER ---


INTRODUCTION
   * Controller for a mecanum drive mobile base
   * This is a 2D planar controller
   * The development of this controller was based on the differential drive controller called diff_drive_controller

RESTRICTIONS and ASSUMPTIONS
   * the controller only works with 4WD mobiles
   * the mobile robot is associated a frame called the CENTER frame
      - the origin is at the center of the wheels' contact points
      - X is front
      - Y is left
   * the 4 wheels are identical in radius
   * the wheels rotation axis is aligned with Y (not -Y)
      - that is the axis of rotation of each wheel is in the Y direction 
   * the wheels need to have the same parent frame
   * the projection of the wheels frame origin on the floor is the contact point of the wheel

DESCRIPTION
   * The velocity command is a body velocity, this body frame is called the BASE frame
      - By default the BASE frame = the CENTER frame
   * The BASE frame can be offset (with respect to the CENTER frame) using the **base_frame_offset** parameter
      - WARNING: the offset of the BASE frame with respect to the CENTER is **not automatically retrieved**, it needs to be indicated by hand
   * The odometry computes the location of a frame which is also the BASE frame 
      - The reference frame of the odometry is the initial position of the BASE frame at t=0
      - This reference frame is called the ODOMETRY frame
   * The wheels are refered to using an index 0, 1, 2, 3
      - wheel 0: front left wheel
      - wheel 1: back left wheel
      - wheel 2: back right wheel
      - wheel 3: front right wheel