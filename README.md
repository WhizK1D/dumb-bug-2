# dumb-bug-2
Simple implementation of the Bug-2 path finding algorithm, implemented using rospy for a simple differential
drive 2-wheeled robot

## Node Description
- Homing Beacon: Pre-provided node that publishes the location of the target
- Overwatch: Node that monitors the current distance of the robot to the target
- Laser Reader: Node to monitor the region in a 3m semi-circle in front for any obstacles and update the robot state
- Robot: Actual robot node that defines operations for the robot to perform depending on its state
