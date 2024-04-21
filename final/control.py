import compas_rrc as rrc
import compas.geometry as cg
import math

def create_frame_from_points(
    point1: cg.Point, point2: cg.Point, point3: cg.Point
) -> cg.Frame:
    """Create a frame from three points.

    Args:
        point1 (cg.Point): The first point (origin).
        point2 (cg.Point): The second point (along x axis).
        point3 (cg.Point): The third point (within xy-plane).

    Returns:
        cg.Frame: The frame that fits the given 3 points.
    """
    frame = None
    # ================================== YOUR CODE HERE ==================================

    # Part 1.c.
    frame = cg.Frame.from_points(
        point1, point2, point3
    )  # TODO see if defining origin differently would help

    # ====================================================================================
    return frame


class EETaskHandler:
    """
    Interface to interact with the robot arm and perform tasks with the end effector.
    In order to properly use the class, all commands should be wrapped in a try block
    and cleanup() being called in a finally block.
    Ex.\n
    try:
        handler = EETaskHandler()\n
        // COMMANDS HERE
    finally:
        handler.cleanup()
    """
    def __init__(self) -> None:
        # Create Ros Client
        ros = rrc.RosClient()
        ros.run()

        # Create ABB Client
        abb_rrc = rrc.AbbClient(ros, "/rob1-rw6")
        print("Connected.")

        abb_rrc.send(rrc.SetTool("vac_gripper"))
        x_axis = cg.Point(-236.99, 498.73, 21.31)
        origin = cg.Point(241.78, 485.88, 21.1)
        other_point = cg.Point(235.89, 193.29, 21.34)
        task_frame = create_frame_from_points(origin, x_axis, other_point)

        self.ros = ros
        self.abb_rrc = abb_rrc
        self.task_frame = task_frame
        self.is_gripper_on = True
        self.gripper_off()

    def cleanup(self):
        self.ros.close()
        self.ros.terminate()

    def reset(self, speed):
        """
        Helper function to return arm back to initial position for sorting runs
        """
        if self.is_gripper_on:
            self.gripper_off()

        home = rrc.RobotJoints([0, 0, 0, 0, 90, 60])
        _ = self.abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))

    def intermediate(self, speed):
        """
        Hard coded intermediate point
        """
        intermediate = rrc.RobotJoints([70, 0, 0, 0, 90, 60])
        _ = self.abb_rrc.send_and_wait(
            rrc.MoveToJoints(intermediate, [], speed, rrc.Zone.FINE)
        )

    def rotate(self, speed, rotation):
        """
        Rotate the arm in place after reaching the block
        """
        angles, _ = self.abb_rrc.send_and_wait(
            rrc.GetJoints()
        )
        angles.rax_6 = 153 + rotation # TODO 153 is where gripper is horizontal - may need to change this
        print(angles)
        _ = self.abb_rrc.send_and_wait(
            rrc.MoveToJoints(angles, [], speed, rrc.Zone.FINE)
        )

    def lift_frame(self, frame: cg.Frame, lifted_height=40):
        """
        Helper function that takes a frame and elevates in the z-direction
        """
        lifted_point = cg.Point(
            frame.point.x, frame.point.y, frame.point.z + lifted_height
        )

        return cg.Frame(
            lifted_point, frame.xaxis, frame.yaxis
        )  # TODO test values for lift, when to call this function

    def lift_and_move_to_world_frame(self, dest: cg.Frame, speed: float, lifted_height=40):
        """
        Convenience function to combine lifting and moving
        """
        self.move_to_world_frame(self.lift_frame(dest, lifted_height), speed)

    def move_to_world_frame(self, dest: cg.Frame, speed: float):
        """
        Move 
        """
        #frame = cg.Frame(dest.point, [1, 0, 0], [0, -1, 0])
        # rotate around x axis rather than redefining axes
        frame = dest.rotated(math.pi, cg.Vector.Xaxis(), dest.point) # TODO see if this works
        _ = self.abb_rrc.send_and_wait(
            rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR)
        )

    def gripper_on(self):
        """
        Turns gripper on and waits
        """
        if not self.is_gripper_on:
            self.is_gripper_on = True
            self.abb_rrc.send_and_wait(rrc.SetDigital("DO00", 1))
            self.abb_rrc.send_and_wait(rrc.WaitTime(2))

    def gripper_off(self):
        """
        Turns gripper off and waits
        """
        if self.is_gripper_on:
            self.is_gripper_on = False
            self.abb_rrc.send_and_wait(rrc.SetDigital("DO00", 0))
            self.abb_rrc.send_and_wait(rrc.WaitTime(2))

