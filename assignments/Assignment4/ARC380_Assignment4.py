import numpy as np
import compas.geometry as cg
import compas_rrc as rrc

# Define any additional imports here if needed



def create_frame_from_points(point1: cg.Point, point2: cg.Point, point3: cg.Point) -> cg.Frame:
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


def transform_task_to_world_frame(ee_frame_t: cg.Frame, task_frame: cg.Frame) -> cg.Frame:
    """Transform a task frame to the world frame.

    Args:
        ee_frame_t (cg.Frame): The end-effector frame defined in task space.
        task_frame (cg.Frame): The task frame.

    Returns:
        cg.Frame: The task frame in the world frame.
    """
    ee_frame_w = None
    # ================================== YOUR CODE HERE ==================================

    # Part 1.d.
    ee_frame_w = task_frame.to_world_coordinates(ee_frame_t)

    # ====================================================================================
    return ee_frame_w


# ====================== Drawing effects and helper functions ============================

# Part 2


class EETaskHandler:
    def __init__(self, abb_rrc: rrc.AbbClient, task_frame: cg.Frame) -> None:
        self.abb_rrc = abb_rrc
        self.task_frame = task_frame

    def lift_pen(self, dest: cg.Point):
        """Helper function that lifts pen
        when we want to move the pen but not draw"""

        return cg.Point(
            dest.x, dest.y, dest.z + 5
        )  # TODO test values for lift, when to call this function

    def move_to_point(self, dest: cg.Point, speed: float, quat: cg.Quaternion = None):
        """Helper function that handles conversion to world frame and command sending"""

        if quat is None:
            frame = cg.Frame(dest, self.task_frame.xaxis, self.task_frame.yaxis)
        else:
            frame = cg.Frame.from_quaternion(quat, dest)

        frame_w = transform_task_to_world_frame(frame, self.task_frame)
        _ = self.abb_rrc.send_and_wait(
            rrc.MoveToFrame(frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR)
        )

    def move_to_origin(self):
        """Moves robot to task space origin"""
        self.move_to_point(cg.Point(0, 0), 10)

    def draw_rectangle(
        self, center: cg.Point, length: float, width: float, theta: float
    ):
        """center is in task frame, theta is rotation around task frame z axis"""

        # construct rectangle corner points
        hl, hw = length / 2, width / 2
        top_left = cg.Point(center.x - hw, center.y + hl)
        top_right = cg.Point(center.x + hw, center.y + hl)
        bot_right = cg.Point(center.x + hw, center.y - hl)
        bot_left = cg.Point(center.x - hw, center.y - hl)
        corners = [top_left, top_right, bot_right, bot_left]

        # rotate each point around the rectangle's center
        rotation = cg.Rotation.from_axis_and_angle(cg.Vector.Zaxis, theta, center)
        cg.Point.transform_collection(corners, rotation)

        # move pen to corners and draw when appropriate
        speed = 30
        self.move_to_point(self.lift_pen(corners[0]), speed)

        for corner in corners:
            self.move_to_point(corner, speed)

        self.move_to_point(corners[0])


# ========================================================================================


if __name__ == '__main__':

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')

    # ================================== YOUR CODE HERE ==================================
    abb_rrc.send(rrc.SetTool(""))  # TODO insert tool name

    # Parts 1.e. and 2

    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
