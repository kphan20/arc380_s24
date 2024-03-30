import compas_rrc as rrc
import compas.geometry as cg
from part2 import process_image
from functools import cmp_to_key


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


def transform_task_to_world_frame(
    ee_frame_t: cg.Frame, task_frame: cg.Frame
) -> cg.Frame:
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


class EETaskHandler:
    def __init__(
        self, abb_rrc: rrc.AbbClient, task_frame: cg.Frame, init_frame: cg.Frame
    ) -> None:
        self.abb_rrc = abb_rrc
        self.task_frame = task_frame
        self.init_frame = init_frame  # Starting point in world frame
        self.is_gripper_on = False
        self.gripper_off()

    def reset(self, speed):
        """
        Helper function to return arm back to initial position for sorting runs
        """
        if self.is_gripper_on:
            self.gripper_off()
        _ = self.abb_rrc.send_and_wait(
            rrc.MoveToFrame(self.init_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR)
        )

    def lift_pen(self, dest: cg.Point):
        """Helper function that lifts pen
        when we want to move the pen but not draw"""

        return cg.Point(
            dest.x, dest.y, dest.z + 5
        )  # TODO test values for lift, when to call this function

    def move_to_point(self, dest: cg.Point, speed: float, quat: cg.Quaternion = None):
        """Helper function that handles conversion to world frame and command sending"""

        dest.z = dest.z - 0

        if quat is None:
            frame = cg.Frame(dest, [1, 0, 0], [0, -1, 0])
        else:
            frame = cg.Frame.from_quaternion(quat, dest)

        frame_w = transform_task_to_world_frame(frame, self.task_frame)
        print(frame_w)
        _ = self.abb_rrc.send_and_wait(
            rrc.MoveToFrame(frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR)
        )
        print("Moved to point")

    def move_to_origin(self):
        """Moves robot to task space origin"""
        self.move_to_point(self.lift_pen(cg.Point(0, 0)), 30)

    def gripper_on(self):
        """
        Turns gripper on and waits
        """
        self.is_gripper_on = True
        self.abb_rrc.SetDigital("DO00", 1)
        self.abb_rrc.WaitTime(2)  # TODO see if these are the right settings to wait

    def gripper_off(self):
        """
        Turns gripper off and waits
        """
        self.is_gripper_on = False
        self.abb_rrc.SetDigital("DO00", 0)
        self.abb_rrc.WaitTime(2)

    def sort_pieces(self):
        """
        Resets the robot and attempts to sort configuration
        """

        # resets the robot to some home position
        speed = 30
        self.reset(speed)
        self.abb_rrc.WaitTime(2)

        def piece_comp(p1, p2):
            """
            Comparator function that should order pieces from largest to smallest
            """

            # if pieces are the same size, break tie with id
            if p1["size"] == p2["size"]:
                return p1["id"] - p2["id"]

            # converts boolean to +-1
            return (p1["size"] > p2["size"]) * -2 + 1

        # extracts pieces by color and features of each individual piece
        pieces = process_image()

        for color in pieces:
            # sort each color of piece from largest to smallest
            sorted_color_pieces = sorted(color, key=cmp_to_key(piece_comp))

            # leave largest piece in place and sort on top of it
            largest_piece = sorted_color_pieces.pop(0)
            sorting_center = largest_piece["pos"]

            for piece in sorted_color_pieces:
                # TODO implement sorting
                # 1. go to piece location while raised
                # 2. lower to pick up piece
                # 3. rotate if the object is a square
                # 4. Move to largest piece while raised
                # 5. drop piece
                pass


if __name__ == "__main__":
    try:
        # Create Ros Client
        ros = rrc.RosClient()
        ros.run()

        # Create ABB Client
        abb_rrc = rrc.AbbClient(ros, "/rob1-rw6")
        print("Connected.")

        # ================================== YOUR CODE HERE ==================================
        abb_rrc.send(rrc.SetTool("vac_gripper"))
        origin = cg.Point(167.21, 465.98, 27.64)  # for 2xx
        other_point = cg.Point(52.75, 465.98, 27.22)  # for 2xx
        x_axis = cg.Point(52.75, 563.31, 27.71)  # for 2xx
        task_frame = create_frame_from_points(origin, x_axis, other_point)
        handler = EETaskHandler(abb_rrc, task_frame)
        handler.sort_pieces()
    finally:
        # Close client
        ros.close()
        ros.terminate()
