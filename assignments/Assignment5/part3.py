import compas_rrc as rrc
import compas.geometry as cg
from part2 import process_image, Shape
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
    def __init__(self, abb_rrc: rrc.AbbClient, task_frame: cg.Frame) -> None:
        self.abb_rrc = abb_rrc
        self.task_frame = task_frame
        self.is_gripper_on = False
        self.gripper_off()

    def reset(self, speed):
        """
        Helper function to return arm back to initial position for sorting runs
        """
        if self.is_gripper_on:
            self.gripper_off()

        home = rrc.RobotJoints([0, 0, 0, 0, 90, 320])
        _ = self.abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))

    def lift_frame(self, frame: cg.Frame, lifted_height=40):
        """Helper function that lifts pen
        when we want to move the pen but not draw"""
        # lifted_height = 25
        lifted_point = cg.Point(
            frame.point.x, frame.point.y, frame.point.z + lifted_height
        )

        return cg.Frame(
            lifted_point, frame.xaxis, frame.yaxis
        )  # TODO test values for lift, when to call this function

    def move_to_world_frame(self, dest: cg.Frame, speed: float):
        frame = cg.Frame(dest.point, [1, 0, 0], [0, -1, 0])
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

    def sort_pieces(self):
        """
        Resets the robot and attempts to sort configuration
        """

        # resets the robot to some home position
        speed = 100
        self.reset(speed)
        self.abb_rrc.send_and_wait(rrc.WaitTime(2))

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

        intermediate = rrc.RobotJoints([60, 0, 0, 0, 90, 320])
        _ = self.abb_rrc.send_and_wait(
            rrc.MoveToJoints(intermediate, [], speed, rrc.Zone.FINE)
        )

        intermediate2 = rrc.RobotJoints([60, 0, 23, 0, 63, 320])
        _ = self.abb_rrc.send_and_wait(
            rrc.MoveToJoints(intermediate2, [], speed, rrc.Zone.FINE)
        )

        colors = dict()

        for piece in pieces:
            color_arr = colors.get(piece["color"], list())
            color_arr.append(piece)
            colors[piece["color"]] = color_arr

        for color_pieces in colors.values():
            # sort each color of piece from largest to smallest
            sorted_color_pieces = sorted(color_pieces, key=cmp_to_key(piece_comp))

            # leave largest piece in place and sort on top of it
            largest_piece = sorted_color_pieces.pop(0)
            sorting_center = largest_piece["pos"]

            for idx, piece in enumerate(sorted_color_pieces):
                # 1. go to piece location while raised
                self.gripper_off()
                raised_frame = self.lift_frame(piece["pos"])
                self.move_to_world_frame(raised_frame, speed)  # TODO raise frame

                # 2. lower to pick up piece
                self.move_to_world_frame(self.lift_frame(piece["pos"], 3), speed)
                self.gripper_on()

                # 3. rotate if the object is a square
                # if piece["shape"] == Shape.SQUARE:
                #     robot_joints, _ = self.abb_rrc.send_and_wait(rrc.GetJoints())
                #     _ = self.abb_rrc.send_and_wait(rrc.MoveToJoints(robot_joints, [], speed, rrc.Zone.FINE))
                #     pass  # TODO implement rotate method

                # 4. Move to largest piece while raised
                self.move_to_world_frame(
                    self.lift_frame(sorting_center), speed
                )  # TODO raise frame

                # 5. Drop piece
                self.move_to_world_frame(
                    self.lift_frame(sorting_center, 6 * (idx + 1)), speed
                )
                self.gripper_off()

                # 6. Raise to avoid hitting stack
                self.move_to_world_frame(
                    self.lift_frame(sorting_center, 6 * (idx + 2)), speed
                )


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
        x_axis = cg.Point(-236.99, 498.73, 21.31)
        origin = cg.Point(241.78, 485.88, 21.1)
        other_point = cg.Point(235.89, 193.29, 21.34)
        task_frame = create_frame_from_points(origin, x_axis, other_point)
        handler = EETaskHandler(abb_rrc, task_frame)
        handler.sort_pieces()
    finally:
        # Close client
        ros.close()
        ros.terminate()
