import numpy as np
import compas.geometry as cg
import compas_rrc as rrc
import random
import math

# Define any additional imports here if needed


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
        rotation = cg.Rotation.from_axis_and_angle(cg.Vector.Zaxis(), theta, center)
        cg.Point.transform_collection(corners, rotation)

        # move pen to corners and draw when appropriate
        speed = 50
        self.move_to_point(self.lift_pen(corners[0]), speed)

        for corner in corners:
            self.move_to_point(corner, speed)

        self.move_to_point(corners[0], speed)
        self.move_to_point(self.lift_pen(corners[0]), speed)

    # draw curved line given set of control points (2e)
    def draw_curve(self, control_points: list[cg.Point]):

        # construct bezier curve
        curve = cg.Bezier(control_points)
        # sample curve
        sampled_points = curve.locus(100)
        #for i in range(0, 1, 0.01):  # TODO test step size (currently 1%)
            #sampled_points.append(curve.point_at(i))

        # move pen to start and draw
        speed = 50
        self.move_to_point(self.lift_pen(sampled_points[0]), speed)

        for point in sampled_points:
            self.move_to_point(point, speed)

        # lift pen at end
        self.move_to_point(self.lift_pen(sampled_points[-1]), speed)

    # create a line that randomly jitters along its length (2h)
    def jitter_line(self, start: cg.Point, end: cg.Point):
        # create line
        line = cg.Line(start, end)

        # set max jitter distance
        max_jitter = line.length / 20  # TODO test value

        # TODO verify shift of y axis to be parallel to line
        local_x = line.direction
        local_y = local_x.cross(cg.Vector.Zaxis())

        # create jittered points
        jittered_points = []
        for i in range(1, 100):  # TODO test step size (currently 1%)
            intervs = i / 100.0
            jitter = random.uniform(-max_jitter, max_jitter)
            jittered_points.append(line.point(intervs) + local_y * jitter)

        # move pen to start and draw
        speed = 50
        self.move_to_point(self.lift_pen(jittered_points[0]), speed)

        for point in jittered_points:
            self.move_to_point(point, speed)

        # lift pen at end
        self.move_to_point(self.lift_pen(jittered_points[-1]), speed)

    def draw_hatch(self, center: cg.Point, length: float, width: float, theta: float):
        """center is in task frame, theta is rotation around task frame z axis"""

        hl, hw = length / 2, width / 2
        delta = 5  # TODO tune distance between hashes
        hatches = []
        x, y = -hw, hl

        # helper function to generate each hatch
        def get_hatch():

            # checks which edge the hatch intersects
            y_potential = -hw - x + y
            x_potential = -hl - y + x
            d1 = hw**2 + y_potential**2
            d2 = hl**2 + x_potential**2
            if d1 < d2:
                hatch = (cg.Point(x, y), cg.Point(-hw, y_potential))
            else:
                hatch = (cg.Point(x, y), cg.Point(x_potential, -hl))

            return hatch

        # find hatches originating from top edge
        while x + delta <= hw:
            x += delta
            hatches.append(get_hatch())

        x = hw

        # find hatches originating from right edge
        while y - delta > -hl:
            y -= delta
            hatches.append(get_hatch())

        # rotate and translate each rectangle to match border rectangle
        rotation = cg.Rotation.from_axis_and_angle(cg.Vector.Zaxis(), theta)
        translation = cg.Translation.from_vector([center.x, center.y, center.z])

        speed = 50
        for hatch in hatches:
            rotated_hatch = cg.Point.transformed_collection(hatch, rotation)
            start_t, end_t = cg.Point.transformed_collection(rotated_hatch, translation)
            self.move_to_point(self.lift_pen(start_t), speed)
            self.move_to_point(start_t, speed)
            self.move_to_point(end_t, speed)

        self.move_to_point(self.lift_pen(end_t), speed)

    # Draw a circle of any size given its radius and center point. (2c)
    def draw_circle(self, radius: float, center: cg.Point):

        # creates a list of points from the circle
        points = []
        for i in range(100):
            angle = (
                2 * np.pi * i / 100
            )  # breaks up the circum of the circle into essentially 100 points
            x = center[0] + radius * np.cos(angle)  # finding points along the radius
            y = center[1] + radius * np.sin(angle)  # finding points along the radius
            z = center[2]  # Circle is in the XY plane, so Z is constant
            points.append(cg.Point(x, y, z))

        # moving to the points & drawing
        speed = 50
        self.move_to_point(self.lift_pen(points[0]), speed)

        for point in points:
            self.move_to_point(point, speed)

        self.move_to_point(points[0], speed)
        self.move_to_point(self.lift_pen(points[0]), speed)


# ========================================================================================


if __name__ == "__main__":

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, "/rob1-rw6")
    print("Connected.")

    # ================================== YOUR CODE HERE ==================================
    abb_rrc.send(rrc.SetTool("group3"))

    # Parts 1.e. and 2
    #origin = cg.Point(163.27, 407.60, 30.92) for 3xx
    #x_axis = cg.Point(163.27, 554.01, 31.41) for 3xx
    #other_point = cg.Point(-26.05, 545.02, 30.88) for 3xx
    origin = cg.Point(167.21, 465.98, 27.64) # for 2xx
    other_point = cg.Point(52.75, 465.98, 27.22) # for 2xx
    x_axis = cg.Point(52.75, 563.31, 27.71) # for 2xx
    task_frame = create_frame_from_points(origin, x_axis, other_point)
    handler = EETaskHandler(abb_rrc, task_frame)

    handler.move_to_origin()
    handler.draw_rectangle(cg.Point(0, 0), 50, 50, math.pi / 4)
    curve_list = [cg.Point(-25, -55), cg.Point(0, -70), cg.Point(25, -55)]
    handler.draw_curve(curve_list)
    handler.jitter_line(cg.Point(0, 25), cg.Point(0, -25))
    handler.draw_hatch(cg.Point(0, 0), 50, 50, math.pi / 4)
    handler.draw_circle(30, cg.Point(0, 0))

    # ====================================================================================

    # End of Code
    print("Finished")

    # Close client
    ros.close()
    ros.terminate()
