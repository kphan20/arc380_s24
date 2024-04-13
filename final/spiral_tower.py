import numpy as np
from compas.geometry import Polygon, Rotation, Translation, Frame
from control import EETaskHandler
from perception import process

class SpiralCentroidCalculator:
    """
    Gets the centroids of the levels of the tower.
    All of the centroids are on the perimeter of a circle that is centered
    around the overall center of the tower, and they will all be evenly spaced
    around the circle.
    """

    def __init__(self, num_positions: int, radius: float) -> None:
        """
        num_positions decides how many times the circle is partitioned
        radius determines the distance from the center of the tower centroids are placed
        """
        self.r = radius
        self.angles = [i * np.pi * 2 / num_positions for i in range(num_positions)]
        self.curr_idx = 0

    def get_centroid(self):
        """
        Gets position of next centroid in the tower
        """
        angle = self.angles[self.curr_idx]
        self.curr_idx = (self.curr_idx + 1) % len(self.angles)

        return self.r * np.cos(angle), self.r * np.sin(angle)
    
class SpiralTriangleCalculator:
    """
    Gets positions that the next triangle of blocks should be places at
    """
    def __init__(self) -> None:
        """
        Creates a triangle polygon and rotation object (triangle is rotated 60 degrees at each level)
        """
        self.triangle = Polygon([[],[],[]])
        self.rotation = Rotation.from_axis_and_angle([0, 0, 1], 60 * np.pi/180, self.triangle.centroid)
    
    def get_triangle(self, centroid):
        """
        Rotates triangle and returns translated copy
        """
        self.triangle.transform(self.rotation)
        return self.triangle.transformed(Translation.from_vector(centroid))

def main():
    try:
        handler = EETaskHandler()

        # First centroid of the tower and overall centroid of the spiral
        starting_frame = Frame()

        # current_frame will be changed by the SpiralCentroidCalulator
        current_frame = starting_frame

        # Get positions of blocks and pieces
        poses, centroids, blocks = process()

        spiral_centroids = SpiralCentroidCalculator()
        block_placer = SpiralTriangleCalculator()

        finished_building = False

        # 1. Clear space around starting frame for construction
        
        # 2. Begin loop
        while not finished_building:
            # 3. place down block triangle around current_frame

            # 4. calculate next centroid
            # TODO change get_centroid output to proper frame
            current_frame = spiral_centroids.get_centroid()

            # 5. place down acrylic piece at current_frame

            # 6. check if tower is finished building
            pass
    
    finally:
        handler.cleanup()

if __name__ == '__main__':
    main()