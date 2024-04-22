from control import EETaskHandler
from perception import process2d
from perception_helpers import Color, task_to_world_frame
import json
import compas.geometry as cg

def parse_json(filename):
    with open(filename, "r") as f:
        objects = json.load(f)

    min_x, min_y, max_x, max_y = float('inf'), float('inf'), float('-inf'), float('-inf')
    min_z = float("inf")
    # convert the coordinates to world frame
    for obj in objects:
        task_coords = cg.Frame(cg.Point(*obj["pos"]), [1, 0, 0], [0, 1, 0])

        if obj["shape"] != "disk":
            task_coords.rotate(obj["rot"], point=task_coords.point)
            print(task_coords.point)

        piece_world_coords: cg.Frame = task_to_world_frame(task_coords)

        min_x, min_y, min_z = min(piece_world_coords.point.x, min_x), min(piece_world_coords.point.y, min_y), min(piece_world_coords.point.z, min_z)
        max_x, max_y = max(piece_world_coords.point.x, max_x), max(piece_world_coords.point.y, max_y)

        obj["pos"] = piece_world_coords
        if obj["shape"] == "block":
            continue
        if "blue" in obj["color"]:
            obj["color"] = Color.BLUE
        elif "red" in obj["color"]:
            obj["color"] = Color.RED
        elif "yellow" in obj["color"]:
            obj["color"] = Color.YELLOW

    print("x", min_x, max_x)
    print("y", min_y, max_y)
    print("z", min_z)
    return objects
    
def match_piece(obj: cg.Frame, piece_list):
    def get_dist(piece):
        piece_world_coords: cg.Frame = piece["pos"]
        x, y = obj["pos"].point.x, obj["pos"].point.y # world coords
        piece_x, piece_y = piece_world_coords.point.x, piece_world_coords.point.y
        return  (x - piece_x)**2 + (y - piece_y)**2 # TODO see how to get x, y of piece

    champ_ind = -1
    champ_dist = float("inf")
    for idx, piece in enumerate(piece_list):
        if obj["shape"] != "block" and obj["color"] != piece["color"]:
            continue
        dist = get_dist(piece)
        if dist < champ_dist:
            champ_ind = idx
            champ_dist = dist
    
    return champ_ind

def main(use_handler=False, debug=False):
    try:
        speed = 50
        if use_handler:
            handler = EETaskHandler()
            handler.reset(speed)

        objects = parse_json("spiral_tower.json")

        # Get positions of blocks and pieces
        blocks, acrylic_disks, acrylic_squares, small_disks = process2d(debug=debug, take_image=False)

        curr_tower_height = 0
        # 1. Clear space around starting frame for construction
        if use_handler:
            handler.intermediate(speed)

        # 2. Begin loop
        for idx, obj in enumerate(objects):
            # 3. Find corresponding piece from perception output
            if obj["shape"] == "block":
                search_list = blocks
            elif obj["shape"] == "square":
                search_list = acrylic_squares
            elif obj["shape"] == "disk" and obj["size"] > 2.6:
                search_list = acrylic_disks
            else:
                search_list = small_disks
            
            search_idx = -1
            should_quit = False
            while search_idx == -1:
                search_idx = match_piece(obj, search_list)
                if search_idx == -1:
                    print(search_list)
                    print(f"No piece of color {obj['color']}, shape {obj['shape']}, and size {obj['size']} found")
                    pressed = input("Press some key when more objects have been placed or q to quit\n")
                    if pressed.strip() == "q":
                        should_quit = True
                        break
            if should_quit:
                break

            piece = search_list.pop(search_idx) # Piece on workbench
            print(piece)
            
            # 4. Move piece into proper location
            if not use_handler:
                continue
            
            print(piece)
            # move over piece
            handler.lift_and_move_to_world_frame(piece["pos"], speed, obj["pos"].point.z + 20)
            # must adjust height of grabber for block vs disk
            if obj["shape"] == "block":
                # adjust z for block
                handler.lift_and_move_to_world_frame(piece["pos"], speed, 16)
                handler.rotate(40, piece["orientation"])
                # turn gripper on
                handler.gripper_on()
            else:
                handler.lift_and_move_to_world_frame(piece["pos"], speed, 4)
                # turn gripper on
                handler.gripper_on()

            # move back up
            handler.lift_and_move_to_world_frame(piece["pos"], speed, obj["pos"].point.z + 20)

            # move over tower
            handler.lift_and_move_to_world_frame(obj["pos"], speed, obj["pos"].point.z + 20)

            # move piece to tower
            handler.move_to_world_frame(obj["pos"], speed)
            handler.gripper_off()

            # move back up
            handler.lift_and_move_to_world_frame(obj["pos"], speed, obj["pos"].point.z + 20)

            curr_tower_height = max(curr_tower_height, obj["pos"].point.z)
        print(f"Reached index {idx}")
        print(curr_tower_height)
    finally:
        if use_handler:
            handler.cleanup()

if __name__ == '__main__':
    main(use_handler=True)