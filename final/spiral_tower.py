from control import EETaskHandler
from perception import process, process2d
import json

def parse_json(filename):
    with open(filename, "r") as f:
        objects = json.load(f)
    
    return objects
    
def match_piece(obj, piece_list):
    def get_dist(piece):
        return 0 # TODO see how to get x, y of piece

    x, y = obj["pos"][0], obj["pos"][1]
    champ_ind = 0
    champ_dist = get_dist(piece_list[0])
    for idx, piece in enumerate(piece_list):
        dist = get_dist(piece)
        if dist < champ_dist:
            champ_ind = idx
            champ_dist = dist
    
    return champ_ind

def main():
    try:
        handler = EETaskHandler()

        objects = parse_json("spiral_tower.json")

        # Get positions of blocks and pieces
        blocks, acrylic = process2d()

        # 1. Clear space around starting frame for construction
        
        # 2. Begin loop
        for obj in objects:
            # 3. Find corresponding piece from perception output
            if obj["shape"] == "block":
                search_list = blocks
            else:
                search_list = acrylic
            
            # TODO see if we need to convert rhino coordinates to world coords
            search_idx = match_piece(obj, search_list)
            piece = search_list.pop(search_idx)
            
            # 4. Move piece into proper location
    
    finally:
        handler.cleanup()

if __name__ == '__main__':
    main()