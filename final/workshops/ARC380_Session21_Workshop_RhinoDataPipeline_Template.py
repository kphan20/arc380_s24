import json

import Rhino.Geometry as rg
import rhinoscriptsyntax as rs


def get_objects():
    objects = []

    # Prompt the user to select Brep objects in Rhino
    brep_ids = rs.GetObjects("Select Breps", rs.filter.polysurface)
    if not brep_ids:
        print("No Breps selected.")
        return list()
    
    # Extract geometry and layer information for each object
    for id in brep_ids:
        brep = rs.coercebrep(id)
        layer_name = rs.ObjectLayer(id)
        
        # get object shape, size, and color
        shape, size, color = parse_layer_name(layer_name)
        
        # Get object pose (located at center of top face which is what vacuum gripper cares about)
        pos, rot = get_pose(brep, shape)
        
        objects.append({"shape": shape, "size": size, "color":color, "pos":pos, "rot":rot})    
        
    return objects


def parse_layer_name(layer_name):
    shape, size, color = None, None, None

    # Get shape, size, and color from layer name
    if "block" in layer_name.lower():
        shape = "block"
    else:
        sublayer = layer_name.split("::")[-1]
        tag, color = sublayer.split("_")
        color = color.lower()
        if tag[0].lower() == 'd':
            shape = "disk"
            size = float(tag[1:])
        elif tag[0].lower() == 's':
            shape = "square"
            size = 2

    return shape, size, color


def get_pose(brep, shape):
    pos, rot = None, None

    # Compute position of the brep by projecting the centroid along the Z axis
    vmp = rg.VolumeMassProperties.Compute(brep, volume=False, firstMoments=True, secondMoments=False, productMoments=False)
    centroid = vmp.Centroid
    pos = rg.Intersect.Intersection.ProjectPointsToBreps([brep], [centroid], rg.Vector3d.ZAxis, 1e-6)[1]
    
    # Compute rotation using the longest axis
    rot = 0.0 # radians
    if shape != "disk":
        # get long edge
        sorted_edges = sorted(brep.Edges, key=lambda e: e.GetLength())
        longest = sorted_edges[-1]
        
        # construct vector from longest edge vertices and compute the rotation angle
        edge_vec = rg.Vector3d(longest.PointAtEnd - longest.PointAtStart)
        rot = rg.Vector3d.VectorAngle(rg.Vector3d.XAxis, edge_vec, rg.Vector3d.ZAxis)

    # Convert position Rhino.Point3d to tuple
    pos_tuple = (pos.X, pos.Y, pos.Z)

    return pos_tuple, rot


def export_JSON(obj_tuples, filename):
    # Convert the tuple list into a dictionary of dictionaries
    obj_dict = {}
    
    # Alternatively, if get_objects returns a list of tuples:
        # keys = [keys in order]
        # dict(zip(keys, obj_tuple)) creates a dictionary with the keys mapped to each value in one tuple
    
    # Save dict as JSON file
    for i in range(len(obj_tuples)):
        obj_dict["object" + str(i)] = obj_tuples[i]
    
    with open(filename, "w") as f:
        json.dump(obj_tuples, f, indent=4)

    print("Objects have been written to " + filename)


# --------------------------- Main ---------------------------

# Get objects and sort them into assembly order (i.e., by height / z coord)
objs = get_objects()

objs.sort(key=lambda obj: obj["pos"][-1])

# Export the object information as a JSON file
export_JSON(objs, "tower.json")