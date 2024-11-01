import bpy
import os
import sys

sys.path.insert(1, '/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/tmp')
import layer_gen_data



def clear_scene():
    """Remove all objects from the scene"""
    # Select all objects
    bpy.ops.object.select_all(action='SELECT')
    # Delete selected objects
    bpy.ops.object.delete()


def import_stl(filepath, color):
    if not os.path.exists(filepath):
        print(f"Error: File {filepath} not found")
        return False
        
    try:
        
        # Import the STL file
        bpy.ops.wm.stl_import(filepath=filepath)
        print("STL imported successfully")
        
        # Select the imported object
        imported_object = bpy.context.selected_objects[0]

        # Set object color
        bpy.context.object.color = color        
        
        return True
        
    except Exception as e:
        print(f"Error importing STL: {str(e)}")
        return False

# ================= edge loops =====================

def import_edge_loops():
    points = layer_gen_data.get_points()
    lines = layer_gen_data.get_lines()
    edge_loop_collection = bpy.data.collections.new("Edge loops")
    bpy.context.collection.children.link(edge_loop_collection)


    def point_cloud(ob_name, coords, edges=[], faces=[]):
        """Create point cloud object based on given coordinates and name.

        Keyword arguments:
        ob_name -- new object name
        coords -- float triplets eg: [(-1.0, 1.0, 0.0), (-1.0, -1.0, 0.0)]
        """

        # Create new mesh and a new object
        me = bpy.data.meshes.new(ob_name + "Mesh")
        ob = bpy.data.objects.new(ob_name, me)

        # Make a mesh from a list of vertices/edges/faces
        me.from_pydata(coords, edges, faces)

        # Display name and update the mesh
        ob.show_name = True
        me.update()
        return ob

    # Create the object
    for i in range(0,len(points)):
        name = "loop" + str(i)
        pc = point_cloud(name, points[i], lines[i])

        # Link object to the active collection
        edge_loop_collection.objects.link(pc)



# Clear existing objects
clear_scene()

# Import your STL file
input = "/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/mesh/bunny.stl"
output = "/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/mesh/output2.stl"

import_stl(input, (1,1,1,0.4))
import_stl(output, (1,0.08,0.08,1))
import_edge_loops()


