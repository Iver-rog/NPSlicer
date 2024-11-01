import bpy
import os
import sys

# temporary directory for storing rust programs output data
tmp = '/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/tmp'

sys.path.insert(1, tmp)
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


def import_stl_files( collection_name='STL_Imports', color=(1.0,0.2,0.2,1.0)):
    directory = layer_gen_data.get_mesh_dir()
    """
    Import all STL files from a specified directory into a new or existing collection.
    
    :param directory: Path to the directory containing STL files
    :param collection_name: Name of the collection to import STL files into
    """
    # Create the collection if it doesn't exist
    if collection_name not in bpy.data.collections:
        new_collection = bpy.data.collections.new(collection_name)
        bpy.context.scene.collection.children.link(new_collection)
    else:
        new_collection = bpy.data.collections[collection_name]
    
    # Get a list of all STL files in the directory
    stl_files = [f for f in os.listdir(directory) if f.lower().endswith('.stl')]
    
    # Import each STL file
    for stl_file in stl_files:
        full_path = os.path.join(directory, stl_file)
        
        # Import the STL file
        bpy.ops.wm.stl_import(filepath=full_path)
        
        # Get the imported object (the last imported object)
        imported_object = bpy.context.selected_objects[0]

        bpy.context.object.color = color
        
        # Move the object to the specified collection
        # First, unlink from the current collection
        for collection in bpy.data.collections:
            if imported_object.name in collection.objects:
                collection.objects.unlink(imported_object)
        
        # Link to the new collection
        new_collection.objects.link(imported_object)
        
        # Optional: Center the object to the world origin
        bpy.ops.object.select_all(action='DESELECT')
        imported_object.select_set(True)
        bpy.context.view_layer.objects.active = imported_object
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
    

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
import_stl(input, (1,1,1,0.4))

# import_stls((1,1,1,0.4))

import_stl_files()

import_edge_loops()

# import_stls((1,1,1,0.4))



