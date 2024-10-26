import bpy
import os

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

# Clear existing objects
clear_scene()

# Import your STL file
input = "/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/mesh/bunny.stl"
output = "/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/mesh/output2.stl"

import_stl(input, (1,1,1,0.4))
import_stl(output, (1,0.08,0.08,1))
