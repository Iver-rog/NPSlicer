import bpy
import os
import sys

import bmesh
import mathutils

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

def import_edge_loops(collection_name="Edge loops"):
    points = layer_gen_data.get_points()
    lines = layer_gen_data.get_lines()
    faces = layer_gen_data.get_faces()
    edge_loop_collection = bpy.data.collections.new(collection_name)
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
        pc = point_cloud(name, points[i], lines[i], faces[i])

        # Link object to the active collection
        edge_loop_collection.objects.link(pc)



def apply_boolean_to_collection(collection_name,boolean_obj):
    # Name of the collection you want to process
    # collection_name = "results"  # <-- change this to your collection's name


    # Get the collection
    collection = bpy.data.collections.get(collection_name)

    if collection:
        # Loop through all objects in the collection
        for obj in collection.objects:
            if obj.type == 'MESH':
                # Set the object as active
                bpy.context.view_layer.objects.active = obj
                obj.select_set(True)

                # Switch to Edit mode
                bpy.ops.object.mode_set(mode='EDIT')

                # Create a bmesh from the object's data
                bm = bmesh.from_edit_mesh(obj.data)

                # Flip normals
                bmesh.ops.reverse_faces(bm, faces=bm.faces)

                # Update the mesh
                bmesh.update_edit_mesh(obj.data, loop_triangles=True)

                # Switch back to Object mode
                bpy.ops.object.mode_set(mode='OBJECT')

#=========================== Boolean ==================================
                # Apply a Boolean modifier to the new cube
                boolean_modifier = obj.modifiers.new(name="Boolean", type='BOOLEAN')
                
                # Set the target object for the Boolean modifier
                boolean_modifier.object = boolean_obj
                
                # Set the operation type (choose from 'INTERSECT', 'UNION', 'DIFFERENCE')
                boolean_modifier.operation = 'DIFFERENCE'  # Change this to 'DIFFERENCE' or 'INTERSECT' if needed
                
                # Apply the modifier (optional: this will apply the boolean operation immediately)
                # bpy.ops.object.modifier_apply(modifier=boolean_modifier.name)
#=========================== Boolean ==================================

                # Deselect the object (optional)
                obj.select_set(False)

                print(f"Flipped normals for '{obj.name}'.")
    else:
        print(f"Collection '{collection_name}' not found.")

def create_boolean_obj():
    scale = 2

    #obj = bpy.context.active_object  # or bpy.data.objects["YourObjectName"]
    obj = bpy.data.objects["input mesh"]

    if obj:
        # Get bounding box corners in local space
        local_bbox = [mathutils.Vector(corner) for corner in obj.bound_box]

        # Convert to world space
        world_bbox = [obj.matrix_world @ corner for corner in local_bbox]

        # Find min and max coordinates
        min_corner = mathutils.Vector((
            min(corner.x for corner in world_bbox),
            min(corner.y for corner in world_bbox),
            min(corner.z for corner in world_bbox),
        ))

        max_corner = mathutils.Vector((
            max(corner.x for corner in world_bbox),
            max(corner.y for corner in world_bbox),
            max(corner.z for corner in world_bbox),
        ))

        # Calculate the width, height, and depth (dimensions)
        dimensions = max_corner - min_corner
        print(dimensions)
        dim_max = max(dimensions.x,dimensions.y,dimensions.z)
        print(dim_max)

        # Get the center of the bounding box
        center = (min_corner + max_corner) / 2
        center.z -= ((scale/2 * dim_max)/2 )

        # Create the new cube at the same origin
        bpy.ops.mesh.primitive_cube_add(size=1, location=center)
        new_cube = bpy.context.active_object

        # Scale the cube to match the bounding box dimensions
        dimensions.x += (scale * dim_max)
        dimensions.y += (scale * dim_max)
        dimensions.z += (scale/2 * dim_max)
        print(dimensions)
        new_cube.scale = dimensions  # Adjusting for default cube size (size=1)

    # Apply a Boolean modifier to the new cube
        boolean_modifier = new_cube.modifiers.new(name="Boolean", type='BOOLEAN')
        
        # Set the target object for the Boolean modifier
        boolean_modifier.object = obj
        
        # Set the operation type (choose from 'INTERSECT', 'UNION', 'DIFFERENCE')
        boolean_modifier.operation = 'DIFFERENCE'  # Change this to 'DIFFERENCE' or 'INTERSECT' if needed
        
        # Apply the modifier (optional: this will apply the boolean operation immediately)
        bpy.ops.object.modifier_apply(modifier=boolean_modifier.name)
        
    # remove redundant mesh edges
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.dissolve_limited()
        bpy.ops.object.mode_set(mode='OBJECT')

        return new_cube

# Clear existing objects
clear_scene()

import_stl_files()

boolean_obj = create_boolean_obj()

import_edge_loops("Edge_loops")

apply_boolean_to_collection("Edge_loops",boolean_obj)
