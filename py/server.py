import socket
import json
import queue
import threading
import bpy
import os

message_queue = queue.Queue()

def handle_client(conn):
    try:
        while True:
            length = int.from_bytes(conn.recv(4), 'big')
            data = b''
            while len(data) < length:
                packet = conn.recv(length - len(data))
                if not packet:
                    break
                data += packet
            msg = json.loads(data.decode('utf-8'))
            message_queue.put(msg)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        conn.close()

def server_loop():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('127.0.0.1', 9000))
    s.listen(1)
    while True:
        conn, _ = s.accept()
        threading.Thread(target=handle_client, args=(conn,), daemon=True).start()

def process_messages():
    while not message_queue.empty():
        msg = message_queue.get()
        for command in msg.keys():
            match command:
                case "CreateMesh":
                    obj = msg["CreateMesh"]
                    b_obj = import_edge_loops(
                        obj["collection"],
                        obj["name"],
                        obj["vertices"],
                        obj["edges"],
                        obj["faces"]
                    )
                    if obj["collection"] == "result":
                        apply_boolean(b_obj)

                case "LoadSTL":
                    stl_path,obj_name = msg["LoadSTL"]
                    import_stl(stl_path,obj_name)
                case _:
                    print("Did not recognice the command")

    return 0.1  # Run this function again in 0.1s

def import_stl(filepath, name, color=(1.0,0.2,0.2,1.0)):
    if not os.path.exists(filepath):
        print(f"Error: File {filepath} not found")
        return False
        
    try:
        # Import the STL file
        bpy.ops.wm.stl_import(filepath=filepath)
        print("STL imported successfully")
        
        # Select the imported object
        imported_object = bpy.context.selected_objects[0]
        imported_object.name = name
        imported_object.color = color
        
        return True
        
    except Exception as e:
        print(f"Error importing STL: {str(e)}")
        return False

def clear_scene():
    """Remove all objects from the scene"""
    # Select all objects
    bpy.ops.object.select_all(action='SELECT')
    # Delete selected objects
    bpy.ops.object.delete()

def import_edge_loops(collection_name,name,points,lines,faces):
    # edge_loop_collection = bpy.data.collections.new(collection_name)

    if collection_name not in bpy.data.collections:
        new_collection = bpy.data.collections.new(collection_name)
        bpy.context.collection.children.link(new_collection)
    else:
        new_collection = bpy.data.collections[collection_name]

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
    obj = point_cloud(name, points, lines, faces)

    # Link object to the active collection
    new_collection.objects.link(obj)
    return obj

def apply_boolean(target_obj):
    boolean_obj = bpy.data.objects["input mesh"]
    
    boolean_modifier = target_obj.modifiers.new(name="Boolean", type='BOOLEAN')
    boolean_modifier.operation = 'INTERSECT'
    
    # Set the target object for the Boolean modifier
    boolean_modifier.object = boolean_obj
    
    # Set the operation type (choose from 'INTERSECT', 'UNION', 'DIFFERENCE')
    # boolean_modifier.operation = 'DIFFERENCE'  # Change this to 'DIFFERENCE' or 'INTERSECT' if needed
    
    # Apply the modifier (optional: this will apply the boolean operation immediately)
    # bpy.ops.object.modifier_apply(modifier=boolean_modifier.name)
#=========================== Boolean ==================================

    # Deselect the object (optional)
    boolean_obj.select_set(False)

    print(f"Flipped normals for '{boolean_obj.name}'.")

# remove default cube n stuff
clear_scene()

# Start server in thread
threading.Thread(target=server_loop, daemon=True).start()

# Run the Blender-safe message processor
bpy.app.timers.register(process_messages)

