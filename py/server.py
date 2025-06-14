import socket
import json
import queue
import threading
import bpy
import os
import bmesh

message_queue = queue.Queue()
client_conn = None

def handle_client(conn):
    global client_conn
    client_conn = conn
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
        client_conn = None

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
                    try:
                        b_obj = import_edge_loops(
                            obj["collection"],
                            obj["name"],
                            obj["vertices"],
                            obj["edges"],
                            obj["faces"]
                        )
                    except:
                        print(obj)
                    if obj["collection"] == "result":
                        apply_boolean_slow(b_obj)

                case "LoadSTL":
                    stl_path,obj_name = msg["LoadSTL"]
                    import_stl(stl_path,obj_name)
                case "ExportLayers":
                    export_dir = msg["ExportLayers"]
                    if export_dir[-1] != "/" or export_dir[-1] != "\\":
                        export_dir += "/"
                    batch_export("result",export_dir)
                    
                case "Ping":
                    send_to_rust({"Ping":"Halla ping from blender"})
                    print("Halla ping from blender")
                case _:
                    print("Did not recognice the command")

    return 0.1  # Run this function again in 0.1s

def send_to_rust(response: dict):
    global client_conn
    if client_conn:
        try:
            msg_bytes = json.dumps(response).encode('utf-8')
            length_bytes = len(msg_bytes).to_bytes(4, 'big')
            client_conn.sendall(length_bytes + msg_bytes)
        except Exception as e:
            print(f"Failed to send response: {e}")


def import_stl(filepath, name, color=(1.0,0.2,0.2,1.0)):
    if not os.path.exists(filepath):
        print(f"Error: File {filepath} not found")
        return False
        
    try:
        bpy.ops.wm.stl_import(filepath=filepath)
        print("STL imported successfully")
        
        imported_object = bpy.context.selected_objects[0]
        imported_object.name = name
        imported_object.color = color
        
        return True
        
    except Exception as e:
        print(f"Error importing STL: {str(e)}")
        return False

def clear_scene():
    """Remove all objects from the scene"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def import_edge_loops(collection_name,name,points,edges,faces):
    # edge_loop_collection = bpy.data.collections.new(collection_name)

    if collection_name not in bpy.data.collections:
        new_collection = bpy.data.collections.new(collection_name)
        bpy.context.collection.children.link(new_collection)
    else:
        new_collection = bpy.data.collections[collection_name]

    me = bpy.data.meshes.new(name + "Mesh")
    ob = bpy.data.objects.new(name, me)

    me.from_pydata(points, edges, faces)

    ob.show_name = True
    me.update()

    new_collection.objects.link(ob)
    return ob

def apply_boolean(target_obj):
    boolean_obj = bpy.data.objects["input mesh"]
    
    boolean_modifier = target_obj.modifiers.new(name="Boolean", type='BOOLEAN')
    boolean_modifier.operation = 'INTERSECT'
    
    boolean_modifier.object = boolean_obj
    
    boolean_obj.select_set(False)

def apply_boolean_slow(target_obj):

    boolean_obj = bpy.data.objects["input mesh"]
    
    boolean_modifier = target_obj.modifiers.new(name="Boolean", type='BOOLEAN')
    boolean_modifier.operation = 'INTERSECT'
    
    boolean_modifier.object = boolean_obj
    
    boolean_obj.select_set(False)

    face_n_after_mod = face_count_after_modifier(target_obj)
    face_n_b_mod = face_count_without_modifier(target_obj)
    if face_n_after_mod > 1.1*face_n_b_mod or face_n_after_mod < 0.9*face_n_b_mod:
        flip_normals(target_obj)
        # print(f"Flipped normals for '{target_obj.name}'.")

def face_count_after_modifier(obj):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = obj
    
    dg = bpy.context.evaluated_depsgraph_get()
    eval_obj = bpy.context.object.evaluated_get(dg)

    eval_mesh = eval_obj.to_mesh()

    return len(eval_mesh.polygons)

def face_count_without_modifier(obj):
    return len(obj.data.polygons)

def flip_normals(obj):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    bpy.ops.object.mode_set(mode='EDIT')

    bm = bmesh.from_edit_mesh(obj.data)

    bmesh.ops.reverse_faces(bm, faces=bm.faces)

    bmesh.update_edit_mesh(obj.data, loop_triangles=True)

    bpy.ops.object.mode_set(mode='OBJECT')

def apply_all_modifiers(collection_name):
    collection = bpy.data.collections.get(collection_name)
    if not collection:
        print(f"Collection '{collection_name}' not found.")
        return

    bpy.ops.object.select_all(action='DESELECT')

    mesh_objs = [obj for obj in collection.objects if obj.type == 'MESH']
    if not mesh_objs:
        print("No mesh objects to convert.")
        return

    for obj in mesh_objs:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_objs[0]

    bpy.ops.object.convert(target='MESH')

def merge_by_distance(collection_name, threshold=0.0001):

    collection = bpy.data.collections.get(collection_name)
    if not collection:
        print(f"Collection '{collection_name}' not found.")
        return

    mesh_objs = [obj for obj in collection.objects if obj.type == 'MESH']
    if not mesh_objs:
        print("No mesh objects to merge.")
        return

    bpy.ops.object.select_all(action='DESELECT')
    for obj in mesh_objs:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_objs[0]

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=threshold)
    bpy.ops.object.mode_set(mode='OBJECT')

    for obj in mesh_objs:
        obj.select_set(False)

    print(f"Merged vertices by distance ({threshold}) for {len(mesh_objs)} objects.")



def batch_export(collection_name, export_directory):

    apply_all_modifiers(collection_name)
    merge_by_distance(collection_name)

    collection = bpy.data.collections.get(collection_name)

    if collection == None:
        return
    if len(collection.objects) == 0: 
        return

    if collection:
        print(f"Exporting all objects from collection: {collection_name}")

        bpy.ops.object.select_all(action='DESELECT')

        for obj in collection.objects:
            if obj.type == 'MESH':
                obj.select_set(True)

        bpy.context.view_layer.objects.active = collection.objects[0]

        bpy.ops.wm.stl_export(
            filepath=export_directory,
            use_batch=True,
            export_selected_objects=True,
            apply_modifiers=True,
            ascii_format=False
        )

        print("Batch export completed successfully.")

    else:
        print(f"Collection '{collection_name}' not found.")

# remove default cube n stuff
clear_scene()

# Start server in thread
threading.Thread(target=server_loop, daemon=True).start()

# Run the Blender-safe message processor
bpy.app.timers.register(process_messages)

