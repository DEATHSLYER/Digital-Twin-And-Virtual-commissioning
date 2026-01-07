import bpy
import bmesh
import snap7
import time
from mathutils.bvhtree import BVHTree
from snap7.util import get_bool, get_real


PLC_IP = '192.168.0.1'
PLC_RACK = 0
PLC_SLOT = 1


FEEDBACK_DB = 1  
ASRS_CTRL_DB = 3
CARR_CTRL_DB = 4

INTERVAL = 0.1 
SPEED_MULTIPLIER = 0.1 

SHUTTLE_ASRS = 'ASRS_Shuttle'
SHUTTLE_CARR = 'Carrier_Shuttle'
BARCODE_PREFIX = 'BARCODE.0'
PALLET_PREFIX = 'Pallet'

last_barcode = 0
last_pallet = ''

def sensor_pallet():
    global last_pallet
    sensor_obj = bpy.data.objects.get('SENSOR_F')
    sensor_bvh = create_bvh(sensor_obj)
    for obj in bpy.data.objects:
        if obj.name.startswith(PALLET_PREFIX):
            target_bvh = create_bvh(obj)
            if sensor_bvh.overlap(target_bvh):
                last_pallet 
                return obj.name
            else:
                continue
                        
    

def execute_pickup():
    shuttle_obj = bpy.data.objects.get(SHUTTLE_ASRS)
    pallet_obj = bpy.data.objects.get(sensor_pallet())
    
    if pallet_obj:
        pallet_obj.parent = shuttle_obj
    
        pallet_obj.matrix_parent_inverse = shuttle_obj.matrix_world.inverted()
    
def get_barcode():
    global last_barcode
    sensor_obj = bpy.data.objects.get('SENSOR_BARCODE')
    if not sensor_obj: return last_barcode

    sensor_bvh = create_bvh(sensor_obj)
    for obj in bpy.data.objects:
        if obj.name.startswith(BARCODE_PREFIX):
            target_bvh = create_bvh(obj)
            if sensor_bvh.overlap(target_bvh):
                try:
                    last_barcode = int(obj.name.split(".")[1])
                except: pass
                return last_barcode
    return last_barcode


def execute_drop():
    pallet_obj = last_pallet
    if pallet_obj and pallet_obj.parent:
        world_matrix = pallet_obj.matrix_world.copy()
        
        pallet_obj.parent = None
        
        pallet_obj.matrix_world = world_matrix
        

def create_bvh(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.transform(obj.matrix_world)
    bvh = BVHTree.FromBMesh(bm)
    bm.free()
    return bvh

def check_collision(sensor_name, parent_name):
    sensor_obj = bpy.data.objects.get(sensor_name)
    parent_obj = bpy.data.objects.get(parent_name)
    if not sensor_obj or not parent_obj: return False

    sensor_bvh = create_bvh(sensor_obj)
    for obj in bpy.data.objects:
        if obj.type != 'MESH' or obj == sensor_obj or obj == parent_obj or obj.parent == parent_obj:
            continue
        target_bvh = create_bvh(obj)
        if sensor_bvh.overlap(target_bvh):
            return True
    return False


def main_system_loop():
    global last_barcode
    client = snap7.client.Client()
    
    try:
        client.connect(PLC_IP, PLC_RACK, PLC_SLOT)
        if not client.get_connected():
            return INTERVAL

        asrs_obj = bpy.data.objects.get(SHUTTLE_ASRS)
        
        if asrs_obj:
            data_asrs = client.db_read(ASRS_CTRL_DB, 0, 6)
            speed_asrs = get_real(data_asrs, 0)
            on_asrs = get_bool(data_asrs, 4, 0)
            rev_asrs = get_bool(data_asrs, 4, 1)
            pickup = get_bool(data_asrs, 4, 2)
            
            if pickup:
                execute_pickup()
            else:
                execute_drop()
            
            if on_asrs:
                move_val = -speed_asrs if rev_asrs else speed_asrs
                asrs_obj.location.x += (move_val * SPEED_MULTIPLIER)

        carr_obj = bpy.data.objects.get(SHUTTLE_CARR)
        if carr_obj:
            data_carr = client.db_read(CARR_CTRL_DB, 0, 6)
            speed_carr = get_real(data_carr, 0)
            on_carr = get_bool(data_carr, 4, 0)
            rev_carr = get_bool(data_carr, 4, 1)
            
            
            if on_carr:
                move_val = -speed_carr if rev_carr else speed_carr
                carr_obj.location.y += (move_val * SPEED_MULTIPLIER)

        
        asrs_sensors = ['SENSOR_F', 'SENSOR_B', 'SENSOR_FL', 'SENSOR_BR', 
                        'SENSOR_FR', 'SENSOR_BL', 'SENSOR_FRONT', 'SENSOR_END']
        asrs_bits = 0
        for i, s_name in enumerate(asrs_sensors):
            if check_collision(s_name, SHUTTLE_ASRS):
                asrs_bits |= (1 << i)

        carrier_sensors = ['SENSOR_L1', 'SENSOR_L2', 'SENSOR_SHUTTLE1', 'SENSOR_SHUTTLE2']
        carrier_bits = 0
        for i, s_name in enumerate(carrier_sensors):
            if check_collision(s_name, SHUTTLE_CARR):
                carrier_bits |= (1 << i)
        
    
        
        client.db_write(FEEDBACK_DB, 0, bytearray([asrs_bits]))
        client.db_write(FEEDBACK_DB, 1, bytearray([carrier_bits]))
        client.db_write(FEEDBACK_DB, 2, bytearray([get_barcode()]))
        
    except Exception as e:
        print(f"System Error: {e}")
    finally:
        if client.get_connected():
            client.disconnect()

    return INTERVAL


def start():
    stop()
    bpy.app.timers.register(main_system_loop)
    print("Dual Shuttle System Active: DB3->ASRS, DB4->Carrier, Feedback->DB1")

def stop():
    if bpy.app.timers.is_registered(main_system_loop):
        bpy.app.timers.unregister(main_system_loop)
        print("System Stopped")

    