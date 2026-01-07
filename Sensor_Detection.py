import bpy
import math
import time
from mathutils import Matrix
from mathutils.bvhtree import BVHTree
import bmesh

OBJECT_NAME = 'ASRS_Shuttle'
SENSOR_PREFIX = 'SENSOR_'
INTERVAL = 1


def create_bvh_tree_from_object(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.transform(obj.matrix_world)
    bvh = BVHTree.FromBMesh(bm)
    bm.free()
    return bvh


def check_bvh_intersection(obj_1, obj_2):
    bvh1 = create_bvh_tree_from_object(obj_1)
    bvh2 = create_bvh_tree_from_object(obj_2)
    return bvh1.overlap(bvh2)



def check_sensor_collision(sensor_name):
    sensor_obj = bpy.data.objects.get(sensor_name)
    shuttle_obj = bpy.data.objects.get(OBJECT_NAME)
   
    if not sensor_obj: return None
    
    
    for obj in bpy.data.objects:
        
        if obj.type != 'MESH' or obj.name == OBJECT_NAME or obj.parent == shuttle_obj:
            continue
        
        if obj == sensor_obj:
            continue
        
    
        
        overlap_result = check_bvh_intersection(sensor_obj,obj)
        
        if overlap_result:
            return True
    return False     
                
#        matrix_world_inverted = obj.matrix_world.inverted()
#        sensor_local = matrix_world_inverted @ sensor_obj.location
#        
#        pallet_dim = obj.dimensions
#        half_x = pallet_dim.x * 2
#        half_y = pallet_dim.y * 2
#        half_z = pallet_dim.z * 2 
#        
#        is_inside = (
#            abs(sensor_local.x) <= half_x and
#            abs(sensor_local.y) <= half_y and
#            abs(sensor_local.z) <= half_z
#        )
#        
#        if is_inside:
#            return True
#    return False

def is_pallet_aligned():
    fr_name = check_sensor_collision('SENSOR_FR')
    fl_name = check_sensor_collision('SENSOR_FL')
    br_name = check_sensor_collision('SENSOR_BR')
    bl_name = check_sensor_collision('SENSOR_BL')
    
    if fr_name and (fr_name == fl_name) and (fr_name == br_name) and (fr_name == bl_name):
        return fr_name
    return None

def get_sensors():
    shuttle_obj = bpy.data.objects.get(OBJECT_NAME)
    if not shuttle_obj:
        print(f"SHUTTLE NOT FOUND: {OBJECT_NAME}")
        return []
    
    sensors = []
    for obj in bpy.data.objects:
        if obj.parent == shuttle_obj and obj.name.startswith(SENSOR_PREFIX):
            sensors.append(obj.name)
    
    return sensors
    
def sensor_monitor_loop():
    current_sensors = get_sensors()
    
    if not current_sensors:
        print("ERROR: NO SENSORS FOUND")
        stop_monitor()
        return None
    
    readings = {}
    
    for name in current_sensors:
        readings[name] = check_sensor_collision(name)
        
    print(f"\n[{time.strftime('%H:%M:%S')}] SHUTTLE Sensor Readings ({len(current_sensors)} sensors found):")
    
    for name, status in readings.items():
        status_text = f"TRUE (Pallet: {status})" if status else "FALSE ()"
        
        if 'FR' in name.upper() or 'FL' in name.upper() or 'BR' in name.upper() or 'BL' in name.upper():
            category = "ALIGNMENT"
        elif 'TRAVEL' in name.upper() or 'FRONT' in name.upper() or 'BACK' in name.upper():
            category = "TRAVEL"
        else:
            category = "GENERAL"
        
        print(f"  [{category:<9}] {name:<20} : {status_text}")
        
        aligned_pallet = is_pallet_aligned()
        
    
    if check_sensor_collision('SENSOR_FRONT'):
        print(f"    END OF RAIL")
    
    if aligned_pallet :
        print(f"  **PALLET ALIGNED** ")
    else:
        print(f"   PALLET NOT ALIGNED")    
    
    
    return INTERVAL

def start_monitor():
    
    try:
        bpy.app.timers.unregister(sensor_monitor_loop)
    except ValueError:
        pass
    
    bpy.app.timers.register(sensor_monitor_loop, first_interval=INTERVAL)
    print("\n\n** SENSOR MONITOR STARTED **")
    
def stop_monitor():
    try:
        bpy.app.timers.unregister(sensor_monitor_loop)
        print("MONITOR STOPPED")    
    except ValueError:
        print("MONITOR NOT RUNNING")
        
if __name__ == "__main__":
    bpy.context.scene.unit_settings.system = 'METRIC'
    