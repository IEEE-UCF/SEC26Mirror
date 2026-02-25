"""
Parse robot.urdf: compute center AND corner wheel Y-positions from base_link.
"""
import xml.etree.ElementTree as ET
import numpy as np, math

URDF = "/home/ubuntu/ros2_workspaces/src/sec26ros/my_robot_description/urdf/robot.urdf"

tree = ET.parse(URDF)
root = tree.getroot()

# Build child-link → joint-name lookup
child_to_joint = {}
joint_data = {}
for j in root.findall("joint"):
    name   = j.get("name")
    parent = j.find("parent").get("link")
    child  = j.find("child").get("link")
    org    = j.find("origin")
    xyz = [float(x) for x in org.get("xyz","0 0 0").split()] if org is not None else [0,0,0]
    rpy = [float(x) for x in org.get("rpy","0 0 0").split()] if org is not None else [0,0,0]
    joint_data[name] = {"parent": parent, "child": child, "xyz": xyz, "rpy": rpy}
    child_to_joint[child] = name

def rpy_to_R(rpy):
    r, p, y = rpy
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

def T(xyz, rpy):
    m = np.eye(4)
    m[:3,:3] = rpy_to_R(rpy)
    m[:3, 3] = xyz
    return m

def world_pos(joint_name, base_link="base_link"):
    j = joint_data[joint_name]
    chain = []
    cur = j["child"]
    # Walk up: child → parent until base_link
    while True:
        jn = child_to_joint.get(cur)
        if jn is None:
            break
        jd = joint_data[jn]
        chain.append(jd)
        cur = jd["parent"]
        if cur == base_link:
            break
    chain.reverse()
    M = np.eye(4)
    for jd in chain:
        M = M @ T(jd["xyz"], jd["rpy"])
    M = M @ T(j["xyz"], j["rpy"])
    return M[:3, 3]

joints_of_interest = [
    ("leftcenter",     "center left  (plugin L)"),
    ("rightcenter",    "center right (plugin R)"),
    ("frontrightwheel","front-right wheel (plugin L-side)"),
    ("backrightwheel", "back-right  wheel (plugin L-side)"),
    ("frontleftwheel", "front-left  wheel (plugin R-side)"),
    ("backleftwheel",  "back-left   wheel (plugin R-side)"),
]

print(f"{'Joint':<22} {'X':>9} {'Y':>9} {'Z':>9}   {'Y from center':>14}")
print("-" * 70)
for jname, label in joints_of_interest:
    pos = world_pos(jname)
    print(f"{jname:<22} {pos[0]:>9.5f} {pos[1]:>9.5f} {pos[2]:>9.5f}   {abs(pos[1]):>14.5f}   {label}")

# Compute separations
c_left  = world_pos("leftcenter")
c_right = world_pos("rightcenter")
ctr_sep = abs(c_right[1] - c_left[1])

fl = world_pos("frontleftwheel")
bl = world_pos("backleftwheel")
fr = world_pos("frontrightwheel")
br = world_pos("backrightwheel")
# In plugin: frontrightwheel/backrightwheel = LEFT side, frontleftwheel/backleftwheel = RIGHT
# Corner track = front-left to front-right lateral separation
corner_sep_front = abs(fl[1] - fr[1])
corner_sep_back  = abs(bl[1] - br[1])

print(f"\nCenter wheel separation:     {ctr_sep:.6f} m  =  {ctr_sep/0.0254:.3f} in")
print(f"Corner wheel sep (front):    {corner_sep_front:.6f} m  =  {corner_sep_front/0.0254:.3f} in")
print(f"Corner wheel sep (back):     {corner_sep_back:.6f} m  =  {corner_sep_back/0.0254:.3f} in")
print(f"\n→ Use wheel_separation = {ctr_sep:.4f} for center plugin")
print(f"→ Use wheel_separation = {(corner_sep_front+corner_sep_back)/2:.4f} for corner plugin")
