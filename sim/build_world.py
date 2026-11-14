"""Self-contained Webots world, physical four-wheel Ackermann car, no external PROTOs."""
import argparse,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def path_point(s,c,mirror=False):
    r=c['track']['curve_radius'];a=(min(s,40)-40)/r
    x,y,h=(r*math.sin(a),r*(1-math.cos(a)),a) if s<40 else (s-40,0,0)
    return x,-y if mirror else y,-h if mirror else h

def box(x,y,z,sx,sy,sz,color,angle=0,solid=False,name=''):
    shape=f'Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }} geometry Box {{ size {sx} {sy} {sz} }} }}'
    if solid:return f'{"DEF "+name+" " if name else ""}Solid {{ translation {x} {y} {z} rotation 0 0 1 {angle} children [ {shape} ] name "{name or "obstacle"}" boundingObject Box {{ size {sx} {sy} {sz} }} }}\n'
    return f'Pose {{ translation {x} {y} {z} rotation 0 0 1 {angle} children [ {shape} ] }}\n'

def wheel(name,x,y,steering):
    # Wheel central axis y, positive angular speed drives +x on a z-up floor.
    tyre='Pose { rotation 1 0 0 1.57079632679 children [ Shape { appearance PBRAppearance { baseColor .05 .05 .05 roughness 1 metalness 0 } geometry Cylinder { radius .045 height .035 subdivision 24 } } ] }'
    bounding='Pose { rotation 1 0 0 1.57079632679 children [ Cylinder { radius .045 height .035 subdivision 24 } ] }'
    joint=f'''HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 0 0 }}
 device [ RotationalMotor {{ name "drive_{name}" maxVelocity 60 maxTorque .5 sound "" }} ]
 endPoint DEF WHEEL_{name} Solid {{ translation 0 0 0 children [ {tyre} ] name "wheel_{name}" contactMaterial "tyre" boundingObject {bounding} physics Physics {{ density -1 mass .06 }} }} }}'''
    if steering:return f'''HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 0 1 anchor {x} {y} 0 }} device [ RotationalMotor {{ name "steer_{name}" maxVelocity 4 maxTorque 2 minPosition -.6 maxPosition .6 sound "" }} ]
 endPoint Solid {{ translation {x} {y} 0 children [ {joint} ] name "knuckle_{name}" boundingObject Sphere {{ radius .008 }} physics Physics {{ density -1 mass .02 }} }} }}'''
    return f'Pose {{ translation {x} {y} 0 children [ {joint} ] }}'

def build(out,c,case):
    t=c['track'];q=c['tasks'];v=c['vehicle'];mirror=case.get('mirror',False);parts=[]
    parts.append('#VRML_SIM R2025a utf8\nWorldInfo { basicTimeStep 10 coordinateSystem "ENU" randomSeed 42 contactProperties [ ContactProperties { material1 "tyre" material2 "road" coulombFriction [ 1.0 ] bounce 0 softERP .2 softCFM .00001 bumpSound "" rollSound "" slideSound "" } ] }\n')
    spawn_view=case.get('start_s',46.3)-40
    yaw=math.atan2(5,4);pitch=math.atan2(4,math.hypot(4,5))
    quat=(-math.sin(yaw/2)*math.sin(pitch/2),math.cos(yaw/2)*math.sin(pitch/2),math.sin(yaw/2)*math.cos(pitch/2),math.cos(yaw/2)*math.cos(pitch/2))
    norm=math.sqrt(sum(a*a for a in quat[:3]));axis=[a/norm for a in quat[:3]];angle=2*math.acos(quat[3])
    parts.append(f'Viewpoint {{ orientation {axis[0]} {axis[1]} {axis[2]} {angle} position {spawn_view-4} -5 4 follow "yidui-autonomous" followType "Tracking Shot" followSmoothness 0 }}\nBackground {{ skyColor .45 .65 .85 }}\n')
    parts.append(f'DirectionalLight {{ direction .3 .2 -1 intensity {case.get("light",1)} ambientIntensity .6 castShadows FALSE }}\n')
    parts.append('Solid { translation 0 0 -.05 children [ Shape { appearance PBRAppearance { baseColor .15 .28 .12 roughness 1 metalness 0 } geometry Box { size 160 160 .1 } } ] contactMaterial "road" boundingObject Box { size 160 160 .1 } }\n')
    # Visual asphalt ribbon and paint. Paint has no collision extrusion.
    for i in range(200):
        s=(i+.5)*.5;x,y,h=path_point(s,c,mirror)
        parts.append(box(x,y,.001,.505,t['width']+.5,.002,'.37 .12 .09',h))
        for side in (-1,1):
            off=side*t['width']/2
            parts.append(box(x-math.sin(h)*off,y+math.cos(h)*off,.003,.505,t['line_width'],.002,'.96 .96 .96',h))
    def marking(s,width=None,offset=0,length=.1):
        x,y,h=path_point(s,c,mirror);return box(x-math.sin(h)*offset,y+math.cos(h)*offset,.005,length,width or t['width'],.002,'1 .8 .02',h)
    for s in range(0,101,10):
        for side in (-1,1):parts.append(marking(s,.15,side*t['width']/2))
    for s in (0,1.5,q['switch_start_s'],q['autonomous_start_s'],q['parking_start_s'],q['parking_start_s']+q['parking_length']):parts.append(marking(s))
    sx,sy,sh=path_point(q['parking_start_s']+q['parking_length']/2,c,mirror)
    parts.append(box(sx,sy,.005,q['parking_length'],.10,.002,'1 .8 .02',sh))
    x,y,h=path_point(q['zebra_s']+.297/2,c,mirror)
    for k in range(-2,3):parts.append(box(x-math.sin(h)*k*.21,y+math.cos(h)*k*.21,.006,.297,.105,.002,'.97 .97 .97',h))
    for i,s in enumerate(q['cone_s']):
        x,y,h=path_point(s,c,mirror);off=(1 if i==0 else -1)*(t['width']/2-q['cone_edge_offset'])*(-1 if mirror else 1)
        x-=math.sin(h)*off;y+=math.cos(h)*off
        parts.append(f'DEF CONE_{i} Solid {{ translation {x} {y} .04 children [ Shape {{ appearance PBRAppearance {{ baseColor .02 .15 .95 roughness 1 metalness 0 }} geometry Cone {{ bottomRadius .039 height .08 }} }} ] boundingObject Cylinder {{ radius .039 height .08 }} name "cone_{i}" }}\n')
    # Start board remains at the switch (behind the parking-stage spawn).
    x,y,h=path_point(q['start_board_s'],c,mirror);parts.append(box(x,y,.25,.02,.40,.50,'.02 .15 .95',h,True,'START_BOARD'))
    for side in ([1,-1] if case.get('both_boards') else ([] if case.get('no_board') else [1 if case.get('blocked_side','left')=='left' else -1])):
        x,y,h=path_point(q['parking_start_s']+q['parking_length'],c,mirror);off=side*t['width']/4
        parts.append(box(x-math.sin(h)*off,y+math.cos(h)*off,.25,.02,.40,.50,'.02 .15 .95',h,True,'PARK_BOARD_'+str(side).replace('-','M')))
    # Above-track red/green light and sensor lines at the specified spacing.
    # Horizontal two-lamp fixture, bottom 0.40m, above vehicle clearance.
    # A small white LED core models the saturated center expected by the original detector.
    x,y,h=path_point(q['light_s'],c,mirror);parts.append(box(x,y,.455,.06,.36,.11,'.04 .04 .04',h,True,'LIGHT_HOUSING'))
    for offset,name in ((.09,'RED'),(-.09,'GREEN')):
        parts.append(f'Pose {{ translation {x-.035} {y+offset} .455 children [ Shape {{ appearance DEF LIGHT_{name} PBRAppearance {{ baseColor 0 0 0 emissiveColor 0 0 0 emissiveIntensity 3 roughness 1 metalness 0 }} geometry Box {{ size .005 .12 .09 }} }} ] }}\n')
        parts.append(f'Pose {{ translation {x-.039} {y+offset} .455 children [ Shape {{ appearance DEF CORE_{name} PBRAppearance {{ baseColor 0 0 0 emissiveColor 0 0 0 emissiveIntensity 3 roughness 1 metalness 0 }} geometry Box {{ size .003 .018 .018 }} }} ] }}\n')
    # Non-payment 10x10cm QR placeholder: deliberately not used as a payment test.
    x,y,h=path_point(q['qr_s'],c,mirror);parts.append(box(x,y+t['width']/2+.12,.45,.01,.10,.10,'.95 .95 .95',h))
    spawn=case.get('start_s',91.0);x,y,h=path_point(spawn,c,mirror);offset=case.get('offset',0);x-=math.sin(h)*offset;y+=math.cos(h)*offset;h+=math.radians(case.get('yaw_deg',0))
    robot=f'''DEF CAR Robot {{ translation {x} {y} .047 rotation 0 0 1 {h} supervisor TRUE controller "full_supervisor" window "<none>" name "yidui-autonomous" children [
 Pose {{ translation 0 0 .02 children [ Shape {{ appearance PBRAppearance {{ baseColor .2 .22 .26 roughness 1 metalness 0 }} geometry Box {{ size .4 .18 .02 }} }} ] }}
 Camera {{ translation .17 0 {v["camera_height"]-.047} rotation 0 1 0 {math.radians(v["camera_pitch_deg"])} name "camera" width 640 height 480 fieldOfView {math.radians(v["camera_hfov_deg"])} near .02 far 12 noise {case.get('noise',0)} }}
 {wheel('FL',.13,.11,True)} {wheel('FR',.13,-.11,True)} {wheel('RL',-.13,.11,False)} {wheel('RR',-.13,-.11,False)}
 ] boundingObject Pose {{ translation 0 0 .02 children [ Box {{ size .4 .18 .02 }} ] }} physics Physics {{ density -1 mass 2.22 centerOfMass [ 0 0 .015 ] }} }}\n'''
    parts.append(robot)
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(''.join(parts),encoding='utf-8')
    manifest={'rules':c,'case':case,'scope':'complete autonomous segment: start board, zebra, light, two cones, parking','world':str(out)}
    out.with_suffix('.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf-8')
    return manifest

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',default=str(ROOT/'worlds/parking.wbt'));p.add_argument('--blocked-side',choices=['left','right'],default='left');a=p.parse_args()
    build(Path(a.output),json.loads((ROOT/'rules_2026_online.json').read_text()),{'blocked_side':a.blocked_side})
