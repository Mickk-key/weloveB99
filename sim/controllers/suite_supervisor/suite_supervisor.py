"""Webots actuator bridge and independent geometry/logging. Never sends truth to C++."""
import json,math,os,socket,struct,traceback
from pathlib import Path
import cv2,numpy as np
from controller import Supervisor

def main():
    out=Path(os.environ['SUITE_RUN']);manifest=json.loads((out/'manifest.json').read_text());rules=manifest['rules'];case=manifest['case'];modules=set(case['modules'])
    sim=Supervisor();car=sim.getSelf();camera=sim.getDevice('camera');camera.enable(50)
    motors={n:sim.getDevice('drive_'+n) for n in ('FL','FR','RL','RR')};steers={n:sim.getDevice('steer_'+n) for n in ('FL','FR')}
    for m in motors.values():m.setPosition(float('inf'));m.setVelocity(0)
    sock=socket.create_connection(('127.0.0.1',int(os.environ['SUITE_PORT'])),timeout=30);stream=sock.makefile('rb')
    writer=cv2.VideoWriter(str(out/'demo.avi'),cv2.VideoWriter_fourcc(*'MJPG'),20,(640,480));trace=(out/'trace.jsonl').open('w')
    tick=0;light_start=None;last='';stopped_since=None;collisions=set();outside=False;events=set();zebra_hold=0;light_hold=0;command={};completed=False;reason='time_limit';red_crossing=False;max_center_error=0
    def track_coordinates(p):
        x,y=p[:2];y=-y if case.get('mirror') else y
        if x>=0:return 40+x,y
        radius=rules['track']['curve_radius']
        return 40+radius*math.atan2(x,radius-y),radius-math.hypot(x,radius-y)
    cones=[(s-40,side*(rules['track']['width']/2-rules['tasks']['cone_edge_offset'])*(-1 if case.get('mirror') else 1)) for s,side in zip(rules['tasks']['cone_s'],(1,-1))]
    try:
        while sim.step(10)!=-1:
            tick+=1;t=sim.getTime();position=car.getPosition()
            progress,_=track_coordinates(position)
            if light_start is None and progress+.2>=rules['tasks']['light_trigger_s']:light_start=t
            red=light_start is not None and t-light_start<case['light_seconds'];green=light_start is not None and not red
            for color,on,rgb in [('RED',red,[1,0,0]),('GREEN',green,[0,1,0])]:
                for prefix,c in [('LIGHT_',rgb),('CORE_',[1,1,1])]:
                    node=sim.getFromDef(prefix+color);node.getField('baseColor').setSFColor(c if on else [0,0,0]);node.getField('emissiveColor').setSFColor(c if on else [0,0,0])
            if tick%5:continue
            raw=np.frombuffer(camera.getImage(),np.uint8).reshape(480,640,4)[:,:,:3].copy();ok,png=cv2.imencode('.png',raw,[cv2.IMWRITE_PNG_COMPRESSION,1]);assert ok
            sock.sendall(struct.pack('!II',len(png),50)+png.tobytes());line=stream.readline()
            if not line:raise RuntimeError('C++ connection closed')
            command=json.loads(line);pwm=command['motor_pwm'];steer=command['servo_pwm']
            speed=max(0,pwm-10000)*rules['simulation']['pwm_speed_scale'];delta=max(-.48,min(.48,(steer-80)*rules['simulation']['steering_rad_per_pwm']));L=.26;W=.22
            for name,m in steers.items():y=W/2 if name=='FL' else -W/2;m.setPosition(math.atan2(L*math.tan(delta),L-y*math.tan(delta)))
            for name,m in motors.items():
                y=W/2 if name.endswith('L') else -W/2;factor=1-y*math.tan(delta)/L
                if name.startswith('F'):factor/=max(.5,math.cos(math.atan2(L*math.tan(delta),L-y*math.tan(delta))))
                m.setAvailableTorque(.5);m.setVelocity(speed*factor/.045)
            p=car.getPosition();rot=car.getOrientation();yaw=math.atan2(rot[3],rot[0]);events.add(command['state'])
            for i,(x,y) in enumerate(cones):
                dx,dy=x-p[0],y-p[1];along=dx*math.cos(yaw)+dy*math.sin(yaw);lateral=-dx*math.sin(yaw)+dy*math.cos(yaw)
                if math.hypot(max(0,abs(along)-.20),max(0,abs(lateral)-.14))<=.039:collisions.add('cone'+str(i))
            sign=1 if case['blocked_side']=='left' else -1
            if abs(p[0]-56.5)<.21 and abs(p[1]-sign*.305)<.34:collisions.add('board')
            wheel_positions=[sim.getFromDef('WHEEL_'+name).getPosition() for name in motors]
            progress,center_error=track_coordinates(p);max_center_error=max(max_center_error,abs(center_error))
            if any(abs(track_coordinates(w)[1])+.0175>.61 for w in wheel_positions):outside=True
            if command['state']=='ZEBRA_HOLD':zebra_hold+=.05
            if command['state']=='LIGHT_STOP':light_hold+=.05
            if 'light' in modules and case['start_s']<rules['tasks']['light_s'] and red and p[0]+40+.2>rules['tasks']['light_s']:red_crossing=True
            trace.write(json.dumps(dict(t=t,command=command,position=p,yaw=yaw,wheels=wheel_positions,red=red,green=green,collisions=sorted(collisions),outside=outside))+'\n')
            frame=raw.copy()
            for i,label in enumerate((f'{t:.2f}s {command["state"]} M{pwm} S{steer}',f'error={command["error"]:.2f} offset={command["offset"]:.3f} L={command["left"]} R={command["right"]}',f'cone={command["cone"]} zebra={command["zebra"]}',f'parking={command["parking"]} passed={command["passed"]}')):cv2.putText(frame,label,(5,20+22*i),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
            writer.write(frame)
            if command['state']!=last:cv2.imwrite(str(out/f'{tick:05d}-{command["state"]}.jpg'),frame);print(t,command['state'],p,flush=True);last=command['state']
            parking_done=command['state'] in ('QR_STOP_LATCHED','PARK_DONE')
            if parking_done:
                stopped_since=t if stopped_since is None else stopped_since
                if t-stopped_since>=1:completed=True;reason='parking_stop';break
            if progress>=case['goal_s']:
                completed='parking' not in modules;reason='goal_reached' if completed else 'parking_not_triggered';break
            if collisions or outside:reason='collision_or_boundary';break
            if t>=case['max_seconds']:break
        expected_side=-1 if case['blocked_side']=='left' else 1
        checks={}
        for module in modules:
            if case['scenario'] not in ('all',module):continue
            if module=='cone':checks['two_cones']=command.get('passed',0)>=2
            if module=='light':checks['light_stop_release']=light_hold>.1 and not red_crossing and command.get('motor_pwm',10000)>10000 if 'parking' not in modules else light_hold>.1 and not red_crossing
            if module=='zebra':checks['zebra_hold']=zebra_hold>=9.85
            if module=='parking':checks['parking_stop']=command.get('state') in ('QR_STOP_LATCHED','PARK_DONE');checks['selected_half']=bool(car.getPosition()[1]*expected_side>0)
        result=dict(completed=completed and all(checks.values()),checks=checks,end_reason=reason,collisions=sorted(collisions),outside=outside,max_center_error_m=max_center_error,red_crossing=red_crossing,sim_seconds=sim.getTime(),last_command=command,states=sorted(events),zebra_hold_seconds=zebra_hold,light_hold_seconds=light_hold,parking_center_in_selected_half=bool(car.getPosition()[1]*expected_side>0),scope='module sandbox; PWM below neutral is modeled as zero-speed braking, not calibrated real ESC reverse dynamics; QR stop does not certify all wheels inside the bay')
    except Exception as e:traceback.print_exc();result=dict(error=str(e),completed=False)
    finally:
        for m in motors.values():m.setVelocity(0)
        trace.close();writer.release();sock.close();(out/'result.json').write_text(json.dumps(result,indent=2));print(result,flush=True);sim.simulationQuit(0)
if __name__=='__main__':main()
