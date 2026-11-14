"""Physical module sandbox: same race binary, no GPIO, camera pixels + clock only."""
from pathlib import Path
import argparse,json,datetime,os,subprocess,socket,signal,hashlib,cv2
from build_world import build,box
ROOT=Path(__file__).resolve().parents[1]
def run(a):
    if not a.no_build:subprocess.run(['sh','build_local.sh'],cwd=ROOT,check=True)
    binary=ROOT/'build/race'
    modules=set(a.modules.split(','));modules={'lane','light','cone','zebra','parking'} if 'all' in modules else modules|{'lane'}
    if not modules<={'lane','light','cone','zebra','parking'}:raise ValueError('Unknown modules')
    scenario=a.scenario
    if scenario=='auto':scenario='all' if len(modules)>2 else next((x for x in ('parking','cone','light','zebra') if x in modules),'lane')
    stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f');out=ROOT/'sim/results'/f'{stamp}-{scenario}';out.mkdir(parents=True)
    rules=json.loads((ROOT/'sim/rules_2026_online.json').read_text())
    rules['vehicle'].update(camera_height=.30,camera_pitch_deg=-10.,camera_hfov_deg=90.)
    starts={'curve':25.,'lane':83.,'light':64.,'cone':74.,'zebra':52.,'parking':92.,'all':54.}
    goals={'curve':40.,'lane':88.,'light':69.,'cone':82.,'zebra':59.,'parking':97.,'all':97.}
    selected=[m for m in modules if m!='lane'] or ['lane']
    start=min(starts[m] for m in selected) if scenario=='all' else starts[scenario]
    goal=max(goals[m] for m in selected) if scenario=='all' else goals[scenario]
    case=dict(start_s=start,blocked_side=a.blocked_side,mirror=a.mirror,scenario=scenario,goal_s=goal,modules=sorted(modules),parking_mode=a.parking_mode,max_seconds=150 if scenario=='all' else 45,light_seconds=a.red_seconds,offset=a.offset,yaw_deg=a.yaw_deg,noise=a.noise)
    if a.parking_mode=='depth':
        rules['vehicle'].update(camera_height=.14,camera_pitch_deg=15.,camera_hfov_deg=120.)
    world=ROOT/'sim/worlds'/f'{stamp}.wbt';manifest=build(world,rules,case)
    manifest['scope']='Selected module sandbox; no payment, no real-car calibration'
    text=world.read_text().replace('controller "full_supervisor"','controller "suite_supervisor"')
    # Add a real non-payment QR on the unblocked side; detection must use pixels.
    assets=ROOT/'sim/assets';assets.mkdir(exist_ok=True)
    qr=cv2.QRCodeEncoder_create().encode('RACE-SIM-NO-PAYMENT');qr=cv2.copyMakeBorder(qr,4,4,4,4,cv2.BORDER_CONSTANT,value=255);qr=cv2.resize(qr,(256,256),interpolation=cv2.INTER_NEAREST);cv2.imwrite(str(assets/'qr.png'),qr)
    sign=-1 if a.blocked_side=='left' else 1;x=56.5;y=sign*.305
    text+=box(x,y,.25,.02,.40,.50,'1 1 1',solid=False)
    text+=f'Pose {{ translation {x-.011} {y} .28 rotation 0 1 0 -1.57079632679 children [ Shape {{ appearance Appearance {{ material Material {{ diffuseColor 1 1 1 }} texture ImageTexture {{ url [ "../assets/qr.png" ] }} }} geometry Plane {{ size .18 .18 }} }} ] }}\n'
    world.write_text(text)
    config=Path(a.config).resolve() if a.config else ROOT/'config/sim.yaml'
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2));(out/'config.yaml').write_bytes(config.read_bytes())
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src').glob('*') if p.is_file()}
    hashes['build/race']=hashlib.sha256(binary.read_bytes()).hexdigest();(out/'source_hashes.json').write_text(json.dumps(hashes,indent=2))
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    prefs=ROOT/'sim/runtime/config/Cyberbotics/Webots-R2025a.conf';prefs.parent.mkdir(parents=True,exist_ok=True);prefs.write_text('[Internal]\nfirstLaunch=false\n[%General]\ncheckWebotsUpdateOnStartup=false\npythonCommand=/usr/bin/python3\n[OpenGL]\ndisableShadows=true\ndisableAntiAliasing=true\nGTAO=0\n[Sound]\nmute=true\n')
    env=dict(os.environ,SUITE_PORT=str(port),SUITE_RUN=str(out),QTWEBENGINE_DISABLE_SANDBOX='1',LIBGL_ALWAYS_SOFTWARE='1',LP_NUM_THREADS='2',XDG_CONFIG_HOME=str(prefs.parents[1]),CI='1')
    with (out/'cpp.log').open('w') as cl,(out/'webots.log').open('w') as wl:
        cmd=[str(binary),'--env','sim','--config',str(config),'--modules',','.join(sorted(modules)),'--parking-mode',a.parking_mode,'--sim-port',str(port),'--out','sim/results/'+out.name+'/control']
        if a.lane_backend:cmd+=['--lane-backend',a.lane_backend]
        cpp=subprocess.Popen(cmd,cwd=ROOT,stdout=cl,stderr=subprocess.STDOUT);web=None
        try:
            web=subprocess.Popen(['xvfb-run','-a','webots','--batch','--mode=fast','--no-rendering','--minimize','--stdout','--stderr',str(world)],env=env,stdout=wl,stderr=subprocess.STDOUT,start_new_session=True)
            web.wait(timeout=a.wall_timeout)
        finally:
            if web and web.poll() is None:os.killpg(web.pid,signal.SIGTERM);web.wait(timeout=10)
            try:cpp.wait(timeout=5)
            except subprocess.TimeoutExpired:
                cpp.terminate()
                try:cpp.wait(timeout=3)
                except subprocess.TimeoutExpired:cpp.kill();cpp.wait()
    if (out/'demo.avi').exists():subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(out/'demo.avi'),'-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'demo.mp4')],check=True)
    r=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {'error':'See webots.log','completed':False}
    print(json.dumps(dict(run=str(out),result=r),indent=2),flush=True);return r
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--modules',default='lane');p.add_argument('--scenario',choices=['auto','lane','light','cone','zebra','parking','all','curve'],default='auto');p.add_argument('--parking-mode',choices=['qr','depth'],default='qr');p.add_argument('--lane-backend',choices=['indoor','outdoor','outdoor_v2']);p.add_argument('--blocked-side',choices=['left','right'],default='left');p.add_argument('--red-seconds',type=float,default=6);p.add_argument('--mirror',action='store_true');p.add_argument('--config');p.add_argument('--no-build',action='store_true');p.add_argument('--offset',type=float,default=0);p.add_argument('--yaw-deg',type=float,default=0);p.add_argument('--noise',type=float,default=0);p.add_argument('--wall-timeout',type=int,default=1200);a=p.parse_args()
    if a.wall_timeout<60 or a.wall_timeout>3600:p.error('wall-timeout must be 60..3600 seconds')
    if not (-.3<=a.offset<=.3 and -15<=a.yaw_deg<=15 and 0<=a.noise<=.1):p.error('Invalid spawn/noise parameters')
    if not 0<=a.red_seconds<=30:p.error('--red-seconds must be 0..30')
    r=run(a);raise SystemExit(0 if r.get('completed') and not r.get('collisions') and not r.get('outside') else 1)
