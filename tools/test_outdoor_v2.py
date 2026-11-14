from pathlib import Path
import cv2,numpy as np,subprocess,json,shlex
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'evidence/outdoor_v2';OUT.mkdir(parents=True,exist_ok=True)
flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True))
subprocess.run(['g++','-std=c++14','-O2','-I'+str(ROOT/'src'),str(ROOT/'tools/outdoor_v2_probe.cpp'),'-o',str(ROOT/'build/outdoor_v2_probe'),*flags],check=True)
images=[];results=[]
for i in range(8):
    subprocess.run([str(ROOT/'build/outdoor_v2_probe'),str(ROOT/'evidence/outdoor_photo'/f'frame{i}.png'),str(OUT/f'probe{i}'),str(ROOT/'config/outdoor_v2.yaml')],check=True)
    r=json.loads((OUT/f'probe{i}.json').read_text());r['frame']=i;results.append(r)
    images.append(cv2.resize(cv2.imread(str(OUT/f'probe{i}.jpg')),(400,300)))
cv2.imwrite(str(OUT/'preview.jpg'),np.vstack([np.hstack(images[:4]),np.hstack(images[4:])]))
(OUT/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results),flush=True)
