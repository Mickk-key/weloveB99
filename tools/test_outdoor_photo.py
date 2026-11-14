from pathlib import Path
import subprocess,shlex,json,cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'evidence/outdoor_photo'
flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True))
binary=ROOT/'build/outdoor_probe'
subprocess.run(['g++','-std=c++14','-O2','-I'+str(ROOT/'src'),str(ROOT/'tools/outdoor_probe.cpp'),'-o',str(binary),*flags],check=True)
results=[];images=[]
for i in range(8):
    subprocess.run([str(binary),str(out/f'frame{i}.png'),str(out/f'probe{i}')],check=True)
    r=json.loads((out/f'probe{i}.json').read_text());r['frame']=i;results.append(r);images.append(cv2.imread(str(out/f'probe{i}.jpg')))
cv2.imwrite(str(out/'comparison.jpg'),np.vstack([np.hstack(images[:4]),np.hstack(images[4:])]))
(out/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
