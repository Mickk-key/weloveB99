from pathlib import Path
import subprocess,shlex,json,itertools,sys
ROOT=Path(__file__).resolve().parents[1]
def main():
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True))
    out=ROOT/'build/integration_tests'
    subprocess.run(['g++','-std=c++14','-O2','-I'+str(ROOT/'src'),'-I'+str(ROOT/'sim/native'),str(ROOT/'tests/integration.cpp'),'-o',str(out),'-pthread',*flags],check=True)
    r=subprocess.run([str(out)],cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT);print(r.stdout,flush=True)
    (ROOT/'evidence/tests.txt').write_text(r.stdout)
    if r.returncode:raise RuntimeError('Integration tests failed')
    for name in ('qr_tests','depth_tests','fixed_maneuver_tests'):
        binary=ROOT/'build'/name
        subprocess.run(['g++','-std=c++14','-O2','-I'+str(ROOT/'src'),'-I'+str(ROOT/'sim/native'),str(ROOT/'tests'/(name+'.cpp')),'-o',str(binary),'-pthread',*flags],check=True)
        r=subprocess.run([str(binary)],cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        print(r.stdout,flush=True)
        with (ROOT/'evidence/tests.txt').open('a') as log:log.write(r.stdout)
        if r.returncode:raise RuntimeError(name+' failed')
    exe=ROOT/'build/race'
    for env in ('indoor','outdoor','outdoor_v2','sim'):
        for bits in itertools.product((False,True),repeat=4):
            modules=','.join(['lane']+[m for m,on in zip(('light','cone','zebra','parking'),bits) if on])
            subprocess.run([str(exe),'--env',env,'--modules',modules,'--check-config'],cwd=ROOT,stdout=subprocess.DEVNULL,check=True)
    for args in (['--modules','typo'],['--env','typo'],['--enable-motion','--image','none.png'],['--camera-only','--enable-motion'],['--env','sim','--enable-motion']):
        r=subprocess.run([str(exe),*args],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert r.returncode!=0,args
    print('PASS CLI configurations and motion isolation',flush=True)
    subprocess.run([sys.executable,str(ROOT/'tools/validate_fixed_config.py')],cwd=ROOT,check=True)
if __name__=='__main__':main()
