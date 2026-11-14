from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT/'config/outdoor_v2.yaml').read_text()
cases = [
    ('negative duration', 'seconds: 0.65', 'seconds: -0.65'),
    ('zero duration', 'seconds: 0.65', 'seconds: 0'),
    ('infinite duration', 'seconds: 0.65', 'seconds: .Inf'),
    ('nan steering', 'steer:  5.0', 'steer: .Nan'),
    ('reverse motor', 'motor_pwm: 10900', 'motor_pwm: 9000'),
    ('neutral motor', 'motor_pwm: 10900', 'motor_pwm: 10000'),
    ('motor too high', 'motor_pwm: 10900', 'motor_pwm: 12001'),
    ('fractional motor', 'motor_pwm: 10900', 'motor_pwm: 10900.5'),
    ('missing motor', 'motor_pwm: 10900', 'unknown: 10900'),
    ('missing plan', 'cone_maneuver_steps:', 'unused_plan:'),
    ('invalid direction', 'cone_fixed_side: 0', 'cone_fixed_side: 2'),
    ('invalid parking start', 'parking_maneuver_start_y: 340.', 'parking_maneuver_start_y: 480.'),
    ('reverse QR seek', 'parking_qr_seek_pwm: 10900', 'parking_qr_seek_pwm: 9000'),
    ('invalid step name', 'name: OUT', 'name: "bad name"'),
]
with tempfile.TemporaryDirectory(dir=ROOT/'build') as folder:
    path=Path(folder)/'config.yaml'
    for name,old,new in cases:
        assert old in source,(name,old)
        path.write_text(source.replace(old,new,1))
        result=subprocess.run([str(ROOT/'build/race'),'--config',str(path),'--modules','all','--check-config'],capture_output=True,text=True)
        assert result.returncode!=0,name
        print('PASS reject',name)
    path.write_text(source.replace('seconds: 0.65','seconds: 0.8',1).replace('motor_pwm: 10900','motor_pwm: 10850',1))
    subprocess.run([str(ROOT/'build/race'),'--config',str(path),'--modules','all','--check-config'],check=True)
    print('PASS accept independently tuned duration and motor')
