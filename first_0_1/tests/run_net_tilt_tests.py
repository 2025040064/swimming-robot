"""Compile real application code with host hardware stubs; no board is driven."""
import argparse
import re
import subprocess
import tempfile
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--cc', required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
sources = [root / 'User' / name for name in (
    'algorithm/algo_filter.c', 'algorithm/algo_pid.c',
    'app/app_control.c', 'app/app_state_machine.c',
    'app/app_task.c', 'app/app_protocol.c',
    'bsp/test/bsp_mpu_motor_test.c')]
with tempfile.TemporaryDirectory(prefix='boat_net_tilt_') as tmp:
    tmp = Path(tmp)
    for test_mode, ranging in ((0, 0), (0, 1), (1, 0)):
        config = (root / 'User/robot_config.h').read_text(encoding='utf-8')
        config = config.replace('#define ROBOT_RANGING_VALIDATED       0',
                                f'#define ROBOT_RANGING_VALIDATED       {ranging}')
        test_header = (root / 'User/bsp/test/bsp_mpu_motor_test.h').read_text(encoding='utf-8')
        test_header = re.sub(r'(#define ROBOT_IMU_MOTOR_TEST_ENABLE\s+)\d+',
                            lambda m: m[1] + str(test_mode), test_header)
        (tmp / 'bsp/test').mkdir(parents=True, exist_ok=True)
        (tmp / 'bsp/test/bsp_mpu_motor_test.h').write_text(test_header, encoding='utf-8')
        (tmp / 'robot_config.h').write_text(config, encoding='utf-8')
        exe = tmp / f'net_tilt_{test_mode}_{ranging}.exe'
        cmd = [args.cc, '-std=gnu89', '-Wall', '-Wextra', '-Werror',
               '-Wno-unused-function', '-I' + str(tmp),
               '-I' + str(root / 'tests/stubs'), '-I' + str(root / 'User'),
               '-I' + str(root / 'User/bsp'),
               str(root / 'tests/net_tilt_test.c'), *map(str, sources),
               '-lm', '-o', str(exe)]
        subprocess.run(cmd, check=True)
        subprocess.run([str(exe)], check=True)
