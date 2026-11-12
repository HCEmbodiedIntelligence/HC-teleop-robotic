#!/usr/bin/env python3
"""依次运行各组仿真测试，避免多个 VR 模拟器同时发包。"""
import argparse
from datetime import datetime
from pathlib import Path
import subprocess
import sys

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--url',default='http://172.168.200.30:7876')
parser.add_argument('--report',type=Path,default=Path('runtime/dagger_boundary')/datetime.now().strftime('%Y%m%d-%H%M%S'))
suites=['body_base','command','fault','aggressive','startup','concurrency','wire','full_session','timing']
parser.add_argument('--only',choices=suites,nargs='+',default=suites)
args=parser.parse_args()
codes=[]
for suite in args.only:
    script='dagger_aggressive.py' if suite=='aggressive' else f'dagger_{suite}_boundaries.py'
    codes.append(subprocess.call([sys.executable,str(Path(__file__).with_name(script)),
        '--url',args.url,'--report',str(args.report/suite)]))
raise SystemExit(1 if any(codes) else 0)
