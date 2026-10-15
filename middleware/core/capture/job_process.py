"""Low-priority child process for file conversion, never a robot control process."""
import argparse
import json
import os


def emit(kind,value):
    print(json.dumps({'kind':kind,'value':value},ensure_ascii=False),flush=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('operation',choices=('export','realign','recording'))
    parser.add_argument('source')
    parser.add_argument('output')
    parser.add_argument('version')
    args=parser.parse_args()
    os.nice(10)
    # All child threads inherit this limit. Leave the other CPUs available to
    # live ROS capture; keep working on hosts without Linux affinity support.
    if hasattr(os,'sched_getaffinity') and hasattr(os,'sched_setaffinity'):
        allowed=os.sched_getaffinity(0)
        if len(allowed)>1:os.sched_setaffinity(0,{max(allowed)})
    try:
        progress=lambda value:emit('progress',value)
        if args.operation=='recording':
            from .catalog import read_json
            from pathlib import Path
            if read_json(Path(args.source)/'session.json').get('recording_mode')=='raw':
                from .raw_bundle import prepare
            else:
                from .native_bundle import prepare
            result=prepare(args.source,args.output,progress)
        elif args.operation=='export':
            from .exporter import export
            result=export(args.source,args.output,args.version,progress)
        else:
            from .offline import realign
            result=realign(args.source,progress)
        emit('result',result)
    except Exception as error:
        emit('error',str(error))
        raise SystemExit(1) from None


if __name__=='__main__':main()
