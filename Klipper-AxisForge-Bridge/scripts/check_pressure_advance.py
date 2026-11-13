#!/usr/bin/env python3
"""Compare adapter/native extrusion queues offline, without a Klipper instance.

Reads trusted Klipper source and a compiled plan. No printer objects, network
connections, stepper queues or hardware are created. The native Move and
PrinterExtruder.process_move implementations run with an in-memory trapq sink.
This verifies native PA eligibility and its inputs, not physical PA calibration.
"""
import argparse
import ast
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from klipper_extras.axisforge_sync import ScheduledExtrusion


def extract(path,name,parent=None):
    tree=ast.parse(path.read_text())
    if parent: tree=next(n for n in tree.body if getattr(n,'name',None)==parent)
    node=next(n for n in tree.body if getattr(n,'name',None)==name)
    scope={'math':math}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


def compare(klipper,plan):
    Move=extract(klipper/'klippy/toolhead.py','Move')
    process=extract(klipper/'klippy/kinematics/extruder.py','process_move','PrinterExtruder')
    counts={'matched_extrusion_moves':0,'pressure_advance_eligible':0,'without_pressure_advance':0}
    for op in plan['operations']:
        if op['kind']!='chunk': continue
        for record in op['moves']:
            if not record['delta_e']: continue
            adapter=ScheduledExtrusion(record,0.)
            head=SimpleNamespace(max_accel=adapter.accel,max_velocity=adapter.cruise_v,
                                 junction_deviation=.01,mcr_pseudo_accel=10.)
            normal=Move(head,adapter.start_pos,adapter.end_pos,adapter.cruise_v)
            normal.accel=adapter.accel
            normal.set_junction(adapter.start_v**2,adapter.cruise_v**2,
                (record['profile'].get('end_velocity',0.)*adapter.move_d)**2)
            queues=[]
            for move in (adapter,normal):
                output=[]
                extruder=SimpleNamespace(trapq=None,trapq_append=lambda *args:output.append(args))
                process(extruder,10.+record['offset_s'],move,3)
                queues.append(output[0])
            if len(queues[0])!=len(queues[1]) or any(a!=b and not math.isclose(a,b,rel_tol=1e-9,abs_tol=1e-9)
                                                   for a,b in zip(*queues)):
                raise ValueError('Native extrusion queue mismatch on source line %s' % record['source_line'])
            expected=record['delta_e']>0 and record['start'][:2]!=record['end'][:2]
            if bool(queues[0][9])!=expected: raise ValueError('Native PA eligibility changed')
            counts['matched_extrusion_moves']+=1
            counts['pressure_advance_eligible' if expected else 'without_pressure_advance']+=1
    return dict(counts,hardware_commands=False,policy=plan.get('motion_options',{}).get('policy','exact'))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--klipper-dir',type=Path,required=True)
    parser.add_argument('--plan',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(compare(args.klipper_dir,json.loads(args.plan.read_text()))))
