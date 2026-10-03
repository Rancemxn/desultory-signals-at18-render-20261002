"""Remove redundant fingers when an existing contact already covers a short Drag."""
import argparse
import copy
import json
from pathlib import Path
import zipfile

from basis import NoteType
from chart import load_chart
from contact_refinement import ContactGeometry, validate_contact
from comfort_audit import closest


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('source',type=Path);ap.add_argument('chart',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args();plan=json.loads(args.source.read_text())
    with zipfile.ZipFile(args.chart) as z:chart=load_chart(z.read('chart.json').decode('utf-8-sig'))
    g=ContactGeometry(chart,physical_screen=plan['physical_screen'],extra_clearance_m=plan['comfort']['extra_block_clearance_m'])
    contacts=plan['contacts'];changes=[]
    for c in list(contacts):
        if c not in contacts or c['end']-c['start']>.02:continue
        if not all(g.notes[n][1].type==NoteType.DRAG for n in c['note_ids']):continue
        choices=[]
        for owner in contacts:
            if owner is c or owner['start']>c['start']+1e-8 or owner['end']<c['end']-1e-8:continue
            distance,t=closest(c,owner,plan['physical_screen'])
            if distance>.02:continue
            if validate_contact(g,c,owner['points']):continue
            choices.append((distance,owner))
        if not choices:continue
        distance,owner=min(choices,key=lambda x:x[0])
        changes.append(dict(drag=c['note_ids'],owner=list(owner['note_ids']),time=c['start'],distance_mm=distance*1000))
        owner['note_ids']=sorted(set(owner['note_ids'])|set(c['note_ids']))
        owner['joint_note_coverage']=True
        contacts.remove(c)
    plan['contacts']=sorted(contacts,key=lambda c:(c['start'],c['pointer']))
    args.output.write_text(json.dumps(plan,indent=2))
    args.output.with_name('shared-drag-refinement.json').write_text(json.dumps(changes,indent=2))
    print(json.dumps({'removed_redundant_contacts':len(changes),'changes':changes}),flush=True)


if __name__=='__main__':main()
