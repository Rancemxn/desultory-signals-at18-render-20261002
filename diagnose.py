import json, zipfile
from chart import load_chart
from algo.algo5 import Planner, Settings, State
z=zipfile.ZipFile(r'C:\Users\Admin\Desktop\[AT 18] Desultory Signals.zip')
c=load_chart(z.read('chart.json').decode())
p=Planner(c,Settings(),json.load(open('hand-profile.json')))
original=p.choices
laststate=None
def choices(task,state,degraded=False):
    global laststate
    result=original(task,state,degraded)
    if task.id==1129:
        laststate=state
        print('LAST',degraded, len(result),[(k['note_ids'],k['hand'],k['finger'],k['start'],k['end']) for k in state.contacts[-8:]],flush=True)
    return result
p.choices=choices
p.tasks=[t for t in p.tasks if t.beat<16000]
try: p.run()
except Exception as e:
    print(e)
    t=next(t for t in p.tasks if t.id==1129)
    print('task',t.start,t.beat,t.end,'angle',t.line.angle@t.note.seconds)
    for side,finger in p.keys:
        seeds=p.seeds(t,laststate,side,finger)
        print(side,finger,'previous',[(k['note_ids'],k['start'],k['end']) for k in laststate.contacts if k['hand']==('left' if side==-1 else 'right') and k['finger']==finger][-1:])

