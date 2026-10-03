"""Run at most two chart workflows concurrently and verify downloaded deliveries."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parent
STATE = ROOT/'.local/batch-state.json'
ORDER = ['AboutTheUniverse','TrueHomeTrueWorldRework','Implexrough','EntrancetotheChaos',
         'ExoplanetaryMirage','Hate','OblivionPHIN']
DEST = ROOT.parent/'delivery/general-charts'
LIVE = {'queued','in_progress','waiting','requested','pending'}


def gh(*args):
    return subprocess.check_output(['gh',*map(str,args)],cwd=ROOT,text=True,encoding='utf-8')


def save(state):
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state,indent=2),encoding='utf-8')
    temporary.replace(STATE)


def runs():
    return json.loads(gh('run','list','--workflow','batch-chart.yml','--limit',100,
                        '--json','databaseId,displayTitle,status,conclusion,url'))


def available_slots(remote):
    return max(0,2-sum(r['status'] in LIVE for r in remote))


def main():
    state = json.loads(STATE.read_text(encoding='utf-8'))
    state['max_parallel_charts']=2
    while True:
        remote = runs()
        for run in remote:
            key = run['displayTitle'].removesuffix(' general 1080p60')
            if key in ORDER and run['status'] in LIVE and not state['charts'].get(key,{}).get('run'):
                state['charts'][key]=dict(run=run['databaseId'],status=run['status'],
                                         conclusion=run['conclusion'],url=run['url'])
        by_id = {r['databaseId']:r for r in remote}
        for key,entry in list(state['charts'].items()):
            if entry.get('status')=='downloaded' or not entry.get('run'):
                continue
            run = by_id.get(entry['run']) or json.loads(gh('run','view',entry['run'],'--json','status,conclusion,url'))
            entry.update({k:run[k] for k in ('status','conclusion','url')})
            if run['status']!='completed':
                continue
            if run['conclusion']!='success':
                entry['status']='failed'
                continue
            target = DEST/key
            target.mkdir(parents=True,exist_ok=True)
            gh('release','download',f"general-{key}-{entry['run']}",'--dir',target,'--clobber')
            report = json.loads((target/'validation.json').read_text(encoding='utf-8'))
            video = next(target.glob('*.mp4'))
            with video.open('rb') as f:
                digest = hashlib.file_digest(f,'sha256').hexdigest()
            assert digest==report['sha256']
            entry.update(status='downloaded',file=str(video),sha256=digest)
            print(key,'DOWNLOADED',video,flush=True)
            save(state)
        # Count actual cloud jobs, so a manually dispatched repair also occupies a slot.
        for _ in range(available_slots(remote)):
            remote = runs()
            if not available_slots(remote):
                break
            key = next((k for k in ORDER if not state['charts'].get(k,{}).get('run')),None)
            if key is None:
                break
            existing = next((r for r in remote if r['displayTitle']==key+' general 1080p60'
                             and r['status'] in LIVE),None)
            if existing is None:
                previous = {r['databaseId'] for r in remote}
                gh('workflow','run','batch-chart.yml','-f','chart='+key)
                for attempt in range(18):
                    time.sleep(5)
                    remote = runs()
                    existing = next((r for r in remote if r['databaseId'] not in previous
                                     and r['displayTitle']==key+' general 1080p60'),None)
                    if existing:
                        break
                else:
                    raise RuntimeError('Dispatched run not found; inspect GitHub before restarting')
            state['charts'][key] = dict(run=existing['databaseId'],status=existing['status'],
                                       conclusion=existing['conclusion'],url=existing['url'])
            print(key,'STARTED',existing['databaseId'],flush=True)
            save(state)
        completed = sum(e.get('status')=='downloaded' for e in state['charts'].values())
        failed = [k for k,e in state['charts'].items() if e.get('status')=='failed']
        state['status']='complete' if completed==len(ORDER) else 'running'
        state['failed_charts']=failed
        save(state)
        if state['status']=='complete':
            break
        if completed+len(failed)==len(ORDER):
            state['status']='needs-fix';save(state);break
        time.sleep(20)


if __name__=='__main__':
    while True:
        try:
            main()
            break
        except subprocess.CalledProcessError as exc:
            print('Temporary GitHub command failure:',exc,flush=True)
            time.sleep(20)
