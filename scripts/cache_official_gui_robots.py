"""Cache exact official robot files and all variant dependencies locally."""
import hashlib,json,urllib.request,urllib.parse,time
from pathlib import Path
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import UsdUtils
ROOT=Path(__file__).resolve().parents[1]
DEST=ROOT/'assets/nvidia_official'
HOST='https://omniverse-content-production.s3-us-west-2.amazonaws.com/'
START=[HOST+'Assets/Isaac/5.1/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd',HOST+'Assets/Isaac/5.1/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd']
queue=list(START);seen=set();manifest=[];errors=[]
while queue:
    url=queue.pop(0)
    if url in seen:continue
    seen.add(url)
    if not url.startswith(HOST):raise ValueError('Unexpected remote host: '+url)
    path=DEST/urllib.parse.unquote(url[len(HOST):])
    path.parent.mkdir(parents=True,exist_ok=True)
    try:
        if not path.exists():
            for attempt in range(3):
                try:
                    with urllib.request.urlopen(url,timeout=30) as response:data=response.read()
                    path.write_bytes(data);break
                except Exception:
                    if attempt==2:raise
                    time.sleep(1)
        data=path.read_bytes()
        deps=[]
        if path.suffix.lower() in ('.usd','.usda','.usdc'):
            groups=UsdUtils.ExtractExternalReferences(str(path))
            deps=[a for group in groups for a in group]
            for asset in deps:
                if asset in ('OmniPBR.mdl','OmniGlass.mdl','OmniSurface.mdl') or not asset:continue
                if asset.startswith(('omniverse:','https:','http:')):
                    if asset.startswith(HOST):target=asset
                    elif '/Isaac/' in asset:
                        target=HOST+'Assets/Isaac/5.1/Isaac/'+asset.split('/Isaac/',1)[1]
                    else:raise ValueError('Unmapped URL '+asset)
                else:target=urllib.parse.urljoin(url,asset)
                queue.append(target)
        manifest.append({'url':url,'file':str(path),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'dependencies':deps})
        print('CACHED',len(manifest),len(data),path.relative_to(DEST),flush=True)
    except Exception as e:
        errors.append({'url':url,'error':repr(e)});print('ERROR',url,repr(e),flush=True)
    (DEST/'manifest.json').write_text(json.dumps({'files':manifest,'errors':errors,'pending':queue},indent=2),encoding='utf8')
print('COMPLETE',len(manifest),'errors',len(errors),'bytes',sum(r['bytes'] for r in manifest),flush=True)
if errors:raise SystemExit(1)
