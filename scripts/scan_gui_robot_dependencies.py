import json,hashlib,urllib.parse
from pathlib import Path
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import UsdUtils
ROOT=Path(__file__).resolve().parents[1]
CACHE=Path.home()/'IsaacAssets/5.1'
HOST='https://omniverse-content-production.s3-us-west-2.amazonaws.com/'
queue=[HOST+'Assets/Isaac/5.1/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd',HOST+'Assets/Isaac/5.1/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd']
seen=set();files=[];pending=[]
while queue:
    url=queue.pop(0)
    if url in seen:continue
    seen.add(url)
    if not url.startswith(HOST):raise ValueError(url)
    path=CACHE/urllib.parse.unquote(url[len(HOST):])
    if not path.is_file():pending.append({'url':url,'file':str(path)});continue
    deps=[]
    if path.suffix.lower() in ('.usd','.usda','.usdc'):
        deps=[a for group in UsdUtils.ExtractExternalReferences(str(path)) for a in group]
        for asset in deps:
            if not asset or asset in ('OmniPBR.mdl','OmniGlass.mdl','OmniSurface.mdl'):continue
            if asset.startswith(('http:','https:','omniverse:')):
                if asset.startswith(HOST):target=asset
                elif '/Isaac/' in asset:target=HOST+'Assets/Isaac/5.1/Isaac/'+asset.split('/Isaac/',1)[1]
                else:raise ValueError(asset)
            else:target=urllib.parse.urljoin(url,asset)
            queue.append(target)
    files.append({'url':url,'file':str(path),'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'dependencies':deps})
result={'files':files,'pending':pending}
(ROOT/'outputs/gui_review/official_robot_dependencies.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print('cached',len(files),'pending',len(pending))
