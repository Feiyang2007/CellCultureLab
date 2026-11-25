"""Prepare a separate movable working copy; the source snapshot is never modified."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,sys

HANDLES=[]
def usd_bootstrap():
    if os.name!='nt':raise RuntimeError('This handoff targets Windows / Isaac Sim 5.1.')
    package=Path(sys.prefix)/'Lib/site-packages/isaacsim'
    if not package.is_dir():raise RuntimeError('Use the Python environment in which Isaac Sim 5.1 is installed.')
    lib=next((package/'extscache').glob('omni.usd.libs-*'))
    sys.path.insert(0,str(lib))
    for d in (lib/'bin',lib/'bin/deps',package/'kit'):
        if d.is_dir():HANDLES.append(os.add_dll_directory(str(d)))
    return package

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def error_text(e):
    try:return str(e)
    except UnicodeError:return type(e).__name__+': native diagnostic could not be decoded as UTF-8; kept as an explicit failure.'

def prepare(source,dest,external,official_manifest):
    source=source.resolve();dest=dest.resolve();external=external.resolve()
    if dest.exists():raise RuntimeError('Destination exists; choose a new empty path. No existing files are replaced.')
    if source==dest or source in dest.parents:raise RuntimeError('Destination must be outside the immutable snapshot.')
    usd_bootstrap();from pxr import Sdf,UsdUtils
    shutil.copytree(source,dest)
    vendor=dest/'third_party/isaac_robots';shutil.copytree(external,vendor)
    official=json.loads(official_manifest.read_text(encoding='utf8'))
    mapping={r['url']:vendor/Path(r['file'].replace('\\','/').split('/IsaacAssets/5.1/',1)[1]) for r in official['files']}
    changes=[];unknown=[];layer_failures=[]
    oldproject='c:/users/17690/documents/feiyangchen/30-learing_by_myself/research/in_myresearch/vla/building_simulation_environment/cellculturelab/'
    oldvendor='c:/users/17690/isaacassets/5.1/'
    def translated(path):
        norm=path.replace('\\','/');low=norm.lower()
        if low.startswith(oldproject):return dest/norm[len(oldproject):]
        if low.startswith(oldvendor):return vendor/norm[len(oldvendor):]
        if norm in mapping:return mapping[norm]
        return None
    for p in sorted(dest.rglob('*')):
        if not p.is_file() or p.suffix.lower() not in ('.usd','.usda','.usdc'):continue
        try:l=Sdf.Layer.FindOrOpen(str(p))
        except Exception as e:layer_failures.append({'file':str(p.relative_to(dest)),'error':error_text(e)});continue
        if l is None:layer_failures.append({'file':str(p.relative_to(dest)),'error':'Layer is None'});continue
        before=sha(p);replacements=[]
        def replace(path):
            target=translated(path)
            if target is None:
                return path
            new=os.path.relpath(target,p.parent).replace('\\','/')
            if path!=new:replacements.append({'old':path,'new':new})
            return new
        UsdUtils.ModifyAssetPaths(l,replace)
        if replacements:
            l.Save();changes.append({'file':str(p.relative_to(dest)),'before_sha256':before,'after_sha256':sha(p),'asset_paths':replacements})
    # Update build pointers only in the disposable working copy. Saved reports remain unchanged.
    for rel in ['outputs/runtime_validation/mini_articulation_build.json','outputs/runtime_validation/c1000_isaac_lock_pilot/build.json']:
        p=dest/rel;m=json.loads(p.read_text(encoding='utf8'));before=sha(p)
        for key in ('asset','reference','scene'):
            if key in m:
                target=translated(m[key])
                if target is not None:m[key]=str(target)
        for key in ('asset','reference'):
            if key in m and Path(m[key]).is_file():m[key+'_sha256']=sha(m[key])
        p.write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf8')
        changes.append({'file':rel,'before_sha256':before,'after_sha256':sha(p),'reason':'Relocated build pointers and hashes for working copy only.'})
    # The bare USD loader had a local installation constant. Resolve from this interpreter.
    p=dest/'scripts/validate_gpu_capacity.py';before=sha(p);s=p.read_text(encoding='utf8')
    old='ISAAC = Path("D:/IsaacSim5/env/Lib/site-packages/isaacsim")'
    if old in s:
        s=s.replace(old,'ISAAC = Path(sys.prefix) / "Lib/site-packages/isaacsim"');p.write_text(s,encoding='utf8')
        changes.append({'file':str(p.relative_to(dest)),'before_sha256':before,'after_sha256':sha(p),'reason':'Resolve Isaac package using selected interpreter.'})
    result={'kind':'packaging_relocation_only','physics_rerun':False,'source':str(source),'destination':str(dest),'changes':changes,'layer_open_failures':layer_failures,
       'note':'Original snapshot and historical reports are untouched. New working-copy hashes do not replace old experimental identities.'}
    (dest/'HANDOFF_RELOCATION.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    return result

def dependency_audit(root,output):
    usd_bootstrap();from pxr import Sdf,UsdUtils
    root=root.resolve();rows=[];fails=[]
    for p in sorted(root.rglob('*')):
        if p.suffix.lower() not in ('.usd','.usda','.usdc') or not p.is_file():continue
        try:l=Sdf.Layer.FindOrOpen(str(p))
        except Exception as e:fails.append({'file':str(p.relative_to(root)),'error':error_text(e)});continue
        if l is None:continue
        refs=set()
        def collect(path):
            refs.add(path)
            return path
        # Identity callback enumerates composition and attribute asset paths; no Save.
        UsdUtils.ModifyAssetPaths(l,collect)
        for ref in sorted(refs):
            if not ref:continue
            norm=ref.replace('\\','/')
            if norm=='OmniPBR.mdl':status='engine_mdl_module_unverified'
            elif '://' in norm:status='external_uri'
            else:status='exists' if (p.parent/norm).resolve().exists() else 'missing'
            rows.append({'layer':str(p.relative_to(root)),'asset_path':ref,'status':status})
    counts={k:sum(r['status']==k for r in rows) for k in ('exists','missing','external_uri','engine_mdl_module_unverified')}
    result={'scope':'USD asset-path existence only. Does not execute MDL, compile shaders, follow every runtime reference, validate physics, or verify visual quality.',
       'counts':counts,'layer_open_failures':fails,'rows':rows}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'counts':counts,'layer_failures':len(fails)},ensure_ascii=False))
    return result

def main():
    ap=argparse.ArgumentParser();sp=ap.add_subparsers(dest='mode',required=True)
    p=sp.add_parser('prepare');p.add_argument('--source',type=Path,required=True);p.add_argument('--dest',type=Path,required=True);p.add_argument('--external',type=Path,required=True);p.add_argument('--official-manifest',type=Path,required=True)
    p=sp.add_parser('audit');p.add_argument('--project',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.mode=='prepare':
        r=prepare(a.source,a.dest,a.external,a.official_manifest);print('Relocated files:',len(r['changes']),'layer failures:',len(r['layer_open_failures']))
        return 1 if r['layer_open_failures'] else 0
    dependency_audit(a.project,a.output);return 0
if __name__=='__main__':sys.exit(main())
