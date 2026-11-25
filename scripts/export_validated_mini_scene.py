"""Export a reviewed diagnostic mini scene with persistent measured capacities.

Flattens only the small override stack, keeping original geometry references.
The result depends on v06 and the candidate mini asset, not outputs/ fixtures.
"""
import argparse
import hashlib
import json
from pathlib import Path
from validate_gpu_capacity import bare_usd,repair,CAPACITY_NAMES

bare_usd()
from pxr import Usd,UsdPhysics,Sdf,UsdUtils

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--review",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    meta=json.loads(args.review.read_text(encoding="utf-8"))
    if not meta.get("runtime_validated") or not meta.get("capacity_run_validated"):
        raise ValueError("Only externally reviewed clean runs can be exported")
    source=Path(meta["source"])
    main_scene=ROOT/"assets/scenes/biolab_main_v06.usda"
    if args.output.resolve()==main_scene.resolve() or args.output.resolve()==source.resolve():
        raise ValueError("Refusing to overwrite input scene")
    attrs=meta["after"]["scenes"][0]["attributes"]
    capacities={name:rec["composed_value"] for name,rec in attrs.items()
        if name.startswith("physxScene:gpu") and rec["composed_type"]=="uint" and rec["composed_value"] is not None}
    total=capacities.pop(CAPACITY_NAMES[0])
    if capacities.pop(CAPACITY_NAMES[1])!=total:
        raise ValueError("Exporter currently requires equal aggregate capacities")
    stage=Usd.Stage.Open(str(source))
    repair(stage,total,Usd,UsdPhysics,Sdf,None,capacities)
    stage.SetEditTarget(stage.GetSessionLayer())
    physics=stage.GetPrimAtPath(meta["selected_scene"])
    hz=meta.get("physics_time_steps_per_second",{}).get(str(physics.GetPath()))
    if not hz:
        raise ValueError("Reviewed run must record actual physics frequency")
    physics.CreateAttribute("physxScene:timeStepsPerSecond",Sdf.ValueTypeNames.UInt).Set(hz)
    physics.CreateAttribute("physxScene:enableGPUDynamics",Sdf.ValueTypeNames.Bool).Set(True)
    physics.CreateAttribute("physxScene:broadphaseType",Sdf.ValueTypeNames.Token).Set("GPU")
    active=[(str(p.GetPath()),p.GetTypeName()) for p in stage.Traverse()]
    result=Sdf.Layer.CreateAnonymous("validated_pilot.usda")
    found_base=False
    for layer in stage.GetLayerStack():
        if layer.realPath and Path(layer.realPath).resolve()==main_scene.resolve():
            found_base=True
            break
        UsdUtils.StitchLayers(result,layer)
    if not found_base:
        raise ValueError("Expected original v06 base layer")
    if args.output.resolve().parent!=main_scene.parent.resolve():
        raise ValueError("Export must live beside original v06")
    result.subLayerPaths=[main_scene.name]
    result.defaultPrim="World"
    result.customLayerData={"validationScope":"candidate native mini with selected tips; Robot, Vortex and C1000 inactive; not complete task acceptance",
        "mujoco_reference_version":"3.16.0 candidate", "original_version_matched":False,
        "review_report":str(args.review.resolve()),"review_sha256":hashlib.sha256(args.review.read_bytes()).hexdigest()}
    result.Export(str(args.output))
    readback=Usd.Stage.Open(str(args.output))
    if active!=[(str(p.GetPath()),p.GetTypeName()) for p in readback.Traverse()]:
        raise RuntimeError("Export changed active topology")
    for name in CAPACITY_NAMES:
        if readback.GetPrimAtPath(str(physics.GetPath())).GetAttribute(name).Get()!=total:
            raise RuntimeError("Export lost aggregate capacity")
    report=dict(scene=str(args.output.resolve()),sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        matching_active_topology=True,review=str(args.review.resolve()),aggregate_capacity=total,
        other_capacities=capacities,physics_hz=hz,full_scene_accepted=False,original_version_matched=False)
    target=ROOT/"outputs/runtime_validation"/(args.output.stem+"_export.json")
    target.write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report),flush=True)


if __name__=="__main__":
    main()
