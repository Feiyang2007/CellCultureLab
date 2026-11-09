"""Offline handoff checks. No physics, GUI, network, or report rewriting."""
from pathlib import Path
import argparse, hashlib, json, math, sys

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))

def review(root):
    root=Path(root).resolve(); data=root/'outputs/runtime_validation'; checks=[]
    def check(name,passed,detail):checks.append(dict(name=name,passed=bool(passed),detail=detail))
    g=read(data/'v5_independent.json');j=read(data/'v5_runtime_acceptance_20261019.json')
    check('six_asset_report',len(g['results'])==6 and all(x['passed'] for x in g['results']),
          {'geoms':sum(x['checked_geoms'] for x in g['results']),'joints':sum(x['movable_joints'] for x in g['results'])})
    ev=j['evidence']; matches=[]
    for value in ev.values():
        for old,expected in value.get('asset_hashes',{}).items():
            rel=old.replace('\\','/').split('/CellCultureLab/',1)[-1]; p=root/rel
            matches.append({'file':rel,'matches':p.is_file() and sha(p)==expected})
    check('21_joint_records_and_asset_hashes',j['verified_joints']==21 and len(ev)==21 and not j['failed'] and all(m['matches'] for m in matches),
          {'scope':j['scope'],'asset_checks':matches})
    m=read(data/'mini_articulation_330_comparison.json'); ref=root/'upstream/physics_reference/centrifuge_tiangen_tgear_mini.json'
    check('8_reference_protocol_records',len(m['rows'])==8 and all(r['passed'] for r in m['rows']) and sha(ref)==m['reference_sha256'],
          {'max_angle_error_deg':max(x['max_angle_error_deg'] for x in m['rows']),'mujoco':m['mujoco_version'],
           'original_version_acceptance':m['original_version_acceptance'],'note':'Only the eight named protocol records; not full task or full trajectory equivalence.'})
    s=read(data/'mini_lid_slow_real_cpu_rev2.json')
    lines=[json.loads(line) for line in (data/'mini_lid_slow_real_cpu_rev2.steps.jsonl').read_text(encoding='utf8').splitlines()]
    header,*rows=lines
    peak=max(abs(v) for r in rows for v in r['v_post'])
    cont=all(a['q_post']==b['q_pre'] and a['v_post']==b['v_pre'] for a,b in zip(rows,rows[1:]))
    finite=all(math.isfinite(v) for r in rows for k in ('q_pre','v_pre','torque_Nm','q_post','v_post') for v in r[k])
    sat=sum(any(abs(t)>=header['control_design']['tau_sat_Nm'] for t in r['torque_Nm']) for r in rows)
    bools={k:v for k,v in s['verdict'].items() if type(v) is bool}
    check('slow_1140_step_integrity',len(rows)==1140 and [r['step'] for r in rows]==list(range(1140)) and cont and finite,
          {'steps':len(rows),'exact_qv_continuity':cont,'finite':finite})
    check('slow_peak_speed_and_saturation_recomputed',abs(peak-s['slow_summary']['peak_v_rad_s'])<1e-12 and sat==0,
          {'peak_v_rad_s':peak,'saturated_steps':sat,'tracking_error_from_report':s['slow_summary']['max_tracking_error_rad']})
    check('slow_recorded_criteria',len(bools)==24 and all(bools.values()) and s['criteria_all_passed'],
          {'stored_boolean_criteria':len(bools),'note':'Criteria count and saved verdict checked; this is not a new simulation or independent recomputation of all criteria.'})
    for key,rel in [('asset_sha256_16','assets/instruments6_pilot/centrifuge_tiangen_tgear_mini.usda'),('scene_sha256_16','assets/scenes/mini_closed_loop_v2.usda')]:
        check('slow_'+key,sha(root/rel).startswith(header[key]),{'file':rel,'recorded':header[key],'current':sha(root/rel)})
    runner=root/'scripts/run_mini_lid_task_v2.py'; note=(root/'docs/低速开合修订2_20261019.md').read_text(encoding='utf8')
    check('slow_runner_matches_revision2_document',sha(runner)[:16] in note,{'sha256':sha(runner),'note':'Revision 2 document contains this prefix; JSONL itself did not record runner hash.'})
    cdir=data/'c1000_isaac_lock_pilot'; c=read(cdir/'reviewed_summary.json')
    runs=[read(cdir/(n+'.json')) for n in ('cycle_diagnostic','cycle_physics_only','cycle_physics_only_repeat')]
    tr=[r['rows'][0]['trajectory'] for r in runs]
    check('c1000_three_saved_trajectories_equal',tr[0]==tr[1]==tr[2],{'recorded_samples_per_run':[len(t) for t in tr],'note':'Saved samples compared exactly; not 3032 complete per-step samples.'})
    check('c1000_cycle_verdicts',all(r['rows'][0]['passed'] and r['rows'][0]['steps']==3032 for r in runs),
          {'steps':[r['rows'][0]['steps'] for r in runs],'criteria_each':c['criteria_each'],'scope':c['scope'],
           'main_scene_integrated':c['main_scene_integrated'],'source_dynamics_equivalent':c['source_dynamics_equivalent']})
    for key,rel in [('runner_sha256','scripts/check_c1000_isaac_lock.py'),('asset_sha256','outputs/runtime_validation/c1000_isaac_lock_pilot/asset.usda')]:
        check('c1000_'+key,sha(root/rel)==c[key],{'file':rel,'current':sha(root/rel),'recorded':c[key]})
    origin=read(root/'outputs/gui_review/original_local_build.json')
    check('v06_source_hash_preserved',sha(root/'assets/scenes/biolab_main_v06.usda')==origin['source_sha256'],{'sha256':origin['source_sha256']})
    return {'kind':'saved_evidence_offline_review','physics_rerun':False,'all_checks_passed':all(x['passed'] for x in checks),'checks':checks,
      'limitations':['Hash consistency proves file identity, not authorship, historical creation time, or real-world accuracy.',
      'mini slow report retains generic suspicious_no_response/zero_response text for hold_open. It is not deleted or silently normalized; interpret with q/v/holding torque and saved hold criteria.',
      'mini protocol report original_version_acceptance=false remains unchanged; use only its eight passed protocols, not blanket original-version/task acceptance.',
      'No robot task, main-scene contact acceptance, full engine trajectory equivalence, or visual data readiness is established.']}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--output',type=Path)
    a=ap.parse_args();r=review(a.project);text=json.dumps(r,ensure_ascii=False,indent=2)
    if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text,encoding='utf8')
    else:print(text)
    print('Offline evidence review:',r['all_checks_passed'],len(r['checks']),'checks')
    return 0 if r['all_checks_passed'] else 1
if __name__=='__main__':sys.exit(main())
