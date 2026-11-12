from pathlib import Path
import json, hashlib, math, sys, subprocess
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import cv2, numpy as np, yaml
from ament_index_python.packages import get_package_share_directory

out=Path(__file__).resolve().parent
root=out.parents[1]
# Keep the recorded source hash intact. The evaluator used a top-level arena
# key, but actual trial archives nest it under config. Save a transparent,
# reproducible analysis-only adapter beside these results.
original_path=root/'scripts/analyze_wingbot_selfplay.py'
original=original_path.read_text(encoding='utf8')
adapted=original.replace('config["arena"]["world_offset"]','config["config"]["arena"]["world_offset"]')
adapted=adapted.replace('ROOT = Path(__file__).resolve().parents[1]',
    'ROOT = Path(__file__).resolve().parents[2]\nsys.path.insert(0,str(ROOT/"scripts"))')
adapter_path=out/'analyze_wingbot_selfplay_compat.py'
adapter_path.write_text(adapted,encoding='utf8')
(out/'analysis_adapter.json').write_text(json.dumps(dict(
    original=str(original_path),original_sha256=hashlib.sha256(original_path.read_bytes()).hexdigest(),
    adapter=adapter_path.name,adapter_sha256=hashlib.sha256(adapter_path.read_bytes()).hexdigest(),
    changes=['Read archived arena from config.arena instead of top-level arena.',
             'Resolve repository root and helper imports from this result subdirectory.'],
    simulation_code_unchanged=True),ensure_ascii=False,indent=2),encoding='utf8')
subprocess.run([sys.executable,str(adapter_path),str(out)],check=True,cwd=root)
def read(name): return json.loads((out/name).read_text(encoding='utf8'))
def save(name,value): (out/name).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf8')

def validate_video(name):
    cap=cv2.VideoCapture(str(out/(name+'.mp4')))
    fps=cap.get(cv2.CAP_PROP_FPS); declared=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    size=[int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))]
    n=0; first=last=None
    while True:
        ok,frame=cap.read()
        if not ok: break
        if first is None: first=frame.copy()
        last=frame; n+=1
    cap.release()
    if last is not None: cv2.imwrite(str(out/(name+'.verified_last.png')),last)
    rows=[json.loads(line) for line in (out/(name+'.frames.jsonl')).read_text().splitlines() if line.strip()]
    changed=int(np.count_nonzero(np.any(first!=last,axis=2))) if first is not None else 0
    return name,dict(fps=fps,declared_frames=declared,decoded_frames=n,duration_s=n/fps if fps else 0,
        size=size,frames_logged=len(rows),held_frames=sum(bool(x.get('held')) for x in rows),
        missing_cards=sum(bool(x.get('missing')) for x in rows),first_last_changed_pixels=changed,
        checks_pass=n==declared==len(rows)==9000 and abs(fps-30)<1e-6 and changed>0)
with ThreadPoolExecutor(max_workers=2) as pool:
    videos=dict(pool.map(validate_video,['uav_native_rgb','uav_image_face_ledger']))
save('recording_validation.json',videos)
share=Path(get_package_share_directory('maicon_arena_gz'))
manifest=read('source_manifest.json')['sha256']; mismatches=[]
for key,expected in manifest.items():
    if key.startswith('repo/'): p=root/key[5:]
    elif key.startswith('installed/share/'): p=share/key[len('installed/share/'):]
    elif key.startswith('installed/lib/'): p=share.parents[1]/'lib/maicon_arena_gz'/key[len('installed/lib/'):]
    else: mismatches.append(dict(key=key,error='unknown prefix')); continue
    actual=hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
    if actual!=expected: mismatches.append(dict(key=key,expected=expected,actual=actual))
source=dict(files_checked=len(manifest),mismatches=mismatches,all_match=not mismatches)
save('source_verification.json',source)
report=read('selfplay_report.json'); analysis=read('selfplay_analysis.json')
ledger=read('uav_image_face_ledger.json')
spec=yaml.safe_load((out/'arena_spec.yaml').read_text())
layout=yaml.safe_load((out/'layout_default.yaml').read_text())
sectors=['R-S1','R-S2','R-S3','CM']
sectors_report={}
for sector in sectors:
    base=ledger['sector_summary'].get(sector,{})
    faces=[f for f in ledger['faces'] if f.get('sector')==sector and f.get('image_state')!='excluded']
    states=Counter(f.get('ground_state','pending') for f in faces)
    rows=analysis['agents']['B-2']['mission_evaluation']['m3']['per_sector_class']
    bins=[r for r in rows if r['sector']==sector]
    sectors_report[sector]=dict(boxes=base.get('boxes',0),identified_boxes=base.get('identified_boxes',0),
        resolved_boxes=base.get('resolved_boxes',0),identified_faces=states['identified'],empty_faces=states['empty'],
        unresolved_faces=sum(v for k,v in states.items() if k not in ('identified','empty','excluded')),
        eligible_faces=len(faces),states=dict(states),
        m3_expected=sum(r['expected'] for r in bins),m3_reported=sum(r['reported'] for r in bins),
        missing={r['type']:-r['difference'] for r in bins if r['difference']<0},
        extra={r['type']:r['difference'] for r in bins if r['difference']>0},
        exact=all(r['difference']==0 for r in bins),
        remaining=[dict(key=f['key'],state=f.get('ground_state'),center=f['center']) for f in faces if f.get('ground_state') not in ('identified','empty','excluded')])
save('sector_box_report.json',dict(observer=ledger.get('ground_observer'),image_stamp_s=ledger.get('image_stamp_s'),
    ground_state_stamp_s=ledger.get('ground_state_stamp_s'),mission_complete=ledger.get('mission_complete'),sectors=sectors_report,
    basis='Final UAV image-associated geometry with B-2 ground observations; M3 uses last server-accepted replacement snapshot. I includes M2/M3; E means empty.'))

# Ground-truth locations are used only here, after the match, to locate sides
# whose class/sector aggregate may be missing. They never enter the Agent.
geometric=[]
width,height=spec['floor']['size']
for f in ledger['faces']:
    if f.get('sector') not in sectors or f.get('image_state')=='excluded': continue
    distance,index=min((math.hypot(f['center'][0]-(b['center'][0]+width/2),f['center'][1]-(b['center'][1]+height/2)),i) for i,b in enumerate(layout['boxes']))
    if distance>.10: continue
    gt=layout['boxes'][index]['faces'].get(f['face'],'blank')
    if gt.startswith('m3:'):
        geometric.append(dict(key=f['key'],sector=f['sector'],expected_class=gt.split(':',1)[1],
            ground_state=f.get('ground_state'),ground_box=f.get('ground_box'),center=f['center'],match_error_m=distance,
            truth_box_index=index,identified=f.get('ground_state')=='identified'))
save('m3_side_audit.json',dict(evaluation_only=True,sides=geometric,
    limitation='Spatial matches to layout after the run. Identified state alone does not prove the correct class; official scoring is sector/class counts, and classification must be assessed separately.'))

agents={}
for slot,item in analysis['agents'].items():
    ev=item['mission_evaluation']; m=ev['m3']
    exact=[s for s in m['scoring_sectors'] if all(r['difference']==0 for r in m['per_sector_class'] if r['sector']==s)]
    agents[slot]=dict(m3_reported=m['reported_total'],m3_expected=m['expected_total'],m3_missing=m['undercount_total'],
        m3_extra=m['overcount_total'],exact_sectors=exact,m2_confirmed_hit=ev['m2']['referee_confirmed_designated_hit'],
        m5_team_capture=ev['m5']['referee_confirmed_capture'],mode=item['mode'],error=item['error'],cleanup_errors=item['cleanup_errors'],
        independent_audit=item['independent_audit'],localization_error_m=item['localization_error_m'])
end=next((e['elapsed_s'] for e in analysis['physical_match']['chronology'] if e['type']=='match_finished'),None)
baseline_dir=root/'results/gazebo_3v3_coverage_uav_20261112'
comparison={}
if (baseline_dir/'summary.json').is_file():
    old=json.loads((baseline_dir/'summary.json').read_text())
    comparison=dict(basis='Single-run descriptive comparison; arena dimensions/calibration/layout changed; not controlled statistical evidence.',
        before_folder=baseline_dir.name,before_B2=old['agents']['B-2'],
        after_B2={k:v for k,v in agents['B-2'].items() if k.startswith('m3_') or k=='exact_sectors'})
save('comparison.json',comparison)
summary=dict(completed=report.get('completed'),active_slots=report.get('active_slots'),policies=report.get('policies'),models=report.get('models'),
    errors=report.get('errors'),agent_returncodes=report.get('agent_returncodes'),requested_s=report.get('duration_requested_s'),
    referee_finished_elapsed_s=end,coordinator_observed_finish_s=report.get('mission_finished_elapsed_s'),
    arena_size_m=spec['floor']['size'],layout_moves=layout.get('meta',{}).get('center_snap'),
    videos=videos,source=source,physical_match=analysis['physical_match'],agents=agents,B2_sectors=sectors_report,
    independent_audit_complete=analysis.get('independent_audit_complete'),independent_stall_ratio=analysis.get('independent_stall_ratio'),
    comparison=comparison,
    limitations=['Local rehearsal evidence, not an official competition score.',
                 'Do not sum overlapping reports from the three robots into a team M3 score.',
                 'M5 capture is alliance-level. Final ledger freezes after early match end.',
                 'UAV footprints are geometry; target identity requires ground camera evidence.',
                 'No UGV hit event means combat performance remains unvalidated.'])
save('summary.json',summary)
page=(out/'report.html').read_text().replace('wingbot_selfplay_third_person.mp4','uav_image_face_ledger.mp4')
page=page.replace('<video controls','<p>UAV 실제 시점 30fps. 오른쪽 장부는 B-2만의 지상 관측 결과입니다.</p><video controls')
(out/'report.html').write_text(page,encoding='utf8')
names={'AMMO_BOX':'탄약상자','DRUM':'드럼통','MILITARY_TRUCK':'군용트럭','MISSILE':'미사일','MORTAR':'박격포','TANK':'전차'}
def labels(counts): return ', '.join(f'{names.get(k,k)} {v}' for k,v in counts.items()) or '없음'
b=agents['B-2']
lines=['# 수정 경기장 3대3 UAV 녹화·B-2 M3 분석','',
'2026-11-12. 주행 영역 6.12×5.00m, 장애물 18개를 중앙 방향 20cm 격자로 이동한 기본 배치에서 B-1·B-2·B-3 대 R-1·R-2·R-3를 실행했다. 전원 같은 자율 코드, seed 0, CPU 렌더러다.',
'',f"실행 완료: {summary['completed']}. 심판의 종료 이벤트: {end}초. 녹화 요청/길이: 300초. 조기 종료 후 구간은 실제 종료 후 화면이며 면 장부는 FINAL로 고정한다.",
'', '## B-2 섹터별 마지막 장부','',
'| 섹터 | UAV 박스 | 식별 면이 있는 박스 | 식별 I | 빈 면 E | 미완료 | 대상 면 |',
'|---|---:|---:|---:|---:|---:|---:|']
for s,r in sectors_report.items():
    lines.append(f"| {s} | {r['boxes']} | {r['identified_boxes']} | {r['identified_faces']} | {r['empty_faces']} | {r['unresolved_faces']} | {r['eligible_faces']} |")
lines+=['','I는 M2/M3 그림의 식별 상태를 포함할 수 있다. E는 빈 면 확인이며 객체 식별 수에 더하지 않는다. UAV 박스 수는 상공에서 확인한 형상 수다.',
'', '## B-2 M3 최종 제출과 누락','',
f"최종 서버 수신 스냅샷은 {b['m3_reported']}/{b['m3_expected']}개, 종류·섹터별 누락 합계 {b['m3_missing']}개, 초과 합계 {b['m3_extra']}개다. 이는 로컬 배치 정답과 집계 비교이며 공식 점수나 개별 사진 식별 정확도가 아니다.",
'','| 섹터 | 제출/정답 | 누락 | 초과 | 완전 일치 |','|---|---:|---|---|---|']
for s,r in sectors_report.items():
    lines.append(f"| {s} | {r['m3_reported']}/{r['m3_expected']} | {labels(r['missing'])} | {labels(r['extra'])} | {'예' if r['exact'] else '아니오'} |")
lines+=['','미완료 면:']
for s,r in sectors_report.items():
    lines.append(f"- {s}: "+(', '.join(f"{f['key']} ({f['state']})" for f in r['remaining']) or '없음'))
lines+=['','배치 정답과 영상 장부의 위치를 사후 대응한 [면별 진단](m3_side_audit.json)은 원인 확인용이다. I 상태라도 종류가 틀릴 수 있으므로 집계 누락과 미관측 면을 같은 수로 간주하지 않는다.',
'', '## 6대의 M3 집계','',
'| 로봇 | 제출/정답 | 누락 | 초과 | 완전 일치 섹터 |','|---|---:|---:|---:|---|']
for slot,a in agents.items():
    lines.append(f"| {slot} | {a['m3_reported']}/{a['m3_expected']} | {a['m3_missing']} | {a['m3_extra']} | {', '.join(a['exact_sectors']) or '없음'} |")
lines+=['','세 로봇의 보고는 서로 중복될 수 있으므로 합산해 팀 점수라고 하지 않는다.',
'', '## 경기·주행 기록','']
for e in analysis['physical_match']['chronology']: lines.append('- '+json.dumps(e,ensure_ascii=False))
p=analysis['physical_match']
lines+=[f"- 발사 {p['shots_recorded']}회, 구조물 표적 명중 {len(p['target_hit_events'])}회, UGV 명중 {len(p['ugv_hit_events'])}회.",
f"- 독립 진척 검사 완료: {analysis.get('independent_audit_complete')}. 전체 정체 비율: {analysis.get('independent_stall_ratio')}. 정체는 충돌 횟수를 뜻하지 않는다.",
'', '## 영상 검증','']
for name,v in videos.items():
    lines.append(f"- [{name}.mp4]({name}.mp4): {v['fps']:g}fps, {v['duration_s']:g}초, 전체 {v['decoded_frames']}프레임 디코딩. 시간 정렬을 위한 이전 프레임 유지 {v['held_frames']}장, 누락 안내 {v['missing_cards']}장. 검증 통과: {v['checks_pass']}.")
lines+=[f"- 소스·설치 자산 {source['files_checked']}개 해시 확인. 모두 일치: {source['all_match']}.",
'- [실행 설정/전체 요약](summary.json), [영상 포함 보고서](report.html), [영상 검증](recording_validation.json), [섹터별 상세](sector_box_report.json).']
if comparison:
    old=comparison['before_B2']
    lines+=['','## 이전 3대3과 비교','',
f"이전 B-2는 {old['m3_reported']}/{old['m3_expected']}, 누락 {old['m3_missing']}·초과 {old['m3_extra']}개였다. 이번 B-2는 {b['m3_reported']}/{b['m3_expected']}, 누락 {b['m3_missing']}·초과 {b['m3_extra']}개다. 경기장 치수·보정·배치가 함께 바뀐 각 1회 실행이므로 일반적 성능 향상의 증거로 단정하지 않는다."]
(out/'RESULTS.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
print(json.dumps(dict(completed=summary['completed'],errors=summary['errors'],returncodes=summary['agent_returncodes'],referee_finished_s=end,
    video=videos,source=source,B2_sectors=sectors_report,agents={k:{q:v for q,v in a.items() if q not in ('independent_audit','localization_error_m')} for k,a in agents.items()},physical_match=p),ensure_ascii=False,indent=2))

