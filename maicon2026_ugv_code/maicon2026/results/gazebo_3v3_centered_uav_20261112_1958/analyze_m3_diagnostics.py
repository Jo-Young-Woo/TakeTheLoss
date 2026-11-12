from pathlib import Path
import json, math
from collections import Counter
out=Path(__file__).resolve().parent
def read(p): return json.loads((out/p).read_text(encoding='utf8'))
m=read('B-2/mission_report.json')
a=read('selfplay_analysis.json')['agents']['B-2']
audit=read('m3_side_audit.json')
trace=[json.loads(s) for s in (out/'B-2/agent_trace.jsonl').read_text().splitlines() if s.strip()]
arrival=m.get('m3_arrival_events',[])
selected=m.get('exploration_diagnostics',{}).get('events',[])
classified=m.get('observed_classified_faces',[])
diagnosed=[]
for side in audit['sides']:
    direction=side['key'].rsplit('-',1)[1]
    ground_key=[side.get('ground_box'),'ENWS'.index(direction)]
    obs=[f for f in classified if f.get('geometry_key')==ground_key]
    votes=Counter()
    for f in obs: votes.update(f.get('votes',{}))
    attempts=[e for e in arrival if e.get('primary_face')==ground_key]
    choices=[e for e in selected if e.get('primary_face')==ground_key]
    dominant=votes.most_common(1)[0][0] if votes else None
    expected=side['expected_class'].upper().replace('MILITARY_TRUCK','TRUCK')
    diagnosed.append(dict(**side,ground_key=ground_key,classified_votes=dict(votes),
        dominant_class=dominant,dominant_class_matches_truth=(dominant==expected if dominant else None),
        retained_arrival_events=attempts,retained_selection_events=choices,
        evidence_note='Retained diagnostics can be bounded; no entry does not prove that a viewpoint was never considered.'))
for d in diagnosed: d['ledger_state_transitions']=[]
for line in (out/'uav_image_face_ledger.jsonl').open(encoding='utf8'):
    frame=json.loads(line)
    for d in diagnosed:
        same=[f for f in frame['faces'] if f['face']==d['key'].rsplit('-',1)[1] and math.dist(f['center'],d['center'])<.05]
        if not same: continue
        state=same[0].get('ground_state','pending')
        history=d['ledger_state_transitions']
        if not history or state!=history[-1]['state']:
            history.append(dict(elapsed_s=frame['image_stamp_s']-m['match_start_sim_s'],state=state))
timeline=[]
previous=None
for r in trace:
    counts=r.get('m3_counts',{})
    if counts!=previous:
        timeline.append(dict(elapsed_s=r.get('elapsed'),counts=counts,mode=r.get('mode')))
        previous=counts
stats=m.get('agent_stats',{})
cov=m.get('coverage',{})
centres={b['bid']:b for b in cov.get('uav_arena_centres',[])}
alignment=cov.get('uav_alignment',{})
geometry=[]
for b in cov.get('cube_candidates',[]):
    if b['bid'] not in centres: continue
    aerial=centres[b['bid']]
    projected=[aerial['cx']+alignment.get('dx',0),aerial['cy']+alignment.get('dy',0)]
    geometry.append(dict(bid=b['bid'],ground_center=[b['cx'],b['cy']],
        raw_uav_center=[aerial['cx'],aerial['cy']],aligned_uav_center=projected,
        residual_m=math.dist([b['cx'],b['cy']],projected),source=b.get('source')))
summary=dict(observer='B-2',evaluation_only=True,
    state_time_s=stats.get('state_time'),mode_time_s=stats.get('mode_time'),
    total_classification_changes=len(timeline),
    final_local_counts=m.get('m3_counts'),
    accepted_m3=m.get('submitted_reports',{}).get('m3'),
    last_classification_change=timeline[-1] if timeline else None,
    arrival_outcomes=dict(Counter(e.get('outcome') for e in arrival)),
    arrival_view_reasons=dict(Counter(e.get('view_reason') for e in arrival)),
    exploration_final=m.get('exploration_diagnostics',{}),
    localization_error_m=a.get('localization_error_m'),
    independent_progress=a.get('independent_audit'),
    detector_errors=stats.get('detector_errors'),
    detector_stats=m.get('detector_stats'),
    registered_box_geometry=geometry,uav_alignment=alignment,
    confirmed_classified_face_count=len(classified),
    m3_sides=diagnosed,classification_timeline=timeline,
    limitation='Final accepted sector/class counts determine the aggregate result. Local per-face states and retained events diagnose coverage only; counts are not individual target recall.')
(out/'m3_diagnostics.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
note='''

## M3 누락 원인 분석

세 누락은 모두 CM 아래쪽 단독 큐브 U09에 대응한다. 실제 중심은 경기장 좌표 (3.06, 1.50)m, Gazebo 중심 좌표 (0.00, -1.00)m다.

| 면 | 실제 그림 | 최종 장부 | 기록에서 확인한 내용 |
|---|---|---|---|
| U09-E | 전차 | pending | 확정 분류 없음. 마지막 후보 생성에서도 해당 면의 관측 위치가 0개였다. |
| U09-N | 박격포 | empty | 약 7.1초에 viewed, 41.9초에 empty로 바뀌었다. 실제 빈 면이 아니다. |
| U09-S | 박격포 | empty | 약 268.0초에 viewed, 271.1초에 empty로 바뀌었다. 실제 빈 면이 아니다. |

주요 공간 오차도 확인했다. 같은 박스의 지상 모델 중심은 (2.616, 1.497)m이지만 UAV 원좌표는 (3.058, 1.497)m이다. 정렬값 (+0.013, -0.013)m을 적용한 기대 중심과 비교해도 잔차가 약 0.455m다. 전체 정렬 잔차 RMS 0.021m와 별개인 특정 박스의 위치 이상이다. 관측 계획은 실제 그림보다 왼쪽으로 치우친 모델을 사용했다. U09-S 접근 기록에는 입사각 불충족과 가림으로 인한 재접근도 있다.

현재 설정은 탐지 없이 3회 이상, 서로 다른 위치 2곳 이상에서 관측하면 EMPTY로 처리하고, 완료된 면의 일반 재탐지는 꺼져 있다. 따라서 탐지 실패가 빈 면 확정으로 이어져 재탐색 우선순위에서 빠질 수 있다. 남은 후보들은 274.39초에 시간 제한으로 거절됐고 약 275초에 거점 임무로 전환했다. 이 기록은 위치 이상과 빈 면 오판정을 입증하지만, 개별 카메라 프레임의 검출 실패 원인 전체를 확정하지는 않는다.

다음 개선 우선순위는 **UAV와 지상 박스 대응의 큰 위치 잔차를 거부하고 재연결하는 것**, 그리고 **정확한 면 위치·촬영 범위가 확인되지 않은 무검출 관측을 EMPTY로 확정하지 않는 것**이다. 이번에는 알고리즘을 바꾸거나 재경기를 추가하지 않았다.

I 상태는 M3 외 표적·장식의 분류도 포함할 수 있어 M3 정답 수와 같지 않다. 예를 들어 R-S3의 마지막 I는 5개이지만 M3 제출은 3개다. CM의 다른 미완료 면 U11-E는 장식 그림이므로 M3 누락에 포함하지 않았다.

- [면별 상태 변화·관측 시도·위치 잔차](m3_diagnostics.json)
- [분석 도구 호환 보정 내역](analysis_adapter.json): 기록된 config.arena를 읽도록 결과 폴더 안에서만 분석 사본을 보정했다. 경기 코드·설치 자산은 변경하지 않았다.
'''
report_path=out/'RESULTS.md'
report_text=report_path.read_text(encoding='utf8').split('\n## M3 누락 원인 분석')[0]
report_path.write_text(report_text.rstrip()+note+'\n',encoding='utf8')
page_path=out/'report.html'
page=page_path.read_text(encoding='utf8')
if '<!-- B2 M3 diagnosis -->' not in page:
    panel='''<!-- B2 M3 diagnosis --><h2>B-2 최종 M3 결과</h2>
    <p>R-S1 6/6 · R-S2 3/3 · R-S3 3/3 · CM 6/9. 합계 18/21, 누락 3개, 초과 0개. 공식 점수가 아닌 로컬 집계 비교입니다.</p>
    <p>누락은 CM 아래 U09의 전차 1면과 박격포 2면입니다. 전차는 pending, 박격포들은 empty로 잘못 처리됐습니다. 지상 박스와 UAV 위치 사이에는 정렬 보정 후에도 약 45.5cm 잔차가 있습니다.</p>
    <p>BLUE의 R-2 점령으로 282.64초에 경기 종료. 영상은 종료 후 장면을 포함한 300초, 30fps입니다. <a href='RESULTS.md'>전체 결과·원인 분석</a> · <a href='m3_diagnostics.json'>면별 근거</a></p>'''
    page=page.replace('<video controls',panel+'<video controls')
    page_path.write_text(page,encoding='utf8')
compact=dict(state_time_s=summary['state_time_s'],arrival_outcomes=summary['arrival_outcomes'],arrival_view_reasons=summary['arrival_view_reasons'],
    localization_error_m=summary['localization_error_m'],last_classification_change=summary['last_classification_change'],
    final_exploration=m.get('exploration_diagnostics',{}).get('last_evaluate'),
    unresolved_M3=[{k:v for k,v in d.items() if k not in ('retained_arrival_events','retained_selection_events')} for d in diagnosed if not d['identified']],
    mapped_class_mismatches=[d['key'] for d in diagnosed if d['dominant_class_matches_truth'] is False])
print(json.dumps(compact,ensure_ascii=False,indent=2))

