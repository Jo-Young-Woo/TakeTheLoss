#!/usr/bin/env python3
"""대회장 현장 실험 도구 (스켈레톤 — 실기에서 실행해 본 적 없음; 시뮬 드라이런으로 코드만 확인).

  python scripts/field_exp.py list                              실험 목록과 소요 시간
  python scripts/field_exp.py run F01 --robot sim               시뮬 드라이런(코드 확인용 — 수치는 현장 값이 아니다)
  python scripts/field_exp.py run F01 --robot ros               실기(ROS2 RosWingbot), 대화형
  python scripts/field_exp.py run F10 --robot ros --allow-laser 레이저 자동 발사 허용(눈 보호!)
  python scripts/field_exp.py suggest field/records/<날짜>/F02_*.json     제안값만 보기
  python scripts/field_exp.py apply   field/records/<날짜>/F02_*.json     제안값을 보고 y/N 로 config/field_overrides.json 에 반영
  python scripts/field_exp.py status                            튜너블 상태(가정/시뮬 튜닝/실측) 집계와 실측으로 바뀐 값
  python scripts/field_exp.py revert map3d.nav_thr              오버라이드를 지워 기본값으로
  python scripts/field_exp.py checklist --write                 field/FIELD_CHECKLIST.md 생성
  python scripts/field_exp.py replay-depth <capture 폴더> --start 1.0 2.5 0 [--set map3d.nav_thr=2.4]   F08 로 저장한 깊이를 설정을 바꿔 다시 돌려 보기
  python scripts/field_exp.py zero-import ~/zeroing/laser_zero.yaml [--distances 0.6 1.0 1.5 2.0] [--apply]   텔레옵 영점 결과 → aim.laser_* 4개(기록 + 선택 반영)

실기 연결: `--robot ros` = RosWingbot(2차 사전교육 ROS2 API로 구현, 로봇에서 미검증). `--robot wingbot` 은 전송 계층이 없는 기본형이라 종료 코드 3.
"""
import argparse, glob, json, math, os, sys, datetime
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.config import Config
from maicon26.field import overrides as ov
from maicon26.field import tunables as tun
from maicon26.field.experiments import EXPERIMENTS, NeedInput, execute, replay_depth
from maicon26.field.runner import ORDER, checklist_markdown, make_real_ctx, make_sim_ctx

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REC_ROOT = os.path.join(ROOT, "field", "records")


def _cfg(a) -> Config:
    return ov.load_config(getattr(a, "config", None), None if getattr(a, "no_overrides", False) else a.overrides)


def cmd_list(a):
    print(f"{'ID':4s} {'소요':>5s}  제목")
    for eid in ORDER:
        e = EXPERIMENTS[eid]
        print(f"{eid:4s} {e.minutes:>3d}분  {e.title}\n       필요: {', '.join(e.needs) or '-'}")


def cmd_run(a):
    exp = EXPERIMENTS[a.exp.upper()]
    cfg = _cfg(a)
    out = os.path.join(a.out or REC_ROOT, datetime.date.today().isoformat())
    if a.robot == "sim":
        ctx = make_sim_ctx(out, seed=a.seed, cfg=cfg)
        print("!! 시뮬 드라이런: 코드가 도는지만 본다. 이 수치를 설정에 적용하지 말 것.")
    else:
        try:
            if a.robot == "ros":
                from maicon26.hal.wingbot import RosWingbot
                robot = RosWingbot(cfg)
            else:
                from maicon26.hal.wingbot import WingbotRobot
                robot = WingbotRobot(cfg)
        except NotImplementedError as e:
            print(f"WingbotRobot 는 전송 계층이 없는 기본형(스텁)이라 단독으로 못 쓴다: {e}\n  → 로봇에서는 --robot ros (RosWingbot = WingbotRobot + ROS2 전송). 로봇 없이 코드만 보려면 --robot sim.")
            sys.exit(3)
        except ImportError as e:
            print(f"ROS2(rclpy 등)를 불러오지 못했다: {e}\n  → 로봇의 ROS2 환경(source /opt/ros/humble/setup.bash)에서 실행할 것.")
            sys.exit(3)
        answers = json.load(open(a.answers, encoding="utf-8")) if a.answers else None
        ctx = make_real_ctx(robot, cfg, out, allow_laser=a.allow_laser, answers=answers, interactive=not a.yes)
    try:
        execute(exp, ctx, note=a.note or "")
    except NeedInput as e:
        print(f"입력이 필요한 질문이 있다: {e} (대화형으로 실행하거나 --answers 파일에 넣을 것)")
        sys.exit(2)
    finally:
        if a.robot != "sim":
            try:
                ctx.robot.close()
            except Exception:
                pass


def _load_rec(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def cmd_suggest(a):
    for p in _expand(a.records):
        r = _load_rec(p)
        print(f"{r['id']} {r['date']} {r['time']} [{r['robot']}] 판정 {r['analysis']['verdict']}  {p}")
        if r.get("WARNING"):
            print("   !!", r["WARNING"])
        for s in r["analysis"]["suggestions"]:
            print(f"   {s['path']} = {s['value']}   # {s['note']}")


def cmd_apply(a):
    cfg = Config.load(a.config)
    path = a.overrides
    data = ov.load(path)
    n = 0
    for p in _expand(a.records):
        r = _load_rec(p)
        if r.get("robot") == "sim" and not a.force_sim:
            print(f"건너뜀(시뮬 기록): {p}  — 정말 쓰려면 --force-sim")
            continue
        print(f"\n[{r['id']}] 판정 {r['analysis']['verdict']}  {p}")
        for s in r["analysis"]["suggestions"]:
            if a.only and s["path"] not in a.only:
                continue
            cur = tun.resolve(cfg, s["path"])
            print(f"  {s['path']}: {cur} → {s['value']}   # {s['note']}")
            ok = a.yes or input("  반영? [y/N] ").strip().lower() == "y"
            if ok:
                ov.record(data, cfg, s["path"], s["value"], r["id"], s["note"], os.path.relpath(p, ROOT))
                n += 1
    if n:
        print(f"\n{n}개 반영 → {ov.save(data, path)}")
    else:
        print("\n반영한 것 없음")


def cmd_status(a):
    cfg = Config.load(a.config)
    data = ov.load(a.overrides)
    prov = data["provenance"]
    c = tun.summary(cfg, prov)
    print(f"튜너블 {len(tun.TUNABLES)}개: PDF {c['PDF']} · ASSUMED {c['ASSUMED']} · SIM_TUNED {c['SIM_TUNED']} · MEASURED {c['MEASURED']}")
    if prov:
        print("\n실측으로 갱신된 값:")
        for p, pv in prov.items():
            print(f"  {p} = {data['values'][p]}   ({pv['exp']} {pv['date']}) {pv.get('note', '')}")
    else:
        print("\n아직 실측으로 갱신된 값이 없다.")
    if a.todo:
        print("\n먼저 확인할 것(우선순위 1, 아직 실측 아님):")
        for t in sorted(tun.TUNABLES, key=lambda t: (ORDER.index(t.exp) if t.exp in ORDER else 99, t.path)):
            if t.prio == 1 and t.path not in prov:
                print(f"  [{t.status:9s}] {t.exp:18s} {t.path} = {tun.resolve(cfg, t.path)}")


def cmd_revert(a):
    data = ov.load(a.overrides)
    print("되돌림" if ov.revert(data, a.path) else "오버라이드에 없음")
    ov.save(data, a.overrides)


def cmd_checklist(a):
    cfg = Config.load(a.config)
    md = checklist_markdown(cfg, ov.load(a.overrides))
    if a.write:
        p = os.path.join(ROOT, "field", "FIELD_CHECKLIST.md")
        os.makedirs(os.path.dirname(p), exist_ok=True)
        open(p, "w", encoding="utf-8", newline="\n").write(md)
        print("쓴 파일:", p)
    else:
        print(md)


def cmd_replay(a):
    cfg = _cfg(a)
    sets = {}
    for kv in a.set or []:
        k, v = kv.split("=", 1)
        sets[k] = json.loads(v)
    rep = replay_depth(a.dir, cfg, tuple(a.start[:2]) + (float(a.start[2]) * 3.141592653589793 / 180.0,), sets)
    print(json.dumps(rep, ensure_ascii=False, indent=2, default=str))


def cmd_zero_import(a):
    """텔레옵 영점(laser_zero.yaml) → aim.laser_lateral_m/vertical_m/laser_offset_pan_deg/laser_offset_tilt_deg.
    각도 변환은 지금 설정의 카메라(K 또는 화각)로 한다. 기록은 field/records/<날짜>/ZERO_<시각>.json, 반영은 --apply(y/N, --yes 면 묻지 않음)."""
    from maicon26.field import laser_zero as lz
    cfg = _cfg(a)
    try:
        data = lz.load_zero(a.file)
        cam = lz.camera_for(cfg)
        hf = 2.0 * math.degrees(math.atan2(cam.w / 2.0, cam.f))
        res = lz.convert(data, cam, distances=a.distances, axis=a.axis, runtime_hfov_deg=hf)
    except (lz.ZeroFormatError, OSError, ValueError) as ex:
        print(f"변환 실패: {ex}")
        sys.exit(2)
    print(f"영점 {a.file}: {res.n_samples}점 ({res.source}), 거리 {res.d_range[0]:.2f}–{res.d_range[1]:.2f} m, 축각 {res.axis_mode}, 판정 {res.verdict}")
    print(f"  모델 대비 오차 rms {res.rms_px:.2f} px / 최대 {res.max_px:.2f} px   (런타임 카메라 f={cam.f:.1f}px, 화각 {hf:.1f}°)")
    for p in lz.PATHS:
        print(f"  {p}: {tun.resolve(cfg, p)} → {res.values[p]}")
    for w in res.warnings:
        print("  !!" if w.startswith("!") else "  ※", w.lstrip("! "))
    for n in res.notes:
        print("  ※", n)
    rec = lz.to_record(res, cfg, os.path.abspath(a.file), robot=a.robot, note=a.note)
    d = a.out or os.path.join(REC_ROOT, rec["date"])
    os.makedirs(d, exist_ok=True)
    out = os.path.join(d, f"ZERO_{rec['time'].replace(':', '')}.json")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(rec, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print("  기록:", out)
    if a.apply:
        cmd_apply(argparse.Namespace(config=a.config, overrides=a.overrides, records=[out], yes=a.yes, only=None, force_sim=a.force_sim))
    else:
        print("  반영하려면 --apply 를 붙이거나:  python scripts/field_exp.py apply", os.path.relpath(out, ROOT) if not a.out else out)


def _expand(pats):
    out = []
    for p in pats:
        out += sorted(glob.glob(p)) or [p]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="기본 설정(json/yaml). 없으면 코드 기본값")
    ap.add_argument("--overrides", default=ov.DEFAULT_PATH, help="현장 오버라이드 파일")
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("list").set_defaults(fn=cmd_list)
    r = sp.add_parser("run")
    r.add_argument("exp")
    r.add_argument("--robot", choices=["sim", "ros", "wingbot"], default="sim")
    r.add_argument("--seed", type=int, default=1)
    r.add_argument("--out", default=None)
    r.add_argument("--note", default="")
    r.add_argument("--answers", default=None, help="미리 채운 답(JSON). 키는 질문 키")
    r.add_argument("--yes", action="store_true", help="대화형 질문 없이 기본값/answers 로(안내만 출력, Enter 대기 없음)")
    r.add_argument("--allow-laser", action="store_true", help="F10 이 레이저를 자동 발사하도록 허용(눈 보호 확인 후)")
    r.add_argument("--no-overrides", action="store_true")
    r.set_defaults(fn=cmd_run)
    s = sp.add_parser("suggest"); s.add_argument("records", nargs="+"); s.set_defaults(fn=cmd_suggest)
    p = sp.add_parser("apply"); p.add_argument("records", nargs="+"); p.add_argument("--yes", action="store_true"); p.add_argument("--only", nargs="*")
    p.add_argument("--force-sim", action="store_true"); p.set_defaults(fn=cmd_apply)
    st = sp.add_parser("status"); st.add_argument("--todo", action="store_true"); st.set_defaults(fn=cmd_status)
    rv = sp.add_parser("revert"); rv.add_argument("path"); rv.set_defaults(fn=cmd_revert)
    ck = sp.add_parser("checklist"); ck.add_argument("--write", action="store_true"); ck.set_defaults(fn=cmd_checklist)
    rp = sp.add_parser("replay-depth"); rp.add_argument("dir"); rp.add_argument("--start", nargs=3, type=float, default=[1.0, 2.5, 0.0], metavar=("X", "Y", "TH_DEG"))
    rp.add_argument("--set", nargs="*", help="경로=JSON값 (예: map3d.nav_thr=2.4)"); rp.set_defaults(fn=cmd_replay)
    z = sp.add_parser("zero-import", help="텔레옵 영점 결과(laser_zero.yaml)를 aim.laser_* 4개로 변환")
    z.add_argument("file")
    z.add_argument("--distances", nargs="*", type=float, default=None, help="식(fit)만 있는 파일일 때 영점을 잰 거리들(m). 없으면 0.6 1.0 1.5 2.0")
    z.add_argument("--axis", choices=["auto", "free", "parallel"], default="auto", help="축각을 맞출지: auto=거리 3곳·폭 0.5 m 이상이면 맞춤, parallel=0 으로 고정(Gazebo 와 같은 모델)")
    z.add_argument("--robot", choices=["real", "sim"], default="real", help="sim 이면 apply 가 거부한다(--force-sim 필요)")
    z.add_argument("--note", default="")
    z.add_argument("--out", default=None, help="기록을 둘 폴더(기본 field/records/<날짜>)")
    z.add_argument("--apply", action="store_true", help="변환 결과를 config/field_overrides.json 에 반영(각 값 y/N)")
    z.add_argument("--yes", action="store_true")
    z.add_argument("--force-sim", action="store_true")
    z.set_defaults(fn=cmd_zero_import)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
