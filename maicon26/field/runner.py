"""현장 실험 실행 환경(시뮬 드라이런 / 실기)과 체크리스트 생성.

make_sim_ctx   시뮬 로봇으로 '실험 코드가 도는지' 확인하는 환경. 환경 배치(블록·벽·박스 치우기)와 줄자 답을 자동으로 채운다.
               !! 여기서 나온 수치는 현장 값이 아니다(기록에 WARNING 이 붙는다).
make_real_ctx  실제 로봇(hal/wingbot.py)에 붙는 환경. 대화형으로 사람에게 안내하고 Enter 를 기다린다.
checklist_markdown  튜너블 레지스트리 + 실험 절차 → field/FIELD_CHECKLIST.md (현재값·상태·근거·합격 기준·체크칸).
"""
from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional

from ..config import Config
from ..hal.base import Robot
from . import tunables as tun
from .experiments import EXPERIMENTS, Ctx

ORDER = ["F09", "F01", "F02", "F04", "F03", "F06", "F05", "F07", "F10", "F08"]     # 현장 실험 권장 순서(앞 것이 뒤 것의 전제)


# ====================================================================== 시뮬 드라이런
def make_sim_ctx(out_dir: str, seed: int = 1, cfg: Optional[Config] = None, say: Callable[[str], None] = print,
                 fast: bool = True) -> Ctx:
    from ..sim.robot import SimRobot
    from ..sim.world import World
    cfg = cfg or Config()
    w = World(cfg, seed=seed)
    u = w.add_ugv("BLUE", 2)
    u.pose.th = 0.0
    robot = SimRobot(w, u)

    def teleport(x: float, y: float, th: float = 0.0) -> None:
        u.pose.x, u.pose.y, u.pose.th = x, y, th
        u.v = u.w = 0.0
        u.cmd_v = u.cmd_w = 0.0

    def clear_corridor(ctx: Ctx) -> None:
        sx, sy = u.pose.x, u.pose.y
        w.cubes = [c for c in w.cubes if not (sx - 0.3 < c.cx < sx + 5.0 and abs(c.cy - sy) < 0.8)]

    def place_block(ctx: Ctx) -> None:
        d = float(ctx.given.get("block_dist", 2.5))
        sx, sy = ctx.given.get("start_x", u.pose.x), ctx.given.get("start_y", u.pose.y)
        clear_corridor(ctx)
        w.add_low_obstacle(sx + d, sy - 0.25, sx + d + 0.2, sy + 0.25, float(ctx.given.get("block_h_cm", 12.0)) / 100.0)

    def front_clear(ctx: Ctx) -> None:
        w.cubes.clear()
        teleport(1.0, 2.5)

    def block_bumper(ctx: Ctx) -> None:
        w.add_low_obstacle(u.pose.x + 0.13, u.pose.y - 0.3, u.pose.x + 0.6, u.pose.y + 0.3, 0.10)

    def wall_at(ctx: Ctx) -> None:
        d = float(ctx.given.get("wall_d", 1.0))
        w.cubes.clear()
        teleport(cfg.arena.W - d - cfg.rig.depth_xyz[0], cfg.arena.H / 2)

    def empty_floor(ctx: Ctx) -> None:
        clear_corridor(ctx)

    hooks = {"place_block": place_block, "front_clear": front_clear, "block_bumper": block_bumper, "wall_at": wall_at, "empty_floor": empty_floor}
    answers: Dict[str, Any] = {
        "start_x": lambda c: u.pose.x, "start_y": lambda c: u.pose.y, "start_th_deg": lambda c: math.degrees(u.pose.th),
        "duration_s": 6.0 if fast else 20.0, "frames": 8, "distances": "0.8,1.5,2.5", "block_dist": 2.2, "block_h_cm": 12.0,
        "tag": "dryrun", "max_depth": 20,
        # F10: 시뮬의 숨은 정답(레이저 발광부 오프셋)을 '읽은 점 위치'로 돌려준다
        "left_cm1": lambda c: 100 * cfg.laser.sim_origin_lateral_m, "left_cm2": lambda c: 100 * cfg.laser.sim_origin_lateral_m,
        "up_cm1": lambda c: 100 * cfg.laser.sim_origin_vertical_m, "up_cm2": lambda c: 100 * cfg.laser.sim_origin_vertical_m,
        # F09: 몇 개만 채운다(나머지는 빈 칸 = 건너뜀)
        "m_rig.depth_xyz": "0.10 0 0.10", "m_arena.W": "6.12", "m_map3d.target_window_z": "0.335",
    }
    ctx = Ctx(robot, cfg, out_dir, interactive=False, answers=answers, hooks=hooks, advance=lambda dt: w.step(dt), say=say, robot_kind="sim")
    ctx.sim_world = w                                                 # type: ignore[attr-defined]
    return ctx


# ====================================================================== 실기
def make_real_ctx(robot: Robot, cfg: Config, out_dir: str, allow_laser: bool = False, answers: Optional[Dict[str, Any]] = None,
                  interactive: bool = True) -> Ctx:
    return Ctx(robot, cfg, out_dir, interactive=interactive, answers=answers, allow_laser=allow_laser, robot_kind="real")


# ====================================================================== 체크리스트
def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:g}"
    if isinstance(v, (tuple, list)):
        return "(" + ", ".join(_fmt(x) for x in v) + ")"
    return str(v)


def checklist_markdown(cfg: Optional[Config] = None, overrides: Optional[Dict[str, Any]] = None) -> str:
    cfg = cfg or Config()
    prov = (overrides or {}).get("provenance", {})
    ov = (overrides or {}).get("values", {})
    cnt = tun.summary(cfg, prov)
    L: List[str] = []
    A = L.append
    A("# 현장 실험 체크리스트 (자동 생성 — 직접 고치지 말 것)")
    A("")
    A("생성: `python scripts/field_exp.py checklist --write` · 기준: `maicon26/field/tunables.py` + `config/field_overrides.json`")
    A("")
    A("> **이 표의 '현재값'은 코드 기본값이거나 시뮬에서 정한 값이다. 대회장 실측값이 아니다.** `MEASURED` 가 된 것만 현장에서 갱신된 값이다.")
    A("")
    A(f"상태 집계: PDF {cnt['PDF']} · ASSUMED(가정) {cnt['ASSUMED']} · SIM_TUNED(시뮬 튜닝) {cnt['SIM_TUNED']} · MEASURED(실측) {cnt['MEASURED']}")
    A("")
    A("## 권장 순서")
    A("")
    A("| 순서 | 실험 | 소요 | 하는 일 | 갱신 대상 수 |")
    A("|---|---|---|---|---|")
    for i, eid in enumerate(ORDER, 1):
        e = EXPERIMENTS[eid]
        A(f"| {i} | **{eid}** | {e.minutes} 분 | {e.title} | {sum(1 for t in tun.TUNABLES if t.exp == eid)} |")
    A("")
    A("실험 코드가 없는 항목(`MANUAL`)과 기존 보정 도구 항목(`calib:*`)은 아래 '실험 코드 없음' 절에 있다.")
    A("")
    A("## 사용 순서(요약)")
    A("")
    A("```bash")
    A("python scripts/field_exp.py list                         # 실험 목록")
    A("python scripts/field_exp.py run F09 --robot ros          # 실행(대화형). 기록: field/records/<날짜>/F09_*.json")
    A("python scripts/field_exp.py apply field/records/<날짜>/F09_*.json     # 제안값을 보고 y/N 로 config/field_overrides.json 에 반영")
    A("python scripts/field_exp.py status                       # 무엇이 실측으로 바뀌었나")
    A("python scripts/run_real.py ... --overrides config/field_overrides.json   # 반영된 값으로 실행")
    A("```")
    A("")
    for eid in ORDER:
        e = EXPERIMENTS[eid]
        rows = sorted([t for t in tun.TUNABLES if t.exp == eid], key=lambda t: (t.prio, t.path))
        A(f"## {eid} · {e.title}")
        A("")
        A(f"소요 약 {e.minutes} 분 · 필요: {', '.join(e.needs) or '-'}")
        A("")
        for s in e.procedure:
            A(f"- {s}")
        if e.safety:
            A(f"- **안전:** {e.safety}")
        A("")
        if rows:
            _table(L, rows, cfg, prov, ov)
        else:
            A("(이 실험이 직접 갱신 제안하는 레지스트리 항목 없음 — 기록 JSON 의 suggestions 를 본다)")
        A("")
        A("측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL")
        A("")
    other = sorted([t for t in tun.TUNABLES if t.exp not in ORDER], key=lambda t: (t.exp, t.prio, t.path))
    A("## 실험 코드 없음 (줄자·눈·ros2 명령 / 기존 `calib` 도구)")
    A("")
    _table(L, other, cfg, prov, ov, with_exp=True)
    A("")
    sim = [t for t in tun.TUNABLES if t.status == tun.SIM_TUNED and t.path not in prov]
    A("## 시뮬에서 정한 값 모음 (실기에서 다시 확인할 것)")
    A("")
    A("시뮬의 잡음·허상·키네매틱 서보 가정에 맞춰 정한 값이다. 근거를 적어 두었으니 현장 결과와 비교해 갱신한다.")
    A("")
    A("| 경로 | 현재값 | 실험 | 정한 근거 |")
    A("|---|---|---|---|")
    for t in sim:
        A(f"| `{t.path}` | {_fmt(tun.resolve(cfg, t.path))} | {t.exp} | {t.basis} |")
    A("")
    miss = _unlisted_check(cfg)
    if miss:
        A("## 레지스트리에 없는 설정 경로")
        A("")
        A("`Config` 에는 있지만 튜너블 표에 없는 값(= 현장 확인 대상으로 분류하지 않은 값): " + ", ".join(f"`{m}`" for m in miss[:60]) + (" …" if len(miss) > 60 else ""))
        A("")
    return "\n".join(L) + "\n"


def _table(L: List[str], rows: List[tun.Tunable], cfg: Config, prov: Dict[str, Any], ov: Dict[str, Any], with_exp: bool = False) -> None:
    head = "| ☐ | 우선 | 경로 | 현재값 | 상태 | " + ("실험 | " if with_exp else "") + "지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |"
    L.append(head)
    L.append("|---|---|---|---|---|" + ("---|" if with_exp else "") + "---|---|---|")
    for t in rows:
        cur = tun.resolve(cfg, t.path)
        st = tun.status_of(t.path, prov)
        shown = _fmt(ov[t.path]) if t.path in ov else _fmt(cur)
        unit = f" {t.unit}" if t.unit else ""
        L.append(f"| ☐ | {'★' * (4 - t.prio) if t.prio < 3 else '·'} | `{t.path}` | {shown}{unit} | {st} | " + (f"{t.exp} | " if with_exp else "")
                 + f"{t.basis} | {t.how} | {t.pass_if} |")


def _unlisted_check(cfg: Config) -> List[str]:
    import dataclasses
    listed = {t.path for t in tun.TUNABLES}
    out: List[str] = []

    def walk(obj: Any, prefix: str) -> None:
        for f in dataclasses.fields(obj):
            v = getattr(obj, f.name)
            p = f"{prefix}{f.name}"
            if dataclasses.is_dataclass(v):
                walk(v, p + ".")
            else:
                out.append(p)
    walk(cfg, "")
    skip = ("team", "slot", "loop_hz", "arena.")
    return [p for p in out if p not in listed and not p.startswith(skip) and not p.startswith(("depth.sim_mount", "laser.sim_", "perception.sim_"))]
