"""임무 어휘. 사전교육 자료 p12~16에 나온 명칭을 영문 canonical key로 정리한다.

서버 보고값은 참가팀 단계별 API 안내 2026-11-04 스냅숏 표를 따른다.
M1/M3/M6 JSON 형식 자체는 안내서의 임시 형식이며 현장 확정본과 대조해야 한다.
"""
from __future__ import annotations

from typing import Dict, List

# 지정표적(M2) — 시설 3종
TARGETS: Dict[str, List[str]] = {
    "GUARD_POST": ["감시초소", "감시 초소", "초소", "감시소"],
    "RADAR_FACILITY": ["레이더시설", "레이더 시설", "레이더기지", "레이다시설", "레이다 시설", "레이더", "레이다"],
    "COMM_FACILITY": ["통신시설", "통신 시설", "통신소", "통신기지"],
}

# 전장 객체(M3) — 6종
OBJECTS: Dict[str, List[str]] = {
    "AMMO_BOX": ["탄약상자", "탄약 상자", "탄약고"],
    "DRUM": ["드럼통", "드럼"],
    "MORTAR": ["박격포"],
    "TRUCK": ["군용트럭", "군용 트럭", "트럭"],
    "TANK": ["전차", "탱크"],
    "MISSILE": ["미사일"],
}

# 목표거점(M5) — 3종
BASES: Dict[str, List[str]] = {
    "COMMAND_POST": ["작전지휘소", "작전 지휘소", "지휘소"],
    "MAINT_DEPOT": ["정비소", "정비창"],
    "SUPPLY_BASE": ["군수보급소", "군수 보급소", "보급소", "보급기지"],
}

# 섹터 별칭(명령문에 어떤 형태로 나올지 모르므로 폭넓게). 값은 '제출 문자열'.
NATO = ["ALPHA", "BRAVO", "CHARLIE", "DELTA", "ECHO", "FOXTROT"]
NATO_KR = {"알파": "ALPHA", "브라보": "BRAVO", "찰리": "CHARLIE", "델타": "DELTA",
           "에코": "ECHO", "폭스트롯": "FOXTROT"}

DISPLAY_KR = {
    **{k: v[0] for k, v in TARGETS.items()},
    **{k: v[0] for k, v in OBJECTS.items()},
    **{k: v[0] for k, v in BASES.items()},
}

UGV_CLASS = "UGV"
BASE_SIGN_CLASSES = tuple(BASES.keys())  # 거점 표지 이미지 인식용 클래스명(사전 제공 이미지)

# Canonical internal names -> facility suffix or literal object wire value.
# Facility prefix is the camp where it stands, not the reporting team's camp.
TARGET_WIRE = {"GUARD_POST": "OUTPOST", "RADAR_FACILITY": "RADAR", "COMM_FACILITY": "COMMS"}
BASE_WIRE = {"COMMAND_POST": "COMMAND_POST", "MAINT_DEPOT": "MAINTENANCE", "SUPPLY_BASE": "SUPPLY_DEPOT"}
OBJECT_WIRE = {"AMMO_BOX": "ammo_box", "DRUM": "drum", "MORTAR": "mortar",
               "TRUCK": "military_truck", "TANK": "tank", "MISSILE": "missile"}
WIRE_SECTORS = ("B-S1", "B-S2", "B-S3", "CM", "R-S1", "R-S2", "R-S3")
SECTOR_WIRE = {**{s: s for s in WIRE_SECTORS if s != "CM"}, "COMMON": "CM"}
FACILITY_WIRE_VALUES = tuple(f"{camp}_{suffix}" for camp in ("BLUE", "RED")
                             for suffix in (*TARGET_WIRE.values(), *BASE_WIRE.values()))
