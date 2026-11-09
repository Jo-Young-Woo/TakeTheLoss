#!/usr/bin/env python3
"""자연어 전술명령 → M1 JSON 규칙 파서 확인. 예: python scripts/check_command.py "찰리와 델타 구역의 레이더 시설을 타격하고 군수보급소로 진입하라" """
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.mission.command import CommandParser

if __name__ == "__main__":
    text = " ".join(sys.argv[1:]) or input("명령> ")
    c = CommandParser().parse(text)
    print(c.to_payload(), "confidence=%.2f" % c.confidence, c.notes)
