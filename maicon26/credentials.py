"""Read mission credentials without putting their values in argv or diagnostics."""
from __future__ import annotations

import getpass
import os
from pathlib import Path
import sys
import warnings

from .link import schema
from .link.secrets import register_secret


def read_mission_code(code_file=None, *, environ=None, prompt=None, warn=None):
    """MAICON_CODE, legacy environment, file, then at most three hidden prompts.

    Environment/file format mismatches warn and continue for field compatibility.
    Interactive input must match the documented format; the server receives the
    original letter case. No diagnostic contains the value or file contents.
    """
    env = os.environ if environ is None else environ
    warning = warn or (lambda text: print("[run_real] WARN " + text, file=sys.stderr, flush=True))
    source, raw = None, None
    for key in ("MAICON_CODE", "MAICON_MISSION_CODE"):
        if key in env:
            source, raw = "environment", env[key]
            break
    if source is None and code_file is not None:
        try:
            raw = Path(code_file).expanduser().read_text(encoding="utf-8-sig")
        except (OSError, UnicodeError):
            raise ValueError("임무코드 파일을 읽을 수 없습니다") from None
        source = "file"
    if source is not None:
        register_secret(raw)
        value = raw.strip()
        register_secret(value)
        if not schema.mission_code_valid(value):
            warning("환경변수/파일의 임무코드가 영문·숫자 8자리 형식과 다릅니다. 현장 서버 확인을 계속합니다")
        return value

    ask = prompt or getpass.getpass
    for attempt in range(3):
        try:
            # getpass's non-TTY fallback echoes input; fail before that fallback.
            with warnings.catch_warnings():
                warnings.simplefilter("error", getpass.GetPassWarning)
                raw = ask("임무코드 (화면에 표시되지 않음): ")
        except (EOFError, getpass.GetPassWarning):
            raise ValueError("숨김 입력을 사용할 수 없습니다. MAICON_CODE 또는 --code-file을 지정하세요") from None
        register_secret(raw)
        value = raw.strip()
        register_secret(value)
        if schema.mission_code_valid(value):
            return value
        warning(f"임무코드는 영문·숫자 8자리여야 합니다 ({attempt+1}/3)")
    raise ValueError("임무코드 입력 형식 확인에 세 번 실패했습니다")
