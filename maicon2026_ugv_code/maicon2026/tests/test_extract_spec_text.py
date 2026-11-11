import hashlib
from pathlib import Path

import pytest

from scripts.extract_spec_text import extract_html


def test_html_extraction_preserves_tables_korean_and_inline_text_without_active_content():
    source = '<style>secretcss</style><script>secretjs</script><svg><text>drawing</text></svg><h1>안내서</h1><p>진행 <b>중</b>만 &amp; 409</p><table><tr><th>값</th><th>뜻</th></tr><tr><td>ready</td><td>준비<br>단계</td></tr></table>'
    assert extract_html(source) == '안내서\n진행 중만 & 409\n| 값 | 뜻 |\n| ready | 준비 단계 |\n'


def test_preformatted_request_lines_remain_separate():
    assert extract_html('<pre>GET /api/report/me\nHost: &lt;서버IP&gt;</pre>') == 'GET /api/report/me\nHost: <서버IP>\n'


@pytest.mark.spec("S0.12", "S0.13")
def test_packaged_snapshot_provenance_and_extraction_preserve_source_authority():
    root = Path(__file__).resolve().parents[1]
    raw = (root/"docs/source/MAICON2026_game-server-api_p0-p12.html").read_bytes()
    assert len(raw) == 115114
    assert hashlib.sha256(raw).hexdigest() == "834c7a098a8fbc6b6b528840f8d8b81b194604aa443f05081106e3b9d2699c88"
    extracted = extract_html(raw.decode("utf-8-sig"))
    stored = (root/"docs/API_SOURCE_EXTRACT.txt").read_text(encoding="utf-8")
    assert stored.split("\n\n", 1)[1] == extracted
    assert "예시의 팀 이름·토큰·시각은 꾸민 값입니다" in extracted
    assert "서버 주소·임무코드는 현장에서 따로 안내합니다" in extracted
    assert "경기 규칙의 원본은 경기 룰북" in extracted
    assert "SERVER ↔ UGV 통신 규약(배포본)" in extracted
