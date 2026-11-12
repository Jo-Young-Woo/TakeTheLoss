# API 안내서 요구사항 추적표

기준은 `docs/source/MAICON2026_game-server-api_p0-p12.html`의 2026-11-04 스냅숏이다.
원본 115114 bytes, SHA256 `834c7a098a8fbc6b6b528840f8d8b81b194604aa443f05081106e3b9d2699c88`.
사용자 작업지시의 97개 S-ID를 원본 603줄 추출문과 SVG 원문에 대조했다.
아래 **구현 70 / 규칙 27 / 해당없음 0**은 코드와 회귀 항목의 연결 상태이며, 전체 테스트 통과나 실서버 검증을 뜻하지 않는다.

2026-11-09 전체 회귀는 사용자 요청으로 중단됐고 이후 새 경기장 재구성 작업이 시작됐다.
전체 및 ZIP 내부 재검증은 아직 완료되지 않았다. 개별 실행 결과는 API_WORKLOG와 최종 실행 로그를 따른다.
요청된 테스트는 책임별 `test_spec_schema.py`, `test_spec_transport.py`, `test_spec_runtime.py`, `test_mockserver_spec.py`, `test_spec_game_rules.py`로 분리했다.
요청 이름 `test_spec_conformance.py`의 Agent→실제 HttpServerLink→MockServer 통합 G6 사례 6개는 선택 검증을 통과했다.
STOP→M1/M3 신규 POST 취소·1초 이내 M6를 실제 HTTP 및 비동기 큐 경로에서 검증한다.

코드 위치는 저장소 상대 경로와 함수/클래스/상수이며 테스트는 실제 spec 마커가 있는 함수다.
`python scripts/check_traceability.py`는 97행·참조 존재·마커를 검사하며 의미상의 사양 검토나 실기 검증을 대신하지 않는다.

| S-ID | 안내서 요지 | 상태(구현/규칙/해당없음) | 코드 위치(파일:함수) | 테스트(파일::이름) | 비고(G-ID) |
| --- | --- | --- | --- | --- | --- |
| S0.1 | 현장 주소와 report/rehearsal 경로만 사용 | 구현 | maicon26/link/http.py:HttpServerLink.__init__ | tests/test_real_runtime.py::test_real_launcher_accepts_explicit_server_environment_without_guessing | G5/G9; 주소 기본값을 추측하지 않음 |
| S0.16 | 주최측과 참가팀 역할, 시작 0:00·보고·종료 수명 | 규칙 | maicon26/mission/manager.py:Agent._match_api | tests/test_spec_conformance.py::test_agent_early_reporting_cancels_m1_m3_and_mock_receives_m6_within_one_second | G6; SVG 역할·레이저 설명은 원본 대조 |
| S0.2 | 코드 8자리 영숫자, GET 쿼리·POST 본문 | 구현 | maicon26/credentials.py:read_mission_code; maicon26/link/schema.py:mission_code_valid | tests/test_spec_credentials.py::test_invalid_noninteractive_format_warns_without_echo_and_continues | G5; 환경·파일 비표준 값은 경고 후 서버 판정 |
| S0.3 | POST JSON Content-Type | 구현 | maicon26/link/http.py:HttpServerLink._post_json | tests/test_mockserver_spec.py::test_transport_adapter_accepts_urllib_request_without_opening_socket | G7 |
| S0.4 | 임무코드·토큰과 대소문자 변형 출력 금지 | 구현 | maicon26/link/secrets.py:REDACT | tests/test_spec_transport.py::test_non_json_and_transport_exception_paths_do_not_leak_credentials | G13 |
| S0.5 | 마감은 서버 수신 시각, 종료 후 M1/M3 거절 | 구현 | maicon26/sim/mockserver.py:MockServer.request; maicon26/mission/submit.py:Submitter.close_for_stop | tests/test_mockserver_spec.py::test_receipt_time_after_early_end_rejects_inflight_m3 | G6; 이미 전송 중인 요청의 서버 판정 유지 |
| S0.6 | 서버 푸시 없이 참가팀 요청 | 구현 | maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_mockserver_spec.py::test_long_poll_uses_fake_clock_publication_instant_and_clamps_wait | G2/G9 |
| S0.7 | 확정·예정·미결 표시와 잠정값 분리 | 규칙 | maicon26/link/schema.py:SCHEMA_PROVISIONAL; maicon26/config.py:LlmCfg | tests/test_spec_runtime.py::test_unannounced_llm_protocol_and_m5_dwell_remain_explicit_unconfigured_slots | G12; 범례는 문서 메타데이터 |
| S0.8 | 0/240/300/330초 시간표 | 구현 | maicon26/sim/mockserver.py:MockServer.start | tests/test_mockserver_spec.py::test_reporting_window_and_all_six_team_early_finish | G7 |
| S0.9 | 시작 신호부터 팀 시계, 서버 남은 시간 없음 | 구현 | maicon26/mission/manager.py:Agent._on_start | tests/test_spec_runtime.py::test_synchronized_restart_restores_clock_without_replacing_navigation_pose | G14; 위치 복원과 별개 |
| S0.10 | M5 조기 종료도 report/phase로 감지 | 구현 | maicon26/mission/manager.py:Agent._serve | tests/test_spec_conformance.py::test_agent_early_reporting_cancels_m1_m3_and_mock_receives_m6_within_one_second | G6 |
| S0.11 | 1·2차 임무 wait로 게시 즉시 수신 | 구현 | maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_mockserver_spec.py::test_long_poll_uses_fake_clock_publication_instant_and_clamps_wait | G2/G9 |
| S0.12 | 접속 정보 미포함, 예시 이름·토큰·시각은 허구 | 규칙 | scripts/extract_spec_text.py:extract_html | tests/test_extract_spec_text.py::test_packaged_snapshot_provenance_and_extraction_preserve_source_authority | G8/G11; 실주소·실코드 없음 |
| S0.13 | 룰북·통신규약 우선, 안내서 2026-11-04 스냅숏 | 규칙 | scripts/extract_spec_text.py:extract_html; maicon26/link/schema.py:SCHEMA_SNAPSHOT | tests/test_extract_spec_text.py::test_packaged_snapshot_provenance_and_extraction_preserve_source_authority | G8/G11; SHA256 고정 |
| S0.14 | 남은 시간·자기 점수·예제 코드 API 없음 | 규칙 | maicon26/mission/threat.py:evidence_from_tracks; maicon26/link/schema.py:ENDPOINTS | tests/test_mockserver_spec.py::test_reporting_window_and_all_six_team_early_finish | G7/G11; 점수는 관찰 추정 |
| S0.15 | JSON·LLM API·동점·M5 체류 잠정 | 규칙 | maicon26/config.py:LlmCfg; maicon26/config.py:MissionCfg; maicon26/link/schema.py:SCHEMA_PROVISIONAL | tests/test_spec_runtime.py::test_unannounced_llm_protocol_and_m5_dwell_remain_explicit_unconfigured_slots | G12; 현장 재확인 |
| S1.1 | 모델·센서 초기화 후 me→ready | 구현 | scripts/run_real.py:main | tests/test_real_runtime.py::test_success_infers_before_ready_and_closes_all | G5/G14 |
| S1.2 | ready,false일 때 코드만 POST; 실패 2초 후 me부터 | 구현 | maicon26/link/http.py:HttpServerLink.ready_handshake | tests/test_mockserver_spec.py::test_ready_guards_and_persisted_acceptance | G3/G7 |
| S1.3 | slot/camp/phase/ready, 미편성 null | 구현 | maicon26/link/http.py:HttpServerLink.me; maicon26/link/schema.py:parse_slot | tests/test_mockserver_spec.py::test_authentication_and_missing_mission_code_have_distinct_errors | G4/G7; me 편성 정보가 영상 헤더보다 우선 |
| S1.4 | idle/waiting/ready/running/reporting/finished | 구현 | maicon26/link/schema.py:PHASE_LABELS; maicon26/link/http.py:HttpServerLink.me | tests/test_mockserver_spec.py::test_ready_guards_and_persisted_acceptance | G7 |
| S1.5 | ready 401/409/422, UGV 설정 확인 전 재시도 | 구현 | maicon26/link/schema.py:ready_error; maicon26/link/http.py:HttpServerLink.ready_handshake | tests/test_mockserver_spec.py::test_complete_published_error_strings_match_source_text | G3/G7/G13; 원문 생략 접미는 미정 |
| S1.6 | 재시작은 me부터, 기존 ready 상태 보존 | 구현 | scripts/run_real.py:main; maicon26/link/http.py:HttpServerLink.ready_handshake | tests/test_real_runtime.py::test_mid_match_restart_cannot_guess_starting_base | G14; 주행 위치 미복원 시 안전 차단 |
| S1.7 | 초기화 성공은 참가팀 책임 | 규칙 | scripts/run_real.py:main | tests/test_real_runtime.py::test_readiness_timeout_closes_robot_stack_without_reporting_ready | G5; 실패 시 준비완료 없음 |
| S2.1 | 0:00 전술명령 게시, 요청으로 받음 | 구현 | maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_mockserver_spec.py::test_long_poll_uses_fake_clock_publication_instant_and_clamps_wait | G2 |
| S2.2 | accepted:false는 정상 대기, true의 tactical_command 사용 | 구현 | maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_mockserver_spec.py::test_order_reasons_and_stable_team_specific_posting | G2 |
| S2.3 | wait 0 즉시/1~120 대기, 게시·미편성 즉시 | 구현 | maicon26/sim/mockserver.py:MockServer._order | tests/test_mockserver_spec.py::test_long_poll_all_teams_observe_same_publication_and_short_timeout | G7/G9; 기본20은 허용범위 선택 |
| S2.4 | HTTP timeout>wait, 제어루프 밖 대기 | 구현 | maicon26/link/http.py:HttpServerLink._wait_loop; maicon26/link/async_link.py:AsyncServerLink | tests/test_real_runtime.py::test_real_launcher_cannot_enable_network_calls_in_control_loop | G9/G14; 실전 sync-link 제거 |
| S2.5 | accepted/reason/order, KST issued_at 형식 | 구현 | maicon26/link/clock_sync.py:ClockSync.elapsed; maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_mockserver_spec.py::test_order_reasons_and_stable_team_specific_posting | G14 |
| S2.6 | 게시 전·미편성·게시누락 세 reason 분리 | 구현 | maicon26/link/schema.py:classify_reason; maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_spec_transport.py::test_reason_backoff_and_warning_only_on_change | G2 |
| S2.7 | order 401/422, 게시 전·미편성은 200 | 구현 | maicon26/sim/mockserver.py:MockServer._handle | tests/test_mockserver_spec.py::test_authentication_and_missing_mission_code_have_distinct_errors | G7/G13 |
| S2.8 | 팀별 명령 보존, 종료·재실행 후 동일, 목표거점 별도 | 구현 | maicon26/sim/mockserver.py:MockServer._order | tests/test_mockserver_spec.py::test_order_reasons_and_stable_team_specific_posting | G7/G14 |
| S2.9 | wait 유무 요청·게시 전후 예시 형식 | 구현 | maicon26/link/http.py:HttpServerLink._get_json | tests/test_mockserver_spec.py::test_order_reasons_and_stable_team_specific_posting | G7; 예시 시각은 실기 정보 아님 |
| S3.1 | 주행 서버 API 없음, 젯슨 match 신호 경계 | 규칙 | maicon26/mission/manager.py:Agent._match_api; maicon26/link/schema.py:ENDPOINTS | tests/test_spec_transport.py::test_restart_recovers_elapsed_once_then_obeys_early_report_stop | G8; 시간·정지 경계 회귀이며 실물 젯슨 미검증 |
| S3.2 | start 순간 팀 시계와 running, 조기 report | 구현 | maicon26/mission/manager.py:Agent._on_start; maicon26/mission/manager.py:Agent._serve | tests/test_spec_conformance.py::test_agent_early_reporting_cancels_m1_m3_and_mock_receives_m6_within_one_second | G6/G14 |
| S4.1 | 선택 기능, 최신 정상 JPEG 1초 캡처 | 구현 | maicon26/link/http.py:HttpServerLink.get_uav | tests/test_mockserver_spec.py::test_drone_capture_retains_good_frame_and_enforces_camp_intervals | G4 |
| S4.2 | 경기는 camera 생략, 파괴 수별1/2/5초·3개403 | 구현 | maicon26/sim/mockserver.py:MockServer._drone | tests/test_mockserver_spec.py::test_drone_capture_retains_good_frame_and_enforces_camp_intervals | G4/G7 |
| S4.3 | 연습 테스트 camera 요청은 편성·파괴저하와 독립 | 구현 | maicon26/sim/mockserver.py:MockServer._drone | tests/test_mockserver_spec.py::test_drone_practice_mode_headers_and_all_error_conditions | G4/G7 |
| S4.4 | main/test1/test2, 테스트 꺼지면 경기 규칙 | 구현 | maicon26/link/http.py:HttpServerLink.__init__; maicon26/sim/mockserver.py:MockServer._drone | tests/test_mockserver_spec.py::test_drone_practice_mode_headers_and_all_error_conditions | G4/G7 |
| S4.5 | 깨진 화면 대신 마지막 정상; 캡처시각 중복 제거 | 구현 | maicon26/link/http.py:HttpServerLink.get_uav | tests/test_mockserver_spec.py::test_drone_capture_retains_good_frame_and_enforces_camp_intervals | G4 |
| S4.6 | JPEG와 여섯 영상 헤더 검증 | 구현 | maicon26/link/http.py:HttpServerLink.get_uav | tests/test_mockserver_spec.py::test_drone_practice_mode_headers_and_all_error_conditions | G4; 실환경 헤더 미검증 |
| S4.7 | 영상 401/403/409/422/503 오류 구분 | 구현 | maicon26/link/schema.py:drone_error; maicon26/sim/mockserver.py:MockServer._drone | tests/test_mockserver_spec.py::test_drone_practice_mode_headers_and_all_error_conditions | G4/G7 |
| S4.8 | 자기 진영 시설 파괴로 영상 저하; 리허설 영상 없음 | 구현 | maicon26/sim/mockserver.py:MockServer._drone | tests/test_mockserver_spec.py::test_drone_capture_retains_good_frame_and_enforces_camp_intervals | G4/G7 |
| S5.1 | 텍스트 LLM M1 3+3+4점, 미보고0, API 미안내 | 규칙 | maicon26/config.py:LlmCfg; maicon26/mission/manager.py:Agent._poll_llm | tests/test_spec_runtime.py::test_unannounced_llm_protocol_and_m5_dwell_remain_explicit_unconfigured_slots | G12; 실제 주최측 LLM 연동 확인필요 |
| S5.2 | target/search_sectors/base, 하나 이상·이름 규격 | 구현 | maicon26/link/schema.py:validate_m1; maicon26/link/schema.py:m1_to_server | tests/test_spec_schema.py::test_m1_invalid_wire_reports_have_structured_422_locations | G1/G15 |
| S5.3 | 매번 전체 대체, 생략 필드는 null | 구현 | maicon26/link/schema.py:validate_m1; maicon26/mission/manager.py:Agent._submit_m1 | tests/test_mockserver_spec.py::test_m1_last_report_replaces_omitted_fields_with_null | G1/G6 |
| S5.4 | 시설이 놓인 진영 접두, 12 보고값 | 구현 | maicon26/link/schema.py:m1_to_server; maicon26/vocab.py:TARGET_WIRE | tests/test_spec_schema.py::test_internal_encoders_keep_opponent_facility_prefix_and_scored_territory | G15 |
| S5.5 | accepted와 정규화 received, 값으로 ACK 비교 | 구현 | maicon26/link/schema.py:equal_received; maicon26/link/http.py:HttpServerLink.submit | tests/test_spec_schema.py::test_m1_received_sector_order_and_case_are_cosmetic_but_values_are_not | G1 |
| S5.6 | M1 표적은 M2, 탐색·거점은 M3/M5와 별도 | 규칙 | maicon26/mission/manager.py:Agent._serve; maicon26/link/schema.py:scoring_sectors | tests/test_spec_runtime.py::test_m5_posted_objective_does_not_overwrite_m1_interpretation | G6/G15 |
| S5.7 | M1 running만, 미리·변경 시 전체 보고 | 구현 | maicon26/mission/manager.py:Agent._poll_llm; maicon26/mission/manager.py:Agent._guard_m3_submission | tests/test_spec_runtime.py::test_m1_rule_report_then_timely_llm_replaces_all_fields_once | G6; 늦은 LLM은 기존 보고를 덮지 않음 |
| S5.8 | M1 401/409/422, 영구 제외와 단계 오류 분리 | 구현 | maicon26/link/schema.py:classify_submission_409; maicon26/link/http.py:HttpServerLink.submit | tests/test_spec_transport.py::test_permanent_409_stops_one_kind_until_next_match | G3/G7/G13 |
| S5.9 | M1 요청·응답 예시와 보고값 표 연결 | 구현 | maicon26/link/schema.py:validate_m1 | tests/test_mockserver_spec.py::test_m1_last_report_replaces_omitted_fields_with_null | G1/G15 |
| S6.1 | M2 표적 장치가 판정·보고, 참가팀 API 없음 | 규칙 | maicon26/link/schema.py:ENDPOINTS; maicon26/sim/world.py:World._contact | tests/test_spec_runtime.py::test_device_events_do_not_create_participant_m2_m4_or_m5_reports | G7; 레이저 UGV 코드 원본 SVG 확인 |
| S6.2 | M2 지정표적10+연합 성공5 | 규칙 | maicon26/sim/run.py:score | tests/test_spec_game_rules.py::test_destroyed_target_is_counted_once_and_benefits_assignee_not_shooter | G7 |
| S6.3 | 2초 연속 조사, 발사자와 무관한 지정팀 점수 | 규칙 | maicon26/sim/world.py:World._contact | tests/test_spec_game_rules.py::test_destroyed_target_is_counted_once_and_benefits_assignee_not_shooter | G7; 물리 수광부 실기 미검증 |
| S6.4 | 표적 첫 파괴 한 번만 인정 | 규칙 | maicon26/sim/world.py:World._contact; maicon26/sim/run.py:score | tests/test_spec_game_rules.py::test_destroyed_target_is_counted_once_and_benefits_assignee_not_shooter | G7; 회피 전략 변경 없음 |
| S6.5 | 지정표적은 명령문으로 판단, 파괴상태 조회 API 없음 | 규칙 | maicon26/mission/manager.py:Agent._on_command; maicon26/link/schema.py:ENDPOINTS | tests/test_spec_game_rules.py::test_destroyed_target_is_counted_once_and_benefits_assignee_not_shooter | G7; 관찰·장치 이벤트만 사용 |
| S7.1 | M3 상대3구역 각7+CM9, 종류·개수 완전일치 | 규칙 | maicon26/sim/run.py:score_m3 | tests/test_spec_game_rules.py::test_m3_no_partial_points_for_wrong_quantity_kind_or_extra_object | G7/G10 |
| S7.2 | 관찰 누적 후 채점구역 전체 스냅숏 | 구현 | maicon26/mission/manager.py:Agent._m3_counts; maicon26/mission/manager.py:Agent._submit_m3_if_due | tests/test_spec_runtime.py::test_changed_m3_full_snapshots_switch_to_one_second_cadence_at_225_seconds | G6/G10 |
| S7.3 | 1~7개 섹터·중복금지·6종·정수0~99 | 구현 | maicon26/link/schema.py:validate_m3 | tests/test_spec_schema.py::test_m3_rejects_bad_sector_shapes_names_duplicates_and_cardinality | G10/G15 |
| S7.4 | M3 사진6종만, 실제 UGV·일반 장애물 제외 | 구현 | maicon26/link/schema.py:m3_to_server; maicon26/vocab.py:OBJECT_WIRE | tests/test_spec_schema.py::test_ground_object_map_distinguishes_tank_picture_from_ugv_and_other_obstacles | G10/G15 |
| S7.5 | M3 accepted와 정렬·대문자 received | 구현 | maicon26/link/schema.py:equal_received | tests/test_spec_schema.py::test_m3_received_sector_and_object_order_do_not_matter_but_omission_and_zero_do | G1 |
| S7.6 | 전체 대체, 진영별 상대3구역+CM만 채점 | 구현 | maicon26/link/schema.py:scoring_sectors; maicon26/link/schema.py:m3_to_server | tests/test_mockserver_spec.py::test_m3_accepts_all_sectors_but_replaces_entire_report | G6/G15; 내부 COMMON→CM |
| S7.7 | M3 running만, 후반 변경 전체본1초 간격 | 구현 | maicon26/mission/manager.py:Agent._submit_m3_if_due; maicon26/mission/manager.py:Agent._guard_m3_submission | tests/test_spec_runtime.py::test_changed_m3_full_snapshots_switch_to_one_second_cadence_at_225_seconds | G6; 225초 전 기본5초 |
| S7.8 | M3 401/409/422, 중복·잘못된 종류·범위 거절 | 구현 | maicon26/link/schema.py:validate_m3; maicon26/link/schema.py:submission_error | tests/test_spec_schema.py::test_m3_rejects_nonclasses_duplicate_types_and_invalid_integer_counts | G3/G7/G13 |
| S8.1 | M4 ESP 피격 판정, 참가팀 보고 API 없음 | 규칙 | maicon26/link/schema.py:ENDPOINTS; maicon26/sim/world.py:World._contact | tests/test_spec_runtime.py::test_device_events_do_not_create_participant_m2_m4_or_m5_reports | G7; 실물 발사는 젯슨 API |
| S8.2 | 4목숨, 피격별15/12/8/3/0점 | 규칙 | maicon26/sim/run.py:score | tests/test_spec_game_rules.py::test_m4_published_survival_points | G7 |
| S8.3 | 2초 연속,10초 무적, 아군 오사도 피격 | 규칙 | maicon26/sim/world.py:World._contact | tests/test_spec_game_rules.py::test_friendly_fire_counts_and_ten_second_invulnerability | G7; 기존 사선 회피 보존 |
| S9.1 | 240초 진영 공통 목표거점 게시 | 구현 | maicon26/sim/mockserver.py:MockServer.start; maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_spec_game_rules.py::test_m5_is_one_shared_alliance_objective_separate_from_m1 | G7 |
| S9.2 | 선착 점령 연합 각10, 종료 후 승리5·무승부 양측5 | 규칙 | maicon26/sim/run.py:score; maicon26/sim/world.py:World._base_sensor_step | tests/test_spec_game_rules.py::test_alliance_victory_bonus_and_draw | G7 |
| S9.3 | target를 시작부터 wait, 게시 즉시·백그라운드 수신 | 구현 | maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_mockserver_spec.py::test_long_poll_all_teams_observe_same_publication_and_short_timeout | G2/G9/G14 |
| S9.4 | objective/label/issued_at, 상대 진영 거점 | 구현 | maicon26/link/schema.py:parse_objective; maicon26/link/http.py:HttpServerLink._wait_loop | tests/test_mockserver_spec.py::test_long_poll_uses_fake_clock_publication_instant_and_clamps_wait | G14/G15 |
| S9.5 | target 게시 전·미편성·게시 누락 reason | 구현 | maicon26/link/schema.py:classify_reason | tests/test_spec_transport.py::test_reason_backoff_and_warning_only_on_change | G2 |
| S9.6 | target 401/422와 accepted:false 정상 응답 | 구현 | maicon26/sim/mockserver.py:MockServer._handle | tests/test_mockserver_spec.py::test_authentication_and_missing_mission_code_have_distinct_errors | G7/G13 |
| S9.7 | 목표 센서 가림으로 해당 연합 점령, UGV 식별 없음 | 규칙 | maicon26/sim/world.py:World._base_sensor_step; maicon26/mission/field_policy.py:OwnBasePolicy; maicon26/mission/manager.py:Agent._update_field_policy; maicon26/config.py:MissionCfg | tests/test_spec_game_rules.py::test_non_target_base_does_not_capture_and_dwell_is_explicit_provisional_setting; tests/test_spec_field_policy.py::test_activation_at_225_only_blocks_own_three_bases_and_can_reset; tests/test_spec_field_policy.py::test_initial_footprint_overlap_can_leave_but_cannot_go_deeper; tests/test_spec_field_policy.py::test_agent_final_output_guard_overrides_acceleration_ramp_for_both_backends | G12; 225초 진입 방지는 요청된 팀 전략. 실제 ROS Nav2 주행 미검증. 10ms 경쟁은 동일 틱/수신순서 대용 |
| S9.8 | M5는 target 따름, M1 거점과 분리, 결과 API 없음 | 구현 | maicon26/mission/manager.py:Agent._serve | tests/test_spec_runtime.py::test_m5_posted_objective_does_not_overwrite_m1_interpretation | G14 |
| S10.1 | 300초/M5로 정지·잠금, M6 동안 프로그램 유지 | 구현 | maicon26/sim/world.py:World.end; maicon26/mission/manager.py:Agent._enter_halt | tests/test_spec_game_rules.py::test_any_robot_in_target_sensor_credits_target_alliance_even_own_base | G6/G7 |
| S10.2 | report 신호보다 먼저 오는 reporting phase도 종료 | 구현 | maicon26/mission/manager.py:Agent._serve; maicon26/mission/manager.py:Agent._enter_halt | tests/test_spec_runtime.py::test_restart_in_reporting_goes_directly_to_stopped_reporting_without_start_time | G6/G14 |
| S10.3 | pause는 종료가 아니며 M6 금지 | 구현 | maicon26/mission/manager.py:Agent._match_api | tests/test_spec_runtime.py::test_robot_pause_keeps_match_active_without_requesting_m6 | G6 |
| S10.4 | 종료 즉시 M1/M3 닫음, M6만30초·6팀 제출시끝 | 구현 | maicon26/mission/submit.py:Submitter.close_for_stop; maicon26/sim/mockserver.py:MockServer._handle | tests/test_spec_conformance.py::test_agent_early_reporting_cancels_m1_m3_and_mock_receives_m6_within_one_second | G6/G7; HTTP/비동기 큐+Mock 통합 회귀 |
| S10.5 | 참가팀 M2/M4/M5 보고 없음 | 규칙 | maicon26/link/schema.py:ENDPOINTS; maicon26/mission/manager.py:Agent._on_event | tests/test_spec_runtime.py::test_device_events_do_not_create_participant_m2_m4_or_m5_reports | G7 |
| S10.6 | reporting/finished me 예시와 늦은 M3 409 | 구현 | maicon26/link/schema.py:submission_error; maicon26/sim/mockserver.py:MockServer._handle | tests/test_mockserver_spec.py::test_receipt_time_after_early_end_rejects_inflight_m3 | G6/G7 |
| S11.1 | M6 상대팀 위험순위,15/10/5점 | 규칙 | maicon26/sim/run.py:score | tests/test_spec_game_rules.py::test_m6_partial_rank_scoring | G7 |
| S11.2 | hp/4+ammo/8+M3/30 동일가중 위험도 | 규칙 | maicon26/mission/threat.py:score; maicon26/sim/run.py:score | tests/test_spec_game_rules.py::test_m6_equal_weight_formula_and_nonnegative_terms | G7; Agent는 관측 추정만 |
| S11.3 | 경기 중 준비, reporting 즉시 M6; 마지막 보고 | 구현 | maicon26/mission/manager.py:Agent._prepare_m6; maicon26/mission/manager.py:Agent._mode_report | tests/test_spec_conformance.py::test_agent_early_reporting_cancels_m1_m3_and_mock_receives_m6_within_one_second | G6; LLM 대기로 첫 보고 지연 없음 |
| S11.4 | 상대 호출부호1~3개, 순위 순서 의미 유지 | 구현 | maicon26/link/schema.py:validate_m6; maicon26/link/schema.py:equal_received | tests/test_spec_schema.py::test_m6_received_case_is_cosmetic_but_rank_order_is_semantic | G1/G15 |
| S11.5 | 호출부호와 섹터 구분, 관찰 추정, 동점 잠정 | 규칙 | maicon26/link/schema.py:callsign; maicon26/mission/threat.py:evidence_from_tracks | tests/test_spec_game_rules.py::test_m6_referee_uses_actual_m3_points_not_travel_and_estimator_stays_observational | G12; 동점 허용은 시뮬 설정 |
| S11.6 | 마감까지 미보고 M1/M3/M6은0점 | 규칙 | maicon26/sim/run.py:score; maicon26/mission/manager.py:Agent._mode_report | tests/test_spec_game_rules.py::test_python_sim_submission_phase_gates_and_six_reports_finish | G6/G7 |
| S11.7 | M6 401/409/422, 자기편·중복·잘못된 호출부호 | 구현 | maicon26/link/schema.py:validate_m6; maicon26/link/schema.py:submission_error | tests/test_spec_schema.py::test_m6_rejects_wrong_side_sector_names_duplicates_and_bad_lengths | G3/G7/G13 |
| SA.1 | 401/409/422 및 영상403/503, detail loc/msg | 구현 | maicon26/link/schema.py:ValidationError; maicon26/link/http.py:HttpServerLink._detail | tests/test_spec_schema.py::test_m1_invalid_wire_reports_have_structured_422_locations | G13; 문자열 detail은 방어 처리이지 공식 보장 아님 |
| SA.2 | 시설물12값과 내부명 매핑 | 구현 | maicon26/vocab.py:TARGET_WIRE; maicon26/vocab.py:BASE_WIRE | tests/test_spec_schema.py::test_central_wire_tables_match_snapshot_and_arena_rows | G15 |
| SA.3 | 팀 시계·관찰 추정 사용, 추가 조회 API 없음 | 규칙 | maicon26/mission/threat.py:evidence_from_tracks; maicon26/link/schema.py:ENDPOINTS | tests/test_spec_game_rules.py::test_m6_referee_uses_actual_m3_points_not_travel_and_estimator_stays_observational | G11 |
| SA.4 | 7구역 이름·행·끝띠 제외·진영별4채점구역 | 구현 | maicon26/link/schema.py:SERVER_SECTORS; maicon26/link/schema.py:scoring_sectors | tests/test_spec_schema.py::test_central_wire_tables_match_snapshot_and_arena_rows | G15 |
| SA.5 | M3 사진 객체6종 표 | 구현 | maicon26/vocab.py:OBJECT_WIRE; maicon26/link/schema.py:validate_m3 | tests/test_spec_schema.py::test_m3_rejects_nonclasses_duplicate_types_and_invalid_integer_counts | G10/G15 |
| SR.1 | 리허설은 prefix만 변경, 형식 동일·공식점수 없음 | 구현 | maicon26/link/http.py:HttpServerLink.__init__; maicon26/sim/mockserver.py:MockServer.start | tests/test_mockserver_spec.py::test_rehearsal_isolated_state_five_second_countdown_and_complete_timeline | G7/G9 |
| SR.2 | 리허설7창구, drone은 report camera로 별도 | 구현 | maicon26/sim/mockserver.py:MockServer._handle | tests/test_mockserver_spec.py::test_rehearsal_isolated_state_five_second_countdown_and_complete_timeline | G7 |
| SR.3 | 리허설 시작 누른5초후0:00,240/300/330초 흐름 | 구현 | maicon26/sim/mockserver.py:MockServer.start | tests/test_mockserver_spec.py::test_rehearsal_isolated_state_five_second_countdown_and_complete_timeline | G7; 포털 UI 자동화 없음 |
| SR.4 | 리허설도 단계 규칙, 비활성 idle와 전용 reason | 구현 | maicon26/link/schema.py:REASON_NO_REHEARSAL; maicon26/sim/mockserver.py:MockServer._order | tests/test_mockserver_spec.py::test_rehearsal_isolated_state_five_second_countdown_and_complete_timeline | G2/G7; 실제 UGV 연결은 현장 전제 |

## 원문 대조와 분류

- 추출문 1~603줄의 각 단계·표·경고·예시를 S0~S11/SA/SR에 연결했다. 단계 제목, 페이지 이동, 화살표, 색상 범례, 요청/응답 표지는 해당 절의 비실행 설명이다. UI 표지를 별도 서버 동작으로 만들지 않았다.
- 추출기가 SVG를 제거하므로 레이저에 발사 UGV 코드가 실린다는 문구는 HTML 406/409줄에서 직접 확인했다(S0.16/S6.1). 누락된 서버 기능으로 오인하지 않는다.
- 원문 422 설명은 `detail` 목록의 `loc/msg`이다. 작업지시가 덧붙인 업무 오류 문자열 형태는 클라이언트 방어 처리로 수용하며 공식 서버의 보장으로 쓰지 않는다(SA.1).
- 생략 부호가 있는 오류 예시의 미안내 접미는 `schema.ERROR_DETAILS_UNSPECIFIED`로 구분했다. 알려진 접두는 분류하되 문장 전체를 추측하지 않는다.
- G14는 HTTP Date/KST 게시시각을 이용한 시간 복구다. 실제 SLAM 지도·위치 앵커를 복원하지 못한 중간 주행 재시작은 `run_real.py`가 정지 상태에서 차단한다. reporting 재시작은 STOP을 선반영하고 M6로 진행한다.
- M5 거의 동시 점령의 10ms 센서시각과 서버 도착시각은 실물 검증이 필요하다. 현재 Python 심판은 같은 틱/처리 순서 대용이며 실제 두 독립 시각을 재현했다고 주장하지 않는다.
- S9.7의 센서 소속 미식별 위험을 피하는 225초 자기 거점 제약은 공식 서버 규칙이 아니라 요청된 팀 전략이다. `OwnBasePolicy`는 감지 지도와 분리된 경로 마스크 및 최종 차체 궤적 검사로 적용한다. 선택 회귀는 통과했으며 실제 ROS Nav2 필터·주행 검증은 남아 있다.
- 사양에 없는 주소·비밀값·LLM 프로토콜·추가 API는 만들지 않았다. 실제 YOLO 인식, 주최측 장치와 통신, 연합 동시 주행 성능은 이 표의 정적 추적 검증 범위 밖이다.

## 미분류 문장
없음.




