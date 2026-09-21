# 웹 백엔드 — AMR 매거진 이송 관제

> WBS 1~7단계의 백엔드. 프론트는 아직 없다.
> 스키마 원본: [`docs/DB구성.md`](../../docs/DB구성.md) · 그림: [`docs/ERD.md`](../../docs/ERD.md)

## 한 줄 요약

**설정은 파일이 주인이고, 이력은 DB 가 주인이고, 실시간 상태는 아무도 주인이 아니다.**
이 백엔드는 그 셋을 화면에 이어 주는 층이며, 그 자신은 아무것도 들고 있지 않는다.

---

## 준비

```bash
# 1) 파이썬 의존성 (rclpy 는 여기 없다 — apt 로 깔린 ROS2 것을 쓴다)
python3 -m venv --system-site-packages .venv     # apt install python3.12-venv 필요
. .venv/bin/activate
pip install -r requirements.txt

# 2) DB — 컨테이너로 올리든 로컬에 깔든 상관없다. DSN 만 맞으면 된다.
#    스키마 4개를 순서대로 적용한다(001 → 002 → 003 → 004).
for f in ../../sql/00*.sql; do psql "$COBOT3_DSN" -f "$f"; done
#   001 트레이스 4표 + 뷰 4개   002 carrier_kind 4 · carrier_pair 8
#   003 관제 2표                004 trace.appended 트리거 (6.4 · 5.1 이 이걸 탄다)

# 3) 실행
cp .env.example .env && set -a && . ./.env && set +a
uvicorn app.main:app --reload --port 8000
```

`http://localhost:8000/docs` 에 스키마가 다 나온다. `GET /health` 로 DB·브리지 상태를 따로 본다.

**ROS 없이 돌린다**(`COBOT3_ROS=0`): 읽기 API·설정 편집·큐 관리는 전부 동작하고,
로봇에게 실제로 시키는 부분만 `503 bridge_unavailable` 로 거절한다.
조용히 성공한 척하지 않는 이유는, 그러면 화면이 배차된 줄 알고 영원히 기다리기 때문이다.

---

## 구조

```
화면 ──REST──> routers ──> services ──> PostgreSQL   (표 5·6 쓰기 / 표 1~4 읽기)
                              │
                              └──> rosbridge ──action·srv──> task_manager

PostgreSQL ──NOTIFY──> notify.py ──┬──> hub (trace.appended)          6.4
 (event_logger 가 INSERT)          └──> pickup.on_place_done          5.1

task_manager ──feedback──> rosbridge ──> dispatcher ──┬──> 표 5·6 갱신  4.8
                                                      └──> hub (task.changed)
```

| 파일 | 맡은 것 |
|---|---|
| `yamlstore.py` | yaml 원자적 읽기/쓰기. **주석을 보존**하고 revision 으로 동시 편집을 막는다 |
| `configstore.py` | 설정 캐시. mtime 이 바뀌면 다시 읽어 사람이 에디터로 고친 것도 잡는다 |
| `services/queue.py` | 표 5 의 모든 쓰기. 불변식은 파이썬이 아니라 SQL 인덱스가 지킨다 |
| `services/pickup.py` | 표 6 + `ready_at` 스케줄러 |
| `services/dispatcher.py` | 큐 → 로봇, 로봇 보고 → 표. 로봇별 루프 하나씩 |
| `services/rosbridge.py` | **rclpy 를 아는 유일한 파일.** 나머지는 `Bridge` 만 본다 |
| `services/notify.py` | `LISTEN trace_appended` 전용 커넥션 |
| `ws.py` | 봉투 `{ch, robot_id, seq, ts, data}`. 채널별 `seq`, 클라이언트별 큐 |

---

## 설계 결정 (되돌리기 전에 읽을 것)

**1. `task` 의 writer 는 웹 하나다** (WBS 0.3)
`task_manager` 는 DB 를 모른다. 진행 상황을 `ExecuteTask` feedback 으로만 올리고,
그걸 표에 적는 것은 `dispatcher` 다. 그래서 로거(`event_logger`)와 웹이 쓰는 표가 겹치지 않는다 —
`docs/DB구성.md` §1 의 "DB 커넥션을 가진 노드가 하나뿐" 이 **로깅 경로에 한해** 유지되는 이유(§10-7).

**2. 작업 1건 = 캐리어 1개, 그리고 복귀까지 끝나야 1건이다**
`scan → pick → nav → place → return(선반)`. return 을 빼면 `more_at_target` 을 판단할 자리가 없다 —
place 직후 로봇은 스테이션에 있어 선반에 뭐가 남았는지 모른다.
복귀해서 "아직 더 있다" 를 알리면 웹이 곧바로 후속 작업을 걸고, `dispatcher.wake()` 가 배차 루프를
즉시 깨워 로봇이 선반 앞에 선 채로 이어 돈다. 사람이 매번 다시 배정하지 않는다.

**3. 순찰을 없앤 게 아니라 '어느 선반' 만 가져왔다**
로봇은 지금처럼 스스로 일을 찾는다(detect → HOLD → scan → 손목캠 보정 → pick).
웹이 가져간 것은 다음 선반의 선택뿐이다. **선반은 잠그지 않는다** — 두 로봇이 같은 선반에
붙어 매거진을 나눠 빼는 것이 정상 운용이고, DB 에도 선반 lock 이 없다.

그럼에도 큐가 필요한 이유는 셋이다: ⓐ 회수가 스캔을 선점해야 한다(스테이션 capacity 가 1이라
산출물을 안 치우면 다음 place 가 `PORT_OCCUPIED` 로 막힌다) ⓑ 두 로봇의 부하를 나눈다
ⓒ 웹이 재시작해도 배정이 남는다. 1대로 돌릴 때는 선반 목록을 순서대로 큐에 넣으면 그게 순찰이다.

> ⚠️ 선반을 안 잠그므로 두 로봇이 같은 캐리어를 노릴 수 있다. 배정 시점에는 선반에 무엇이
> 있는지 모르므로 막을 수 없다. 부딪히면 배타를 **캐리어(`qr_payload`) 수준**에 걸어야 한다 —
> `feedback.carrier_id` 로 이미 올라오므로 감지 자체는 새 인터페이스 없이 된다.

**4. 실시간 상태는 저장하지 않는다**
위치·배터리·현재 단계는 WebSocket 패스스루로 끝난다. 초당 수십 번 바뀌고 과거값을 아무도 안 본다.
그래서 `GET /robots` 는 큐와 브리지 상태만 주고 위치를 주지 않는다 — 줄 스냅샷이 없다.

**5. 결번은 프론트가 푼다**
허브는 놓친 메시지를 재전송하지 않는다. 재전송하려면 버퍼가 필요하고, 버퍼는 곧
"실시간 상태를 서버가 들고 있다" 는 뜻이라 4번과 어긋난다. `seq` 가 튀면 그 리소스를 REST 로 다시 읽는다.

---

## 실패 코드

| HTTP | code | 언제 |
|---|---|---|
| 404 | `not_found` | 없는 작업 · 없는 선반/스테이션 |
| 409 | `conflict` | 같은 선반에 다른 로봇이 실행 중 · RUNNING 이관 · 큐가 그 사이 바뀜 |
| 412 | `revision_mismatch` | 설정 저장 중 다른 사람이 먼저 저장했다 |
| 422 | `unprocessable` | 티칭 미완료 선반 배정 · 모르는 로봇 · 라우팅 분기 |
| 428 | `if_match_required` | 설정 PUT 에 `If-Match` 를 안 달았다 |
| 503 | `bridge_unavailable` | ROS 브리지가 꺼짐 · 액션 서버 없음 · 인터페이스 미정 |

---

## 아직 안 된 것

| | 막는 것 |
|---|---|
| `task_manager` 가 `ExecuteTask` 를 아직 안 받는다 | 지금은 `START_DETECTED_FOR_TEST` 로 스스로 시작한다. goal 대기로 바꿔야 배차가 실제로 돈다 |
| `TraceEvent` 새 판을 `task_manager`·`event_logger` 가 아직 안 쓴다 | 표 2·3 이 안 채워진다 → 로그 화면·회수 예약(5.1)이 빈 채로 돈다 |
| `stations.yaml` 의 `process_sec` 가 추정값 | 회수 시각이 틀린다. 실측 필요 |
| 인증·접근 범위 | WBS 1.5 대로 이번 스코프 밖 |
| 카메라 스트림 (8단계) | `web_video_server` 별도 셋업. JSON 채널과 분리 |
