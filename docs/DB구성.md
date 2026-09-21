# 🗄 생산 트래킹 DB 구성 — 트레이스 4표 + 관제 2표

> Isaac Sim_협동3 / 24.04+5.x / Development Process
> 상위: [04_Backend_Carrier_Research.md](04_Backend_Carrier_Research.md) §12-1 「바코드/QR 기반 캐리어 추적」
> 관련: [05_QR_Traceability.md](05_QR_Traceability.md) §2-8 · [02_2, 3차 목표 설계도.html](02_2,%203차%20목표%20설계도.html) FIG.2
> 그림: [ERD.md](ERD.md) — 표 6개 관계도 한 장. **이 문서가 원본이고 그쪽은 그림만 갖는다**
> 상태: **트레이스 4표 스키마 확정 · 미구현** / **관제 2표(§10) 제안**

---

## 이 문서의 위치

`05_QR_Traceability.md` §2-8이 **SQLite 2테이블(`carrier` / `event`)** 로 잡아 둔 것을,
페이로드 체계가 `F1-MGZB-1`로 확정되고 캐리어 에셋이 16종으로 늘어난 뒤의 실제 스키마로 다시 내린 것이다.

| §2-8이 정한 것 | 이 문서가 바꾼 것 | 왜 |
|---|---|---|
| SQLite | **PostgreSQL** | AMR 2대가 동시에 쓴다. §2-9도 *"다중 PC·다중 AMR로 커지면 PostgreSQL + 오프라인 스풀"* 로 퇴로를 열어 뒀다 |
| `carrier` 1표에 매거진·스택 혼재 | **`magazine_log` / `stack_log` 2표로 분리** | 매거진과 스택은 서로 다른 시각에 다른 로봇이 옮긴다 |
| `event` 1표 (전 단계 공용) | 위 2표가 곧 이벤트 표 | 같음 — **append-only** 원칙은 그대로 |
| `spec_json` 한 칸 | **`carrier_kind` 4행** + `spec_extra` JSONB | 규격은 개체가 아니라 **종류**의 속성이다 |
| `produced_from` 자기참조 | **`carrier_pair` 8행** | 매거진↔스택이 1:1이라 별도 표가 더 곧다 |
| 페이로드 `C3.MAG.A17.9` | **`F1-MGZB-1`** | `isaacpjt/assets/carrier_code.py` 가 규칙과 파서를 갖는다 |

> ⚠️ 충돌이 생기면 **이 문서가 우선**한다. §2-8 / §2-9의 SQLite DDL·로거 코드는 이 문서로 대체됐다.

### 한 줄 요약

> 로봇이 캐리어를 한 단계 다룰 때마다 **행을 하나 append 한다.** 고치지 않는다.
> 한 행에는 **QR 원문 · 종류 · 로봇 · 단계와 시각 · 물체 상태 · 실패 사유**가 들어간다.
> 매거진과 스택은 표를 나누고, 둘의 1:1 대응은 정적인 표 하나가 갖는다.

---

# 1. 전체 구조

```
task_manager (robot1) ─┐
                       ├─ /trace/event  (TraceEvent · 절대이름 · reliable/50)
task_manager (robot2) ─┘          │
                                  ▼
                           event_logger  ── 전역 1개. DB 커넥션은 여기에만 있다
                              │      └─ 큐가 차거나 DB가 죽으면 → runs/trace_spool.jsonl
                              ▼
                          PostgreSQL
```

| 표 | 행 수 | 성격 | 내용 |
|---|---|---|---|
| **표 1** `carrier_pair` | 8 | 정적 · 시드 | 매거진 1개 ↔ 스택 1개 |
| **표 2** `magazine_log` | N | **append-only** | 매거진 이송 로그 |
| **표 3** `stack_log` | N | **append-only** | 스택 이송 로그 (표 2와 구조 동일) |
| **표 4** `carrier_kind` | 4 | 정적 · 시드 | 종류별 규격 |
| **표 5** `task` | N | **가변 · 공유** | 작업 큐 — 화면이 쓰고 orchestrator가 읽는다 (§10) |
| **표 6** `pending_pickup` | N | **가변 · 공유** | 산출물 회수 대기 타이머 (§10) |

표 1~4는 **로거만 쓴다.** 표 5·6은 성격이 반대다 — 웹 백엔드와 orchestrator가 **양쪽에서 UPDATE 한다.**
그래서 append-only 원칙(§8-1)은 표 1~4에만 적용된다. 왜 이 둘만 표가 되었는지는 §10-1.

**DB 커넥션을 가진 노드가 `event_logger` 하나뿐인 것이 설계의 핵심이다.**
`task_manager`는 토픽에 던지고 끝이라, 로거가 안 떠 있어도 DB가 죽어도 로봇은 평소대로 돈다.
§2-9의 *"기록 실패가 로봇 동작을 막지 않는다. 그렇다고 조용히 버리지도 않는다"* 를 배선으로 보장한 것이다.

---

# 2. ENUM 2개

```sql
CREATE TYPE run_status  AS ENUM ('IN_TRANSIT', 'COMPLETED', 'FAILED');
CREATE TYPE fail_reason AS ENUM ('pick_error', 'nav_error', 'place_error', 'dock_error');
```

`fail_reason`을 ENUM으로 못 박은 이유: **다섯 번째 값을 DB가 거부한다.** 참조 테이블이면 새 사유가 조용히 늘어난다.
`stage`는 반대로 `TEXT`로 둔다 — 트리에 가지가 늘어날 자리가 이미 나 있다(`task_manager.py:772` 주석: 배터리 선점 · 외부 작업 지시).

---

# 3. 표 1 — `carrier_pair` (8행, 정적)

```sql
CREATE TABLE carrier_pair (
    magazine_payload TEXT PRIMARY KEY,        -- 'F1-MGZB-1'  이 매거진이
    stack_payload    TEXT NOT NULL UNIQUE     -- 'F3-STKB-1'  이 스택이 된다
);
```

양쪽에 UNIQUE가 걸려 **1:1이 DB 레벨에서 보장된다** — 한 매거진이 두 스택이 되거나, 두 매거진이 같은 스택이 되는 행을 거부한다.

`carriers.yaml`의 `produced_from`이 하던 일을 이 표가 가져간다. 그 파일은 삭제한다(§8-5).

### 시드 — 확정

```sql
INSERT INTO carrier_pair (magazine_payload, stack_payload) VALUES
  ('F1-MGZB-1', 'F3-STKB-1'),   -- 파랑
  ('F1-MGZB-2', 'F3-STKB-2'),
  ('F2-MGZB-1', 'F3-STKB-3'),
  ('F2-MGZB-2', 'F3-STKB-4'),
  ('F1-MGZO-1', 'F3-STKO-1'),   -- 주황
  ('F1-MGZO-2', 'F3-STKO-2'),
  ('F2-MGZO-1', 'F3-STKO-3'),
  ('F2-MGZO-2', 'F3-STKO-4');
```

`carrier_code.py` 의 `CODES` 16개와 대조해 확인한 것 — 매거진 8개·스택 8개가 **각각 한 번씩만** 나오고,
모든 쌍에서 **매거진 색 == 스택 색**이다 (파랑 4쌍 · 주황 4쌍).

> 📌 저장하는 값은 **페이로드 형태**(`F1-MGZB-1`)다. 파일명(`F1_MGZB_1.usda`)은
> `carrier_code.usd_name()` 이 `-` 를 `_` 로 바꾼 것이고, QR 에 구워진 문자열은 페이로드 쪽이다.

> 이 표가 비어 있어도 **표 2·3은 정상으로 채워진다.** `production_tracking` 뷰만 빈 결과를 낸다.

---

# 4. 표 2 `magazine_log` / 표 3 `stack_log` (append-only)

두 표는 **컬럼이 완전히 같다.** 아래 DDL에서 이름만 바꾼 것이 표 3이다.

미션 1회당 최대 4행(`pick` · `nav` · `place` · `return`). **UPDATE도 DELETE도 하지 않는다.**

```sql
CREATE TABLE magazine_log (
    log_id       BIGSERIAL PRIMARY KEY,
    run_id       UUID        NOT NULL,        -- 미션 1회를 묶는 끈
    qr_payload   TEXT        NOT NULL,        -- ① 'F1-MGZB-1' 원문 그대로
    kind_code    TEXT        NOT NULL,        -- ② 'MGZB' → carrier_kind 조인
    plant_code   TEXT                         --   'F1' 공장. 첫 토큰은 항상 첫 자리다(§8-3)
                 GENERATED ALWAYS AS (split_part(qr_payload, '-', 1)) STORED,
    robot_id     TEXT        NOT NULL,        -- ③ 'robot1'

    stage        TEXT        NOT NULL,        -- ④ pick | nav | place | return
    attempt      INT         NOT NULL DEFAULT 1,   -- ④ 같은 단계의 몇 번째 시도인가 (§4-7)
    started_at   TIMESTAMPTZ NOT NULL,        -- ④ 그 단계 시작 (벽시계)
    ended_at     TIMESTAMPTZ NOT NULL,        -- ④ 그 단계 끝 (벽시계) ← 표시하는 값
    started_sim  DOUBLE PRECISION,            --    같은 순간의 시뮬 시각
    ended_sim    DOUBLE PRECISION,            --    같은 순간의 시뮬 시각
    duration_sec DOUBLE PRECISION
                 GENERATED ALWAYS AS (ended_sim - started_sim) STORED,

    succeeded    BOOLEAN     NOT NULL,        --    그 '단계' 의 성패
    status       run_status  NOT NULL,        -- ⑤ 그 시점 '물체' 의 상태
    fail_reason  fail_reason,                 -- ⑤ 넷 중 하나. 성공 행은 NULL
    fail_detail  TEXT,                        -- ⑤ 'PORT_OCCUPIED(3)' 액션 상수명
    port         TEXT,                        --    'test_loader' 어디서

    CONSTRAINT mag_reason_iff_failed
        CHECK (succeeded = (fail_reason IS NULL)),
    CONSTRAINT mag_detail_needs_reason
        CHECK (fail_detail IS NULL OR fail_reason IS NOT NULL),
    CONSTRAINT mag_run_stage_once
        UNIQUE (run_id, stage, attempt)
);

CREATE INDEX idx_mag_carrier ON magazine_log (qr_payload, ended_at DESC);
CREATE INDEX idx_mag_robot   ON magazine_log (robot_id,   ended_at DESC);
CREATE INDEX idx_mag_open    ON magazine_log (status) WHERE status = 'IN_TRANSIT';
```

## 4-1. 컬럼별 설명

| 컬럼 | 무엇 | 없으면 |
|---|---|---|
| `log_id` | DB가 매기는 삽입 순번 | 같은 초에 들어온 두 행의 순서가 비결정적이 된다 |
| `run_id` | **미션 1회.** `ScanLeaf`가 성공한 순간 `uuid4()` 발행 → `CycleDone`에서 폐기 | 같은 캐리어를 두 번 돌렸을 때 "이번 PICK"과 "지난번 PICK"을 시간으로만 갈라야 한다 |
| `qr_payload` | `CarrierScan` 응답 원문 **그대로**. 가공 없음 | 파싱 규칙이 바뀌면(로트 날짜 추가) 과거 행을 다시 해석할 수 없다 |
| `kind_code` | `carrier_code.parse_code()` 결과 | 조인 키라 인덱스가 필요하고, **SQL로는 못 푼다**(§8-3) |
| `plant_code` | 생성열. 공장 `F1`·`F2`·`F3` | 공장별 집계를 `qr_payload` 문자열 검색으로 해야 한다 |
| `robot_id` | `self.get_namespace().strip('/')` | 전역 로거 하나가 두 로봇 이벤트를 받으므로 메시지에 실려야 한다 |
| `stage` | BT 잎 이름 상수 그대로 — **소문자** | |
| `attempt` | 같은 `(run_id, stage)`의 몇 번째 시도. 기본 1 | 웹 복구로 재시도한 행이 제약에 걸려 **기록을 잃는다**(아래 4-7) |
| `started_at` / `ended_at` | 잎의 `initialise()` / SUCCESS·FAILURE 반환 시각. **`task_manager`가 찍는다** | 로거의 `now()`를 쓰면 큐 지연·스풀 재적재분이 전부 그때로 찍힌다 |
| `started_sim` / `ended_sim` | 같은 순간의 시뮬 시각 (`use_sim_time`) | 아래 4-2 |
| `duration_sec` | 생성열. **시뮬 기준** | 애플리케이션이 계산해 넣으면 두 시각과 어긋날 수 있다 |
| `succeeded` | 그 **단계**의 성패 | 아래 4-3 |
| `status` | 그 시점 **물체**의 상태 | 아래 4-3 |
| `fail_reason` | ENUM 4개 중 하나 | 집계 키 |
| `fail_detail` | `_reason_name()`이 만든 문자열 | `pick_error 12건`을 봐도 **고칠 게 뭔지 모른다** |
| `port` | 어디서 일어난 일인가 | 지금 `task_manager`는 대부분 모른다(아래 4-5) |

## 4-2. 🔧 시계를 두 벌 저장하는 이유

`05` §2-6이 적어 둔 그대로다 — *"하나만 저장하면 반드시 나중에 후회한다."*

Isaac Sim을 GUI 렌더로 돌리면 실시간보다 느리다. 같은 PICK을 재도

| | PICK | NAV | PLACE |
|---|---|---|---|
| 벽시계 | 72.2 s | 207.2 s | 101.5 s |
| 시뮬 | **43.3 s** | **124.3 s** | **60.9 s** |

**사이클 타임은 시뮬 시각이 물리적으로 옳다.** 그래서 `duration_sec`은 `ended_sim - started_sim`이다.
*"몇 시에 배달됐나"* 는 `ended_at`을 본다. 벽시계 소요가 필요하면 `ended_at - started_at`을 쿼리에서 빼면 된다.

## 4-3. ⭐ `succeeded`와 `status`를 따로 두는 이유

`RETURN`(순찰 시작점 복귀)은 `PLACE` **뒤**다. 그때 물체는 이미 배달됐다.

```
F1-MGZB-1 | robot1 | place  | succeeded=true  | status=COMPLETED |
F1-MGZB-1 | robot1 | return | succeeded=false | status=COMPLETED | nav_error / BLOCKED(2)
```

복귀 주행은 실패했지만 **물체는 COMPLETED가 맞다.** 한 칸으로 합치면 둘 중 하나가 거짓말이 된다.

### `status` 채우는 규칙 (`task_manager`가 정해서 싣는다)

| `stage` | `succeeded=true` | `succeeded=false` |
|---|---|---|
| `pick` · `nav` | `IN_TRANSIT` | `FAILED` |
| `place` | `COMPLETED` | `FAILED` |
| `return` | `COMPLETED` | **`COMPLETED`** |

> 📌 **로거는 이 판단을 하지 않는다.** `TraceEvent.msg`의 *"받은 그대로 append 한다 — 판단하지 않는다"* 를 지킨다.
> 로거가 "마지막 단계인가"를 추론하면 위 `return` 행에서 틀린다.

`status`가 행마다 반복되는 것은 **의도한 비정규화**다. 로그 한 줄만 떼어 봐도 그때 물체가 어떤 상태였는지 알 수 있어야 증거로 쓰인다.

## 4-4. `fail_reason` ↔ BT 잎

사유는 **단계가 정한다.** 그래서 액션 enum에 없는 실패(타임아웃·서버 없음)도 갈 곳이 있다.

| BT 잎 (`Freeze` stage) | `fail_reason` | 비고 |
|---|---|---|
| `pick` | `pick_error` | |
| `nav` | `nav_error` | |
| `place` | `place_error` | |
| `return` | `nav_error` | 같은 `NavigateTo` 액션 |
| `dock` | `dock_error` | 미구현 |
| `scan` | **행 없음** | 아래 4-6 |

### `fail_detail` 카탈로그 — DB에 실제로 들어오는 문자열 전부

`task_manager.py:295 _reason_name()`이 `"{상수명}({코드})"` 로 만든다.
값의 경로: `ActionLeaf.update():414` → `Freeze.update():659` → `on_freeze():843` → 이 칸.

**모든 `ActionLeaf`가 공통으로 내는 3종**

| 값 | 코드 | 언제 |
|---|---|---|
| `TIMEOUT({n}s)` | `:378` | 단계별 상한 초과 (`pick`/`place` 120 s, `nav`/`return` 300 s) |
| `SERVER_UNAVAILABLE({서버이름})` | `:386` | `SERVER_WAIT_S`(5 s) 기다려도 액션 서버 없음 |
| `GOAL_REJECTED` | `:400` | 서버가 goal 거부 |

**`pick` → `pick_error` · 총 10종** (`PickCarrier.action`)

`NO_IK(1)` · `COLLISION(2)` · `NO_FLANGE(3)` · `NO_ATTACH(4)` · `SLIP(5)` · `OFF_FLANGE(6)` · `CANCELED(7)` + 공통 3종

**`nav` · `return` → `nav_error` · 각 7종** (`NavigateTo.action`)

`PLAN_FAIL(1)` · `BLOCKED(2)` · `CANCELED(3)` · `TOLERANCE_NOT_MET(4)` + 공통 3종

**`place` → `place_error` · 총 9종** (`PlaceCarrier.action`)

`NO_IK(1)` · `COLLISION(2)` · `PORT_OCCUPIED(3)` · `NOT_GRIPPED(4)` · `UNKNOWN_VARIANT(5)` · `CANCELED(6)` + 공통 3종

> ⚠️ **`CANCELED`가 지금은 실패로 기록된다.** `ok_fail_reasons`를 쓰는 건 주석처리된 PATROL 하나뿐이라(`:751`),
> mission의 `nav`·`ret`은 기본값 `()`이다. 지금은 Selector 자식이 `mission` 하나뿐이라 선점이 없어 발생하지 않지만,
> **배터리 선점 가지를 mission 위에 넣는 순간**(`:772`) 정상적인 선점이 전부 `nav_error`로 쌓인다.
> 그때 `nav`·`ret`에도 `ok_fail_reasons=(NavigateTo.Result.CANCELED,)`를 넣어야 한다. **스키마는 안 바뀐다.**

> ⚠️ **`NONE(0)`이 들어올 수 있다.** 서버가 `success=false`인데 `fail_reason=0`으로 돌려주면 나온다.
> 원인 없는 실패라 서버 쪽 버그 신호다. 걸러내지 않고 그대로 기록한다 — 집계에 뜨면 `pick_place_server` / `nav_server`를 보라는 뜻이다.

## 4-5. `port`가 대부분 비어 있는 이유

`task_manager`의 NAV 목표가 `TEST_LOADER` 상수(`:210`) 하나뿐이고, **순찰 중 발견** 방식이라 슬롯 번호를 지정받지 않는다.
당분간 `place` 행만 `test_loader` 고정이고 `pick` 행은 NULL이다. dispatcher가 붙어 슬롯을 지정하면 채워진다.

컬럼은 지금 만들어 둔다 — 나중에 `ALTER TABLE` 하는 것보다 빈 칸을 두는 게 싸다.

## 4-6. `scan` 행이 없는 이유

행의 키가 QR 원문인데, 판독에 실패하면 원문이 없다. 그리고 **판독 성공은 행이 필요 없다** —
`ScanLeaf` SUCCESS와 `pick` 시작은 1 tick(`TICK_PERIOD_S = 0.1`) 차이라, `pick` 행의 `started_at`이 곧
*"CarrierScan으로 QR 받아온 직후"* = `IN_TRANSIT` 진입 시각이다.

`SCAN`의 네 갈래 실패는 **전부 DB에 안 들어간다:**

| 값 | soft / hard |
|---|---|
| `NOT_FOUND(found=false) xN` `:499` | **soft** — 2회 재시도 후 순찰 복귀. 에러가 아니다 |
| `TIMEOUT(40s)` `:469` | hard — 그 자리 정지 |
| `SERVICE_UNAVAILABLE(/perception/carrier_scan)` `:475` | hard — 그 자리 정지 |
| `UNKNOWN_PAYLOAD('…')` `:505` | hard — 그 자리 정지 |

> 🔧 **hard 3종도 DB에 남기지 않는다 — 대신 없앤다.** `ScanLeaf.initialise():461`의 `self.soft = False` 를
> `True` 로 바꾸면 `ScanLeaf`가 내는 **모든** FAILURE가 soft가 되어 `Freeze:660`이 절대 얼리지 않는다.
> 판독이 안 되면 무조건 순찰로 돌아간다 — **SCAN 은 에러가 아니다.**
>
> ⚠️ 같이 봐야 할 것: `on_scan_not_found()`(쿨다운 30초)가 지금은 `found=false` 경로에서만 불린다.
> `TIMEOUT`·`SERVICE_UNAVAILABLE`도 soft가 되면 쿨다운 없이 즉시 재시도라, `carrier_code_reader`가
> 안 떠 있을 때 5초마다 계속 돈다. **세 경로 모두에서 부르는 게 맞다.**
>
> 📌 그래서 **`robot_log` 표는 만들지 않는다.** 표는 4개, `fail_reason`은 4개로 끝이다.

## 4-7. ⭐ `attempt` — 웹 복구로 같은 단계를 다시 시도할 때

`PLACE`가 `PORT_OCCUPIED`로 실패하면 **매거진은 아직 흡착에 붙어 있다.** 화면이 `/orchestrator/resume`으로
풀어주면 그 단계를 다시 시도한다. 이때 **새 미션이 아니라 같은 미션의 재시도**다 — 구조가 그렇게 생겼다.

```python
# Freeze.tick()  :640
if self.frozen:
    self.status = Status.RUNNING      # ← 얼어 있는 동안 RUNNING 을 돌려준다
    yield self
    return
yield from super().tick()
```

`Freeze`가 RUNNING을 돌려주므로 `memory=True`인 mission Sequence가 이 잎을 **"진행 중인 자식"으로 붙들고** 있다.
`frozen = False` 한 줄이면 자식이 `initialise()`부터 다시 돌아 goal 이 새로 나가고,
**`pick`·`nav`는 재실행되지 않는다.** 그래서 `run_id`는 그대로 두고 `attempt`만 올린다.

```
run_id  stage   attempt  succeeded  status      fail_reason  fail_detail
aaa…    pick    1        t          IN_TRANSIT  ·            ·
aaa…    nav     1        t          IN_TRANSIT  ·            ·
aaa…    place   1        f          FAILED      place_error  PORT_OCCUPIED(3)
aaa…    place   2        t          COMPLETED   ·            ·
aaa…    return  1        t          COMPLETED   ·            ·
```

**한 미션의 이야기가 한 묶음에 남는다.** 새 `run_id`를 발행하는 안은 `place` 행 하나짜리 기형 미션을 만든다 —
로봇이 이미 들고 있으므로 `pick`도 `nav`도 없는 미션이 되기 때문이다.

### 누가 세는가

`task_manager`가 `stage → 횟수` dict 로 들고 있고, **`ScanLeaf`가 `run_id`를 새로 발행할 때 비운다.**
미션이 바뀌면 `attempt`도 1로 돌아간다.

```python
def new_run(self):
    self.bb.run_id = str(uuid.uuid4())
    self._attempts = {}                 # ← 새 미션이면 초기화
    return self.bb.run_id
```

resume 서비스(`std_srvs/SetBool`, **상대이름** — 로봇마다 하나다)는 얼어붙은 `Freeze`를 알아야 하므로,
`on_freeze()`가 자기 자신을 넘긴다(인자 하나 추가). 푸는 것은 `node.frozen = False` 한 줄이다.

> ⚠️ **집계 주의 — `run_outcome` 뷰를 쓴다.** 위 로그에는 `FAILED` 행과 `COMPLETED` 행이 **같은 미션 안에** 있다.
> `WHERE status='FAILED'` 로 미션 실패를 세면 이 미션이 실패로도 성공으로도 잡힌다.
> 미션의 최종 결과는 **그 `run_id`의 마지막 행**이고, 그것이 §6의 `run_outcome`이다.

> 📌 `data=false`(포기)는 미구현이다. 로봇이 캐리어를 든 채 미션을 버리는 처리가 따로 필요하다.

---

# 5. 표 4 — `carrier_kind` (4행, 정적)

```sql
CREATE TABLE carrier_kind (
    kind_code    TEXT PRIMARY KEY,            -- 'MGZB'  QR 원문의 품목 코드
    family       TEXT NOT NULL,               -- 'magazine' | 'stack'
    color        TEXT NOT NULL,               -- 'orange'   | 'blue'
    carrier_type TEXT GENERATED ALWAYS AS (family || '_' || color) STORED UNIQUE,
                                              -- 'magazine_blue'
                                              --   ②「어떤 물체인지」이자 grasp/place.yaml 키

    length_mm    NUMERIC(6,1) NOT NULL,       -- 긴 변
    width_mm     NUMERIC(6,1) NOT NULL,       -- 짧은 변
    height_mm    NUMERIC(6,1) NOT NULL,       -- 전체 높이 = 플랜지 판 윗면
    mass_kg      NUMERIC(5,2) NOT NULL,

    spec_extra   JSONB NOT NULL DEFAULT '{}'::jsonb   -- 종류별로 축이 다른 것만
);

INSERT INTO carrier_kind
    (kind_code, family, color, length_mm, width_mm, height_mm, mass_kg, spec_extra) VALUES
 ('MGZO', 'magazine', 'orange', 250.0, 140.0, 142.0, 1.00, '{"slots":20,"slot_pitch_mm":5}'),
 ('MGZB', 'magazine', 'blue',   300.0, 200.0, 106.0, 1.00, '{"slots":12,"slot_pitch_mm":5}'),
 ('STKO', 'stack',    'orange', 328.6, 135.9,  77.7, 1.00, '{"trays":6,"tray_pitch_mm":7.62}'),
 ('STKB', 'stack',    'blue',   328.6, 135.9,  97.0, 1.00, '{"trays":8,"tray_pitch_mm":7.62}');
```

수치는 전부 `isaacpjt/assets/gen_carrier_assets.py`의 규격 요약(`:19~52`)에서 옮긴 것이다.
`edcdf14`에서 네 에셋 플랜지 판이 80×80으로 통일되고 두께만 6/10으로 갈린 것까지 반영했다.

## 5-1. `carrier_type`을 생성열로 둔 이유

`carrier_code.py`가 이미 `FAMILY = {"MGZ": "magazine", "STK": "stack"}` 과 `COLOR = {"O": "orange", "B": "blue"}` 를 갖는다.
`carrier_type = family + '_' + color` 로 두면 **`grasp.yaml` / `place.yaml` 키와 `family`/`color`가 절대 어긋날 수 없다.**

그리고 `task_manager`의 종류 조회가 한 줄이 된다 — 매핑표도, DB 조회도 없다:

```python
info    = carrier_code.parse_code(res.payload)   # "F1-MGZB-1"
variant = f"{info.family}_{info.color}"          # "magazine_blue" = grasp/place.yaml 키
```

> 📌 이것이 성립하려면 `grasp.yaml:98` · `place.yaml:25`의 `magazines:` 절 키를
> `magazine_1_orange` → `magazine_orange`, `magazine_2_blue` → `magazine_blue` 로 바꾸고
> `stack_orange` · `stack_blue` 를 **추가**해야 한다. `pick_place_server.py:176-177`이
> `yaml[...]["magazines"]` 로 읽으므로, 스택이 들어가면 절 이름도 `carriers:` 로 바꿔야 한다.
>
> 스택 항목이 없으면 표 3에는 이렇게만 쌓인다 —
> `grasp.yaml`에 없으면 `pick_place_server.py:218`이 KeyError → `pick` 행에 `NO_FLANGE(3)`,
> `place.yaml`에 없으면 `:310`이 None → `place` 행에 `UNKNOWN_VARIANT(5)`.

## 5-2. 넣지 않은 칸과 그 주인

같은 숫자가 두 곳에 있으면 반드시 갈라진다. **다른 파일이 이미 주인인 값은 넣지 않았다.**

| 뺀 칸 | 주인 |
|---|---|
| `variant` | = `carrier_type`. 같은 값이다 |
| `base_asset` | 개체 파일명은 페이로드에서 나온다 (`carrier_code.usd_name()` → `F1_MGZB_1.usda`) |
| `flange_plate_mm` · `flange_thick_mm` | **`grasp.yaml:98`의 `flange_size`** (`[0.08, 0.08, 0.006]`). 로봇이 파지에 쓰는 값이라 파일이 주인 |
| `qr_size_mm` | **`frames.yaml:165`의 `label_side_m: 0.050`** (+ `data_side_m`, 판독 거리 실측표). 생성물이라 파일이 주인 |

남은 `L×W×H` · `mass_kg` · `spec_extra`는 **`gen_carrier_assets.py`에만 있고 어느 yaml에도 없다.**
DB가 유일한 조회처라 여기 둘 값이 맞다.

---

# 6. 뷰 5개

append-only의 유일한 비용이 *"지금 상태가 뭐냐"* 를 바로 못 읽는 것이다. 뷰가 그것을 갚는다.

```sql
-- 캐리어별 최신 행 = 현재 상태
CREATE VIEW magazine_latest AS
SELECT DISTINCT ON (qr_payload) *
FROM magazine_log ORDER BY qr_payload, log_id DESC;

CREATE VIEW stack_latest AS
SELECT DISTINCT ON (qr_payload) *
FROM stack_log ORDER BY qr_payload, log_id DESC;

-- 매거진 · 스택을 한 줄로 (표 1이 잇는다)
CREATE VIEW production_tracking AS
SELECT p.magazine_payload,  km.carrier_type AS magazine_type,
       m.robot_id AS magazine_robot,  m.status AS magazine_status,
       m.fail_reason AS magazine_fail, m.fail_detail AS magazine_fail_detail,
       m.ended_at AS magazine_at,
       p.stack_payload,     ks.carrier_type AS stack_type,
       s.robot_id AS stack_robot,     s.status AS stack_status,
       s.fail_reason AS stack_fail,   s.fail_detail AS stack_fail_detail,
       s.ended_at AS stack_at
FROM carrier_pair p
LEFT JOIN magazine_latest m ON m.qr_payload = p.magazine_payload
LEFT JOIN stack_latest    s ON s.qr_payload = p.stack_payload
LEFT JOIN carrier_kind   km ON km.kind_code = m.kind_code
LEFT JOIN carrier_kind   ks ON ks.kind_code = s.kind_code;

-- 전체 집계용 — 표 2 UNION ALL 표 3
CREATE VIEW carrier_log AS
SELECT 'MAGAZINE' AS role, log_id, run_id, qr_payload, kind_code, plant_code, robot_id,
       stage, attempt, started_at, ended_at, duration_sec, succeeded, status,
       fail_reason, fail_detail, port
FROM magazine_log
UNION ALL
SELECT 'STACK',           log_id, run_id, qr_payload, kind_code, plant_code, robot_id,
       stage, attempt, started_at, ended_at, duration_sec, succeeded, status,
       fail_reason, fail_detail, port
FROM stack_log;

-- 미션별 최종 결과 = 그 run_id 의 마지막 행. 성공률 집계는 전부 이걸로 (§4-7)
CREATE VIEW run_outcome AS
SELECT DISTINCT ON (run_id)
       run_id, role, qr_payload, kind_code, plant_code, robot_id,
       status AS final_status, fail_reason, fail_detail,
       (SELECT max(attempt) FROM carrier_log c2 WHERE c2.run_id = c.run_id) AS max_attempt
FROM carrier_log c
ORDER BY run_id, log_id DESC;
```

> 한 번도 이송하지 않은 개체는 `production_tracking`에서 종류가 NULL이다 — **종류는 로그 행이 들고 오기 때문**이다.
> 표 1은 페이로드 두 개만 갖는다.

---

# 7. 이 DB로 무엇을 답하는가

```sql
-- 한 미션의 이력
SELECT upper(stage) AS 단계,
       to_char(ended_at, 'YYYY.MM.DD HH24:MI') AS 시각,
       round(duration_sec::numeric, 1) AS 소요초,
       status, fail_reason, fail_detail
FROM magazine_log WHERE run_id = '…' ORDER BY log_id;
```
```
 단계  |      시각        | 소요초 |   status   | fail_reason | fail_detail
-------+------------------+--------+------------+-------------+------------------
 PICK  | 2026.09.21 12:17 |   43.3 | IN_TRANSIT |             |
 NAV   | 2026.09.21 12:20 |  124.3 | IN_TRANSIT |             |
 PLACE | 2026.09.21 12:22 |   60.9 | FAILED     | place_error | PORT_OCCUPIED(3)
```

> ⚠️ **미션 단위 집계는 반드시 `run_outcome`을 쓴다.** 재시도한 미션에는 `FAILED` 행과 `COMPLETED` 행이
> 같이 들어 있어서, `magazine_log`를 직접 세면 한 미션이 실패로도 성공으로도 잡힌다(§4-7).

```sql
-- 종류별 성공률 (미션 단위)
SELECT k.carrier_type,
       count(*)                                            AS 미션,
       count(*) FILTER (WHERE r.final_status = 'COMPLETED') AS 성공,
       count(*) FILTER (WHERE r.max_attempt > 1)            AS 사람이_살린것
FROM run_outcome r LEFT JOIN carrier_kind k USING (kind_code)
GROUP BY k.carrier_type;

-- 공장별 통과량 — plant_code 가 생성열이라 파서 없이 나온다 (§8-3)
SELECT plant_code,
       count(*) FILTER (WHERE final_status = 'COMPLETED') AS 성공,
       count(*) FILTER (WHERE final_status = 'FAILED')    AS 실패
FROM run_outcome GROUP BY plant_code ORDER BY plant_code;

-- 어느 단계 · 어느 사유에 실패가 몰리는가  ← 04 §12-1 이 요구한 것
SELECT stage, fail_reason, fail_detail, count(*)
FROM carrier_log WHERE NOT succeeded
GROUP BY 1,2,3 ORDER BY 4 DESC;

-- 단계별 평균 소요 (시뮬 시각 기준)
SELECT stage, round(avg(duration_sec)::numeric,1) AS 평균초, count(*)
FROM carrier_log WHERE succeeded GROUP BY stage;

-- 지금 이송 중
SELECT role, robot_id, qr_payload, stage, ended_at
FROM carrier_log WHERE status = 'IN_TRANSIT' AND stage IN ('pick','nav');

-- 로트 현황 — 매거진 · 스택 한 줄
SELECT * FROM production_tracking ORDER BY magazine_at DESC NULLS LAST;
```

---

# 8. 설계 결정 기록

## 8-1. 왜 append-only인가

`05` §2-8의 첫 번째 설계 의도가 *"추적성 데이터를 나중에 고치면 그 순간 증거 능력이 사라진다"* 였다.
행 하나를 `IN_TRANSIT` → `COMPLETED`로 UPDATE하는 안도 검토했지만, 그러면

- 중간 단계(`pick` 성공 시각, `nav` 소요)가 남지 않는다
- 노드가 죽으면 행이 영구 `IN_TRANSIT`으로 남아 **고아 행 정리(reaper)** 가 필요하다
- 정정이 곧 덮어쓰기가 된다

append-only면 셋 다 없다. 정정은 **삭제가 아니라 정정 행 추가**다.

## 8-2. 왜 표를 둘로 나눴나 — 그리고 그 대가

매거진과 스택은 **서로 다른 시각에 다른 로봇이** 옮긴다. 한 행에 열 칸을 다 넣으면
매거진 처리 직후부터 스택이 생길 때까지 오른쪽 절반이 NULL이고, 그 NULL이 *"아직"* 인지 *"영영 없음"* 인지 구분되지 않는다.

**대가는 둘이다:**

1. 전체 집계가 `UNION ALL`이 된다 → `carrier_log` 뷰가 갚는다
2. **"한 로봇이 동시에 하나만 든다"를 DB가 못 막는다.** UNIQUE INDEX는 한 테이블 안에서만 유효하다.
   다만 BT의 `mission` 가지가 하나뿐이고 로봇당 `task_manager`가 하나라 **애플리케이션이 이미 보장**한다.
   → 표 5가 들어오면 `task`의 부분 UNIQUE INDEX가 **작업 단위로는** 이것을 DB 레벨로 끌어올린다(§10-4).
   표 2·3 사이는 여전히 못 막지만, 미션을 만드는 쪽이 하나로 좁혀지므로 실질적으로 같은 보장이 된다.

## 8-3. 왜 파서를 SQL에 두지 않았나

`gen_carrier_assets.py:39`의 QR 내용이 `"F1-260921-MGZO-1"` — **토큰 4개**다.
`carrier_code.py`의 `kind_of()`가 인덱스가 아니라 *"아는 코드와 같은 토큰 찾기"* 로 되어 있고,
자가 테스트에 `assert kind_of("F1-260921-MGZO-1") == "MGZO"` 까지 있는 이유가 이것이다.

`split_part(qr_payload, '-', 2)` 같은 생성열을 쓰면 **로트 날짜를 넣는 순간 `260921`을 반환한다.**
정규식으로 같은 규칙을 SQL에 복사하면 파서가 두 벌이 된다.

> **파서는 `carrier_code.py` 하나. DB는 그 결과를 평범한 컬럼에 저장한다.**

📌 **첫 토큰은 예외다.** 날짜가 끼어 밀리는 것은 **두 번째 토큰부터**이고, 공장 코드 `F1`은 어떤 경우에도
첫 자리다. 그래서 `plant_code`만은 `split_part(qr_payload, '-', 1)` 생성열로 둬도 안전하다 —
이 경고는 **`kind_code`에만** 해당한다.

## 8-4. 왜 `SCAN` 실패가 4개 사유에 없나

말 그대로 **에러가 아니기 때문**이다. `found=false`는 `ScanLeaf:497`에서 `self.soft = True`가 되고
`Freeze:660`이 얼리지 않아 순찰로 돌아간다. 물건은 그 자리에 그대로 있으므로 다음 순찰에 다시 보인다.
미션이 시작되지도 않았으니 남길 행도 없다. (hard 3종도 §4-6대로 soft 로 바꿔 없앤다.)

## 8-5. 왜 `carriers.yaml`을 없애나

순수 관계형 데이터(16행 × 6열)이고 DB가 더 잘한다. 그리고 **독자가 사라진다** —
유일한 독자가 `task_manager.py:793`의 `lookup_carrier_id()`(`:911`)인데,
그 docstring이 *"QR 이 종류만 담아서 개체는 못 가린다"* 이다. 새 페이로드는 **개체를 가린다.**
`F1-MGZB-1` 자체가 개체 ID라 이 조회 함수가 통째로 없어진다.

### 반대로 파일로 남기는 것

| 파일 | 왜 |
|---|---|
| `grasp.yaml` · `place.yaml` | `pick_place_server`가 **매 동작마다** 읽는다. DB에 넣으면 **DB가 죽으면 로봇이 못 움직인다** |
| `frames.yaml` | `measure_layout.py` 생성물. 헤더에 *"이 파일을 직접 손으로 고치지 않는다"* 고 적혀 있다 |

> **경계선: 로봇이 움직이는 데 필요한 것은 파일, 무슨 일이 있었는지 남기는 것은 DB.**

---

# 9. 이 표를 채우는 배선 (요약)

상세는 구현 시 `05_QR_Traceability.md` §2-9를 이 내용으로 갱신한다.

| 만들 것 | 내용 |
|---|---|
| `sql/001_schema.sql` | 위 ENUM 2 · 표 4 · 뷰 5 · 인덱스 |
| `sql/002_seed.sql` | `carrier_kind` 4행 · `carrier_pair` 8행 |
| `cobot3_orchestrator/event_logger.py` | `/trace/event` 구독 → 큐 → psycopg INSERT. 스풀 + 재접속 |

| 고칠 것 | 내용 |
|---|---|
| `msg/TraceEvent.msg` | `robot_id` · `run_id` · `status` · `attempt` · `wall_stamp` **5필드** 추가. `CMakeLists.txt`는 이미 등록돼 있어 안 고친다 |
| `task_manager.py` | ⓐ `initialise()` 두 곳에 `started_at`/`started_sim` ⓑ `ScanLeaf` 성공부에 `new_run()` ⓒ **`Freeze.update()`에서 발행** ⓓ `__init__`에 publisher·`robot_id` ⓔ `emit_trace()` / `_status()` / `attempt_of()` 추가, `lookup_carrier_id()` 삭제 ⓕ `ScanLeaf.initialise():461` 을 soft 로(§4-6) ⓖ `on_freeze()`에 인자 하나 + `/orchestrator/resume` 서비스(§4-7) |
| `carrier_code.py` 용어 | `line=parts[0]` → `plant=parts[0]`, docstring `라인 코드` → `공장 코드`. **라인은 페이로드에 없다**(§11) |
| `carrier_code.py` | `isaacpjt/assets/` → `cobot3_orchestrator/` (ament 패키지가 아니라 노드가 import 못 한다) |
| `setup.py` · `package.xml` · `mission_nodes.launch.py` | 엔트리포인트 · 의존성 · **네임스페이스 없이 전역 1개**로 로거 추가 |

| 지울 것 |
|---|
| `src/cobot3_bringup/config/carriers.yaml` · `task_manager.py`의 `CARRIERS_YAML` · `self.carriers` · `lookup_carrier_id()` |

**발행 지점이 `Freeze` 하나인 것이 핵심이다.** `pick` · `nav` · `place` · `return` 넷이 전부 `Freeze`로 감싸져 있고
(`:704` `:711` `:717` `:728`), 이 데코레이터가 자식의 SUCCESS도 FAILURE도 다 보며 `self.stage`를 이미 들고 있다.
클래스 하나만 고치면 네 단계가 전부 걸린다.

---

# 10. 관제 화면이 추가로 요구하는 표 2개

웹 관제 UI(2대 운용 · 작업 배정 · 산출물 자동 회수)를 얹을 때 **DB에 새로 들어가는 것은 두 표뿐**이다.
선반 좌표 · 스테이션 place 자세 · 라우팅 규칙은 §8-5의 경계선 그대로 **파일이 주인으로 남는다.**

## 10-1. ⭐ 판정 기준

> **여러 곳이 동시에 쓰거나 · 프로세스가 죽어도 남아야 하거나 · 집계 질의의 대상인 것만 DB.**
> 한 주인이 시작할 때 한 번 읽는 값은 **파일**. 매 틱 바뀌고 지나가면 그만인 값은 **아무 데도 저장하지 않는다.**

§8-5의 *"로봇이 움직이는 데 필요한 것은 파일, 무슨 일이 있었는지 남기는 것은 DB"* 를 화면 요구사항까지 넓힌 것이다.

📌 **화면에서 편집한다는 사실 자체는 DB 사유가 아니다.** 편집기는 파일도 쓴다 —
`isaacpjt/tools/capture_pose.py`가 이미 `taught_poses.yaml`에 그렇게 쓰고 있다.

| 화면이 요구한 것 | 판정 | 왜 |
|---|---|---|
| 선반 좌표 · 층별 티칭 자세 | **파일** | 주인이 하나(엔지니어), 미션 시작 때 한 번 읽는다 |
| 스테이션 place 자세 | **파일** | `place.yaml`을 `pick_place_server`가 매 동작마다 읽는다. DB로 옮기면 §8-5가 경고한 *"DB가 죽으면 로봇이 못 움직인다"* 에 정확히 걸린다 |
| 라우팅 규칙 | **파일** | 선형 체인 3홉짜리 상수. 바뀌는 빈도가 배포 주기보다 낮다 |
| 로봇 위치 · 배터리 · 현재 상태 | **저장 안 함** | 초당 수십 번 바뀌고 과거값을 아무도 안 본다. ROS 토픽 → WebSocket 패스스루로 끝. DB에 쓰면 쓰기 부하만 남는다 |
| 작업 큐 · 순서 | **표 5** | 화면이 쓰고 orchestrator가 읽는 **공유 가변 상태**. 웹이 재시작해도 배정이 남아야 하고 두 로봇 큐의 순서에 트랜잭션이 필요하다 |
| 산출물 회수 대기 | **표 6** | *"n초 뒤에 큐잉"* 은 **타이머가 도는 중에 프로세스가 죽으면 사라지는** 상태다 |
| 중단 지점(`resume_point`) | **표 5의 칸** | 중단 지점은 로봇의 속성이 아니라 *"그 작업을 어디까지 했나"* 다. → 별도 `robot` 표가 필요 없어진다 |
| 선반 점유 lock | **칸 없음** | 표 5에서 파생된다(§10-4) |

> 이 기준을 적용하면 `shelf` · `station` · `routing_rule` · `robot` 표가 전부 사라진다.
> 특히 **`robot` 표에 남을 칸이 `resume_point` 하나뿐**이었고, 그게 작업의 속성으로 옮겨가면서 표 자체가 없어졌다.

## 10-2. 표 5 — `task` (작업 큐)

```sql
CREATE TYPE task_kind   AS ENUM ('SCAN', 'RECOVER');
CREATE TYPE task_status AS ENUM ('QUEUED', 'RUNNING', 'DONE', 'FAILED');

CREATE TABLE task (
    task_id         BIGSERIAL PRIMARY KEY,
    robot_id        TEXT        NOT NULL,      --   'robot1' · 표 2·3과 같은 값
    kind            task_kind   NOT NULL,
    target_ref      TEXT        NOT NULL,      -- ① 'shelf_01' | 'test_loader' — yaml 키 (FK 아님)
    queue_order     INT         NOT NULL,      --   화면 드래그 재정렬
    status          task_status NOT NULL DEFAULT 'QUEUED',

    run_id          UUID,                      -- ② ScanLeaf 성공 순간 stamp → 표 2·3과 공유
    resume_pass     INT,                       -- ③ 중단한 층
    resume_progress DOUBLE PRECISION,          -- ③

    created_at      TIMESTAMPTZ NOT NULL DEFAULT now(),
    started_at      TIMESTAMPTZ,
    ended_at        TIMESTAMPTZ
);

CREATE INDEX idx_task_run ON task (run_id);
```

| # | 칸 | 왜 이 모양인가 |
|---|---|---|
| ① | `target_ref` | **FK가 아니다.** 설정의 주인이 파일이라 DB가 참조 무결성을 걸 수 없다. *"티칭 미완료 선반은 배정 불가"* 는 배정 시점에 백엔드가 yaml을 읽어서 검사한다 |
| ② | `run_id` | 배정 시점에는 NULL이다. **QR을 읽어야 미션이 시작**되므로(§4-6) `ScanLeaf` 성공 순간 발행된 값을 여기에 찍는다 |
| ③ | `resume_*` | 웹 복구용. §4-7이 *"같은 `run_id` + `attempt`"* 로 정해지면서 의미가 고정됐다 — **"이 미션을 어디까지 했나"** 다. 재개가 같은 미션이라 `task.run_id`를 덮을 일도 없다 |

**큐는 별도 표가 아니다** — `robot_id` + `queue_order`가 큐이고, 드래그 재정렬은 UPDATE 한 번이다.
회수 우선순위도 `priority` 칸 없이 `kind='RECOVER'` 를 앞으로 정렬하면 된다.

## 10-3. 표 6 — `pending_pickup` (회수 대기)

```sql
CREATE TYPE pickup_status AS ENUM ('WAITING', 'QUEUED', 'DONE', 'GAVE_UP');

CREATE TABLE pending_pickup (
    pending_id    BIGSERIAL PRIMARY KEY,
    station_ref   TEXT          NOT NULL,      --   'test_loader' — yaml 키
    source_run_id UUID          NOT NULL,      -- ① 이 산출물을 만든 미션 (표 2의 run_id)
    ready_at      TIMESTAMPTZ   NOT NULL,      --   place 완료 + stations.yaml 의 process_sec
    retry_count   INT           NOT NULL DEFAULT 0,   -- ② 가 보니 없더라 → 재예약
    status        pickup_status NOT NULL DEFAULT 'WAITING',
    task_id       BIGINT REFERENCES task(task_id),    --   큐잉된 회수 작업

    CONSTRAINT pickup_task_when_queued
        CHECK (status IN ('WAITING', 'GAVE_UP') OR task_id IS NOT NULL)
);

CREATE INDEX idx_pickup_due ON pending_pickup (ready_at) WHERE status = 'WAITING';
```

① **`expected_payload` 를 두지 않았다.** 복사하지 않아도 조인으로 나온다 — §5-2의 *"같은 숫자가 두 곳에 있으면 반드시 갈라진다"* 와 같은 이유다.

```sql
SELECT cp.stack_payload                        -- 이 스테이션에서 나올 산출물
FROM pending_pickup pp
JOIN magazine_log ml ON ml.run_id = pp.source_run_id AND ml.stage = 'place'
JOIN carrier_pair cp ON cp.magazine_payload = ml.qr_payload
WHERE pp.pending_id = …;
```

> 📌 이 조인은 §3의 `carrier_pair` 8쌍이 확정돼 바로 결과가 나온다 — 회수하러 가기 전에 무엇이 나올지 안다.

② `retry_count`가 이 표를 못 없애는 이유다. *"언제 회수 가능한가"* 만이면 `magazine_latest`에서 파생할 수 있지만,
**비전으로 확인했더니 아직 없어서 재예약** 한 횟수는 어디에도 파생할 근거가 없다.

## 10-4. 🔧 선반 lock을 칸으로 두지 않는 이유

*"두 로봇이 같은 선반에 동시에 진입하지 않는다"* 는 `locked_by` 같은 칸 없이 인덱스가 거부한다.

```sql
-- 한 선반을 동시에 두 작업이 RUNNING 할 수 없다
CREATE UNIQUE INDEX task_one_running_per_ref
    ON task (target_ref) WHERE status = 'RUNNING';

-- 한 로봇이 동시에 두 작업을 RUNNING 할 수 없다   ← §8-2 의 '대가 2' 를 부분적으로 갚는다
CREATE UNIQUE INDEX task_one_running_per_robot
    ON task (robot_id) WHERE status = 'RUNNING';

-- 같은 로봇 큐 안에서 순서가 겹치지 않는다
CREATE UNIQUE INDEX task_queue_order
    ON task (robot_id, queue_order) WHERE status = 'QUEUED';
```

lock을 칸으로 들면 **해제를 잊은 행(고아 lock)** 이 생기고 청소기가 필요해진다 — §8-1이 `IN_TRANSIT` UPDATE 안을 버린 이유와 정확히 같다.
파생으로 두면 작업이 끝나는 순간 lock도 같이 사라진다.

## 10-5. `run_id` — 두 세계를 잇는 칸

| 시점 | 일어나는 일 | `run_id` |
|---|---|---|
| 화면에서 배정 | `task` 행 생성 · `QUEUED` | NULL |
| QR 판독 성공 | `ScanLeaf`가 `uuid4()` 발행 → `task`에 stamp | **발행** |
| `pick`·`nav`·`place`·`return` | `magazine_log`에 같은 값으로 최대 4행 | 공유 |
| 사이클 완료 | `task.status = DONE` | 고정 |

*"이 작업이 어떻게 됐나"* 가 `task` → `run_id` → `carrier_log` 조인 한 번이다. **로그 화면의 작업별 필터가 새 표 없이 성립한다.**

## 10-6. 파일이 계속 주인인 것

| 파일 | 담는 것 | 화면이 고치는 법 |
|---|---|---|
| `taught_poses.yaml` | 선반 좌표 + 층별 티칭 자세 | *"현재 자세로 저장"* 이 append. `capture_pose.py`가 이미 하는 일 |
| `place.yaml` | 스테이션 place 6D | 화면이 값만 덮어쓴다 |
| `grasp.yaml` | 종류별 파지 레시피 | 편집 대상 아님 (§8-5) |
| `frames.yaml` | `measure_layout.py` 생성물 | 손대지 않는다 (§8-5) |
| **`stations.yaml`** (신설) | 처리시간 · 산출물 종류 · 완료 신호 · capacity · 라우팅 체인 | 지금 `task_manager.py:210`의 `TEST_LOADER` 상수가 하던 일을 여기로 옮긴다 |

> 📌 **reload 신호는 만들지 않는다.** 노드는 **뜰 때 한 번** 읽고(`pick_place_server.py:176-177`),
> 설정을 바꾸면 **노드를 재시작한다.** 새 서비스도 토픽도 없다.
> 운전 중에 설정을 고칠 일이 없다는 전제이고, 그 전제가 깨지면 그때 미션 시작마다 재독하는 쪽으로 바꾼다
> (파일 저장을 `os.replace()` 로 원자적으로 하는 것만 지키면 배선 없이 된다).

## 10-7. 배선 (§9에 더해지는 것)

| 만들 것 | 내용 |
|---|---|
| `sql/003_task.sql` | ENUM 3 · 표 5·6 · 부분 UNIQUE INDEX 3 |
| 웹 백엔드 | 표 5·6에 대한 **유일한 writer이자 스케줄러.** `ready_at` 도래 → `task` 행 생성 |
| `task_manager.py` | 큐 소비 — `QUEUED` 중 `queue_order` 최소를 `RUNNING`으로. `ScanLeaf`에서 이미 만드는 `run_id`를 `task`에도 stamp |

📌 **`event_logger`는 손대지 않는다.** 표 5·6은 트레이스가 아니라 제어 상태라 `/trace/event` 경로를 타지 않는다.
§1의 *"DB 커넥션을 가진 노드가 하나뿐"* 은 **로깅 경로에 한해** 유지되고, 큐 소비는 별도 커넥션이 된다.

---

# 11. 결정 기록 · 남은 것

**표 1~4는 미정이 없다.** 먼저 열려 있던 넷은 이렇게 닫혔다.

| 열려 있던 것 | 결론 | 어디에 |
|---|---|---|
| `SCAN` hard 실패를 어디에 (`robot_log`) | **표를 만들지 않는다.** 대신 `ScanLeaf`를 전부 soft 로 바꿔 hard 실패 자체를 없앤다. **DB는 SCAN 성공 이후부터** | §4-6 |
| 웹 복구 후 `run_id`를 이어갈지 | **같은 `run_id` + `attempt`.** 제약은 `UNIQUE (run_id, stage, attempt)`, 집계는 `run_outcome` | §4-7 · §6 |
| 화면이 yaml을 고쳤을 때 reload | **만들지 않는다.** 노드는 뜰 때 한 번 읽고, 설정을 바꾸면 재시작한다 | §10-6 |
| 페이로드에 라인이 없다 | **공장만 쓴다.** `F1`·`F2`·`F3` = 공장 1~3. 라인은 QR 이 복잡해져 의도적으로 뺐다. `plant_code` 생성열로 공장별 집계는 된다 | §8-3 · §9 |

> 🔧 마지막 항목은 **용어 정리를 남긴다** — `carrier_code.py`가 첫 토큰을 `line` 이라 부르고 docstring도
> *"라인 코드"* 라고 적는데, 실제로는 공장이다. `plant` 로 고쳐야 나중에 진짜 라인이 생겨도 안 부딪힌다.

## 남은 것 — 전부 뒤 단계에서

| # | 내용 | 언제 |
|---|---|---|
| 1 | `/orchestrator/resume` 의 `data=false`(포기) — 로봇이 캐리어를 든 채 미션을 버리는 처리 | resume 구현 때 |
| 2 | `DOCK` 실패(`dock_error`)를 어디에 — `return` 처럼 캐리어 로그에 붙일지, 미션 밖 도킹은 어떻게 할지 | `DOCK` 구현 때 |
| 3 | 표 5·6(§10)은 **제안 상태** | 관제 화면을 실제로 붙일 때 |

**표 1~4 · 뷰 5개 · §9 배선은 지금 그대로 구현할 수 있다.**
