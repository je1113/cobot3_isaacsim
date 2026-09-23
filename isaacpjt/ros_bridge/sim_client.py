"""
sim_backend.py 용 클라이언트 — 시스템 python(3.12)에서 쓴다. Isaac/rclpy
의존성이 전혀 없는 순수 stdlib 라서 pick_place_server/nav_server/
carrier_code_reader 노드가 그대로 import 해서 쓴다.

    from sim_client import SimClient
    sim = SimClient()
    sim.call("reset_magazine", robot_id="robot1")
    sim.get_status()
"""

import json
import os
import socket
import threading
import time


class SimClientError(RuntimeError):
    pass


class SimClient:
    def __init__(self, host=None, port=None):
        self.host = host or os.environ.get("SIM_BACKEND_HOST", "127.0.0.1")
        self.port = int(port or os.environ.get("SIM_BACKEND_PORT", "8765"))
        self._lock = threading.Lock()
        self._next_id = 0

    def _new_conn(self, read_timeout_s=5.0):
        # 연결 자체는 로컬호스트라 5s 로 충분하다. 그 뒤 읽기 타임아웃은
        # read_timeout_s 로 늘린다 — GUI(렌더 활성) 모드에서는 world.step()
        # 하나가 headless 보다 훨씬 오래 걸려서, 이걸 고정 5s 로 두면 서버가
        # 아직 일하고 있는데도 클라이언트가 먼저 TimeoutError 를 낸다(실측:
        # pick_phase2_finish 도중 GUI 로 재현). socket 의 timeout 은 connect
        # 뿐 아니라 그 뒤 모든 blocking 연산(읽기 포함)에도 적용된다.
        s = socket.create_connection((self.host, self.port), timeout=5.0)
        s.settimeout(read_timeout_s)
        return s.makefile("rwb")

    def call(self, method, timeout_s=60.0, connect_retry_s=20.0, **params):
        """블로킹 호출 — 결과가 올 때까지 기다린다. 오래 걸리는 호출
        (pick_phase1_approach 등) 은 진행 중 get_status() 를 별도 연결로
        폴링해서 액션 feedback 을 만든다.

        ★ 연결 자체가 안 되는 경우(sim_backend 가 아직 안 떴거나 재시작
        중이라 ConnectionRefusedError 등)를 SimClientError 로 감싸지 않고
        그냥 흘려보냈었다 — 호출하는 쪽(carrier_code_reader/nav_server/
        pick_place_server)은 전부 SimClientError 만 잡고 있어서, 원시
        OSError 가 그대로 서비스 콜백 밖으로 튀어나가 rclpy executor 가
        그 예외를 못 삼키고 노드 자체가 죽었다(실측 재현: carrier_code_reader
        가 "ConnectionRefusedError" 로 process died). sim_backend 를
        재시작하는 동안 터미널을 다시 실행해도 안전하려면 이 클라이언트가
        연결 실패를 "그 한 번의 SimClientError" 로 통일해서 돌려줘야 한다.

        ★ connect_retry_s — sim_backend 가 재시작 중이면(stage 를 다시 열고
        로봇을 다시 세우는 데 수십 초가 걸린다) 포트가 잠깐 안 열려 있어
        ConnectionRefusedError 가 난다. 그 창을 그냥 에러로 흘리면 순찰
        중이던 노드가 그 한 번의 실패로 상태를 잃는다 — 그래서 "연결 자체가
        안 되는" 실패만 이 시간 동안 재시도한다. 연결된 뒤(요청을 이미
        보낸 뒤)의 실패는 재시도하지 않는다 — 그 요청이 이미 물리적으로
        절반쯗 실행됐을 수 있어서, 같은 걸 다시 보내면 동작이 중복될 수
        있다."""
        with self._lock:
            self._next_id += 1
            req_id = self._next_id

        deadline = time.monotonic() + connect_retry_s
        while True:
            try:
                f = self._new_conn(read_timeout_s=timeout_s + 5.0)
                break
            except OSError as e:
                if time.monotonic() >= deadline:
                    raise SimClientError(
                        f"{method}: sim_backend 와 연결할 수 없다 "
                        f"({connect_retry_s:.0f}s 재시도 끝 — {e})") from e
                time.sleep(0.5)

        try:
            try:
                f.write((json.dumps({"id": req_id, "method": method, "params": params,
                                     "timeout_s": timeout_s}) + "\n").encode())
                f.flush()
                line = f.readline()
                if not line:
                    raise SimClientError(f"{method}: 연결이 끊겼다 (응답 없음)")
                resp = json.loads(line)
            finally:
                f.close()
        except OSError as e:
            raise SimClientError(f"{method}: sim_backend 와 통신할 수 없다 ({e})") from e
        except json.JSONDecodeError as e:
            raise SimClientError(f"{method}: 응답을 파싱할 수 없다 ({e})") from e
        if "error" in resp and resp["error"] is not None:
            raise SimClientError(f"{method}: {resp['error']}")
        return resp.get("result")

    def get_status(self, robot_id=None):
        """짧은 타임아웃의 폴링 전용 호출. 큐를 거치지 않아 빠르다.
        robot_id 를 안 주면 sim_backend 쪽 기본값(robot1)을 본다 — 로봇
        두 대를 한 소켓으로 관리하므로 로봇별 phase 를 보려면 넘겨야 한다.

        connect_retry_s 를 짧게 둔다 — 이건 폴링용이라 sim_backend 가
        아예 안 떠 있을 때도 자주 불린다. call() 의 기본 20s 재시도를
        그대로 쓰면 sim_backend 가 없는 동안 폴링 하나하나가 20s 씩
        블로킹돼서 "빠른 폴링"이라는 이 메서드의 목적이 깨진다."""
        params = {"robot_id": robot_id} if robot_id else {}
        return self.call("get_status", timeout_s=3.0, connect_retry_s=3.0, **params)
