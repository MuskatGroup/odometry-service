"""Real API tests. Set RUN_API_TESTS=1 after building the .NET project."""

import json
import os
import socket
import subprocess
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import pytest

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(os.environ.get("RUN_API_TESTS") != "1", reason="Set RUN_API_TESTS=1")


def call(base, route, value=None, method=None):
    body = json.dumps(value).encode() if value is not None else None
    request = Request(base + route, data=body, method=method, headers={"Content-Type": "application/json"})
    try:
        with urlopen(request, timeout=3) as response:
            return response.status, json.loads(response.read())
    except HTTPError as exc:
        return exc.code, None


@pytest.fixture
def api(tmp_path):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    base = f"http://127.0.0.1:{port}"
    environment = {**os.environ, "ASPNETCORE_URLS": base, "ODOMETRY_DATA_DIR": str(tmp_path)}
    dll = ROOT / "apps/api/bin/Debug/net10.0/Odometry.Api.dll"
    assert dll.exists(), "Build apps/api first"
    with (tmp_path / "server.log").open("w") as log:
        process = subprocess.Popen(["dotnet", str(dll)], env=environment, stdout=log, stderr=log)
        try:
            for _ in range(100):
                if process.poll() is not None:
                    pytest.fail((tmp_path / "server.log").read_text())
                try:
                    if call(base, "/api/health")[0] == 200:
                        break
                except (OSError, URLError):
                    pass
                time.sleep(0.1)
            else:
                pytest.fail("API startup timeout")
            yield base
        finally:
            process.terminate()
            process.wait(timeout=10)


def frame():
    return {
        "run_id": "test",
        "seq": 1,
        "stamp_ns": "1789000000123456789",
        "s_m": 0,
        "v_mps": 10,
        "mode": "FUSED",
        "valid": True,
        "uncertainty_available": False,
        "sigma_s_m": None,
        "sigma_v_mps": None,
    }


def test_dedup_reports_precision_and_not_found(api):
    assert call(api, "/api/runs", {"run_id": "test", "status": "completed"})[0] == 200
    assert call(api, "/api/runs/test/telemetry", [frame()])[1]["accepted"] == 1
    assert call(api, "/api/runs/test/telemetry", [frame()])[1]["duplicates"] == 1
    assert call(api, "/api/runs/test/telemetry")[1][0]["stamp_ns"] == frame()["stamp_ns"]
    assert call(api, "/api/runs/test/telemetry?afterSeq=1")[1] == []
    assert call(api, "/api/runs/test/report", {"metrics": {"rmse": None}}, "PUT")[0] == 200
    assert call(api, "/api/runs/test/report")[1]["metrics"]["rmse"] is None
    assert call(api, "/api/runs/missing")[0] == 404
    assert call(api, "/api/unknown")[0] == 404


def test_validation_is_atomic(api):
    call(api, "/api/runs", {"run_id": "test"})
    bad = {**frame(), "run_id": "other", "seq": 2}
    assert call(api, "/api/runs/test/telemetry", [frame(), bad])[0] == 400
    assert call(api, "/api/runs/test/telemetry")[1] == []
    assert call(api, "/api/runs", {"run_id": "../bad"})[0] == 400
    assert call(api, "/api/runs/test/telemetry", [{}])[0] == 400
    assert call(api, "/api/runs/test/telemetry", [frame()] * 501)[0] == 400
