"""Identify the gray-box longitudinal model from GNSS-derived reference tables.

    python dataset/analysis/build_reference_table.py      # once
    python dataset/analysis/identify_model.py [--out configs/models]

Model (docs/23 section 3.3):

    a_cmd = traction(u, v) for u > 0,  brake(u, v) for u < 0,  0 for u = 0      (u = notch / 15)
    tau * da_act/dt = a_cmd - a_act
    dv/dt = a_act - c1 * v - c2 * v * |v| - g * grade

For a fixed ``tau`` the model is linear in the table values and in (c1, c2), so the fit is a
non-negative least squares problem: tables are parametrised by non-negative increments along the
controller axis (monotone in |u|), with a smoothness penalty along the speed axis. ``tau`` is chosen
on the validation bags. Test bags are never touched here. GNSS-derived ``a_ref`` is the only target.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
G = 9.81
DT = 0.1
NOTCHES = 15
U_NODES = np.array([1, 2, 3, 5, 7, 10, 15]) / NOTCHES
V_NODES = np.arange(0.0, 12.1, 2.0)
TAUS = (0.1, 0.2, 0.3, 0.5, 0.8, 1.2, 2.0)
SMOOTH = 30.0  # weight of the speed-axis second-difference penalty
HORIZON_S = 10.0


# --- data --------------------------------------------------------------------------------------


def load_bag(path: Path) -> dict[str, np.ndarray]:
    """One contiguous 10 Hz series; missing values become NaN."""
    rows = list(csv.DictReader(path.open(encoding="utf-8")))

    def col(name):
        return np.array([float(r[name]) if r[name] != "" else np.nan for r in rows])

    return {
        "t": col("stamp_ns") / 1e9,
        "v": col("v_ref"),
        "a": col("a_ref"),
        "u": col("controller") / NOTCHES,
        "grade": col("grade"),
        "vehicle": rows[0]["vehicle_id"] if rows else "",
    }


def segments(bag: dict) -> list[slice]:
    """Runs without gaps in time (a gap breaks the actuator filter history)."""
    t = bag["t"]
    breaks = np.flatnonzero(np.diff(t) > 3 * DT) + 1
    edges = [0, *breaks.tolist(), len(t)]
    return [slice(a, b) for a, b in zip(edges[:-1], edges[1:], strict=True) if b - a > 50]


# --- linear basis ------------------------------------------------------------------------------


def hat_weights(nodes: np.ndarray, x: np.ndarray) -> np.ndarray:
    """Clamped linear-interpolation weights, shape (len(x), len(nodes))."""
    xc = np.clip(x, nodes[0], nodes[-1])
    idx = np.clip(np.searchsorted(nodes, xc) - 1, 0, len(nodes) - 2)
    frac = (xc - nodes[idx]) / (nodes[idx + 1] - nodes[idx])
    w = np.zeros((len(x), len(nodes)))
    rows = np.arange(len(x))
    w[rows, idx] = 1.0 - frac
    w[rows, idx + 1] += frac
    return w


def table_basis(u_abs: np.ndarray, v: np.ndarray, active: np.ndarray) -> np.ndarray:
    """Columns for non-negative increments delta[i, j]; T[i, j] = sum_{i' <= i} delta[i', j]."""
    wu = np.cumsum(hat_weights(U_NODES, u_abs)[:, ::-1], axis=1)[:, ::-1]  # sum_{i>=i'} h_i(u)
    wv = hat_weights(V_NODES, np.abs(v))
    basis = (wu[:, :, None] * wv[:, None, :]).reshape(len(u_abs), -1)
    return basis * active[:, None]


def filtered(columns: np.ndarray, tau: float) -> np.ndarray:
    """First-order lag applied to every column (forward Euler, exact enough at 10 Hz)."""
    alpha = 1.0 - np.exp(-DT / tau)
    out = np.zeros_like(columns)
    state = np.zeros(columns.shape[1])
    for k in range(len(columns)):
        state = state + alpha * (columns[k] - state)
        out[k] = state
    return out


def design(bags: list[dict], tau: float) -> tuple[np.ndarray, np.ndarray]:
    xs, ys = [], []
    n_table = len(U_NODES) * len(V_NODES)
    for bag in bags:
        for seg in segments(bag):
            u, v, a, grade = (bag[k][seg] for k in ("u", "v", "a", "grade"))
            usable_u = np.where(np.isnan(u), 0.0, u)
            v0 = np.where(np.isnan(v), 0.0, v)
            phi = np.hstack(
                [
                    table_basis(np.abs(usable_u), v0, (usable_u > 0).astype(float)),
                    -table_basis(np.abs(usable_u), v0, (usable_u < 0).astype(float)),
                ]
            )
            f_phi = filtered(phi, tau)
            drag = np.stack([-v0, -v0 * np.abs(v0)], axis=1)
            x = np.hstack([f_phi, drag])
            ok = ~(np.isnan(v) | np.isnan(a) | np.isnan(grade) | np.isnan(u))
            xs.append(x[ok])
            ys.append((a + G * grade)[ok])
    assert xs, "no usable rows"
    x, y = np.vstack(xs), np.concatenate(ys)
    assert x.shape[1] == 2 * n_table + 2
    return x, y


# --- solver ------------------------------------------------------------------------------------


def nnls_gram(gram: np.ndarray, rhs: np.ndarray, max_iter: int = 2000) -> np.ndarray:
    """Lawson-Hanson active set on the normal equations (small problems only)."""
    n = len(rhs)
    x = np.zeros(n)
    passive = np.zeros(n, dtype=bool)
    tol = 1e-10 * max(1.0, np.abs(rhs).max())
    for _ in range(max_iter):
        w = rhs - gram @ x
        w[passive] = -np.inf
        j = int(np.argmax(w))
        if w[j] <= tol:
            break
        passive[j] = True
        while True:
            idx = np.flatnonzero(passive)
            s = np.zeros(n)
            s[idx] = np.linalg.solve(gram[np.ix_(idx, idx)] + 1e-12 * np.eye(len(idx)), rhs[idx])
            if np.all(s[idx] > 0):
                x = s
                break
            neg = idx[s[idx] <= 0]
            alpha = np.min(x[neg] / (x[neg] - s[neg] + 1e-30))
            x = x + alpha * (s - x)
            passive &= x > 1e-12
            x[~passive] = 0.0
    return x


def smoothness_matrix() -> np.ndarray:
    """Second differences along speed of T = A delta (per controller node), applied to both tables."""
    ni, nj = len(U_NODES), len(V_NODES)
    cumulative = np.tril(np.ones((ni, ni)))
    rows = []
    for i in range(ni):
        for j in range(1, nj - 1):
            row = np.zeros((ni, nj))
            for ip in range(ni):
                weight = cumulative[i, ip]
                row[ip, j - 1] += weight
                row[ip, j] -= 2 * weight
                row[ip, j + 1] += weight
            rows.append(row.ravel())
    d = np.array(rows)
    size = ni * nj
    out = np.zeros((2 * len(d), 2 * size + 2))
    out[: len(d), :size] = d
    out[len(d) :, size : 2 * size] = d
    return out


def fit(bags: list[dict], tau: float) -> np.ndarray:
    x, y = design(bags, tau)
    penalty = smoothness_matrix()
    gram = x.T @ x + SMOOTH * len(y) / len(penalty) * penalty.T @ penalty
    return nnls_gram(gram, x.T @ y)


def unpack(theta: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, float]:
    ni, nj = len(U_NODES), len(V_NODES)
    size = ni * nj
    traction = np.cumsum(theta[:size].reshape(ni, nj), axis=0)
    brake = -np.cumsum(theta[size : 2 * size].reshape(ni, nj), axis=0)
    return traction, brake, float(theta[-2]), float(theta[-1])


# --- evaluation ----------------------------------------------------------------------------------


def a_cmd(traction, brake, u: float, v: float) -> float:
    if u == 0.0:
        return 0.0
    table = traction if u > 0 else brake
    wu = hat_weights(U_NODES, np.array([abs(u)]))[0]
    wv = hat_weights(V_NODES, np.array([abs(v)]))[0]
    return float(wu @ table @ wv)


def horizon_errors(bags: list[dict], theta: np.ndarray, tau: float) -> dict[str, float]:
    """Open-loop (model-only) speed error over HORIZON_S windows vs the GNSS reference speed.

    Baseline: hold the speed at the window start. Windows must be fully on the graph with reference
    and control available.
    """
    traction, brake, c1, c2 = unpack(theta)
    steps = round(HORIZON_S / DT)
    model_sq, hold_sq, count = 0.0, 0.0, 0
    for bag in bags:
        for seg in segments(bag):
            u, v, a, grade = (bag[k][seg] for k in ("u", "v", "a", "grade"))
            good = ~(np.isnan(u) | np.isnan(v) | np.isnan(a) | np.isnan(grade))
            for start in range(0, len(u) - steps, steps):
                window = slice(start, start + steps + 1)
                if not good[window].all():
                    continue
                vk, a_act = v[start], a[start]
                for k in range(steps):
                    cmd = a_cmd(traction, brake, float(u[start + k]), vk)
                    a_act += DT / tau * (cmd - a_act)
                    vk += DT * (a_act - c1 * vk - c2 * vk * abs(vk) - G * grade[start + k])
                    vk = max(vk, 0.0) if v[start + k] >= 0 else vk
                    model_sq += (vk - v[start + k + 1]) ** 2
                    hold_sq += (v[start] - v[start + k + 1]) ** 2
                    count += 1
    if not count:
        return {"model_rmse": float("nan"), "hold_rmse": float("nan"), "n": 0}
    return {"model_rmse": (model_sq / count) ** 0.5, "hold_rmse": (hold_sq / count) ** 0.5, "n": count}


# --- artifact ------------------------------------------------------------------------------------


def profile(
    vehicle_id: str, theta: np.ndarray, tau: float, id_bags: list[str], val_bags: list[str], metrics: dict
) -> dict:
    traction, brake, c1, c2 = unpack(theta)
    return {
        "schema_version": 1,
        "model_version": "tram-graybox-v1",
        "vehicle_id": vehicle_id,
        "identified_at_utc": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "identification_method": "nnls_monotone_tables_gnss_reference",
        "identification_bags": sorted(id_bags),
        "validation_bags": sorted(val_bags),
        "longitudinal": {
            "tau_s": float(tau),
            "c1_inv_s": round(c1, 6),
            "c2_inv_m": round(c2, 6),
            "disturbance_limit_mps2": 1.5,
        },
        "traction": {
            "controller_u": [round(float(u), 6) for u in U_NODES],
            "speed_mps": [float(v) for v in V_NODES],
            "acceleration_mps2": [[round(float(x), 4) for x in row] for row in traction],
        },
        "braking": {
            "controller_u": [round(-float(u), 6) for u in U_NODES],
            "speed_mps": [float(v) for v in V_NODES],
            "acceleration_mps2": [[round(float(x), 4) for x in row] for row in brake],
        },
        "noise": {"q_v": 0.01, "q_a": 0.05, "q_d": 0.001, "wheel_variance_floor": 0.04},
        "wheel_health": {
            "gate_normal": 9.0,
            "gate_reject": 36.0,
            "max_wheel_accel_mps2": 6.0,
            "freeze_s": 1.0,
            "reacquire_s": 10.0,
            "recover_updates": 5,
        },
        "metrics": metrics,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--derived", type=Path, default=ROOT / "dataset/derived")
    parser.add_argument("--out", type=Path, default=ROOT / "configs/models")
    args = parser.parse_args()

    split = json.loads((args.derived / "split.json").read_text(encoding="utf-8"))
    load = {
        name: {b: load_bag(args.derived / "reference" / f"{b}.csv") for b in split[name]}
        for name in ("identification", "validation")
    }
    args.out.mkdir(parents=True, exist_ok=True)

    for vehicle in ("default", "30618", "30639"):
        def pick(name):
            return {
                b: d for b, d in load[name].items() if vehicle == "default" or d["vehicle"] == vehicle
            }

        train, val = pick("identification"), pick("validation")
        if not train or not val:
            print(f"skip {vehicle}: identification={len(train)} validation={len(val)}")
            continue
        best = None
        for tau in TAUS:
            theta = fit(list(train.values()), tau)
            errors = horizon_errors(list(val.values()), theta, tau)
            print(vehicle, f"tau={tau}", {k: round(float(v), 4) for k, v in errors.items()}, flush=True)
            if best is None or errors["model_rmse"] < best[0]["model_rmse"]:
                best = (errors, tau, theta)
        errors, tau, theta = best
        coverage = float(
            np.mean([np.mean(~np.isnan(d["v"])) for d in train.values()])
        )
        metrics = {
            "reference_coverage": round(coverage, 4),
            "validation_velocity_rmse_mps": round(float(errors["model_rmse"]), 4),
            "validation_hold_speed_rmse_mps": round(float(errors["hold_rmse"]), 4),
            "validation_horizon_s": HORIZON_S,
            "validation_windows_steps": int(errors["n"]),
        }
        data = profile(vehicle, theta, tau, list(train), list(val), metrics)
        path = args.out / f"{vehicle}.yaml"
        path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
        print("wrote", path, metrics)


if __name__ == "__main__":
    main()
