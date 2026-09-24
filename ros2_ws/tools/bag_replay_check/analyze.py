import bisect, json, math
truth = [json.loads(l) for l in open("/tmp/truth.jsonl")]
est = [json.loads(l) for l in open("/tmp/est.jsonl")]
t0 = truth[0]["stamp_ns"]
ts = [r["stamp_ns"] for r in truth]

def interp(ns, key):
    i = bisect.bisect_left(ts, ns)
    if i == 0 or i >= len(ts): return None
    a, b = truth[i - 1], truth[i]
    f = (ns - a["stamp_ns"]) / (b["stamp_ns"] - a["stamp_ns"])
    return a[key] + f * (b[key] - a[key])

print("estimate frames:", len(est), " first stamp (s from t0):", round((est[0]["stamp_ns"] - t0) / 1e9, 3),
      " last:", round((est[-1]["stamp_ns"] - t0) / 1e9, 3))
windows = [("clean 0-6s", 0, 6), ("dropout 6-7.5s", 6, 7.5), ("after dropout 7.5-11s", 7.5, 11),
           ("slip 11-12s", 11, 12), ("after slip 12-20s", 12, 20)]
for name, a, b in windows:
    rows = [e for e in est if a <= (e["stamp_ns"] - t0) / 1e9 < b]
    modes = {}
    for e in rows: modes[e["mode"]] = modes.get(e["mode"], 0) + 1
    errs = []
    for e in rows:
        if e["has"]:
            tv = interp(e["stamp_ns"], "v")
            if tv is not None: errs.append(e["v"] - tv)
    rmse = math.sqrt(sum(x * x for x in errs) / len(errs)) if errs else None
    print(f"{name:24s} n={len(rows):3d} modes={modes} rmse_v={rmse if rmse is None else round(rmse, 3)}")
# raw wheel error in the slip window for comparison
raw = [(r["wheel"] - r["v"]) for r in truth if r["wheel"] is not None and 11 <= (r["stamp_ns"] - t0) / 1e9 < 12]
print("raw wheel rms error in slip window:", round(math.sqrt(sum(x * x for x in raw) / len(raw)), 3))
last = est[-1]
tv, tsn = interp(last["stamp_ns"], "v"), interp(last["stamp_ns"], "s")
print("final: est s=%.2f v=%.2f | truth s=%.2f v=%.2f | sigma_s=%.2f mode=%s" % (last["s"], last["v"], tsn, tv, last["sigma_s"], last["mode"]))
seen = []
for e in est:
    if not seen or seen[-1][1] != e["mode"]: seen.append(((e["stamp_ns"] - t0) / 1e9, e["mode"]))
print("mode transitions:", [(round(t, 2), m) for t, m in seen])
