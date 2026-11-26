import hashlib
import json
from pathlib import Path
import struct
import sys


def beam_digest(scan):
    digest = hashlib.sha256()
    for key in ("ranges", "intensities"):
        values = scan[key]
        digest.update(struct.pack("<I", len(values)))
        for value in values:
            number = float("inf") if value is None or value == "inf" else float(value)
            digest.update(struct.pack("<f", number))
    return digest.hexdigest()


def verify(root):
    trial = json.loads((root / "trial.json").read_text())
    official = json.loads((root / "official.json").read_text())
    managed = json.loads((root / "managed.json").read_text())
    capture = json.loads((root / "managed_capture.json").read_text())
    replay_statistics = json.loads((root / "managed_replay_stats.json").read_text())
    if not trial["runtime_checks_passed"] or not trial["factory_restored"]:
        raise ValueError("runtime or restoration checks failed")
    if not official["passed"] or not managed["passed"] or not capture["complete"] or replay_statistics["errors"]:
        raise ValueError("driver/capture/replay failed")
    raw_frames = {}
    for line in (root / "managed_replay.jsonl").read_text().splitlines():
        scan = json.loads(line)
        raw_frames.setdefault(beam_digest(scan), []).append(scan)
    matched = 0
    beam_count = 0
    previous_stamp = None
    for line in (root / "managed_scans.jsonl").read_text().splitlines():
        scan = json.loads(line)
        stamp = scan["header_stamp"]
        if previous_stamp is not None and stamp <= previous_stamp:
            raise ValueError("non-monotonic published stamp")
        previous_stamp = stamp
        count = len(scan["ranges"])
        geometry = scan["geometry"]
        error = geometry["angle_max"] - geometry["angle_min"] - (count - 1) * geometry["angle_increment"]
        if count != 1075 or len(scan["intensities"]) != count or abs(error) > 1e-6:
            raise ValueError("invalid managed beam geometry")
        candidates = raw_frames.get(beam_digest(scan), [])
        if not any(abs(candidate["received"] - scan["metadata"]["received_wall"]) < 0.5 for candidate in candidates):
            raise ValueError("published range/intensity pair not found in contemporaneous raw packet replay")
        matched += 1
        beam_count += count
    if matched < 60:
        raise ValueError("fewer than 60 matched published scans")
    official_geometry = official["first_phase"][0]["geometry"]
    result = {
        "status": "实车静态通过（雷达对照）",
        "passed": True,
        "navigation_accepted": False,
        "factory_restored": True,
        "absolute_time_alignment_verified": False,
        "clock_modified": False,
        "managed_matched_scans": matched,
        "managed_matched_range_intensity_pairs": beam_count,
        "matching_basis": "same captured packets replayed with frozen C++ core; both arrays compared at float32 precision",
        "managed_geometry": managed["first_phase"][0]["geometry"],
        "official_geometry": official_geometry,
        "official_known_bias_observed": abs(official_geometry["endpoint_error_rad"]) > 1e-6,
        "inter_driver_point_equality_required": False,
    }
    return result


if __name__ == "__main__":
    result = verify(Path(sys.argv[1]))
    Path(sys.argv[1], "acceptance.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))
