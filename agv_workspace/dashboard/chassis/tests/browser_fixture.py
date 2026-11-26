import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    chrome = sys.argv[1]
    output = Path(sys.argv[2]).resolve()
    output.mkdir(parents=True, exist_ok=True)
    static = Path(__file__).resolve().parents[1] / "static"
    mock = """
window.fixtureRequests = [];
window.fixtureState = {ready: true, sampled_at: 100,
  control: {epoch: "browser-fixture", sequence: 0, action: "stop", remaining_s: 0, pulse_s: 0.5, error: ""},
  samples: {odom: {value: {x: 0, y: 0, yaw: 0, vx: 0, wz: 0}},
    joints: {value: {left_wheel_joint: 0, right_wheel_joint: 0}}}};
window.fetch = async (url, options = {}) => {
  if (url === "/api/command") {
    const payload = JSON.parse(options.body);
    window.fixtureRequests.push(payload);
    Object.assign(window.fixtureState.control, {action: payload.action, sequence: payload.sequence,
      remaining_s: payload.action === "stop" ? 0 : payload.duration_s});
    if (payload.action !== "stop") window.fixtureState.control.pulse_s = payload.duration_s;
  } else if (url !== "/api/state") throw new Error("Unexpected URL: " + url);
  return {ok: true, json: async () => structuredClone(window.fixtureState)};
};
"""
    checks = """
setTimeout(async () => {
  const assert = (condition, message) => { if (!condition) throw new Error(message); };
  const pause = () => new Promise(resolve => setTimeout(resolve, 10));
  try {
    assert(window.fixtureRequests.length === 0, "startup sent a command");
    assert(document.getElementById("duration").value === "0.5", "wrong default duration");
    for (const duration of [0.5, 5, 10]) {
      document.getElementById("duration").value = String(duration);
      document.querySelector('[data-action="forward"]').click();
      await pause();
      assert(window.fixtureRequests.at(-1).duration_s === duration, "duration not forwarded");
      assert(document.getElementById("result").textContent.includes(`${duration} 秒`), "wrong acknowledgement");
      assert(document.getElementById("duration").disabled, "duration remains editable during pulse");
      assert(document.querySelector('[data-action="forward"]').disabled, "motion button remains enabled");
      assert(!document.getElementById("stop").disabled, "stop unavailable");
      document.getElementById("stop").click();
      await pause();
      assert(window.fixtureRequests.at(-1).action === "stop", "stop not forwarded");
      assert(!document.getElementById("duration").disabled, "duration not restored after stop");
    }
    assert(window.fixtureRequests.length === 6, "unexpected queued commands");
    document.body.dataset.fixture = "PASS";
    document.body.dataset.requests = JSON.stringify(window.fixtureRequests);
  } catch (error) {
    document.body.dataset.fixture = "FAILED";
    document.body.dataset.error = error.message;
  }
}, 100);
"""
    html = static.joinpath("index.html").read_text()
    html = html.replace('<link rel="stylesheet" href="/style.css">', "")
    html = html.replace('<script src="/app.js"></script>',
                        "<script>" + mock + "</script><script>" + static.joinpath("app.js").read_text() +
                        "</script><script>" + checks + "</script>")
    page = output / "browser_fixture.html"
    page.write_text(html)
    with tempfile.TemporaryDirectory(prefix="feedbot-browser-fixture-") as profile:
        result = subprocess.run([chrome, "--headless", "--disable-gpu", "--disable-background-networking",
                                 "--no-first-run", "--user-data-dir=" + profile,
                                 "--virtual-time-budget=1500", "--dump-dom", page.as_uri()],
                                capture_output=True, text=True, timeout=30)
    (output / "browser_fixture_dom.html").write_text(result.stdout)
    (output / "browser_fixture.log").write_text(result.stderr)
    assert result.returncode == 0 and 'data-fixture="PASS"' in result.stdout, "browser fixture failed"
    print(json.dumps({"status": "PASS", "hardware": "mock fetch only; no network commands",
                      "checks": ["default 0.5 s", "0.5/5/10 s selected commands", "acknowledgement",
                                 "disable selection/motion during pulse", "stop available", "no startup/queued commands"]}))


if __name__ == "__main__":
    main()
