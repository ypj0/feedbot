import json


class StopRequest:
    def __init__(self):
        self.requested = False
        self.signum = None
        self.wait_process = None

    def handle(self, signum, unused_frame):
        self.requested = True
        self.signum = signum
        if self.wait_process is not None:
            try:
                self.wait_process.terminate()
            except OSError:
                pass


def execute_session(supervisor, hooks, store, stop_request, utc_now):
    report = {"passed": False, "agent_motion_commands_sent": False, "factory_restored": False,
              "started_at": utc_now(), "candidate": str(hooks.root)}
    primary_error = None
    cleanup_errors = []
    restore_error = None
    try:
        supervisor.start()
        hooks.ready()
        report["ready"] = True
        hooks.hold(stop_request)
        report["intentional_stop_signal"] = stop_request.signum
    except BaseException as error:
        primary_error = error
        report["error"] = "%s: %s" % (type(error).__name__, error)
    finally:
        cleanup_errors = hooks.shutdown_lifecycle()
        try:
            if supervisor.needs_restore:
                transaction = json.loads((supervisor.root / "transaction.json").read_text())
                if transaction.get("state") == "rollback_failed":
                    raise RuntimeError("prior restoration failed: " + str(transaction.get("error")))
                supervisor.restore()
        except BaseException as error:
            restore_error = error
        finally:
            supervisor.close()
        if cleanup_errors:
            report["cleanup_errors"] = cleanup_errors
        if restore_error is not None:
            report["restore_error"] = "%s: %s" % (type(restore_error).__name__, restore_error)
        if hooks.native_stop_error is not None:
            report["native_stop_error"] = hooks.native_stop_error
        report["factory_restored"] = not supervisor.needs_restore
        report["passed"] = bool(primary_error is None and report.get("ready") and
                                stop_request.requested and
                                not cleanup_errors and restore_error is None and
                                hooks.native_stop_error is None and report["factory_restored"])
        report["finished_at"] = utc_now()
        store.write_json("session.json", report)
    return 0 if primary_error is None and report["passed"] else 1
