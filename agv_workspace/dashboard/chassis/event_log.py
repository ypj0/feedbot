from collections import deque
import threading


class EventLog:
    def __init__(self, stream):
        self.stream = stream
        self.condition = threading.Condition()
        self.records = deque()
        self.error = None
        self.closed = False
        self.worker = threading.Thread(target=self.consume, name="web-event-log", daemon=True)
        self.worker.start()

    def check(self):
        with self.condition:
            if self.error is None and len(self.records) >= 128:
                self.error = RuntimeError("web event log backlog reached 128 records")
                self.condition.notify_all()
            if self.error is not None:
                raise RuntimeError("web event log failed: " + str(self.error)) from self.error

    def submit(self, record):
        with self.condition:
            self.check()
            if self.closed:
                raise RuntimeError("web event log is closed")
            self.records.append(record)
            self.condition.notify()

    def consume(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.records or self.closed or self.error is not None)
                    if self.error is not None or not self.records:
                        return
                    record = self.records.popleft()
                self.stream.write(record)
                self.stream.flush()
        except Exception as error:
            with self.condition:
                self.error = error
                self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.worker.join(timeout=2)
        if self.worker.is_alive():
            raise RuntimeError("web event log writer did not stop within 2 seconds")
        self.check()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        try:
            self.close()
        finally:
            if not self.worker.is_alive():
                self.stream.close()
