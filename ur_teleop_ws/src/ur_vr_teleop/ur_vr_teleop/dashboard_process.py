"""Single worker owns every managed-process mutation and publishes UI snapshots."""
import queue
import threading


class ManagerWorker:
    def __init__(self, manager):
        self.manager = manager
        self.events = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.closing = threading.Event()
        self.action = None
        self.busy = False
        self.thread = threading.Thread(target=self.run, name='teleop-process-owner', daemon=False)
        self.thread.start()

    def submit(self, action, mode=None):
        with self.lock:
            if self.closing.is_set():
                return False
            # Stop supersedes a queued restart; restart can follow an in-flight start.
            if (self.busy or self.action) and action not in ('stop', 'restart', 'setup_preview', 'setup_live', 'routine_live'):
                return False
            if self.action and self.action[0] == 'stop' and action != 'stop':
                return False
            self.action = (action, mode)
            self.busy = True
        self.wake.set()
        return True

    def snapshot(self):
        try:
            self.events.get_nowait()
        except queue.Empty:
            pass
        self.events.put_nowait((self.manager.message, self.busy))

    def run(self):
        try:
            while not self.closing.is_set():
                with self.lock:
                    action, self.action = self.action, None
                try:
                    if action:
                        name, mode = action
                        if name in ('setup_preview', 'setup_live', 'routine_live'):
                            self.manager.engineering_session = name != 'routine_live'
                            mode = name != 'setup_preview'
                            name = 'restart' if self.manager.children else 'start'
                        if name == 'stop':
                            self.manager.stop()
                            self.manager.message = 'Stopping owned ROS processes'
                        elif name == 'rviz':
                            self.manager.open_rviz()
                        elif name == 'restart' or self.manager.children:
                            self.manager.stop(restart=self.manager.live if mode is None else mode)
                        else:
                            self.manager.start(bool(mode))
                    # Close can arrive during a long start; never begin a pending restart afterward.
                    if self.closing.is_set():
                        break
                    self.manager.poll()
                except Exception as error:
                    self.manager.message = 'Process operation failed: ' + str(error)
                with self.lock:
                    self.busy = bool(self.action) or self.manager.stopping
                self.snapshot()
                self.wake.wait(.1)
                self.wake.clear()
        finally:
            self.manager.close()

    def close(self):
        self.closing.set()
        self.wake.set()

    def join(self):
        self.thread.join()
