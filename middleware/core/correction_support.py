"""Optional correction capture on the existing local/task recording pipeline."""
import copy
import threading


class CorrectionSupport:
    corrections = None

    def configure_corrections(self, ros_config):
        self.ros_config = copy.deepcopy(ros_config)
        self.correction_lock = threading.RLock()

    def prepare_corrections(self, metadata=None):
        from .correction_recording import CorrectionSession
        with self.correction_lock:
            if self.corrections is not None:
                return self.corrections.status()
            if self.is_recording():
                raise ValueError('stop ordinary recording before preparing corrections')
            session = CorrectionSession(self, metadata or {})
            self.corrections = session
            try:
                session.prepare()
            except Exception as exc:
                session.stop()
                self.corrections = None
                if getattr(exc, 'action', None) and getattr(exc, 'code', None):
                    raise ValueError(exc.action) from exc
                raise
            return session.status()

    def stop_corrections(self):
        with self.correction_lock:
            if self.corrections is not None:
                self.corrections.stop()
                self.corrections = None

    def correction_open(self, token, stamp_ns):
        if self.corrections is None:
            raise RuntimeError('correction recording has not been prepared')
        return self.corrections.open(token, stamp_ns)

    def correction_close(self, token, stamp_ns, last_command_seq, reason):
        if self.corrections:
            self.corrections.close(token, stamp_ns, last_command_seq, reason)

    def correction_action(self, evidence):
        if self.corrections:
            self.corrections.action(evidence)
