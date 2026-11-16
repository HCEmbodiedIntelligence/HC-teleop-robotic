"""Stable errors shared by the data service and its IPC client."""


class DataError(Exception):
    def __init__(self, code: str, message: str = "", *, retryable: bool = False):
        super().__init__(message or code)
        self.code = code
        self.message = message or code
        self.retryable = retryable

    def as_dict(self):
        return {"code": self.code, "message": self.message, "retryable": self.retryable}
