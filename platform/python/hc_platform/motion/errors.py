class MotionError(ValueError):
    def __init__(self, code, message="", details=None):
        super().__init__(message or code)
        self.code, self.details = code, details or {}

    def as_dict(self):
        return {"code": self.code, "message": str(self), "details": self.details}
