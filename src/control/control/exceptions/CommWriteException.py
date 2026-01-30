class CommWriteError(Exception):
    """Raised when a communication protocol fails to write data"""
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)