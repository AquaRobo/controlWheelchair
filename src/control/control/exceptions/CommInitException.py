class CommInitError(Exception):
    """Raised when a communication protocol fails to be initialized"""
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)