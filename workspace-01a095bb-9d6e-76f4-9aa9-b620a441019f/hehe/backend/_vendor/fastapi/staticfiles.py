"""FastAPI static files shim."""
class StaticFiles:
    def __init__(self, directory="", **kwargs):
        self.directory = directory
