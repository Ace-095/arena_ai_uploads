"""
Minimal FastAPI shim for development without pip access.
Provides just enough API surface for app.py to import and define routes.
"""

import asyncio
import json
import logging
from typing import Any, Optional
from contextlib import asynccontextmanager

log = logging.getLogger("fastapi_shim")


class HTTPException(Exception):
    def __init__(self, status_code=500, detail=""):
        self.status_code = status_code
        self.detail = detail
        super().__init__(detail)


class _Request:
    pass


class WebSocket:
    def __init__(self):
        self._accepted = False

    async def accept(self):
        self._accepted = True

    async def send_text(self, data):
        pass

    async def send_json(self, data):
        pass

    async def receive_text(self):
        await asyncio.sleep(3600)
        return ""

    async def receive_json(self):
        await asyncio.sleep(3600)
        return {}


class WebSocketDisconnect(Exception):
    pass


class Response:
    def __init__(self, content=b"", media_type="application/json", status_code=200):
        self.body = content if isinstance(content, bytes) else content.encode()
        self.media_type = media_type
        self.status_code = status_code


class _Route:
    def __init__(self, path, method, handler, response_model=None):
        self.path = path
        self.method = method
        self.handler = handler
        self.response_model = response_model


class FastAPI:
    def __init__(self, title="", lifespan=None, **kwargs):
        self.title = title
        self._lifespan = lifespan
        self._routes = []
        self._middleware = []

    def add_middleware(self, middleware_class, **kwargs):
        pass

    def mount(self, path, app=None, name=None):
        pass

    def get(self, path, **kwargs):
        def decorator(func):
            self._routes.append(_Route(path, "GET", func, kwargs.get("response_model")))
            return func
        return decorator

    def post(self, path, **kwargs):
        def decorator(func):
            self._routes.append(_Route(path, "POST", func, kwargs.get("response_model")))
            return func
        return decorator

    def delete(self, path, **kwargs):
        def decorator(func):
            self._routes.append(_Route(path, "DELETE", func, kwargs.get("response_model")))
            return func
        return decorator

    def websocket(self, path):
        def decorator(func):
            self._routes.append(_Route(path, "WS", func))
            return func
        return decorator


class StaticFiles:
    def __init__(self, directory="", **kwargs):
        self.directory = directory


class CORSMiddleware:
    pass
