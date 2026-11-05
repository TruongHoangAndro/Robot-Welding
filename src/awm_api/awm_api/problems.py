"""RFC 9457 problem details for every error under /api/v1."""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

MEDIA_TYPE = 'application/problem+json'

_TITLES = {400: 'Bad request', 401: 'Not authenticated', 403: 'Forbidden', 404: 'Not found',
           405: 'Method not allowed', 409: 'Conflict', 413: 'Upload too large', 422: 'Unprocessable content',
           500: 'Internal error'}


class ProblemError(Exception):
    """Raised by handlers; rendered as ``application/problem+json``."""

    def __init__(self, status: int, type_: str, title: Optional[str] = None, detail: Optional[str] = None,
                 errors: Optional[List[Dict[str, str]]] = None, headers: Optional[Dict[str, str]] = None,
                 **extra: Any) -> None:
        super().__init__(detail or title or type_)
        self.status = status
        self.type = type_ if type_.startswith('/') else f'/problems/{type_}'
        self.title = title or _TITLES.get(status, 'Error')
        self.detail = detail
        self.errors = errors
        self.headers = headers
        self.extra = extra

    def body(self, instance: Optional[str] = None) -> Dict[str, Any]:
        out: Dict[str, Any] = {'type': self.type, 'title': self.title, 'status': self.status}
        if self.detail:
            out['detail'] = self.detail
        if instance:
            out['instance'] = instance
        if self.errors:
            out['errors'] = self.errors
        out.update(self.extra)
        return out


def problem_response(p: ProblemError, request: Optional[Request] = None) -> JSONResponse:
    return JSONResponse(p.body(request.url.path if request else None), status_code=p.status,
                        media_type=MEDIA_TYPE, headers=p.headers)


def _pointer(loc) -> str:
    parts = [str(x) for x in loc if x not in ('body', 'query', 'path', 'form', 'header')]
    return '/' + '/'.join(parts)


def install(app: FastAPI) -> None:
    @app.exception_handler(ProblemError)
    async def _problem(request: Request, exc: ProblemError):  # noqa: ANN202
        return problem_response(exc, request)

    @app.exception_handler(RequestValidationError)
    async def _validation(request: Request, exc: RequestValidationError):  # noqa: ANN202
        errors = [{'pointer': _pointer(e.get('loc', ())), 'message': str(e.get('msg', ''))} for e in exc.errors()]
        return problem_response(ProblemError(422, 'validation', 'Request validation failed',
                                             detail='; '.join(f"{e['pointer']}: {e['message']}" for e in errors),
                                             errors=errors), request)

    @app.exception_handler(StarletteHTTPException)
    async def _http(request: Request, exc: StarletteHTTPException):  # noqa: ANN202
        kind = {404: 'not-found', 405: 'method-not-allowed'}.get(exc.status_code, 'http-error')
        detail = exc.detail if isinstance(exc.detail, str) and exc.detail not in ('Not Found',) else None
        return problem_response(ProblemError(exc.status_code, kind, detail=detail,
                                             headers=getattr(exc, 'headers', None)), request)
