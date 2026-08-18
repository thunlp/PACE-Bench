"""Credential-injecting allowlist gateway used by isolated agent containers.

This file intentionally depends only on the Python standard library because the
container runner mounts it into the lightweight agent image as a standalone
program.  The untrusted agent can reach this gateway but cannot inspect its
filesystem, environment, or the real provider credentials.
"""

from __future__ import annotations

import argparse
import http.client
import json
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

_HOP_HEADERS = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailers",
    "transfer-encoding",
    "upgrade",
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evaluator-url", required=True)
    parser.add_argument("--secret-file", type=Path, required=True)
    parser.add_argument("--custom-base-url")
    parser.add_argument("--port", type=int, default=8080)
    return parser


class Gateway:
    def __init__(
        self,
        evaluator_url: str,
        secrets: dict[str, str],
        custom_base_url: str | None,
    ) -> None:
        self.routes: dict[str, tuple[str, str | None, str]] = {
            "openai": ("https://api.openai.com", secrets.get("openai"), "openai"),
            "anthropic": (
                "https://api.anthropic.com",
                secrets.get("anthropic"),
                "anthropic",
            ),
            "evaluator": (evaluator_url.rstrip("/"), None, "passthrough"),
        }
        if custom_base_url:
            self.routes["custom"] = (
                custom_base_url.rstrip("/"),
                secrets.get("custom"),
                "bearer",
            )

    def resolve(self, request_path: str) -> tuple[str, str | None, str, str] | None:
        path = urlsplit(request_path).path
        parts = path.lstrip("/").split("/", 1)
        route = self.routes.get(parts[0])
        if route is None:
            return None
        suffix = "/" + parts[1] if len(parts) == 2 else "/"
        query = urlsplit(request_path).query
        if query:
            suffix += "?" + query
        return (*route, suffix)


def _handler(gateway: Gateway) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "PACEBenchGateway/1.0"

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/health":
                self._small_response(HTTPStatus.OK, b"ok\n")
                return
            self._forward()

        def do_POST(self) -> None:  # noqa: N802
            self._forward()

        def do_DELETE(self) -> None:  # noqa: N802
            self._forward()

        def do_PATCH(self) -> None:  # noqa: N802
            self._forward()

        def do_PUT(self) -> None:  # noqa: N802
            self._forward()

        def do_OPTIONS(self) -> None:  # noqa: N802
            self._forward()

        def log_message(self, format_string: str, *args: object) -> None:
            print(format_string % args, file=sys.stderr, flush=True)

        def _forward(self) -> None:
            resolved = gateway.resolve(self.path)
            if resolved is None:
                self._small_response(HTTPStatus.FORBIDDEN, b"route not allowed\n")
                return
            base_url, secret, auth_mode, upstream_path = resolved
            upstream = urlsplit(base_url)
            if upstream.scheme not in {"http", "https"} or not upstream.hostname:
                self._small_response(HTTPStatus.BAD_GATEWAY, b"invalid upstream\n")
                return

            length_text = self.headers.get("Content-Length")
            try:
                length = int(length_text) if length_text else 0
            except ValueError:
                self._small_response(HTTPStatus.BAD_REQUEST, b"invalid length\n")
                return
            if length > 10_000_000:
                self._small_response(
                    HTTPStatus.REQUEST_ENTITY_TOO_LARGE, b"request too large\n"
                )
                return
            body = self.rfile.read(length) if length else None
            headers = {
                name: value
                for name, value in self.headers.items()
                if name.lower() not in _HOP_HEADERS
                and name.lower() not in {"host", "authorization", "x-api-key"}
            }
            if auth_mode == "passthrough":
                authorization = self.headers.get("Authorization")
                if authorization:
                    headers["Authorization"] = authorization
            elif auth_mode in {"openai", "bearer"}:
                if not secret:
                    self._small_response(
                        HTTPStatus.SERVICE_UNAVAILABLE, b"provider key unavailable\n"
                    )
                    return
                headers["Authorization"] = "Bearer " + secret
            elif auth_mode == "anthropic":
                if not secret:
                    self._small_response(
                        HTTPStatus.SERVICE_UNAVAILABLE, b"provider key unavailable\n"
                    )
                    return
                headers["x-api-key"] = secret

            connection_class = (
                http.client.HTTPSConnection
                if upstream.scheme == "https"
                else http.client.HTTPConnection
            )
            connection = connection_class(
                upstream.hostname,
                upstream.port,
                timeout=600,
            )
            prefix = upstream.path.rstrip("/")
            response_started = False
            try:
                connection.request(
                    self.command,
                    prefix + upstream_path,
                    body=body,
                    headers=headers,
                )
                response = connection.getresponse()
                self.send_response(response.status, response.reason)
                for name, value in response.getheaders():
                    lowered = name.lower()
                    if lowered not in _HOP_HEADERS and lowered != "content-length":
                        self.send_header(name, value)
                self.send_header("Connection", "close")
                self.end_headers()
                response_started = True
                while True:
                    chunk = response.read(64 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (OSError, http.client.HTTPException) as exc:
                if not response_started and not self.wfile.closed:
                    try:
                        self._small_response(
                            HTTPStatus.BAD_GATEWAY,
                            f"gateway error: {exc}\n".encode("utf-8"),
                        )
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    print(f"gateway stream error: {exc}", file=sys.stderr, flush=True)
            finally:
                self.close_connection = True
                connection.close()

        def _small_response(self, status: HTTPStatus, body: bytes) -> None:
            self.send_response(int(status))
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
            self.close_connection = True

    return Handler


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    secrets = json.loads(args.secret_file.read_text(encoding="utf-8"))
    if not isinstance(secrets, dict):
        raise ValueError("secret file must contain an object")
    gateway = Gateway(
        args.evaluator_url,
        {str(key): str(value) for key, value in secrets.items() if value},
        args.custom_base_url,
    )
    server = ThreadingHTTPServer(("0.0.0.0", args.port), _handler(gateway))
    server.daemon_threads = True
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
