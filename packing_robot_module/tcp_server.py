# 클라이언트 접속을 받아 \r\n 라인 단위로 명령을 읽고 핸들러에 넘기는 TCP 서버
from __future__ import annotations

import logging
import socket

log = logging.getLogger("packing_robot_module.tcp_server")

DELIMITER = b"\r\n"
ENCODING = "UTF-8"


class TcpServer:
    """한 번에 한 클라이언트만 처리하는 단순 라인 서버.

    로봇 모션이 블로킹이라 여러 클라이언트를 동시에 처리할 필요가 없다.
    접속이 끊기면 다음 접속을 다시 accept한다.
    """

    def __init__(self, host: str, port: int, handler: object) -> None:
        self._host = host
        self._port = port
        self._handler = handler

    def serve_forever(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
            server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_sock.bind((self._host, self._port))
            server_sock.listen(1)
            log.info("리스닝 시작 %s:%d", self._host, self._port)
            while True:
                conn, addr = server_sock.accept()
                log.info("클라이언트 접속: %s", addr)
                try:
                    self._serve_client(conn=conn)
                finally:
                    conn.close()
                    log.info("클라이언트 연결 종료: %s", addr)

    def _serve_client(self, conn: socket.socket) -> None:
        def send(line: str) -> None:
            conn.sendall(line.encode(ENCODING) + DELIMITER)
            log.info(">> %s", line)

        buf = b""
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                return
            buf += chunk
            while DELIMITER in buf:
                raw, buf = buf.split(DELIMITER, 1)
                line = raw.decode(ENCODING, errors="replace").strip()
                if line:
                    log.info("<< %s", line)
                    self._handler.handle_line(line=line, send=send)
