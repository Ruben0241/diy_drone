#!/usr/bin/env python3
"""Configurator starten (Windows/Linux/macOS): python tools/configurator/start_configurator.py

Startet einen Webserver nur fuer diesen Computer (127.0.0.1:8722) und oeffnet den Browser.
Web Serial (USB) funktioniert nur in Chrome oder Edge.
"""
import functools
import http.server
import threading
import webbrowser
from pathlib import Path

PORT = 8722
handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(Path(__file__).parent))
server = http.server.ThreadingHTTPServer(("127.0.0.1", PORT), handler)
url = f"http://localhost:{PORT}/index.html"
threading.Timer(0.5, lambda: webbrowser.open(url)).start()
print(f"Configurator laeuft: {url}  (Ctrl+C beendet)")
try:
    server.serve_forever()
except KeyboardInterrupt:
    pass
