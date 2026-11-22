#!/bin/bash
# Configurator starten (macOS: im Finder doppelklicken).
# Startet einen kleinen Webserver nur fuer diesen Computer und oeffnet Chrome.
# Web Serial (USB) funktioniert nur in Chrome oder Edge.
cd "$(dirname "$0")" || exit 1
PORT=8722
python3 -m http.server "$PORT" --bind 127.0.0.1 >/dev/null 2>&1 &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT
sleep 0.6
URL="http://localhost:$PORT/index.html"
open -a "Google Chrome" "$URL" 2>/dev/null || open -a "Microsoft Edge" "$URL" 2>/dev/null || open "$URL"
echo "Configurator laeuft: $URL"
echo "Dieses Fenster offen lassen. Schliessen (oder Ctrl+C) beendet den Configurator."
wait $SERVER
