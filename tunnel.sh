#!/usr/bin/env bash
# Start the drone simulation server and expose it via Cloudflare Tunnel.
# Usage: ./tunnel.sh [port]
#
# Requires:
#   - Python deps: pip install -r requirements.txt
#   - cloudflared: curl -sL https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64 -o /usr/local/bin/cloudflared && chmod +x /usr/local/bin/cloudflared

set -e

PORT="${1:-8000}"

# Find cloudflared
if command -v cloudflared &>/dev/null; then
    CLOUDFLARED=cloudflared
elif [[ -x /tmp/cloudflared ]]; then
    CLOUDFLARED=/tmp/cloudflared
else
    echo "cloudflared not found. Installing to /tmp/cloudflared..."
    curl -sL https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64 -o /tmp/cloudflared
    chmod +x /tmp/cloudflared
    CLOUDFLARED=/tmp/cloudflared
fi

cleanup() {
    echo ""
    echo "Shutting down..."
    kill "$SERVER_PID" 2>/dev/null || true
    kill "$TUNNEL_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
    wait "$TUNNEL_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Start the server
echo "Starting server on port $PORT..."
python run.py &
SERVER_PID=$!
sleep 2

if ! curl -s -o /dev/null http://localhost:"$PORT"/; then
    echo "ERROR: Server failed to start on port $PORT"
    exit 1
fi
echo "Server running (PID $SERVER_PID)"
echo ""

# Start the tunnel (its output includes the trycloudflare.com URL)
echo "Starting Cloudflare Tunnel — look for the URL below..."
echo ""
$CLOUDFLARED tunnel --url http://localhost:"$PORT" &
TUNNEL_PID=$!

wait
