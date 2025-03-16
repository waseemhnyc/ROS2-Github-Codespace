#!/bin/bash

# A simpler script that just starts VNC services and exits
# This is intended to be run from the entrypoint script

# Log file for debugging
LOGFILE=/tmp/vnc-startup.log

# Function to log messages
log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> $LOGFILE
}

# Kill any existing VNC processes
pkill -f Xvfb || true
pkill -f x11vnc || true
pkill -f fluxbox || true
pkill -f novnc_proxy || true
sleep 1

# Clear previous log
echo "Starting VNC service: $(date)" > $LOGFILE

# Start VNC services
log "Starting Xvfb..."
export DISPLAY=:0
Xvfb :0 -screen 0 1280x800x24 &
sleep 2

log "Starting window manager..."
fluxbox &
sleep 1

log "Starting VNC server..."
x11vnc -display :0 -forever -nopw -shared -rfbport 5900 &
sleep 1

log "Starting noVNC proxy..."
/opt/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &

log "All VNC services started successfully"
log "Access VNC via port 6080 in your browser"

# Don't block container startup - just exit after starting services
exit 0 