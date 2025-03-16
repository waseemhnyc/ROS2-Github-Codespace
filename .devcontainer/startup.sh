#!/bin/bash

# Log file for debugging
LOGFILE=$HOME/vnc-startup.log

# Clear previous log
echo "Starting VNC service: $(date)" > $LOGFILE

# Function to log messages
log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> $LOGFILE
}

# Kill any existing VNC processes
log "Cleaning up any existing VNC processes..."
pkill -f Xvfb || true
pkill -f x11vnc || true
pkill -f fluxbox || true
pkill -f novnc_proxy || true
sleep 2

# Start VNC services
log "Starting Xvfb..."
export DISPLAY=:0
Xvfb :0 -screen 0 1280x800x24 >> $LOGFILE 2>&1 &
XVFB_PID=$!
log "Xvfb started with PID: $XVFB_PID"

# Wait for Xvfb to initialize
sleep 3
log "Starting window manager..."
fluxbox >> $LOGFILE 2>&1 &
FLUXBOX_PID=$!
log "Fluxbox started with PID: $FLUXBOX_PID"

sleep 2
log "Starting VNC server..."
x11vnc -display :0 -forever -nopw -shared -rfbport 5900 >> $LOGFILE 2>&1 &
VNC_PID=$!
log "VNC server started with PID: $VNC_PID"

sleep 2
log "Starting noVNC proxy..."
/opt/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080 >> $LOGFILE 2>&1 &
NOVNC_PID=$!
log "noVNC proxy started with PID: $NOVNC_PID"

log "All VNC services started successfully"
log "Access VNC via port 6080 in your browser"

# Save PIDs for monitoring
echo "$XVFB_PID $FLUXBOX_PID $VNC_PID $NOVNC_PID" > $HOME/.vnc_pids

# Verify services are running
sleep 5
if ! ps -p $XVFB_PID > /dev/null || ! ps -p $VNC_PID > /dev/null || ! ps -p $NOVNC_PID > /dev/null; then
  log "ERROR: Some VNC services failed to start. Check the log for details."
  exit 1
else
  log "Verified all VNC services are running."
fi 