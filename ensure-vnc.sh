#!/bin/bash
set -e

# This script will be copied to /usr/local/bin/ensure-vnc.sh during container build
# and will be added to container startup

echo "VNC Startup Script - $(date)" | tee -a /var/log/vnc-startup.log

# Create the systemd service file
cat > /tmp/vnc-service.sh << 'EOF'
#!/bin/bash

# Log file for debugging
LOGFILE=/var/log/vnc-startup.log

# Function to log messages
log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $LOGFILE
}

# Kill any existing VNC processes
pkill -f Xvfb || true
pkill -f x11vnc || true
pkill -f fluxbox || true
pkill -f novnc_proxy || true
sleep 2

# Clear previous log
log "Starting VNC service: $(date)"

# Start VNC services
log "Starting Xvfb..."
export DISPLAY=:0
Xvfb :0 -screen 0 1280x800x24 >> $LOGFILE 2>&1 &
sleep 3

log "Starting window manager..."
fluxbox >> $LOGFILE 2>&1 &
sleep 2

log "Starting VNC server..."
x11vnc -display :0 -forever -nopw -shared -rfbport 5900 >> $LOGFILE 2>&1 &
sleep 2

log "Starting noVNC proxy..."
/opt/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080 >> $LOGFILE 2>&1 &
log "All VNC services started successfully"

# Keep script running to prevent services from being terminated
while true; do
  sleep 3600
done
EOF

chmod +x /tmp/vnc-service.sh

# Start the VNC service script in the background
echo "Starting VNC service..." | tee -a /var/log/vnc-startup.log
nohup /tmp/vnc-service.sh > /tmp/vnc-nohup.log 2>&1 &

echo "VNC service should now be running" | tee -a /var/log/vnc-startup.log
echo "You can access it at http://localhost:6080/" | tee -a /var/log/vnc-startup.log
echo "If it's not working, check logs at /var/log/vnc-startup.log and /tmp/vnc-nohup.log" | tee -a /var/log/vnc-startup.log 