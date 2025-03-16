#!/bin/bash

echo "========== VNC Monitor =========="
echo "Checking VNC services status..."

# Check if Xvfb is running
if pgrep -f Xvfb > /dev/null; then
    echo "✅ Xvfb is running"
else
    echo "❌ Xvfb is NOT running"
    HAS_FAILURE=true
fi

# Check if fluxbox is running
if pgrep -f fluxbox > /dev/null; then
    echo "✅ Fluxbox window manager is running"
else
    echo "❌ Fluxbox window manager is NOT running"
    HAS_FAILURE=true
fi

# Check if x11vnc is running
if pgrep -f x11vnc > /dev/null; then
    echo "✅ x11vnc server is running"
else
    echo "❌ x11vnc server is NOT running"
    HAS_FAILURE=true
fi

# Check if noVNC proxy is running
if pgrep -f novnc_proxy > /dev/null; then
    echo "✅ noVNC proxy is running"
else
    echo "❌ noVNC proxy is NOT running"
    HAS_FAILURE=true
fi

# If any service is not running, offer to restart
if [ "$HAS_FAILURE" = true ]; then
    echo ""
    echo "Some VNC services are not running."
    echo "Restarting VNC services..."
    ./.devcontainer/startup.sh
    echo ""
    echo "VNC services have been restarted."
    echo "You can access the VNC through http://localhost:6080/"
else
    echo ""
    echo "All VNC services are running correctly!"
    echo "You can access the VNC through http://localhost:6080/"
fi

# Check if VNC log exists and show recent entries
if [ -f ~/vnc-startup.log ]; then
    echo ""
    echo "Recent log entries:"
    tail -n 10 ~/vnc-startup.log
fi 