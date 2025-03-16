  #!/bin/bash
  export DISPLAY=:1
  Xvfb :1 -screen 0 1280x800x24 &
  sleep 2
  fluxbox &
  x11vnc -display :1 -forever -nopw -shared -rfbport 5901 &
  /opt/noVNC/utils/launch.sh --vnc localhost:5901 --listen 6080 &
  sleep 2
  bash
