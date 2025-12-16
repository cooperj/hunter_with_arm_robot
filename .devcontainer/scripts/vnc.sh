#!/bin/bash

# If run as root, re-exec the script as 'ros' to ensure
# all desktop/VNC processes run under the normal user.
if [ "$(id -u)" -eq 0 ]; then
    export NOVNC_VERSION=${NOVNC_VERSION}
    exec sudo -u ros -H bash "$0" "$@"
fi

echo
echo "****************************************************************************************************************************************"
echo "L-CAS Container Desktop starting..."
echo "****************************************************************************************************************************************"


echo "starting turbovnc"
sudo rm -rf /tmp/.X1-lock /tmp/.X11-unix/X1 > /dev/null 2>&1
screen -dmS turbovnc bash -c 'VGL_DISPLAY=egl VGL_FPS=30 /opt/TurboVNC/bin/vncserver :1 -depth 24 -noxstartup -securitytypes TLSNone,X509None,None 2>&1 | tee /tmp/vnc.log; read -p "Press any key to continue..."'
# wait for VNC to be running

echo "waiting for display to be up"
while ! xdpyinfo -display :1 2> /dev/null > /dev/null; do
    sleep .1
done
echo "display is up"

echo "starting xfce4"
screen -dmS xfce4 bash -c 'DISPLAY=:1 /usr/bin/xfce4-session 2>&1 | tee /tmp/xfce4.log'
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    # Start a user dbus session if not already available
    eval "$(dbus-launch)"
fi
echo "xfce4 up"

echo "starting novnc ${NOVNC_VERSION}"
screen -dmS novnc bash -c '/usr/local/novnc/noVNC-${NOVNC_VERSION}/utils/novnc_proxy --vnc localhost:5901 --listen 5801 2>&1 | tee /tmp/novnc.log'

export DISPLAY=:1

# make sure .Xauthority has an entry for :1
touch /home/ros/.Xauthority 2>/dev/null || true
xauth generate :1 . trusted 2>/dev/null || true

# change background
(sleep 30; xfconf-query -c xfce4-desktop -p $(xfconf-query -c xfce4-desktop -l | grep "workspace0/last-image") -s /usr/share/backgrounds/xfce/lcas.jpg > /dev/null 2>&1)&

echo 
echo "****************************************************************************************************************************************"
echo "Desktop ready. Open your browser at http://localhost:5801/vnc.html?autoconnect=true or another hostname and port you may have forwarded."
echo "****************************************************************************************************************************************"
echo