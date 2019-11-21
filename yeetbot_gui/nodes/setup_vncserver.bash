#!/usr/bin/env bash
sudo apt update
sudo apt install tightvncserver -y
tightvncserver

rm ~/.vnc/xstartup
touch ~/.vnc/xstartup
echo '#!/bin/sh' >> ~/.vnc/xstartup
echo 'xrdb $HOME/.Xresources' >> ~/.vnc/xstartup
echo 'xsetroot -solid pink' >> ~/.vnc/xstartup
echo 'vncconfig -iconic &' >> ~/.vnc/xstartup
echo 'export XKL_XMODMAP_DISABLE=1' >> ~/.vnc/xstartup
echo '/etc/X11/Xsession' >> ~/.vnc/xstartup
echo '/usr/bin/xterm -geometry 80x24+100+100 -ls -title "Launch Me" &' >> ~/.vnc/xstartup
chmod a+x ~/.vnc/xstartup
