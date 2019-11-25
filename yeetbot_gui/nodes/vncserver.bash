#!/usr/bin/env bash

# If a server exists already, kill it
vncserver -kill :1

# Start the server
vncserver :1 -geometry 1600x2560 -depth 24 -dpi 96
