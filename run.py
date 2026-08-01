#!/usr/bin/env python3
"""Start the bridge with dialout group access."""
import os
import sys
import subprocess

venv = os.path.expanduser("~/Downloads/so101-bridge/.venv/bin/python3")
server = os.path.expanduser("~/Downloads/so101-bridge/server.py")
log = "/tmp/so101-bridge.log"

# Start with newgrp to get dialout access
cmd = f'newgrp dialout <<"EOF"\n{venv} {server} > {log} 2>&1 &\nEOF'
subprocess.run(["bash", "-c", cmd])
