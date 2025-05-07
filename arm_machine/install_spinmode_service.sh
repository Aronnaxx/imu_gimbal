#!/bin/bash

# --- CONFIGURE THESE VARIABLES ---
WORKDIR="/home/operator/imu_gimbal"
SCRIPT="arm_machine/spin_mode.py"
PYENV="$WORKDIR/venv/bin/activate"  # Path to your virtualenv activate script
USER="operator"
SERVICE_NAME="spinmode.service"

# --- CREATE SYSTEMD SERVICE FILE ---
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

sudo bash -c "cat > $SERVICE_PATH" <<EOF
[Unit]
Description=Spin Mode Dynamixel Service
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$WORKDIR
Environment=PYTHONUNBUFFERED=1
ExecStart=/bin/bash -c 'source $PYENV && python $SCRIPT'
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

# --- RELOAD SYSTEMD, ENABLE AND START SERVICE ---
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_NAME
sudo systemctl start $SERVICE_NAME

echo "Service $SERVICE_NAME installed and started."
echo "Check status with: sudo systemctl status $SERVICE_NAME" 