#! /bin/sh

sudo apt-get update

# Set power and screensaver settings for convenience
sudo gsettings set org.gnome.desktop.screensaver lock-enabled false
sudo gsettings set org.gnome.desktop.screensaver lock-delay 0
sudo gsettings set org.gnome.desktop.screensaver ubuntu-lock-on-suspend false
sudo gsettings set org.gnome.settings-daemon.plugins.power button-power shutdown
sudo gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
sudo gsettings set org.gnome.desktop.session idle-delay uint32 0

# Install open SSH server
sudo apt-get install openssh-server -y

# Install SVN
sudo apt-get install subversion -y

# Install git
sudo apt-get install git -y

# Step 1 - Install X11VNC  
# ################################################################# 
sudo apt-get install x11vnc -y

# Step 2 - Specify Password to be used for VNC Connection 
# ################################################################# 

sudo x11vnc -storepasswd /etc/x11vnc.pass 


# Step 3 - Create the Service Unit File
# ################################################################# 

cat > /lib/systemd/system/x11vnc.service << EOF
[Unit]
Description=Start x11vnc at startup.
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/bin/x11vnc -auth guess -forever -loop -noxdamage -repeat -rfbauth /etc/x11vnc.pass -rfbport 5900 -shared

[Install]
WantedBy=multi-user.target
EOF

# Step 4 -Configure the Service 
# ################################################################ 

echo "Configure Services"
sudo systemctl enable x11vnc.service
sudo systemctl daemon-reload
