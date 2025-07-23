# step 1

##install ros2 jazzy

follow this

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html


# step 2

##install visual studio

sudo apt update
sudo apt install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] \
https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code


u can call VS using "code"


# step 3

##install gazebo harmonic

follow this
https://gazebosim.org/docs/harmonic/install_ubuntu/

# step 4

create ws

