# FIXME: Its just a collection of commands, needs rework
echo "Run the script as ROOT"

sudo apt-get update
sudo apt-get upgrade

# Few installs, for future.
sudo apt-get install python-dev python-opencv python-wxgtk3.0 python-matplotlib python-pygame python-lxml python-yaml vim git 

sudo apt install python-pip3
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager

# git clone only copter branch for demo, now.
git clone -b Copter-3.5.5 https://github.com/ArduPilot/ardupilot.git

cd ardupilot; git submodule update --init --recursive

# Add to system $PATH variable 
echo "Do the following Manually, for now:"
echo "cd Tools/ardutest; pwd"
echo "vim ~/.bashrc (at the end of the file add/insert the following)"
echo "export PATH="pwd-output-from-above":$PATH"
echo "Quit/Save and then execute: source ~/.bashrc"
echo "cd ardupilot dir and fire ./waf list_boards"

echo "Install with pip3"
pip3 install requirements.txt

#git clone https://github.com/dronekit/dronekit-python.git
#cd ./dronekit-python
#sudo python setup.py build
#sudo python setup.py install



