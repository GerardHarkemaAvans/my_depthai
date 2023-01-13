#pip install requests
#sudo apt update
#sudo apt-get install ros-melodic-ros-numpy
#sudo apt-get install ros-melodic-cv-bridge
#sudo apt install libopencv-dev

git clone https://github.com/luxonis/depthai-ros.git ../../depthai-ros



cd ../../..

rosdep install --from-paths src --ignore-src -r -y

cd src

catkin b

echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
