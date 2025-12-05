Install CasAdi:
git clone https://github.com/casadi/casadi.git
cd casadi
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install



Compile and build:
cd ~/ws
colcon build 
source install/setup.bash