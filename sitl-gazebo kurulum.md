--- ardupilot kodları - gazebo harmonic - sitl - mavproxy pymavlink ---


1. Sistem Güncellemesi ve Temel Paketler
cd
sudo apt update && sudo apt dist-upgrade -y
sudo apt install -y htop neofetch git gcc make curl bzip2 tar unzip python3-pip

2. ArduPilot Kaynak Kodu ve Bağımlılıklar
cd
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

3. ArduPilot SITL Derleme

cd ardupilot 
./waf configure --board sitl
./waf copter
./waf plane

4. MAVProxy ve Pymavlink Kurulumu
cd

pip install --user --upgrade pymavlink MAVProxy --break-system-packages

ya da 

sudo apt update
sudo apt install python3-pymavlink python3-mavproxy

burada kütüphaneler eksik yükleniyor, eksikleri yüklemek için bunları da çalıştırın yoksa sitl açılmıyor

pip install --user future --break-system-packages

pip install --user future lxml pymavlink MAVProxy --break-system-packages

5. gazebo harmonic
cd
sudo apt-get update
sudo apt-get install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libgz-transport13-dev libgz-msgs10-dev

6. ArduPilot Gazebo Plugin Kurulumu
cd
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)

7. Ortam Değişkenlerini Tanımlama
cd
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc

--- ÇALIŞTIRMA ---

link 1 down 
pkill -9 -f "ardupilot|mavproxy|gz|sim_vehicle|ruby"

TERMİNAL 1

# Drone için:
cd
gz sim -v4 -r iris_runway.sdf

# Sabit Kanat için:
cd
gz sim -v4 -r zephyr_runway.sdf

TERMİNAL 2 - SITL 

dizin:
eğer copter çalıştıracaksan

cd ~/ardupilot/ArduCopter

# Drone için (ArduCopter dizininde):
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console


eğer plane calıstıracaksan

cd ~/ardupilot/ArduPlane

# Sabit Kanat için (ArduPlane dizininde):
../Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console

