sudo fallocate /var/swapfile -l 1G
sudo chmod 600 /var/swapfile
sudo mkswap /var/swapfile
sudo swapon /var/swapfile
sudo apt-get install cmake git
sudo apt-get install build-essential pkg-config
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng-dev
sudo apt-get install libtbb-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk2.0-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python2.7-dev python3-dev python-numpy python3-numpy


cd ~
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.1.0.zip
unzip opencv.zip
cd opencv-3.1.0/
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install
sudo ldconfig

sudo apt-get install libeigen3-dev libblas-dev liblapack-dev libglew-dev

cd ~
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
mkdir build
cd build
cmake ..
make

cd ~
git clone https://github.com/raulmur/ORB_SLAM2
cd ORB_SLAM2



cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../..
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz

cd ..
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make


sudo swapoff /var/swapfile
sudo rm /var/swapfile
