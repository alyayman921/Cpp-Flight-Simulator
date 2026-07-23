current_dir="$(pwd)"
mkdir /tmp/simdepends && cd /tmp/simdepends
git clone https://github.com/brechtsanders/xlsxio.git && cd xlsx*
rm -rf build
mkdir build && cd build && cmake -G"Unix Makefiles" ..
make 
sudo make install
sudo ldconfig 
cd  "$current_dir"
g++ -std=c++23 flightsim.cpp -o FlightSimulator -I/usr/include/eigen3 -lm -lxlsxio_read
