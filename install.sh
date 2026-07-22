git clone https://github.com/brechtsanders/xlsxio.git && cd xlsx*
cmake -G"Unix Makefiles" .
make
sudo make install
sudo ldconfig 
cd ../
g++ flightsim.cpp -o FlightSimulator -I/usr/include/eigen3 -lm -lxlsxio_read
