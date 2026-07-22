cd stm
make -j8 all
cd build
st-flash --reset write stm.bin 0x8000000
cd ../..
