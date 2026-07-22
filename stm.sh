cd stm || exit 1
make -j8 all || exit 1
cd build || exit 1
st-flash --reset write stm.bin 0x8000000
cd ../..
