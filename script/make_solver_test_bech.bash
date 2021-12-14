rm -r ../runtime_img
mkdir ../runtime_img
cd ../build
rm -r *
cmake ..
make
./solver_test_bench
cd ../script