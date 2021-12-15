rm -r ../runtime_img
mkdir ../runtime_img
cd ../build
./solver_test_bench ../src/experiments/config/config1_same_img.yaml
cd ../script
