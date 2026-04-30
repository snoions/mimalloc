LD_LIBRARY_PATH=../build ./producer & 
sleep 1
LD_LIBRARY_PATH=../build ./consumer
