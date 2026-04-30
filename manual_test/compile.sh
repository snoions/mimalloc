gcc -O2 -o producer cxl_example.c -I../include -L../build -lmimalloc -lrt -pthread
gcc -O2 -o consumer cxl_example.c -I../include -L../build -lmimalloc -lrt -pthread -DCONSUMER

