#!bin/sh
./a.out 6 0 0 1000 30 64 16 10 4 2000 12 > logs/1121/6_base
echo exp 6nn base fin!
./a.out 6 1 0 1000 30 64 16 10 4 2000 12 > logs/1121/6_ex
echo exp 6nn ex fin!
./a.out 6 0 1 1000 30 64 16 10 4 2000 12 > logs/1121/6_rsu
echo exp 6nn rsu fin!
./a.out 6 1 1 1000 30 64 16 10 4 2000 12 > logs/1121/6_ex_rsu
echo exp 6nn ex rsu fin!