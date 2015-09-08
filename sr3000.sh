#!bin/sh
./a.out 6 1 1 1000 30 64 16 10 4 3000 60 > logs/1121/r3000ex_rsu
echo exp 6nn ex rsu fin!
./a.out 6 0 1 1000 30 64 16 10 4 3000 60 > logs/1121/r3000rsu
echo exp 6nn rsu fin!
./a.out 6 1 0 1000 30 64 16 10 4 3000 60 > logs/1121/r3000ex
echo exp 6nn ex fin!
./a.out 6 0 0 1000 30 64 16 10 4 3000 60 > logs/1121/r3000base
echo exp 6nn base fin!