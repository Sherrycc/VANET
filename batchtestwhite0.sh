#!bin/sh
./a.out 10 0 0 1000 1 30 64 16 5 4 > logs/1021/log1
echo exp 10nn base fin!
./a.out 10 1 0 1000 1 30 64 16 5 4 > logs/1021/log2
echo exp 10nn ex fin!
./a.out 10 0 1 1000 1 30 64 16 5 4 > logs/1021/log3
echo exp 10nn rsu fin!
./a.out 10 1 1 1000 1 30 64 16 5 4 > logs/1021/log4
echo exp 10nn rsu ex fin!