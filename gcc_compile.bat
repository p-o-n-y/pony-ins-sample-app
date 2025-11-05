echo off
echo This is a sample shell script to compile using GCC, which may not work on your particular system
echo Better use own preferred compilation workflow or IDE
echo when compiling in C++ project, compile as C++ code (or use g++, etc.)
echo clone pony repository into ../pony/ folder
echo amend path to GCC execulable as necessary
echo edit GCC flags and arguments according to own version
echo if GCC does not support wildcards, specify each file separately
echo on
gcc -lm -O3 ../pony/*.c ../pony/ins/*.c *.c -o pony-ins-sample-app.exe