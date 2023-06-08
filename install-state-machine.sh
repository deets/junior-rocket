#!/bin/zsh
BASE=$(realpath $(dirname $0))
TFA=~/Dropbox/projects/timed-finite-automaton
E=examples/junior-rocket-state
FILE_LIST=(include/statistics.hpp include/timed-finite-automaton.hpp $E/junior-rocket-state.hpp $E/junior-rocket-state.cpp)
echo $BASE
echo $TFA

for fname ("$FILE_LIST[@]")
do
    cp -v $TFA/$fname $BASE
done
