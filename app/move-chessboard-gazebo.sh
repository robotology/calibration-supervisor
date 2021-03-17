#!/bin/bash

TIME=2.0

echo "-0.06 -0.05 0.0 0.0 0.0 0.17" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.06 -0.02 0.0 0.0 0.0 -0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.06 0.0 0.0 0.0 0.0 -0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.06 0.03 0.0 0.0 0.0 -0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.06 0.05 0.0 0.0 0.0 -0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.04 -0.05 0.0 0.0 -0.35 0.3" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.035 -0.03 0.0 0.0 0.0 -0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.04 0.0 0.0 0.0 -0.35 0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.04 0.02 0.0 -0.1 0.1 -0.3" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "-0.04 0.05 -0.01 -0.15 0.2 0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.0 -0.05 0.0 -0.2 0.2 -0.25" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.0 -0.03 0.0 -0.2 0.2 0.15" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.0 0.0 0.0 0.0 -0.2 -0.2" | yarp write ... /chessboard/mover:i
sleep $TIME

echo "0.0 0.02 0.0 0.0 -0.2 0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.01 0.045 0.0 0.0 0.0 0.3" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.04 -0.05 -0.03 -0.2 -0.15 0.35" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.03 -0.03 0.0 0.0 0.2 -0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.035 0.0 0.0 0.2 -0.2 0.3" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.03 0.03 0.0 0.1 0.1 0.3" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.03 0.05 0.0 0.3 0.0 0.2" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.07 -0.05 0.0 0.15 -0.25 -0.15" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.06 -0.03 0.0 0.1 -0.2 0.3" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.06 0.01 0.0 -0.1 0.1 -0.1" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.06 0.02 0.0 0.1 0.25 0.05" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.07 0.04 -0.01 0.0 0.1 0.25" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.01 -0.05 0.0 0.1 0.1 0.15" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.0 -0.03 0.0 -0.1 0.25 -0.2" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.0 0.0 0.0 0.1 -0.4 0.35" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.0 0.02 0.0 -0.2 0.0 -0.15" | yarp write ... /chessboard/mover:i 
sleep $TIME

echo "0.0 0.05 0.0 -0.2 -0.1 -0.2" | yarp write ... /chessboard/mover:i
sleep $TIME

echo "0.0 0.0 0.0 0.0 0.0 0.0" | yarp write ... /chessboard/mover:i
