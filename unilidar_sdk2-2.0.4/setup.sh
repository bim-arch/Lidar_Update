#!/bin/bash

echo "==========================================="
echo "   UNITREE LIDAR - AUTO SETUP SCRIPT"
echo "==========================================="

# 1. Update Ubuntu
echo "[1/4] Updating System..."
sudo apt-get update -y

# 2. Install Dependencies (Compilers, Python, Flask, USB tools)
echo "[2/4] Installing Dependencies..."
sudo apt-get install -y build-essential cmake python3 python3-pip setserial nano git
pip3 install flask

# 3. Compile the Code
echo "[3/4] Compiling Lidar Backend..."
cd unitree_lidar_sdk
mkdir -p build
cd build
cmake ..
make -j4

# 4. Permissions
echo "[4/4] Setting Permissions..."
cd ../bin
chmod +x server.py

echo "==========================================="
echo "   SETUP COMPLETE!"
echo "   To start the scanner, run:"
echo "   cd unitree_lidar_sdk/bin"
echo "   sudo python3 server.py"
echo "==========================================="