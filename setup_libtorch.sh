#!/bin/bash
set -euo pipefail

echo "Do you want CPU or GPU version?"
echo "1) CPU (cxx11-abi)"
echo "2) GPU CUDA 12.1 (cxx11-abi)"
read -p "Enter choice (1 or 2): " choice

# Create third_party directory if it doesn't exist
mkdir -p aow_controllers/third_party
cd aow_controllers/third_party

# clean any old libtorch first (prevents mixed headers/libs)
rm -rf libtorch

if [ "$choice" = "1" ]; then
    echo "Downloading CPU (cxx11-abi)..."
    url="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcpu.zip"
    file="libtorch-cxx11-abi-shared-with-deps-2.1.0+cpu.zip"
elif [ "$choice" = "2" ]; then
    echo "Downloading GPU CUDA 12.1 (cxx11-abi)..."
    url="https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcu121.zip"
    file="libtorch-cxx11-abi-shared-with-deps-2.1.0+cu121.zip"
else
    echo "Invalid choice"
    exit 1
fi

wget -O "$file" "$url"
unzip "$file"
rm "$file"

echo "Done! LibTorch installed in aow_controllers/third_party/libtorch"
