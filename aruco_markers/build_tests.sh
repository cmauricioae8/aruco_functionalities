#!/bin/bash

info()   { printf "\n\033[1;34m[INFO]\033[0m %s\n" "$*"; }
ok()  { printf "\033[1;32m[OK]\033[0m %s\n" "$*"; }
warn()  { printf "\033[1;33m[WARN]\033[0m %s\n" "$*"; }
fail()  { printf "\033[1;31m[FAIL]\033[0m %s\n" "$*"; exit 1; }


# Create build directory if it doesn't exist
if [ ! -d build ]; then
    mkdir -p build
    info "Build directory created."
else
    ok "Build directory already exists."
fi

# Copy calibration values if they don't exist in 'build' directory
if [ ! -f build/calibration_values.txt ]; then
    cp dummy_calibration_values.txt build/calibration_values.txt
    info "Calibration values copied."
else
    ok "Calibration values already exist."
fi

# Change to build directory
cd build

# CMake configuration
info "CMake configuration..."
cmake ..

if [ $? -ne 0 ]; then
    fail "CMake failed."
fi

# Build executables
info "Building executables..."
make

if [ $? -ne 0 ]; then
    fail "Make failed."
fi

# Post build instructions
ok "Executables built successfully. To run:"
info "./markers_generator"
