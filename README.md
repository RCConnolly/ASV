# Autonomous Surface Vehicle Library

## Authors:
 - Ryan Connolly
 
## Acknowledgements
 
 Much of the Fast Marching Method code in this repository was created by Javier V. Gomez and cloned from https://github.com/jvgomez/fast_methods.
 
## Building
This code using C++11 so a compiler g++ 4.8 or equivalent is required. All testing has been done using Ubuntu 14.04 LTS.

### Dependencies

First, CMake must be installed if it is not already on your system. One method of doing this is to download the latest CMake tar.gz file into your root directory, and then run the following commands.

```
$ tar xzf cmake-XX.XX.XX.tar.gz
$ cd cmake-XX.XX.XX
$ ./configure
$ make
$ make install
```
With CMake installed, it is recommended to install GeographicLib using CMake as described [here](https://geographiclib.sourceforge.io/html/install.html). I have summarized the commands below, which again should be run from the root directory. 

```
  tar xfpz GeographicLib-X.XX.tar.gz  
  cd GeographicLib-1.49 
  mkdir BUILD
  cd BUILD 
  cmake ..
  make 
  make test
  make install
```

The remaining dependendcies are Boost, imagemagick, and CImg which can be installed by running.

`$ sudo apt-get install imagemagick libboost-all-dev cimg-dev`

### Compiling & Running

From the home ASV directory, type the following commands to compile the FM2 executable.
'''
cd full_build
cmake ..
make
'''
The program can now be run and requires two command line arguments - a relative path to an obstacle image and a relative path to the start/goal coordinate file. An example that can be run is:

` ./FM2 ../images/channels.png ../examples/channels/start/points1.txt`
 
## Disclaimer
 
 This repository is an on-going development and is not yet ready for use.
