# ECE398-Fa26
ECE398 Foundations for Robotics (Fall 2026)

## Installation
To run the programs on your local machine, make sure you have installed conda. We recommend using miniconda.

Assuming you have conda installed, you can create a conda environment `ece398` with the following:
```
conda create -n ece398 python=3.10.12
conda activate ece398
pip3 install numpy
pip3 install scikit-image
pip3 install matplotlib
pip3 install opencv-python
pip3 install pupil-apriltags
pip3 install pybullet==3.2.7
```

Then, for running the programs, all you need to do is make sure that you activate the environment before running. E.g., if you want to run `main.py`, do:
```
conda activate ece398
python3 main.py
```

## Running Assignment Code
To run the code for each assignment, you should run from the root folder of this repo (i.e., the same folder as this README).
For example, for the `intro` lab, do:
```
conda activate ece398
python3 assignments/intro/main.py
```

## If you have issues
The conda environment process has been tested on Ubuntu 22.04. If you have issues (e.g., if using ARM like Apple Silicon), I recommend running an Ubuntu virtual machine using a setup like VirtualBox.
