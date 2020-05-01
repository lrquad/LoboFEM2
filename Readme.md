# LoboFEM
![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/scene.png)

## Todo

09/22/2019
- [ ] cublas
- [ ] optimize NN 

## Introduction
- Imgui
- OpenGL 
- libigl 
- c++14 (for imfilebrowser)
- hyperelastic material based FEM simulator
- boost


Multi objects

![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/screen_recored2.gif)


![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/screen_recored3.gif)

Constraint

![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/screen_recored4.gif)

Collision (no friction force yet)

![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/screen_recored5.gif)

Collision (friction force stiffnes 0.1)

![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/screen_recored6.gif)

Collision (friction force stiffnes 1.0)

![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/screen_recored7.gif)

Grasp (friction test failed)

![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/sphere_grasp_failed.gif)

Grasp (stable, squeeze more)

![scene](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/sphere_grasp.gif)


## Install

Require MKL
set MKL root by {MKLROOT}
put below code into ~/.bashrc
CPRO_PATH=/opt/intel/compilers_and_libraries_2019.4.243/linux
export MKLROOT=${CPRO_PATH}/mkl

Eigen only support ip64 not the ilp64.
If use ilp64, pardiso will give a long long int conversion error.


    sudo apt-get install libomp-dev
    sudo apt-get install cmake libx11-dev xorg-dev libglu1-mesa-dev libglm-dev


    cd external
    git clone https://github.com/eigenteam/eigen-git-mirror.git

    cd ..

    mkdir build
    cd build
    cmake ..
    make
    ../bin/LoboFEM


## Structure
![Structure](https://github.com/lrquad/LoboFEMCmake/blob/master/demo/default/Images/LoboFEM.jpeg)


## Note
    sudo nano /etc/modprobe.d/zz-nvidia-modeset.conf
    options nvidia_drm modeset=1
    sudo update-initramfs -u
This helps to remove tearing on my laptop. However, it locks FPS to 60fps.
My laptop: Razer Blade RTX 2070

To support cuda 10.0 need cmake version 3.12 or later

https://stackoverflow.com/questions/993352/when-should-i-make-explicit-use-of-the-this-pointer
https://stackoverflow.com/questions/2812470/why-does-gcc-need-extra-declarations-in-templates-when-vs-does-not/2812501#2812501


bash ./run.sh c x ./config/DinosaurDemo/Dinosaur_demo_gen.xml