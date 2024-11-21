# Non Planar Slicing
This is a work in progres program for doing suport free 3D printing through non planar slicing. The code in this repo is not yet at a stage where the entire slicing pipeline can be preformed. At the moment only the first part of the slicing pipeline is implemented, That beeing a partly working layer generation script. 

![all layers](https://github.com/user-attachments/assets/915c0d88-418c-471f-b9cf-0dfd3c7f9381)

## Info
This project uses Blender to visualize generated geometry. In order to use this functionality Blender needs to be installed. If everything is set up correctly running the binary should automaticaly launch blender with the output generated form the non planar slicing algorithm.

## Compiling from source
The code in this repo can be buildt with either the nix package manager or rust's build system Cargo. The two possible methods are presented bellow:

### On Linux with the nix package manager:  
On a Linux system with the nix package manager installed and the experimental cli enabled, the code can be run directly using the following command:
'''console
$ nix run https://github.com/Iver-rog/NPSlicer
'''

### On other opperating systems using cargo:
For unix systems (linux and macOS) without the nix package manager installed the code can be downloaded and buildt manualy using Rust's build system cargo. At the moment the non planar slicin algorithm uses os spesific code to launch blender for visualization. As a result the project can only be buildt on unix systems.

#### prerequisite: 
The rust toolchain and cargo must be installed on your system:
[download the rust toolchan](https://www.rust-lang.org/tools/install)

First clone the repo:
'''console
$ git clone https://github.com/Iver-rog/NPSlicer
'''
Then build and run the binary using cargo inside the newly downloaded project:
'''console
$ cargo run
'''
