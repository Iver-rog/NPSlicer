# A mesh based Non Planar Slicing algorithm
A non-planar slicer for ordinary 3-axis 3d printers

![all layers](https://github.com/user-attachments/assets/915c0d88-418c-471f-b9cf-0dfd3c7f9381)

## Info
This project uses Blender to visualize generated geometry. In order to use this functionality Blender needs to be installed. If everything is set up correctly running the binary will launch blender with the output generated form the non planar slicing algorithm.

## Compiling from source

#### prerequisite: 
The [rust toolchan and cargo](https://www.rust-lang.org/tools/install) must be installed on your system.

First clone the repo:
```console
$ git clone https://github.com/Iver-rog/NPSlicer
```
Then build and run the binary using cargo inside the src directory of the newly downloaded project:
```console
$ cargo run
```
