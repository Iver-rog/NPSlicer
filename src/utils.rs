use std::{os::unix::process::CommandExt, process::Command};


pub fn show(){
    let python_launch_script = "/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/mesh/view2.py";
    let _ = Command::new("blender")
        .arg("-P")
        .arg(python_launch_script)
        .exec();
}




