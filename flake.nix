{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [ "x86_64-linux" ];
      perSystem = { config, self', pkgs, lib, system, ... }:
        let
          runtimeDeps = with pkgs; [ alsa-lib speechd
            freetype
            expat
            fontconfig
            openssl

            # necessary for building wgpu in 3rd party packages (in most cases)
            libxkbcommon
            wayland libX11 libXcursor libXrandr libXi
            alsa-lib
            fontconfig freetype
            directx-shader-compiler

            libGL
            vulkan-headers vulkan-loader
          ];
          buildDeps = with pkgs; [ pkg-config rustPlatform.bindgenHook 
            # openssl
            pkg-config

            # necessary for building wgpu in 3rd party packages (in most cases)
            pkg-config cmake

            # necessary for developing (all of) wgpu itself
            cargo-nextest cargo-fuzz
          ];
          devDeps = with pkgs; [ 
            mold
            gdb renderdoc cargo-flamegraph 

            shaderc directx-shader-compiler
            vulkan-tools vulkan-tools-lunarg
            vulkan-extension-layer
            vulkan-validation-layers # don't need them *strictly* but immensely helpful
          ];

          cargoToml = builtins.fromTOML (builtins.readFile ./Cargo.toml);
          msrv = cargoToml.package.rust-version;

          rustPackage = features:
            (pkgs.makeRustPlatform {
              cargo = pkgs.rust-bin.stable.latest.minimal;
              rustc = pkgs.rust-bin.stable.latest.minimal;
            }).buildRustPackage {
              # inherit (cargoToml.package) name version;
              name = "NPSlicer";
              version = "0.1";
              src = ./.;
              cargoLock.lockFile = ./Cargo.lock;
              buildFeatures = features;
              buildInputs = runtimeDeps;
              nativeBuildInputs = buildDeps;
              # Uncomment if your cargo tests require networking or otherwise
              # don't play nicely with the Nix build sandbox:
              # doCheck = false;
            };

          mkDevShell = rustc:
            pkgs.mkShell {
              shellHook = ''
                export RUST_SRC_PATH=${pkgs.rustPlatform.rustLibSrc}
                export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${builtins.toString (pkgs.lib.makeLibraryPath runtimeDeps)}";
                export RUSTFLAGS="-C link-arg=-fuse-ld=mold"
              '';
              buildInputs = runtimeDeps;
              nativeBuildInputs = buildDeps ++ devDeps ++ [ rustc ];
            };
        in {
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [ (import inputs.rust-overlay) ];
          };


          packages.default = self'.packages.example-base;
          devShells.default = self'.devShells.nightly;

          packages.example = (rustPackage "foobar");
          packages.example-base = (rustPackage "");

          devShells.nightly = (mkDevShell (pkgs.rust-bin.selectLatestNightlyWith
            (toolchain: toolchain.default)));
          devShells.stable = (mkDevShell pkgs.rust-bin.stable.latest.default);
          devShells.msrv = (mkDevShell pkgs.rust-bin.stable.${msrv}.default);
        };
    };
}
