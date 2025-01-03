{
  inputs = {
    cargo2nix.url = "github:cargo2nix/cargo2nix/";
    flake-utils.follows = "cargo2nix/flake-utils";
    nixpkgs.follows = "cargo2nix/nixpkgs";
  };

  outputs = inputs: with inputs;
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [cargo2nix.overlays.default];
        };

        rustPkgs = pkgs.rustBuilder.makePackageSet {
          rustVersion = "1.75.0";
          packageFun = import ./Cargo.nix;
        };

      in rec {
        packages = {
          layer-gen-rs = (rustPkgs.workspace.layer-gen-rs {});
          default = packages.layer-gen-rs;
        };
      }
    );
}

