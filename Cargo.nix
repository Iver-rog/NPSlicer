# This file was @generated by cargo2nix 0.11.0.
# It is not intended to be manually edited.

args@{
  release ? true,
  rootFeatures ? [
    "layer-gen-rs/default"
  ],
  rustPackages,
  buildRustPackages,
  hostPlatform,
  hostPlatformCpu ? null,
  hostPlatformFeatures ? [],
  target ? null,
  codegenOpts ? null,
  profileOpts ? null,
  cargoUnstableFlags ? null,
  rustcLinkFlags ? null,
  rustcBuildFlags ? null,
  mkRustCrate,
  rustLib,
  lib,
  workspaceSrc,
  ignoreLockHash,
}:
let
  nixifiedLockHash = "ed50f9c0792b960bb0803ccceed99f51399bd96659124770789e54d9ea9f2cd5";
  workspaceSrc = if args.workspaceSrc == null then ./. else args.workspaceSrc;
  currentLockHash = builtins.hashFile "sha256" (workspaceSrc + /Cargo.lock);
  lockHashIgnored = if ignoreLockHash
                  then builtins.trace "Ignoring lock hash" ignoreLockHash
                  else ignoreLockHash;
in if !lockHashIgnored && (nixifiedLockHash != currentLockHash) then
  throw ("Cargo.nix ${nixifiedLockHash} is out of sync with Cargo.lock ${currentLockHash}")
else let
  inherit (rustLib) fetchCratesIo fetchCrateLocal fetchCrateGit fetchCrateAlternativeRegistry expandFeatures decideProfile genDrvsByProfile;
  profilesByName = {
  };
  rootFeatures' = expandFeatures rootFeatures;
  overridableMkRustCrate = f:
    let
      drvs = genDrvsByProfile profilesByName ({ profile, profileName }: mkRustCrate ({ inherit release profile hostPlatformCpu hostPlatformFeatures target profileOpts codegenOpts cargoUnstableFlags rustcLinkFlags rustcBuildFlags; } // (f profileName)));
    in { compileMode ? null, profileName ? decideProfile compileMode release }:
      let drv = drvs.${profileName}; in if compileMode == null then drv else drv.override { inherit compileMode; };
in
{
  cargo2nixVersion = "0.11.0";
  workspace = {
    layer-gen-rs = rustPackages.unknown.layer-gen-rs."0.1.0";
  };
  "registry+https://github.com/rust-lang/crates.io-index".approx."0.5.1" = overridableMkRustCrate (profileName: rec {
    name = "approx";
    version = "0.5.1";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "cab112f0a86d568ea0e627cc1d6be74a1e9cd55214684db5561995f6dad897c6"; };
    dependencies = {
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".autocfg."1.4.0" = overridableMkRustCrate (profileName: rec {
    name = "autocfg";
    version = "1.4.0";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "ace50bade8e6234aa140d9a2f552bbee1db4d353f69b8217bc503490fc1a9f26"; };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".bytemuck."1.20.0" = overridableMkRustCrate (profileName: rec {
    name = "bytemuck";
    version = "1.20.0";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "8b37c88a63ffd85d15b406896cc343916d7cf57838a847b3a6f2ca5d39a5695a"; };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".byteorder."1.5.0" = overridableMkRustCrate (profileName: rec {
    name = "byteorder";
    version = "1.5.0";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "1fd0f2584146f6f2ef48085050886acf353beff7305ebd1ae69500e27c67f64b"; };
    features = builtins.concatLists [
      [ "default" ]
      [ "std" ]
    ];
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".float-cmp."0.9.0" = overridableMkRustCrate (profileName: rec {
    name = "float-cmp";
    version = "0.9.0";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "98de4bbd547a563b716d8dfa9aad1cb19bfab00f4fa09a6a4ed21dbcf44ce9c4"; };
    features = builtins.concatLists [
      [ "default" ]
      [ "num-traits" ]
      [ "ratio" ]
    ];
    dependencies = {
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".hashbrown."0.12.3" = overridableMkRustCrate (profileName: rec {
    name = "hashbrown";
    version = "0.12.3";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "8a9ee70c43aaf417c914396645a0fa852624801b24ebb7ae78fe8272889ac888"; };
    features = builtins.concatLists [
      [ "raw" ]
    ];
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".indexmap."1.9.3" = overridableMkRustCrate (profileName: rec {
    name = "indexmap";
    version = "1.9.3";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "bd070e393353796e801d209ad339e89596eb4c8d430d18ede6a1cced8fafbd99"; };
    dependencies = {
      hashbrown = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".hashbrown."0.12.3" { inherit profileName; }).out;
    };
    buildDependencies = {
      autocfg = (buildRustPackages."registry+https://github.com/rust-lang/crates.io-index".autocfg."1.4.0" { profileName = "__noProfile"; }).out;
    };
  });
  
  "unknown".layer-gen-rs."0.1.0" = overridableMkRustCrate (profileName: rec {
    name = "layer-gen-rs";
    version = "0.1.0";
    registry = "unknown";
    src = fetchCrateLocal workspaceSrc;
    dependencies = {
      nalgebra = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".nalgebra."0.32.6" { inherit profileName; }).out;
      nalgebra_glm = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".nalgebra-glm."0.18.0" { inherit profileName; }).out;
      ordered_float = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".ordered-float."3.9.2" { inherit profileName; }).out;
      priority_queue = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".priority-queue."1.4.0" { inherit profileName; }).out;
      stl_io = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".stl_io."0.8.3" { inherit profileName; }).out;
      thiserror = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".thiserror."1.0.69" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".matrixmultiply."0.3.9" = overridableMkRustCrate (profileName: rec {
    name = "matrixmultiply";
    version = "0.3.9";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "9380b911e3e96d10c1f415da0876389aaf1b56759054eeb0de7df940c456ba1a"; };
    features = builtins.concatLists [
      [ "default" ]
      [ "std" ]
    ];
    dependencies = {
      rawpointer = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".rawpointer."0.2.1" { inherit profileName; }).out;
    };
    buildDependencies = {
      autocfg = (buildRustPackages."registry+https://github.com/rust-lang/crates.io-index".autocfg."1.4.0" { profileName = "__noProfile"; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".nalgebra."0.32.6" = overridableMkRustCrate (profileName: rec {
    name = "nalgebra";
    version = "0.32.6";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "7b5c17de023a86f59ed79891b2e5d5a94c705dbe904a5b5c9c952ea6221b03e4"; };
    features = builtins.concatLists [
      [ "default" ]
      [ "macros" ]
      [ "matrixmultiply" ]
      [ "nalgebra-macros" ]
      [ "std" ]
    ];
    dependencies = {
      approx = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".approx."0.5.1" { inherit profileName; }).out;
      matrixmultiply = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".matrixmultiply."0.3.9" { inherit profileName; }).out;
      nalgebra_macros = (buildRustPackages."registry+https://github.com/rust-lang/crates.io-index".nalgebra-macros."0.2.2" { profileName = "__noProfile"; }).out;
      num_complex = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-complex."0.4.6" { inherit profileName; }).out;
      num_rational = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-rational."0.4.2" { inherit profileName; }).out;
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
      simba = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".simba."0.8.1" { inherit profileName; }).out;
      typenum = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".typenum."1.17.0" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".nalgebra-glm."0.18.0" = overridableMkRustCrate (profileName: rec {
    name = "nalgebra-glm";
    version = "0.18.0";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "e68879ff227a94627e63bbd518b4f82b8f0cc56bb01a498251507de6d1c412d6"; };
    features = builtins.concatLists [
      [ "default" ]
      [ "std" ]
    ];
    dependencies = {
      approx = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".approx."0.5.1" { inherit profileName; }).out;
      nalgebra = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".nalgebra."0.32.6" { inherit profileName; }).out;
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
      simba = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".simba."0.8.1" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".nalgebra-macros."0.2.2" = overridableMkRustCrate (profileName: rec {
    name = "nalgebra-macros";
    version = "0.2.2";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "254a5372af8fc138e36684761d3c0cdb758a4410e938babcff1c860ce14ddbfc"; };
    dependencies = {
      proc_macro2 = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".proc-macro2."1.0.92" { inherit profileName; }).out;
      quote = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".quote."1.0.37" { inherit profileName; }).out;
      syn = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".syn."2.0.89" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".num-complex."0.4.6" = overridableMkRustCrate (profileName: rec {
    name = "num-complex";
    version = "0.4.6";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "73f88a1307638156682bada9d7604135552957b7818057dcef22705b4d509495"; };
    dependencies = {
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".num-integer."0.1.46" = overridableMkRustCrate (profileName: rec {
    name = "num-integer";
    version = "0.1.46";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "7969661fd2958a5cb096e56c8e1ad0444ac2bbcd0061bd28660485a44879858f"; };
    features = builtins.concatLists [
      [ "i128" ]
    ];
    dependencies = {
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".num-rational."0.4.2" = overridableMkRustCrate (profileName: rec {
    name = "num-rational";
    version = "0.4.2";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "f83d14da390562dca69fc84082e73e548e1ad308d24accdedd2720017cb37824"; };
    dependencies = {
      num_integer = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-integer."0.1.46" { inherit profileName; }).out;
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" = overridableMkRustCrate (profileName: rec {
    name = "num-traits";
    version = "0.2.19";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "071dfc062690e90b734c0b2273ce72ad0ffa95f0c74596bc250dcfd960262841"; };
    features = builtins.concatLists [
      [ "i128" ]
      [ "std" ]
    ];
    buildDependencies = {
      autocfg = (buildRustPackages."registry+https://github.com/rust-lang/crates.io-index".autocfg."1.4.0" { profileName = "__noProfile"; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".ordered-float."3.9.2" = overridableMkRustCrate (profileName: rec {
    name = "ordered-float";
    version = "3.9.2";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "f1e1c390732d15f1d48471625cd92d154e66db2c56645e29a9cd26f4699f72dc"; };
    features = builtins.concatLists [
      [ "default" ]
      [ "std" ]
    ];
    dependencies = {
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".paste."1.0.15" = overridableMkRustCrate (profileName: rec {
    name = "paste";
    version = "1.0.15";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "57c0d7b74b563b49d38dae00a0c37d4d6de9b432382b2892f0574ddcae73fd0a"; };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".priority-queue."1.4.0" = overridableMkRustCrate (profileName: rec {
    name = "priority-queue";
    version = "1.4.0";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "a0bda9164fe05bc9225752d54aae413343c36f684380005398a6a8fde95fe785"; };
    dependencies = {
      indexmap = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".indexmap."1.9.3" { inherit profileName; }).out;
    };
    buildDependencies = {
      autocfg = (buildRustPackages."registry+https://github.com/rust-lang/crates.io-index".autocfg."1.4.0" { profileName = "__noProfile"; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".proc-macro2."1.0.92" = overridableMkRustCrate (profileName: rec {
    name = "proc-macro2";
    version = "1.0.92";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "37d3544b3f2748c54e147655edb5025752e2303145b5aefb3c3ea2c78b973bb0"; };
    features = builtins.concatLists [
      [ "default" ]
      [ "proc-macro" ]
    ];
    dependencies = {
      unicode_ident = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".unicode-ident."1.0.14" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".quote."1.0.37" = overridableMkRustCrate (profileName: rec {
    name = "quote";
    version = "1.0.37";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "b5b9d34b8991d19d98081b46eacdd8eb58c6f2b201139f7c5f643cc155a633af"; };
    features = builtins.concatLists [
      [ "default" ]
      [ "proc-macro" ]
    ];
    dependencies = {
      proc_macro2 = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".proc-macro2."1.0.92" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".rawpointer."0.2.1" = overridableMkRustCrate (profileName: rec {
    name = "rawpointer";
    version = "0.2.1";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "60a357793950651c4ed0f3f52338f53b2f809f32d83a07f72909fa13e4c6c1e3"; };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".safe_arch."0.7.2" = overridableMkRustCrate (profileName: rec {
    name = "safe_arch";
    version = "0.7.2";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "c3460605018fdc9612bce72735cba0d27efbcd9904780d44c7e3a9948f96148a"; };
    features = builtins.concatLists [
      [ "bytemuck" ]
      [ "default" ]
    ];
    dependencies = {
      bytemuck = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".bytemuck."1.20.0" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".simba."0.8.1" = overridableMkRustCrate (profileName: rec {
    name = "simba";
    version = "0.8.1";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "061507c94fc6ab4ba1c9a0305018408e312e17c041eb63bef8aa726fa33aceae"; };
    features = builtins.concatLists [
      [ "std" ]
      [ "wide" ]
    ];
    dependencies = {
      approx = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".approx."0.5.1" { inherit profileName; }).out;
      num_complex = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-complex."0.4.6" { inherit profileName; }).out;
      num_traits = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".num-traits."0.2.19" { inherit profileName; }).out;
      paste = (buildRustPackages."registry+https://github.com/rust-lang/crates.io-index".paste."1.0.15" { profileName = "__noProfile"; }).out;
      wide = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".wide."0.7.30" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".stl_io."0.8.3" = overridableMkRustCrate (profileName: rec {
    name = "stl_io";
    version = "0.8.3";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "f985fe7ba9bd93bb8f0158689a0601d4bc5480a7df11c6f22c220fa1bedaefe7"; };
    dependencies = {
      byteorder = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".byteorder."1.5.0" { inherit profileName; }).out;
      float_cmp = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".float-cmp."0.9.0" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".syn."2.0.89" = overridableMkRustCrate (profileName: rec {
    name = "syn";
    version = "2.0.89";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "44d46482f1c1c87acd84dea20c1bf5ebff4c757009ed6bf19cfd36fb10e92c4e"; };
    features = builtins.concatLists [
      [ "clone-impls" ]
      [ "default" ]
      [ "derive" ]
      [ "full" ]
      [ "parsing" ]
      [ "printing" ]
      [ "proc-macro" ]
    ];
    dependencies = {
      proc_macro2 = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".proc-macro2."1.0.92" { inherit profileName; }).out;
      quote = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".quote."1.0.37" { inherit profileName; }).out;
      unicode_ident = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".unicode-ident."1.0.14" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".thiserror."1.0.69" = overridableMkRustCrate (profileName: rec {
    name = "thiserror";
    version = "1.0.69";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "b6aaf5339b578ea85b50e080feb250a3e8ae8cfcdff9a461c9ec2904bc923f52"; };
    dependencies = {
      thiserror_impl = (buildRustPackages."registry+https://github.com/rust-lang/crates.io-index".thiserror-impl."1.0.69" { profileName = "__noProfile"; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".thiserror-impl."1.0.69" = overridableMkRustCrate (profileName: rec {
    name = "thiserror-impl";
    version = "1.0.69";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "4fee6c4efc90059e10f81e6d42c60a18f76588c3d74cb83a0b242a2b6c7504c1"; };
    dependencies = {
      proc_macro2 = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".proc-macro2."1.0.92" { inherit profileName; }).out;
      quote = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".quote."1.0.37" { inherit profileName; }).out;
      syn = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".syn."2.0.89" { inherit profileName; }).out;
    };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".typenum."1.17.0" = overridableMkRustCrate (profileName: rec {
    name = "typenum";
    version = "1.17.0";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "42ff0bf0c66b8238c6f3b578df37d0b7848e55df8577b3f74f92a69acceeb825"; };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".unicode-ident."1.0.14" = overridableMkRustCrate (profileName: rec {
    name = "unicode-ident";
    version = "1.0.14";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "adb9e6ca4f869e1180728b7950e35922a7fc6397f7b641499e8f3ef06e50dc83"; };
  });
  
  "registry+https://github.com/rust-lang/crates.io-index".wide."0.7.30" = overridableMkRustCrate (profileName: rec {
    name = "wide";
    version = "0.7.30";
    registry = "registry+https://github.com/rust-lang/crates.io-index";
    src = fetchCratesIo { inherit name version; sha256 = "58e6db2670d2be78525979e9a5f9c69d296fd7d670549fe9ebf70f8708cb5019"; };
    features = builtins.concatLists [
      [ "std" ]
    ];
    dependencies = {
      bytemuck = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".bytemuck."1.20.0" { inherit profileName; }).out;
      safe_arch = (rustPackages."registry+https://github.com/rust-lang/crates.io-index".safe_arch."0.7.2" { inherit profileName; }).out;
    };
  });
  
}
