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
  nixifiedLockHash = "aea4a67d740d2fce27f815f177630ce78d560beb24aee89bcb9627572513b2a3";
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
  "unknown".layer-gen-rs."0.1.0" = overridableMkRustCrate (profileName: rec {
    name = "layer-gen-rs";
    version = "0.1.0";
    registry = "unknown";
    src = fetchCrateLocal workspaceSrc;
  });
  
}
