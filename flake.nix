{
  description = "A Nix-flake-based C/C++ development environment";

  inputs.nixpkgs.url = "https://flakehub.com/f/NixOS/nixpkgs/0.1";

  outputs = inputs:
    let
      supportedSystems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forEachSupportedSystem = f: inputs.nixpkgs.lib.genAttrs supportedSystems (system: f {
        pkgs = import inputs.nixpkgs { inherit system; };
      });
    in
    {
      devShells = forEachSupportedSystem ({ pkgs }: 
        let
          pico-sdk-with-submodules = pkgs.pico-sdk.override { withSubmodules = true; };
        in
        {
          default = pkgs.mkShell.override
            {
              # Override stdenv in order to change compiler:
              # stdenv = pkgs.clangStdenv;
            }
            {
              packages = with pkgs; [
                gcc-arm-embedded-14
                clang-tools
                cmake
                codespell
                conan
                cppcheck
                doxygen
                gtest
                lcov
                vcpkg
                vcpkg-tool
                pico-sdk-with-submodules
                picotool
              ] ++ (if system == "aarch64-darwin" then [ ] else [ gdb ]);
              
              
              shellHook = ''
                export PICO_SDK_PATH=${pico-sdk-with-submodules}/lib/pico-sdk
              '';
            };
      });
    };
}