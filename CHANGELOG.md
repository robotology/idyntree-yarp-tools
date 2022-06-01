# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Fixed
- Remove use of deprecated YARP headers and classes to ensure compatibility with YARP 3.8 (https://github.com/robotology/idyntree-yarp-tools/pull/32, https://github.com/robotology/idyntree-yarp-tools/pull/33).

## [0.0.5] - 2022-05-27

### Fixed
- Fixed compatibility with YARP 3.7 (https://github.com/robotology/idyntree-yarp-tools/pull/30).

## [0.0.4] - 2022-02-09

### Fixed
- Fixed the conversion from ``iDynTree`` to ``yarp`` pixels after https://github.com/robotology/idyntree/pull/903. Removed some deprecation warnings (https://github.com/robotology/idyntree-yarp-tools/pull/16)

## [0.0.3] - 2021-11-23

### Fixed
- Fixed compatibility with YARP 3.5.100 (https://github.com/robotology/idyntree-yarp-tools/pull/20).

## [0.0.2] - 2021-07-09

### Added
- Added `IDYNTREE_YARP_TOOLS_USES_QT_CHARTS` option to disable just the compilation of `idyntree-plotter` that depends on Qt5 Charts.

## [0.0.1] - 2021-07-08

### Added
- First release of the repo. Contains the new tool `idyntree-yarp-visualizer` and the tools `urdf2dh`, `yarprobotstatepublisher` `idyntree-plotter` and `idyntree-sole-gui` migrated from iDynTree.
