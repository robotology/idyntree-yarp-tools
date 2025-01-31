# Changelog

This file documents notable changes to this project done before December 2023. For changes after that date, plase refers to the release notes of each release at https://github.dev/robotology/idyntree-yarp-tools/releases .


## [0.0.9] - 2023-12-08

### Fixed

- Fix use of transformServer device in yarprobotstatepublisher https://github.com/robotology/idyntree-yarp-tools/pull/49

## [0.0.8] - 2023-11-17

### Changed

- The remapper connector will not fail if some board of joint is not available https://github.com/robotology/idyntree-yarp-tools/pull/47

## [0.0.7] - 2023-02-24

### Changed
- Share connection methods to the robot among tools https://github.com/robotology/idyntree-yarp-tools/pull/43

## [0.0.6] - 2022-09-13

### Changed
- idyntree-yarp-visualizer : Read the net external wrenches using a vector collection matching the change done in `whole-body-estimators` 0.9.0 in https://github.com/robotology/whole-body-estimators/pull/155 (https://github.com/robotology/idyntree-yarp-tools/pull/38).

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
