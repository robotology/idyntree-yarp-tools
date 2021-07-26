# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

## [0.0.2] - 2021-07-09

### Added
- Added `IDYNTREE_YARP_TOOLS_USES_QT_CHARTS` option to disable just the compilation of `idyntree-plotter` that depends on Qt5 Charts.

### Fixed
- Fixed the conversion from ``iDynTree`` to ``yarp`` pixels after https://github.com/robotology/idyntree/pull/903. Removed some deprecation warnings (https://github.com/robotology/idyntree-yarp-tools/pull/16)

## [0.0.1] - 2021-07-08

### Added
- First release of the repo. Contains the new tool `idyntree-yarp-visualizer` and the tools `urdf2dh`, `yarprobotstatepublisher` `idyntree-plotter` and `idyntree-sole-gui` migrated from iDynTree.
