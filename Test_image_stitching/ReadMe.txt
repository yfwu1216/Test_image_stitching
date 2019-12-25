This project was created to test opencv image stitching module with CUDA.
You should use CMake to compile the opencv all module with CUDA and TBB.

VS version: VS2015 recommended.
CUDA version: CUDA8.0 or later.
Opencv version: The default podfile will install openCV v3.4.1, with a hotfix for arm64 compataibility.
TBB version: 2018_20180822oss or later, depend on your CPU.

test modules:
1.image reading, 3-4x faster depend on your CPU.
2.distortion correction, 4x faster.
3.orb and surf descriptor computing, 3-5x faster.
4.descriptor matching, 5-10x faster.
