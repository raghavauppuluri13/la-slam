# la-slam

**Doesn't work and not clean yet**

Monocular SLAM in CPP started on a drive to LA with some friends.

### Dependencies
- OpenCV (only basic image processing), most geometry is done from scratch 
  - Install OpenCV from https://opencv.org/releases/
- Eigen
  - `sudo apt install libeigen3-dev`
- Pangolin (visualization)
  - https://github.com/stevenlovegrove/Pangolin

### Usage 

1. Install dependencies
2. Unzip contents of `sandbox` undistorted dataset from https://www.eth3d.net/datasets into `data/undistorted` folder
3. Add build dir: `cmake --B build` 
4. Build: `cmake --build build`
5. Run VO: `./build/src/vo`

### Benchmarking/Results

- VO runs at ~150ms
- Not accurate right now

### To-do
- [x] Dataset loading, feature generating
- [x] Feature matching
- [x] VO from epipolar geometry
- [ ] Bundle adjustment least squares graph optimization using g2o
- [ ] Loop closure with bag-of-words

### Useful Resources used during development 
- https://github.com/gaoxiang12/slambook-en
- Multiple View Geometry in Computer Vision (Hartley et al): https://www.robots.ox.ac.uk/~vgg/hzbook/
