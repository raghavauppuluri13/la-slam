# la-slam

**Doesn't work yet**

Monocular SLAM in CPP started on a drive to LA with some friends.

For Monocular VO:
- Uses [David Nister's 5 Point algorithm](https://ieeexplore.ieee.org/document/1288525) on correspondences to generate relative camera transformation hypotheses
- Uses [David Nister's Preemptive RANSAC algorithm](https://ieeexplore.ieee.org/document/1238341) to select the correct hypothesis

### Dependencies
- OpenCV (only basic image processing), most geometry is done from scratch 
  - Install OpenCV from https://opencv.org/releases/
- Eigen
  - `sudo apt install libeigen3-dev`
- Pangolin (visualization)
  - https://github.com/stevenlovegrove/Pangolin

### Usage 

1. Install dependencies

```
# pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout tags/v0.6

# GNinja builds faster
cmake -B build -GNinja
cmake --build build
sudo cmake --install build
```

2. Unzip contents of `sandbox` undistorted dataset from https://www.eth3d.net/datasets into `data/undistorted` folder (make sure the dataset structure is the same as in the docs)
3. Add build dir: `cmake -B build -GNinja` 
4. Build: `cmake --build build`
5. To run VO on the `sandbox` dataset for example: 

```
ROOT_DIR=$(readlink -f .) DATASET_NAME="sandbox" ./build/src/main
```
6. Run tests: `./build/test/run_tests`

### Benchmarking/Results

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
