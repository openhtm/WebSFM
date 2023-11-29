# WebSFM

![avatar](https://badgen.net/badge/Linux/C++17/green)


## Introduction

WebSFM is a browser-based SfM system through [WebRTC](https://webrtc.org/). 

The WebRTC service is build based on [aiortc](https://github.com/aiortc/aiortc) to provide real-time data transmission between browser and server, and the common web service is build based on [aiohttp](https://github.com/aio-libs/aiohttp).

The core SfM system is based on monocular ORB-SLAM2, modified with [ceres-solver](https://github.com/ceres-solver/ceres-solver) and [fbow](https://github.com/rmsalinas/fbow) to increase processing speed. Check [ceres_mono_orb_slam2](https://github.com/b51/ceres_mono_orb_slam2) and [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) for fundamental information . [Pybind](https://github.com/pybind/pybind11) is integrated to construct the whole pipeline. [Osmap](https://github.com/AlejandroSilvestri/osmap) is also integrated for map serialization and relocalization.

The WebSFM system construct a pipeline from feature-based indirect oneline SLAM (ORB-SLAM2) and multi-view stereo reconstruction (OpenMVS). Through the [Interface](https://github.com/cdcseacave/openMVS/blob/master/libs/MVS/Interface.h) of OpenMVS, it is possible to generate dense pointcloud and textured mesh from original sparse data. Check [OpenMVS](https://github.com/cdcseacave/openMVS) for more information about stereo reconstruction.

- **Online SFM** for real-time sparse mapping and localization
- **Offline MVS** for dense pointcloud and textured mesh generation
- **WebRTC** for browser-based fast data(media) transmission


## Build

Be sure you have installed `OpenMVS` and `OpenCV 3`.

Modify `SFM/build_3rd.sh` and `SFM/CMakeLists.txt` if necessary (such that you have not installed ceres-solver or Sophus globally) .
  
```bash
# clone the repo
git clone --recursive https://github.com/Jianxff/WebSFM.git

# build 3rd party
cd SFM
./build_3rd.sh

# build SFM system
mkdir build
cd build
cmake ..
make -j4
```


## Run

**!Note:** This is the `backend` service, a frontend WebRTC application is needed to start running pipeline.

Check `handler/session.py` for api information.

```bash
# install required python libraries
pip install numpy aiortc aiohttp

# start backend at http://0.0.0.0:8081
python main.py
```

After end a session, you can find the following results:
- **Osmap File** at `static/usr/test/map.yaml`
- **Dense PointCloud** at `static/usr/test/scene/dense.ply`
- **Plain Mesh** at `static/usr/test/scene/mesh.ply`
- **Textured Mesh** at `static/usr/test/scene/texture.ply` with **Texture File** at `static/usr/test/scene/texture.png`

The `.mvs` files (OpenMVS format) can be found in `static/usr/test/scene/*.mvs`, and the temp files while reconstructing (log files and depth maps) can be found in `static/usr/test/scene/tmp/`.


## Preview

*Comming Soon*