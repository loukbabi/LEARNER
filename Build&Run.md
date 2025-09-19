## Quick Features

- Real‑time monocular and inertial SLAM  
- SuperPoint + ORB front‑end  
- Loop‑closure & relocalization  
- ROS 2 wrappers  
- Tested on EuRoC, TUM VI and OpenLoris datasets  

---

## Tested Setup

|            |                         |
|------------|-------------------------|
| **OS**     | Ubuntu 20.04 / 22.04    |
| **CPU**    | Intel Core i9‑11900K    |
| **GPU**    | NVIDIA GeForce RTX 3080 |
| **CUDA**   | 11.8                    |

*(Other hardware / software versions may also work.)*

### Dependencies

| Library         | ≥ Version | Notes / Install tip                                           |
|-----------------|-----------|---------------------------------------------------------------|
| Pangolin        | –         | Viewer & UI                                                   |
| OpenCV          | 4.2       | `sudo apt install libopencv-dev`                              |
| Eigen           | 3.1       | Header‑only                                                   |
| ONNXRuntime     | 1.16.3    | Edit `CMakeLists.txt:63` with your ONNX path                  |
| ROS 2 (optional)| Humble / Iron | Live camera & bagfile demos                              |

---

## Build

```bash
git clone https://github.com/anastaga/semantic_sporbslam3.git
cd semantic_sporbslam3
mkdir build && cd build
cmake ..
make -j$(nproc)
```

---

## Quick Start (Monocular / EuRoC V1_01)

```bash
cd ~/Workspaces/PhD_ws/src/SLAM_Methods/semantic_sporbslam3
./Examples/Monocular/mono_euroc     Vocabulary/SuperPointVoc.dbow3     Examples/Monocular/OpenLoris.yaml     ~/Workspaces/Datasets/office1-1/color/     ~/Workspaces/Datasets/office1-1/color_clean.txt
```

Arguments: **vocabulary file**, **camera config**, **image folder**, **timestamp file**.

More launch scripts live under `Examples/*`.

---

## Acknowledgments

This project stands on the shoulders of giants. Huge thanks to:

1. **Rover‑SLAM**  
2. ORB‑SLAM3  
3. AIRVO  
4. SP‑Loop  
5. ORB_SLAM3_detailed_comments  
6. SuperPoint_SLAM  

---