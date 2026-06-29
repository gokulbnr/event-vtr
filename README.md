# Event-Based Visual Teach-and-Repeat via Fast Fourier-Domain Cross-Correlation

[![Pixi](https://img.shields.io/badge/pixi-supported-brightgreen?style=for-the-badge&logo=pixi)](https://pixi.sh)
[![RoboStack](https://img.shields.io/badge/robostack-supported-brightgreen?style=for-the-badge&logo=ros)](https://robostack.github.io)

[![Dataset](https://img.shields.io/badge/Dataset-Download-green)](https://huggingface.co/datasets/gokulbnr/QUT-Event-VTR-Dataset) [![Preprint](https://img.shields.io/badge/Preprint-Read-orange)](https://arxiv.org/abs/2509.17287)

Welcome to the official repository for the paper [**Event-Based Visual Teach-and-Repeat via Fast Fourier-Domain Cross-Correlation**](https://arxiv.org/abs/2509.17287), to be presented at the 2026 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2026). 

This work demonstrates teach-and-repeat navigation for a mobile robot using an event-based camera as the sole exteroceptive sensor.

<table border="0">
  <tr>
    <!-- Left Column: Main System Diagram -->
    <td width="50%" valign="top">
      <img src="./assets/system_diagram.png" alt="System Overview" width="100%">
    </td>
    <!-- Right Column: Hardware, GIF, and Tracks Grid -->
    <td width="50%" valign="top">
      <!-- <img src="./assets/hardware_robot.png" alt="Hardware Setup" width="33%"> -->
      <img src="./assets/output.gif" alt="Event VT&R Demo" width="100%">
      <br><br>
      <img src="./assets/tracks_performance.png" alt="Performance Plots" width="90%">
    </td>
  </tr>
</table>

---

## 📊 Dataset

Recordings from our reported real-world demonstrations are publicly available in the Hugging Face dataset:
🔗 [**QUT-Event-VTR-Dataset**](https://huggingface.co/datasets/gokulbnr/QUT-Event-VTR-Dataset)

---

## ⚙️ Prerequisites & Dependencies

### 1. Setup via Pixi
We utilize [Pixi](https://pixi.prefix.dev/latest/) environment structures paired with [RoboStack](https://robostack.github.io/index.html):

```bash
# Clone this repository
git clone https://github.com/gokulbnr/event-vtr.git
cd event-vtr

# Install dependencies and build workspace natively
pixi install
```

### 2. Hardware
* **Prophesee EVK4 HD** (or any compatible Metavision event camera).

### 3. Software Dependencies
This project uses **OpenEB** (the open-source core of the Metavision SDK). To support our pipeline, we utilize a custom fork that implements overlapping event-count window binning:
* **Custom OpenEB Fork:** [gokulbnr/openeb (patch-1)](https://github.com/gokulbnr/openeb/tree/patch-1)

```bash
# Clone the repository
git clone git@github.com:gokulbnr/openeb.git
cd openeb
git checkout patch-1
mkdir build && cd build

# 2. Configure the build directory
ccmake ..
```
⚠️ Crucial Configuration Step:
Inside the ccmake TUI, navigate to CMAKE_INSTALL_PREFIX and update it to point directly inside your project's local Pixi environment:

```bash
CMAKE_INSTALL_PREFIX = /path/to/your/event-vtr/.pixi/envs/default
```
This ensures that OpenEB's custom C++ binaries and Python bindings are isolated and natively accessible to the Pixi environment framework.

```bash
# 3. Build and install into the Pixi environment prefix
make -j$(nproc)
sudo make install
```

### 4. ROS Workspace Setup (`cam_ws`)

The `cam_ws` workspace contains the ROS packages required to interface with your sensors and mobile platform.
Navigate to your repository root and create the workspace structure:
```bash
# From your event-vtr repository root:
cd cam_ws
catkin build
```
---

## 💻 Usage & Running the Code

The workspace workflow is split into initializing the live sensor stream, building your topometric mapping path (Teach Phase), and running the autonomous tracking pipeline (Repeat Phase).

### 1. Initialize the Environment & ROS Workspace
Before running any scripts, ensure your environment is activated. 

### 📹 2. Start the Camera Publisher
Connect your Prophesee EVK4 HD camera and run the event camera publisher node using:
```bash
pixi run cam-count-sliding
```

### 🗺️ 3. Teach Phase (Building the Topometric Map)

### 🤖 4. Repeat Phase (Real-Time Fourier Localization)

---

## Cite us at
```
@article{nair2025event,
  title={Event-Based Visual Teach-and-Repeat via Fast Fourier-Domain Cross-Correlation},
  author={Nair, Gokul B and Fontan, Alejandro and Milford, Michael and Fischer, Tobias},
  journal={arXiv preprint arXiv:2509.17287},
  year={2025}
}
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
