# Event-Based Visual Teach-and-Repeat via Fast Fourier-Domain Cross-Correlation

[![Pixi](https://img.shields.io/badge/pixi-supported-brightgreen?style=for-the-badge&logo=pixi)](https://pixi.sh)
[![RoboStack](https://img.shields.io/badge/robostack-supported-brightgreen?style=for-the-badge&logo=ros)](https://robostack.github.io)

Welcome to the official repository for the paper [**Event-Based Visual Teach-and-Repeat via Fast Fourier-Domain Cross-Correlation**](https://arxiv.org/abs/2509.17287), to be presented at the 2026 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2026). 

This work demonstrates teach-and-repeat navigation for a mobile robot using an event-based camera as the sole exteroceptive sensor.

---

## 📊 Dataset

Recordings from our reported real-world demonstrations are publicly available in the Hugging Face dataset:
🔗 [**QUT-Event-VTR-Dataset**](https://huggingface.co/datasets/gokulbnr/QUT-Event-VTR-Dataset)

---

## Dependency: OpenEB

The event camera device used for this project is a [Prophesee EVK4 HD](https://docs.prophesee.ai/stable/hw/evk/evk4.html). We use OpenEB, the open-source project associated with the Metavision SDK, to interface with the event-camera device. We use a version of the project with an additional feature of binning events from the camera by overlapping windows of event-count, available here: https://github.com/gokulbnr/openeb/tree/patch-1. 

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
