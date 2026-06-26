# GraphDECO GPU Smoke Test

This procedure is the first real 3D Gaussian Splatting handoff test for the
current lidar/camera export. It uses the official GraphDECO/Inria trainer, but
it should be treated as a loader/training smoke test, not a reconstruction
quality benchmark.

## Why this is a notebook path

The local Windows/WSL workstation currently does not expose `nvidia-smi`.
GraphDECO's optimizer uses PyTorch and CUDA extensions, so the practical next
step is to run the test in a CUDA GPU notebook or another CUDA-capable machine.

## 1. Package the current export on Windows

From the project root:

```powershell
cd "C:\Users\Neel\Documents\Fuse Lidar and Camera"

& "$env:LOCALAPPDATA\Programs\Python\Python313\python.exe" `
  reconstruction\package_graphdeco_dataset.py `
  data\exports\colmap\20260625T214456Z-steady-undistorted `
  --output data\exports\gaussian-splatting\20260625T214456Z-steady-undistorted-graphdeco.zip
```

Expected output:

```text
Wrote ...20260625T214456Z-steady-undistorted-graphdeco.zip
Files: 19
Images: 9 missing=0
Sparse points: 327 track_refs=0
```

The zero track references warning is expected for this first dataset. It means
the sparse points came from our lidar/camera diagnostic pipeline, not true
COLMAP feature tracks.

## 2. Open the notebook

Open:

```text
notebooks/GraphDECO_3DGS_Smoke_Test.ipynb
```

Run it in Google Colab or another notebook environment with a CUDA GPU. In
Colab, set:

```text
Runtime -> Change runtime type -> GPU
```

Then upload:

```text
data\exports\gaussian-splatting\20260625T214456Z-steady-undistorted-graphdeco.zip
```

## 3. Success criteria

The smoke test passes if:

- `nvidia-smi` reports a GPU;
- GraphDECO clones and compiles its CUDA submodules;
- the dataset unpack check finds `images/` and `sparse/0`;
- `train.py` starts and reaches iteration 300.

Do not judge visual quality yet. With only 9 views and no true COLMAP feature
tracks, the useful result is a loader/training compatibility answer.

## 4. Common failure meanings

- CUDA extension build failure: notebook GPU/PyTorch/CUDA toolchain mismatch.
- Missing `cameras.bin`, `images.bin`, or `points3D.bin`: package the dataset
  again after running the local GraphDECO checker.
- Loader says camera model unsupported: the export is not using `PINHOLE` or
  `SIMPLE_PINHOLE`.
- Training runs but splat is ugly: expected for this first dataset; next capture
  needs more views, more texture, and better camera pose support.

## References

- GraphDECO/Inria Gaussian Splatting:
  <https://github.com/graphdeco-inria/gaussian-splatting>
- COLMAP model format:
  <https://colmap.github.io/format.html>
