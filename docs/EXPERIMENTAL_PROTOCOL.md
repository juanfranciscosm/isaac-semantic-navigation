# Experimental Protocol

## Research objective

Evaluate CLIPSeg and VLMaps for semantic object localization
and autonomous indoor navigation using a simulated Jackal robot.

## Simulation environment

- NVIDIA Isaac Sim 5.1.0.
- ROS 2 Jazzy.
- Clearpath Jackal.
- RGB-D camera.
- Nav2.
- Indoor warehouse environment.

## Semantic approaches

CLIPSeg:
Online text-guided segmentation, depth projection,
3D localization and navigation goal generation.

VLMaps:
Offline RGB-D mapping, semantic voxel retrieval,
instance selection and navigation goal generation.

## Object categories

- Boxes
- Cargo vehicle
- Chair
- Computer
- Cylindrical container
- Door
- Drinks machine
- Electrical panel
- Extinguisher
- Medical bag

## Experimental conditions

Illumination: I0, I25, I50.
Occlusion: O20, O50, O80.

Total planned combinations: 60.

## Evaluation

- Semantic object localization accuracy.
- Navigation success.
- Navigation goal accuracy.
- Execution time.
- Sensitivity to illumination and occlusion.
- Repeatability across initial robot positions.

## Experimental files

Configurations: evaluation/config/
Results: evaluation/benchmark_clipseg_results/
Automated runs: evaluation/automated_runs/
Preliminary episodes: evaluation/episodes/
Dataset inventory: evaluation/manifests/

## Current status

CLIPSeg CSV records exist for the 60 planned combinations.

VLMaps acquisition remains partially completed.

The computer_O80 acquisition was interrupted after five samples.

Map availability does not establish map validity or navigation success.


