# Experimental Data Management

## Version-controlled files

- Source code.
- Experimental configurations.
- Ground truth coordinates.
- USD simulation scenarios.
- Benchmark CSV results.
- Experimental metadata.
- Semantic target configurations.
- Dataset acquisition inventory.

## External datasets

Original location:

    ~/vlmaps_data_benchmark/

Local backup:

    ~/project_backups/vlmaps_data_benchmark/

Total size: approximately 14 GiB.

Backup verified using rsync checksums on 2026-11-24.

Both copies are stored on the same workstation disk.

## Dataset structure

    camera_info.txt
    rgb/
    depth/
    poses.txt
    timestamps.txt
    vlmap_cam/vlmaps_cam.h5df

## Dataset inventory

    evaluation/manifests/vlmaps_dataset_inventory.csv

The inventory records sample counts and map availability.

## Raw experimental evidence

Screenshots, point clouds, depth arrays and segmentation
images remain in the local workspace and backup archive.
