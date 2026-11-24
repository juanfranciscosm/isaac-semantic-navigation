# VLMaps Integration

## Customized repository

https://github.com/juanfranciscosm/vlmaps

Branch: jackal-vlmaps-benchmark

Pinned commit:
bd5bb23ee0936af29155e08bf19764eae83509af

Original project:
https://github.com/vlmaps/vlmaps

## Workspace structure

    ~/indoor/
    ~/vlmaps_ws/vlmaps/
    ~/vlmaps_data_benchmark/
    ~/vlmaps_build_root/

## Dataset acquisition

    cd ~/indoor
    ./tools/capture_vlmaps_scenario.sh boxes I0

Each scenario requires 24 RGB-D samples with poses and timestamps.

## Map construction

    cd ~/vlmaps_ws/vlmaps
    ./tools/build_vlmaps_scene_map.sh boxes_I0

Expected output:

    ~/vlmaps_data_benchmark/boxes_I0/vlmap_cam/vlmaps_cam.h5df

## Semantic target localization

The localization algorithm performs:

1. Text-based semantic voxel identification.
2. Conversion to metric coordinates.
3. Connected-component clustering.
4. Semantic component merging.
5. Nearest-instance selection.
6. Navigation goal generation.
7. YAML export.

Default merging distance: 2.0 m.
Default goal standoff: 0.80 m.

## Example

    cd ~/vlmaps_ws/vlmaps
    conda activate vlmaps
    export PYTHONPATH="$PWD:$PYTHONPATH"
    python tools/localize_vlmaps_target.py \
      --scenario boxes_I0 \
      --query "box" \
      --positions-file ~/indoor/evaluation/config/boxes_positions.yaml \
      --output /tmp/boxes_I0_example.yaml

## Limitations

Review absolute paths in the configuration before running
the system on another computer.

The complete comparative benchmark remains unfinished.
