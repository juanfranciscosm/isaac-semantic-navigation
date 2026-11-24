# Changelog

## Research snapshot — 2026-11-24

### Semantic navigation
- Extended semantic component processing.
- Added semantic instance selection and target reset.
- Updated navigator goal-state handling.
- Expanded global rolling costmap from 24 x 24 m to 40 x 40 m.
- Updated MPPI controller configuration.

### Experimental evaluation
- Added semantic_nav_evaluation ROS 2 package.
- Implemented automated experimental evaluation.
- Added ground truth and initial robot positions.
- Added benchmark prompts and mapping positions.
- Preserved illumination and occlusion results.

### VLMaps integration
- Added triggered RGB-D dataset acquisition.
- Implemented reproducible mapping pose runner.
- Automated scenario-based data acquisition.
- Added sample-count validation.
- Integrated semantic map construction.
- Implemented semantic target localization and goal generation.
- Added dataset inventory for 60 planned scenarios.

### Simulation
- Added warehouse scenarios for ten object categories.
- Preserved experimental USD scene snapshots.

### Data management
- Organized experimental results and metadata.
- Separated large datasets and semantic maps from Git.

## Previous development

- Isaac Sim and ROS 2 integration.
- CLIPSeg semantic segmentation.
- Three-dimensional object localization.
- Autonomous Jackal navigation with Nav2.
- Installation and architecture documentation.
