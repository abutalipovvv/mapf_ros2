# fleet_experiments

This package provides a single entry point for thesis experiments across three modes:

- `discrete_cbs`: centralized discrete MAPF on the landmark grid
- `nav2_only`: decentralized baseline logging format
- `hybrid`: FleetManager + CBS + Nav2 logging format

Raw logs are written to `results/raw/`.
Aggregated CSV summaries are written to `results/summary/`.
