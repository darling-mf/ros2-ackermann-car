# Documentation Index

This repository keeps the public documentation deliberately small: an
architecture page, a validation page, a troubleshooting page, and one README
per package. Deeper personal engineering notes are not part of the public tree.

## Recommended Reading Order

1. [README](../README.md) — project scope, build, run, measured results.
2. [ARCHITECTURE.md](ARCHITECTURE.md) — layers, packages, interfaces, data flow,
   and the exact files to edit for each subsystem.
3. [VALIDATION.md](VALIDATION.md) — test method, measured numbers, and the
   scenarios that still fail.
4. [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — reusable diagnosis paths for
   Nav2, TF, costmap, and AMCL.

## Package Documentation

| Package / area | Document | Contents |
|---|---|---|
| Chassis controller | [src/r5_steering_controller/README.md](../src/r5_steering_controller/README.md) | Ackermann controller, limits, parameters, tests |
| Bringup / Nav2 | [src/car_bringup/README.md](../src/car_bringup/README.md) | Launch modes, config files, maps, scripts |
| Robot model | [src/car_description/README.md](../src/car_description/README.md) | URDF/xacro, frames, Gazebo world, RViz |
| Navigation behavior | [src/car_navigation/README.md](../src/car_navigation/README.md) | Ackermann behavior trees and recovery policy |
| Serial bridge | [src/r5_serial/README.md](../src/r5_serial/README.md) | X-Protocol, virtual STM32, host tests |
| STM32 modules | [firmware/openctr_b20/README.md](../firmware/openctr_b20/README.md) | C modules, integration points, host tests |
| Upstream patch | [patches/README.md](../patches/README.md) | `ros2_controllers` patch and apply steps |

## Documentation Rules

- Every statement about a subsystem should point to the file that implements it.
- Test results must state the scenario, parameters, and evidence.
- Known failures stay in [VALIDATION.md](VALIDATION.md); do not hide them in a
  change log.
- Do not publish vendor firmware, manuals, personal notes, or interview
  material.
