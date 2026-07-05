# Cool Earth Project

Team: Hagar, Liran, Ilan, Joana, Orel, Eric, Alex

## Overview

This repository contains the software workspace for the Cool Earth demonstrator.

At a high level, the project combines:

- platform state-machine logic and hardware-facing scripts at repository root
- optical navigation processing in the Cool-Earth-NAV sub-module
- the GIANT library (included under Cool-Earth-NAV/giant) as the core vision/opnav backend

This README is intentionally high level. Detailed navigation pipeline behavior and configuration are documented in Cool-Earth-NAV/README.md.

## Repository Layout

- StateMachine.py: main mission-state orchestration script
- ReadIMU.py: IMU read utility script
- ControCheck,.py: additional root-level utility script
- Cool-Earth-NAV/: navigation processing module and data
	- config.py: runtime configuration
	- process_star_image.py: single-image optical processing entry point
	- data/: scenarios and captured frames
	- README.md: detailed navigation module documentation

## Quick Start

1. Create the shared environment (root plus OpNav dependencies) with one command.
2. Activate the environment.
3. Run the root state machine or run the navigation processor directly for focused tests.

Example commands:

```bash
conda env create -f environment.yml
conda activate coolearth-rt

# Full flow
python StateMachine.py

# Navigation-only test
cd Cool-Earth-NAV
python process_star_image.py
```

## Documentation Guide

- Start here for repository context and entry points.
- Use Cool-Earth-NAV/README.md for navigation pipeline setup, configuration, runtime behavior, and troubleshooting.

## Notes

- Keep generated or environment-specific artifacts out of version control.
- Prefer updating module-level README files when changing module-specific behavior.
- The shared dependency file is environment.yml at repository root.
