# GP-ACO

GP-ACO combines genetic programming with ant colony optimization for dynamic
ore-flow blending in open-pit truck dispatching. Offline training constructs a
behaviorally diverse archive of dispatching rules using quality-gated semantic
anchors and CVT-MAP-Elites. Online search uses a frozen rule library to guide
ant construction and improve dispatching plans as operating conditions change.

This repository contains one workflow: **joint offline training on the three
D1 training sets, followed by online testing on all 12 configurations**. The
software can be built without the experimental datasets, a GPU, Python, or
LibTorch.

## Data availability

The GP-ACO source code is publicly available. The experimental training and
test datasets are not authorized for public release before the associated
manuscript is accepted for publication. The datasets are therefore excluded
from this repository. Both the quick check and the full experimental workflow
require separately obtained, authorized local copies of the data.

## Build

Requirements: CMake 3.16 or later and a C++17 compiler, such as Visual Studio
2022, GCC 9 or later, or Clang 10 or later. Run the following commands from this
repository's root directory.

```bash
git clone https://github.com/Townhome/GP-ACO.git
cd GP-ACO
```

Windows, with Visual Studio 2022 and its C++ tools installed:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --parallel
```

Linux:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

## Run

If you already have authorized access to the datasets, place them outside the
repository in a local data directory that contains a `graph/` subdirectory.
Pass the parent of `graph/` to `--data`. Start with a small workload:

```powershell
# Windows
.\build\Release\gpaco.exe --data "C:\path\to\authorized-data" --quick --threads 20 --output results-quick
```

```bash
# Linux
./build/gpaco --data /path/to/authorized-data --quick --threads 20 --output results-quick
```

Then run the full protocol by omitting `--quick`:

```powershell
.\build\Release\gpaco.exe --data "C:\path\to\authorized-data" --threads 20 --output results
```

```bash
./build/gpaco --data /path/to/authorized-data --threads 20 --output results
```

The full run trains one archive on 1,600 instances per mine and transfers it to
C1–C3 × D1–D4. Each configuration contains 200 test instances, with 40 instances
per E1–E5 group; each instance passes through five consecutive environments.
D1–D4 contain 3, 5, 7 and 9 shovels. C1 and C2 use two trucks per shovel, and C3
uses three. Their numbers of unloading sites are 2, 3 and 4, respectively.

Default offline settings are a population of 100, an initial pool of 2,000,
102,000 rule evaluations, 20 tasks, tree initialization depth 6, five anchors,
2,000 reference states and 200 CVT cells. Online search uses five selected
rules, 40 ants, a budget of 1,000 objective evaluations per environment,
α = 0.5, β = 4, ρ = 0.3, temperature 0.1 and up to 100 local trials after an
improving ant generation. Repair and local evaluations count toward the same
budget. Previously executed route prefixes are preserved at transitions.

`--quick` reduces offline training to 96 rules and 10 instances per mine, then
tests one instance from each E1–E5 group in every configuration with 200
evaluations per environment. It checks the workflow and is not a substitute
for the full experiment.

Optional arguments are `--threads` (0 uses available hardware threads),
`--seed` (2026), `--data` (`data`) and `--output` (`results`). Use a separate
output directory for each setting. Repeating an identical command reuses a
completed, verified archive and reruns the online tests. An interrupted offline
stage restarts from its seed. Random streams are assigned per logical search;
changing worker count does not change the intended sampling order. Numerical
results can differ across compiler and standard-library implementations.

## Outputs and layout

- `offline/model.json`: frozen archive, trees, anchors, centroids and training
  normalization; `offline/history.csv`: training progress.
- `C*_D*.csv`: per-instance, per-environment RMSE, elapsed time and budget checks.
- `C*_D*_routes.json`: the corresponding dispatching routes.
- `summary.csv`: mean RMSE across instances and environments, and mean elapsed
  time per complete instance, calculated by summing its five environment times.
- `run.json`: settings, input fingerprints and completion status.

```text
src/
  main.cpp       # single training-and-testing entry point
  core/          # shared types, random streams, CPU executor and file utilities
  problem/       # mine simulator, ore-flow objective and local route moves
  gp/            # trees, semantic archive, reference states and offline training
  aco/           # GP-guided online search and dynamic rescheduling
third_party/     # bundled JSON header
```

The software is licensed under [GPLv3](LICENSE). The simulator and GP trees are
adapted from SIL-TS, Copyright (C) 2025 HaoTang. The uniform generator derives
from the GPLv2-or-later [OFEC](https://github.com/Changhe160/OFEC) adaptation of
[NewRan](https://www.robertnz.net/nr02doc.htm) by Changhe Li and Hai Xia; this
distribution uses GPLv3. The bundled JSON header is Copyright (C) 2013–2022
Niels Lohmann and retains its MIT license.
