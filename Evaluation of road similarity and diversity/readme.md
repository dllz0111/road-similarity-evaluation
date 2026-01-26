\# Road Similarity \& Diversity Evaluation



Lightweight C implementation using \*\*discrete Fréchet distance\*\* to measure road segment similarity and evaluate map diversity based on OpenStreetMap GeoJSON data.



\- `diversity`: computes statistical similarity/diversity metrics

\- `rank`: interactively visualizes and ranks most similar road segments (requires raylib)



\## Features

\- Discrete Fréchet distance for road/trajectory comparison

\- Supports GeoJSON input from OpenStreetMap

\- No external math libraries required (pure C)

\- Optional interactive visualization with raylib



\## Requirements

\- CMake ≥ 3.20

\- C compiler with C17 support (gcc, clang, MSVC)

\- raylib (only required for the `rank` tool)



\## Dependencies Installation – raylib



\### Linux (Ubuntu/Debian)

```bash

sudo apt update

sudo apt install libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev libxcursor-dev libxinerama-dev

git clone https://github.com/raysan5/raylib.git

cd raylib/src

make PLATFORM=PLATFORM\_DESKTOP

sudo make install

macOS (Homebrew)

Bashbrew install raylib

Windows (vcpkg recommended)

Bashvcpkg install raylib

Then add to cmake command:

text-DCMAKE\_TOOLCHAIN\_FILE=\[path\_to\_vcpkg]/scripts/buildsystems/vcpkg.cmake

Or build from source: https://github.com/raysan5/raylib#working-on-windows

Project Structure

text.

├── src/

│   ├── rank.c

│   └── diversity.c

├── lib/

│   ├── cJSON.c

│   └── cJSON.h

├── assets/

│   ├── data/

│   │   ├── map.geojson

│   │   └── target\_road.geojson

│   └── font/

│       └── PretendardJP-Bold.otf

├── CMakeLists.txt

└── README.md

Data Preparation



Download area of interest from OpenStreetMap

Export as GeoJSON

Rename the map file to map.geojson and place in assets/data/

Prepare your target road:

Should be a GeoJSON FeatureCollection containing one LineString feature

Recommended name: target\_road.geojson (no spaces)

Place in assets/data/





Note: Coordinates should be in WGS84 (lon, lat). The current implementation uses raw geographic coordinates; for high accuracy consider projecting to a local metric CRS.

Build

Bashmkdir build \&\& cd build

cmake ..

cmake --build . --config Release

Run

Bash# Compute diversity metrics (average/median/min/max distance, etc.)

./diversity



\# Interactive visualization \& ranking (needs raylib)

./rank

Output



diversity: prints statistical results of Fréchet distances between target road and all road segments in the map

(lower average distance = roads more similar to target → lower diversity)

rank: opens window showing top similar road segments with interactive navigation

