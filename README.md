This program uses Discrete Normalized Frechet Distance to quantify road similarity and diversity for testing autonomous driving systems.

Note: Raylib is not installed by default. Please install it at "lib/" folder manually if you want to visualize the result.

How to use：

Rank：
1. Download map from OpenStreetMap,extract as geojson file.
2. Name it as "similarity_map.geojson", and put it at "assets/data"
3. Rename target road as "target road.geojson", and put it at "assets/data"

It will print all the roads' name, type and Frechet Distance with the target road in increasing order.
It will also create a window to visualize. (You need to install raylib manually)
Change the macro VISUALIZE to 0 to shut down visualization.

This code has length and nodecount filter to filter too short and straight roads that are not suitable for testing ADSs.
You can change the macros to modify the parameters.

Diversity:
1. Download map from OpenStreetMap,extract as geojson file.
2. Name it as "diversity_map.geojson", and put it at "assests/data"

It will print the diversity (average Normalized Discrete Procrustes Distance)





Directory Structure:  
.  
├── src/  
│   ├── rank.c  
│   └── diversity.c  
├── lib/ # please install raylib folder here  
├── cJSON.c  
├── cJSON.h  
├── assets/  
│   └── data/  
│       ├── similarity_map.geojson  
│       ├── target_road.geojson  
│       └── diversity_map.geojson  
├── CMakeLists.txt  
└── README.md  
