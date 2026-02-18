This program is a simple tool to quantify road similarity and diversity using Discrete Normalized Fréchet Distance, mainly intended for testing autonomous driving systems.  

This program has a C head file "NDPF.h", which includes some functions to calculate Fréchet Distance.  
Copy and paste into your project to use it.

Some core functions:

(double) rotate_theta(const group *a, const group *b)
    -> compute the optimal rotation angle that aligns road b to road a.

(long double) frecdis(const group *a, const group *b)
    -> compute the Discrete Fréchet Distance between two polylines.

(long double) normalized_frecdis(const group *a, const group *b)
    -> compute the Fréchet Distance after arc-length normalization 
       (the longer road is truncated to match the shorter one).

(long double) NDPF(group *a, group *b)
    -> full Normalized Direction-Preserving Fréchet Distance
       (translation + rotation + optional reverse matching).

(long double) sliding_NDPF(group *A, group *B)
    -> compute the minimal NDPF under sliding window matching.

Raylib is not in the folder, but it should be installed automatically.

How to use：

Rank：
1. Download map from OpenStreetMap,extract as geojson file.
2. Name it as "similarity_map.geojson", and put it at "assets/data"
3. Rename target road as "target road.geojson", and put it at "assets/data"

It will print all the roads' name, type and Frechet Distance with the target road in increasing order.  
It will also create a window to visualize.  
Change the macro VISUALIZE to 0 to shut down visualization.  

This code has length and nodecount filter to filter too short and straight roads.  
You can change the macros to modify the parameters in "NDPF.h".  

Diversity:
1. Download map from OpenStreetMap,extract as geojson file.
2. Name it as "diversity_map.geojson", and put it at "assests/data"

It will print the diversity (average Normalized Discrete Procrustes Distance)


If your map includes too many roads, please modify the macro MAX_FEATURE_COUNT.  
The default is 1000.

  
Directory Structure:  
.  
├── src/  
│   ├── rank.c  
│   └── diversity.c  
├── lib/
│   ├── cJSON.c  
│   └── cJSON.h  
├── assets/  
│   └── data/  
│       ├── similarity_map.geojson  
│       ├── target_road.geojson  
│       └── diversity_map.geojson  
├── CMakeLists.txt  
└── README.md  
