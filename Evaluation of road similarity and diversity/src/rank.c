#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include "cJSON.h"
#include "raylib.h"
#include "NDPF.h"

#define VISUALIZE 1

Vector2 ToV2(Point p) {
    return (Vector2){ (float)p.x, (float)p.y };
}


int main(void) {
    double start_time, end_time;
    start_time = (double)clock();
    system("chcp 65001");
    long double lPI = 3.14159265358979323846;
    long double radius = 6371393.0;

    group *A = malloc(sizeof(group));
    A->coordinates = malloc(sizeof(Point) * MAX_NODE_COUNT);
    A->n = 0;

    group *B = malloc(sizeof(group) * MAX_FEATURE_COUNT);
    for (int i = 0; i < MAX_FEATURE_COUNT; i++) {
        B[i].coordinates = malloc(sizeof(Point) * MAX_NODE_COUNT);
        B[i].n = 0;
    }

    FILE *fp1 = fopen(TARGET_ROAD_FILEADDRESS, "r");
    if (!fp1) { printf("Error opening target road\n"); return 1; }
    fseek(fp1, 0, SEEK_END);
    long filesize1 = ftell(fp1);
    rewind(fp1);
    char *json1 = malloc(filesize1 + 1);
    fread(json1, 1, filesize1, fp1);
    json1[filesize1] = '\0';
    fclose(fp1);

    cJSON *root1 = cJSON_Parse(json1);
    free(json1);
    if (!root1) return 1;

    cJSON *feature1_arr = cJSON_GetObjectItem(root1, "features");
    cJSON *first_feature = cJSON_GetArrayItem(feature1_arr, 0);
    cJSON *geom1 = cJSON_GetObjectItem(first_feature, "geometry");
    cJSON *coords1 = cJSON_GetObjectItem(geom1, "coordinates");
    cJSON *prop1 = cJSON_GetObjectItem(first_feature, "properties");

    A->n = cJSON_GetArraySize(coords1);
    if (A->n > MAX_NODE_COUNT) A->n = MAX_NODE_COUNT;

    cJSON *name1 = cJSON_GetObjectItem(prop1, "name");
    strcpy(A->name, name1 ? name1->valuestring : "target road");

    for (int i = 0; i < A->n; i++) {
        cJSON *node = cJSON_GetArrayItem(coords1, i);
        long double lon = cJSON_GetArrayItem(node, 0)->valuedouble;
        long double lat = cJSON_GetArrayItem(node, 1)->valuedouble;
        A->coordinates[i].x = lon * radius * lPI * cosl(lat * lPI / 180.0) / 180.0;
        A->coordinates[i].y = lat * radius * lPI / 180.0;
    }
    cJSON_Delete(root1);

    relocate(A);

    long double arcl_A=0;
    for (int i = 0; i < A->n-1; i++) {
        arcl_A += distance(A->coordinates[i], A->coordinates[i+1]);
    }



    FILE *fp = fopen(MAP_FILEADDRESS, "r");
    if (!fp) { printf("cannot open map file\n"); return 1; }
    fseek(fp, 0, SEEK_END);
    long filesize = ftell(fp);
    rewind(fp);
    char *json = malloc(filesize + 1);
    fread(json, 1, filesize, fp);
    json[filesize] = '\0';
    fclose(fp);

    cJSON *root = cJSON_Parse(json);
    free(json);
    cJSON *features = cJSON_GetObjectItem(root, "features");
    int featurecount = cJSON_GetArraySize(features);
    int roadcount = 0;
    int lengthfilteredroadcount = 0;
    int nodesfilteredroadcount = 0;
    int unnamedroadcount = 0;

    for (int i = 0; i < featurecount && roadcount < MAX_FEATURE_COUNT; i++) {
        cJSON *f = cJSON_GetArrayItem(features, i);
        cJSON *props = cJSON_GetObjectItem(f, "properties");
        cJSON *highway = cJSON_GetObjectItem(props, "highway");

        if (!cJSON_IsString(highway)) continue;


        const char* type = highway->valuestring;
        if (strcmp(highway->valuestring,"motorway")==0
            || strcmp(highway->valuestring,"trunk")==0
            || strcmp(highway->valuestring,"primary")==0
            || strcmp(highway->valuestring,"secondary")==0
            || strcmp(highway->valuestring,"tertiary")==0
            || strcmp(highway->valuestring,"residential")==0
            || strcmp(highway->valuestring,"service")==0
            || strcmp(highway->valuestring,"motorway_link")==0
            || strcmp(highway->valuestring,"trunk_link")==0
            || strcmp(highway->valuestring,"primary_link")==0
            || strcmp(highway->valuestring,"secondary_link")==0
            || strcmp(highway->valuestring,"tertiary_link")==0) {
            cJSON *geom = cJSON_GetObjectItem(f, "geometry");
            cJSON *coords = cJSON_GetObjectItem(geom, "coordinates");
            int nodes = cJSON_GetArraySize(coords);
            if (nodes < NODES_FILTER) {
                nodesfilteredroadcount+=1;
                continue;
            }
            if (nodes > MAX_NODE_COUNT) nodes = MAX_NODE_COUNT;

            B[roadcount].n = nodes;
            strcpy(B[roadcount].type, type);
            cJSON *nm = cJSON_GetObjectItem(props, "name");
            if (cJSON_IsString(nm)) {
                strcpy(B[roadcount].name, nm->valuestring);
            }else{
                unnamedroadcount++;
                snprintf(B[roadcount].name,sizeof(B[roadcount].name),"Unnamed road %d",unnamedroadcount);
            }


            long double arcl = 0;
            for (int j = 0; j < nodes; j++) {
                cJSON *node = cJSON_GetArrayItem(coords, j);
                long double lon = cJSON_GetArrayItem(node, 0)->valuedouble;
                long double lat = cJSON_GetArrayItem(node, 1)->valuedouble;
                B[roadcount].coordinates[j].x = lon * radius * lPI * cosl(lat * lPI / 180.0) / 180.0;
                B[roadcount].coordinates[j].y = lat * radius * lPI / 180.0;
                if (j > 0) arcl += distance(B[roadcount].coordinates[j-1], B[roadcount].coordinates[j]);
            }

            if (TARGETROAD_LENGTH_FILTER==0){
                if (arcl < LENGTH_FILTER) {
                    lengthfilteredroadcount++;
                    continue;
                }
             }else if (TARGETROAD_LENGTH_FILTER==1) {
                if (arcl < arcl_A){
                    lengthfilteredroadcount++;
                    continue;
                }
            }

            roadcount++;
            }
    }
    cJSON_Delete(root);

    if (SLIDED_NDPF==1){
        for (int i = 0; i < roadcount; i++) {
        B[i].f = (double)sliding_NDPF(A, &B[i]);
        }
    }else if (SLIDED_NDPF==0) {
        for (int i = 0; i < roadcount; i++) {
            B[i].f = (double)NDPF(A, &B[i]);
        }
    }

    int *Rank = malloc(sizeof(int) * roadcount);
    int *visited = calloc(roadcount, sizeof(int));
    for (int i = 0; i < roadcount; i++) {
        double MIN = 1e18;
        int best = -1;
        for (int j = 0; j < roadcount; j++) {
            if (!visited[j] && B[j].f < MIN) {
                MIN = B[j].f;
                best = j;
            }
        }
        if (best != -1) {
            Rank[i] = best;
            visited[best] = 1;
        }
    }


    for (int i = 0; i < roadcount; i++) {
        int idx = Rank[i];
        printf("%d. %s [%s] - Frechet Distance: %.6f\n", i + 1, B[idx].name, B[idx].type, B[idx].f);
    }

    end_time = (double)clock();
    printf("\nProcessed roads: %d\n", roadcount);
    printf("total: %d\n",roadcount+nodesfilteredroadcount+lengthfilteredroadcount);
    printf("Runtime: %f seconds\n", (end_time - start_time) / CLOCKS_PER_SEC);


    if (VISUALIZE == 0) {
        free(Rank);
        free(visited);
        free(A->coordinates);
        free(A);
        for (int i = 0; i < MAX_FEATURE_COUNT; i++) {
            free(B[i].coordinates);
        }
        free(B);

        return 0;
    }

if (VISUALIZE == 1) {
    SetTraceLogLevel(LOG_NONE);
    InitWindow(1600, 900, "Visualization");
    SetTargetFPS(60);

    int capacity = 23000;
    int *codepoints = malloc(sizeof(int) * capacity);
    int count = 0;

    for (int i = 32; i < 256; i++)
        codepoints[count++] = i;
    for (int i = 0x3040; i <= 0x30FF; i++)
        codepoints[count++] = i;
    for (int i = 0x4E00; i <= 0x9FFF; i++)
        codepoints[count++] = i;

    Font myFont = LoadFontEx(FONT_ADDRESS, 96, codepoints, count);


    int show = SHOW;
    if (show > roadcount) show = roadcount;
         int i = 0;
        while (!WindowShouldClose()) {
            if (IsKeyPressed(KEY_ENTER)
                || IsMouseButtonPressed(MOUSE_LEFT_BUTTON)
                ||IsKeyPressed(KEY_KP_ADD)) {
                i = (i + 1)%show;
            }else if (IsKeyPressed(KEY_KP_SUBTRACT)
                || IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)
                 && i != 0) {
                i = i - 1;
            }

            int best_idx = Rank[i];
            group *current_B = &B[best_idx];

            group drawB = copy_group(current_B, 0, current_B->n);
            relocate(&drawB);
            rotate(A, &drawB);

            BeginDrawing();
            ClearBackground(RAYWHITE);

            Camera2D camera = { 0 };
            camera.target = (Vector2){ 0, 0 };
            camera.offset = (Vector2){ 800, 450 };
            camera.rotation = 0.0f;
            camera.zoom = 20.0f;

            float max_coord = 1.0f;
            for(int i=0; i<A->n; i++) {
                if(fabsl(A->coordinates[i].x) > max_coord) max_coord = fabsl(A->coordinates[i].x);
                if(fabsl(A->coordinates[i].y) > max_coord) max_coord = fabsl(A->coordinates[i].y);
            }
            camera.zoom = 350.0f / max_coord;

            BeginMode2D(camera);
                for (int i = 0; i < A->n - 1; i++) {
                    DrawLineEx(ToV2(A->coordinates[i]), ToV2(A->coordinates[i+1]), 3.0f/camera.zoom, RED);
                }
                for (int i = 0; i < drawB.n - 1; i++) {
                    DrawLineEx(ToV2(drawB.coordinates[i]), ToV2(drawB.coordinates[i+1]), 2.0f/camera.zoom, BLUE);
                }
            EndMode2D();

            DrawRectangle(10, 10, 600, 160, Fade(LIGHTGRAY, 0.8f));

            DrawTextEx(myFont, "Press [ESC] to Exit", (Vector2){20, 20}, 24, 2, DARKGRAY);
            DrawTextEx(myFont, TextFormat("Target: %s", A->name), (Vector2){20, 55}, 24, 2, RED);
            DrawTextEx(myFont, TextFormat("Match Rank %d: %s", i + 1, current_B->name), (Vector2){20, 90}, 24, 2, BLUE);
            DrawTextEx(myFont, TextFormat("Frechet Distance: %.6f", current_B->f), (Vector2){20, 125}, 24, 2, BLACK);

            DrawCircle(800,450,5,RED);
            EndDrawing();

            free(drawB.coordinates);
        }
        CloseWindow();
    }


    free(Rank);
    free(visited);
    free(A->coordinates);
    free(A);
    for (int i = 0; i < MAX_FEATURE_COUNT; i++) {
        free(B[i].coordinates);
    }
    free(B);

    return 0;
}