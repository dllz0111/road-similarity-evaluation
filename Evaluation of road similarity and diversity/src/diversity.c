#include <stdio.h>
#include<math.h>
#include<time.h>
#include <string.h>
#include <stdlib.h>
#include "../lib/cJSON.h"

#define AVERAGE_SLIDED_NDFP 1

#define MAX_FEATURE_COUNT 1000
#define MAX_NODE_COUNT 200
#define NODES_FILTER 3
#define LENGTH_FILTER 10


#define MAP_FILEADDRESS "assets/data/Toyosu.geojson"

long double max(long double a,long double b) {
    return (a > b) ? a : b;
}

long double min(long double a,long double b) {
    return (a < b) ? a : b;
}

typedef struct{
    long double x;
    long double y;
}Point;

typedef struct {
    int n;
    Point *coordinates;
    char name[256];
    char type[32];
    double f;
}group;


long double distance_sq(Point a, Point b){
    return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}
long double distance(Point a,Point b) {
    return sqrtl(distance_sq(a,b));
}

void relocate(group *a) {
    long double dx = a->coordinates[0].x;
    long double dy = a->coordinates[0].y;
    for (int i = 0; i < a->n;i++) {
        a->coordinates[i].x-=dx;
        a->coordinates[i].y-=dy;
    }
}



double rotate_theta(const group *a,const group *b) {
    long double arcla=0,arclb=0;
    for (int i=0;i<a->n-1;i++) {
        arcla += sqrt(distance_sq(a->coordinates[i],a->coordinates[i+1]));
    }
    for (int i=0;i<b->n-1;i++) {
        arclb += sqrt(distance_sq(b->coordinates[i],b->coordinates[i+1]));
    }
    int count=0;
    if (arcla>arclb) {
        arcla=0;
        while (count < a->n - 1 && arcla<arclb) {
            arcla+=sqrt(distance_sq(a->coordinates[count],a->coordinates[count+1]));
            count++;
        }
    }else {
        arclb=0;
        while (count < a->n - 1 && arclb<arcla) {
            arclb+=sqrt(distance_sq(b->coordinates[count],b->coordinates[count+1]));
            count++;
        }
    }

    long double rotate_theta;

    long double arctan1=0;
    long double arctan2=0;
    for (int i=0;i<min(count+1,min(a->n,b->n));i++) {
        arctan1+=a->coordinates[i].x*b->coordinates[i].y-a->coordinates[i].y*b->coordinates[i].x;
        arctan2+=a->coordinates[i].x*b->coordinates[i].x+a->coordinates[i].y*b->coordinates[i].y;
    }
    rotate_theta = (double)atan2l(arctan1, arctan2);
    return rotate_theta;
}


void rotate(const group *a,group *b) {
    long double theta = -rotate_theta(a, b);

    long double c = cosl(theta);
    long double s = sinl(theta);
    Point *old = b->coordinates;
    Point *newp = malloc(sizeof(Point) * b->n);

    for (int i = 0; i < b->n; i++) {
        newp[i].x = c * old[i].x - s * old[i].y;
        newp[i].y = s * old[i].x + c * old[i].y;
    }

    b->coordinates = newp;
    free(old);
}


#define DISTANCE2_TABLE(dis,col_d,i,j)     dis[i*col_d+j]
#define F_TABLE(f,col_f,i,j)       f[i*col_f+j]


long double frecdis(const group *a, const group *b) {
    if (a->n <= 0 || b->n <= 0) return 0;

    int rows = a->n;
    int cols = b->n;

    long double *dis = malloc(rows * cols * sizeof(long double));
    long double *f = malloc(rows * cols * sizeof(long double));

    if (!dis || !f) {
        if (dis) free(dis);
        if (f) free(f);
        return 1e18;
    }

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            dis[i * cols + j] = distance_sq(a->coordinates[i], b->coordinates[j]);
        }
    }

    f[0] = dis[0];

    for (int i = 1; i < rows; i++) {
        f[i * cols + 0] = max(f[(i - 1) * cols + 0], dis[i * cols + 0]);
    }

    for (int j = 1; j < cols; j++) {
        f[0 * cols + j] = max(f[0 * cols + (j - 1)], dis[0 * cols + j]);
    }

    for (int i = 1; i < rows; i++) {
        for (int j = 1; j < cols; j++) {
            f[i * cols + j] = max(dis[i * cols + j], min(f[(i - 1) * cols + (j - 1)], min(f[(i - 1) * cols + j], f[i * cols + (j - 1)])));
        }
    }


    long double final_sq = f[(rows - 1) * cols + (cols - 1)];
    long double final_res = sqrtl(final_sq);

    free(dis);
    free(f);
    return final_res;
}

long double normalized_frecdis(const group *a,const group *b) {
    int na=a->n;
    int nb=b->n;
    long double arcla=0,arclb=0;

    for(int i=0;i<na-1;i++) {
        arcla+=distance(a->coordinates[i],a->coordinates[i+1]);
    }
    for(int i=0;i<nb-1;i++) {
        arclb+=distance(b->coordinates[i],b->coordinates[i+1]);
    }

    group ap = *a, bp = *b;
    int count = 0;
    long double deltalength = 0;
    if (arcla > arclb) {
        while (count < a->n - 1 && deltalength < arclb) {
            deltalength += distance(a->coordinates[count], a->coordinates[count + 1]);
            count++;
        }
        ap.n = count + 1;
    } else {
        while (count < b->n - 1 && deltalength < arcla) {
            deltalength += distance(b->coordinates[count], b->coordinates[count + 1]);
            count++;
        }
        bp.n = count + 1;
    }
    return frecdis(&ap, &bp);
}

long double NDPF(group *a,group *b) {
    relocate(a);
    relocate(b);
    rotate(a,b);
    return normalized_frecdis(a,b);
}



void print_group(const char *name, const group *g) {
    printf("%s (n=%d):\n", name, g->n);
    for (int i = 0; i < g->n; i++) {
        printf("  (%.3Lf, %.3Lf)\n", g->coordinates[i].x, g->coordinates[i].y);
    }
}


group copy_group(const group *src, int start_index, int count) {
    group dest;
    dest.n = count;
    strcpy(dest.name, src->name);
    strcpy(dest.type, src->type);
    dest.f = src->f;
    dest.coordinates = malloc(sizeof(Point) * count);
    for (int i = 0; i < count; i++) {
        dest.coordinates[i] = src->coordinates[start_index + i];
    }
    return dest;
}

void reverse_group(group g) {
    int n = g.n;
    for (int i = 0; i < n / 2; i++) {
        Point temp = g.coordinates[i];
        g.coordinates[i] = g.coordinates[n - 1 - i];
        g.coordinates[n - 1 - i] = temp;
    }
}

long double sliding_NDPF(group *A, group *B) {
    long double average =0;
    if (B->n > A->n) {
        int window_size = A->n;
        for (int i = 0; i <= B->n - window_size; i++) {
            group tempB = copy_group(B, i, window_size);
            group tempA = copy_group(A, 0, A->n);

            long double current_f = NDPF(&tempA, &tempB);

            reverse_group(tempA);
            long double reverse_f = NDPF(&tempA, &tempB);

            average += reverse_f;
            average += current_f;

            free(tempA.coordinates);
            free(tempB.coordinates);
        }
        average = average /(2 * B->n);
    }
    else if (A->n > B->n) {
        int window_size = B->n;
        for (int i = 0; i <= A->n - window_size; i++) {
            group tempA = copy_group(A, i, window_size);
            group tempB = copy_group(B, 0, B->n);

            long double current_f = NDPF(&tempA, &tempB);
            average += current_f;

            reverse_group(tempA);
            long double reverse_f = NDPF(&tempA, &tempB);

            average += reverse_f;

            free(tempA.coordinates);
            free(tempB.coordinates);
        }
        average = average /(2 * A->n);
    }
    else {
        group tempA = copy_group(A, 0, A->n);
        group tempB = copy_group(B, 0, B->n);
        reverse_group(tempA);
        long double reverse_f = NDPF(&tempA, &tempB);
        average = NDPF(&tempA, &tempB);
        average += reverse_f;
        average /= 2;
        free(tempA.coordinates);
        free(tempB.coordinates);
    }

    return average;
}

int main(void) {
    double start_time, end_time;
    start_time = (double)clock();
    system("chcp 65001");
    long double lPI = 3.14159265358979323846;
    long double radius = 6371393.0;

    group *B = malloc(sizeof(group) * MAX_FEATURE_COUNT);
    for (int i = 0; i < MAX_FEATURE_COUNT; i++) {
        B[i].coordinates = malloc(sizeof(Point) * MAX_NODE_COUNT);
        B[i].n = 0;
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
    int nodefilteredcount = 0;
    int lengthfilteredcount = 0;

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
                nodefilteredcount++;
                continue;
            }
            if (nodes > MAX_NODE_COUNT) nodes = MAX_NODE_COUNT;

            B[roadcount].n = nodes;
            strcpy(B[roadcount].type, type);
            cJSON *nm = cJSON_GetObjectItem(props, "name");
            strcpy(B[roadcount].name, nm ? nm->valuestring : "Unnamed road");

            long double arcl = 0;
            for (int j = 0; j < nodes; j++) {
                cJSON *node = cJSON_GetArrayItem(coords, j);
                long double lon = cJSON_GetArrayItem(node, 0)->valuedouble;
                long double lat = cJSON_GetArrayItem(node, 1)->valuedouble;
                B[roadcount].coordinates[j].x = lon * radius * lPI * cosl(lat * lPI / 180.0) / 180.0;
                B[roadcount].coordinates[j].y = lat * radius * lPI / 180.0;
                if (j > 0) arcl += distance(B[roadcount].coordinates[j-1], B[roadcount].coordinates[j]);
            }

            if (arcl < LENGTH_FILTER) {
                lengthfilteredcount++;
                continue;
            }
            roadcount++;
            }
    }
    cJSON_Delete(root);

    long double *fd_table = malloc(sizeof(long double) * roadcount * roadcount);
#define FD_TABLE(i,j)  fd_table[i*roadcount+j]

    if (AVERAGE_SLIDED_NDFP==1){
        for (int i = 0; i < roadcount; i++) {
            FD_TABLE(i, i) = 0.0;
            for (int j = i + 1; j < roadcount; j++) {
                long double d = sliding_NDPF(&B[i], &B[j]);
                FD_TABLE(i, j) = d;
                FD_TABLE(j, i) = d;
            }
        }
    }else if (AVERAGE_SLIDED_NDFP == 0) {
        for (int i = 0; i < roadcount; i++) {
            FD_TABLE(i, i) = 0.0;
            for (int j = i + 1; j < roadcount; j++) {
                long double d = NDPF(&B[i], &B[j]);
                FD_TABLE(i, j) = d;
                FD_TABLE(j, i) = d;
            }
        }
    }
    if (roadcount < 1) return 1;
    long double sum = 0;
    for (int i = 0; i < roadcount-1; i++) {
        for (int j = 0; j < i; j++) {
            sum+=FD_TABLE(i, j);
        }
    }
    long double average = sum / (0.5*roadcount*roadcount-0.5*roadcount);
    printf("diversity = %Lf\n", average);
    printf("Processed roads: %d\n", roadcount);
    printf("total: %d\n",roadcount+nodefilteredcount+lengthfilteredcount);

    end_time = (double)clock();
    printf("runtime: %lf",(end_time-start_time)/CLOCKS_PER_SEC);

    for (int i = 0; i < MAX_FEATURE_COUNT; i++) {
        free(B[i].coordinates);
    }
    free(B);
    free(fd_table);

    int esc=1;
    for (;;){
        printf("\n\nPress [0] to exit\n");
        scanf("%d",&esc);
        if (esc==0) break;
    }
    return 0;
}