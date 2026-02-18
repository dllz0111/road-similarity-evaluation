//
// Created by dllz0111 on 2026/2/18.
//

#ifndef NDPF_H
#define NDPF_H


#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#define SLIDED_NDPF 1
#define REVERSE 1
#define TARGETROAD_LENGTH_FILTER 0

#define MAX_FEATURE_COUNT 1000
#define MAX_NODE_COUNT 150
#define NODES_FILTER 3
#define LENGTH_FILTER 300//in meter
#define SHOW 100

#define TARGET_ROAD_FILEADDRESS "target road3.geojson"
#define MAP_FILEADDRESS "map.geojson"

#define FONT_ADDRESS "PretendardJP-Bold.otf"
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

void reverse_group(group *g) {
    int n = g->n;
    for (int i = 0; i < n / 2; i++) {
        Point temp = g->coordinates[i];
        g->coordinates[i] = g->coordinates[n - 1 - i];
        g->coordinates[n - 1 - i] = temp;
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
    if (REVERSE == 0){
        relocate(a);
        relocate(b);
        rotate(a,b);
        return normalized_frecdis(a,b);
    }else if (REVERSE == 1) {
        relocate(a);
        relocate(b);
        rotate(a,b);
        long double forward = normalized_frecdis(a,b);

        reverse_group(a);
        relocate(a);
        reverse_group(b);
        relocate(b);
        rotate(a,b);
        long double reverse =  normalized_frecdis(a,b);

        return min(forward,reverse);
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


long double sliding_NDPF(group *A, group *B) {
    long double min_dist = 1e18;

    if (B->n > A->n) {
        int window_size = A->n;
        for (int i = 0; i <= B->n - window_size; i++) {
            group tempB = copy_group(B, i, window_size);
            group tempA = copy_group(A, 0, A->n);

            long double current_f = NDPF(&tempA, &tempB);
            if (current_f < min_dist) min_dist = current_f;

            free(tempA.coordinates);
            free(tempB.coordinates);
        }
    }
    else if (A->n > B->n) {
        min_dist = NDPF(A, B);
    }
    else {
        group tempA = copy_group(A, 0, A->n);
        group tempB = copy_group(B, 0, B->n);
        min_dist = NDPF(&tempA, &tempB);
        free(tempA.coordinates);
        free(tempB.coordinates);
    }

    return min_dist;
}



#endif //NDPF_H
