#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

void update_physics(double* x, double* y, double* vx, double* vy, int n, double radius) {
    for (int i = 0; i < n; i++) {
        x[i] += vx[i];
        y[i] += vy[i];

        if (x[i] < -10.0 + radius) {
            vx[i] *= -1.0;
            x[i] = -10.0 + radius;
        } else if (x[i] > 10.0 - radius) {
            vx[i] *= -1.0;
            x[i] = 10.0 - radius;
        }

        if (y[i] < -10.0 + radius) {
            vy[i] *= -1.0;
            y[i] = -10.0 + radius;
        } else if (y[i] > 10.0 - radius) {
            vy[i] *= -1.0;
            y[i] = 10.0 - radius;
        }
    }

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double dx = x[i] - x[j];
            double dy = y[i] - y[j];
            double dist = hypot(dx, dy);

            if (dist > 0 && dist < radius * 2.0) {
                double nx = dx / dist;
                double ny = dy / dist;

                double p = (vx[i] * nx + vy[i] * ny - vx[j] * nx - vy[j] * ny);

                vx[i] -= p * nx;
                vy[i] -= p * ny;
                vx[j] += p * nx;
                vy[j] += p * ny;

                double overlap = radius * 2.0 - dist;
                x[i] += nx * overlap / 2.0;
                y[i] += ny * overlap / 2.0;
                x[j] -= nx * overlap / 2.0;
                y[j] -= ny * overlap / 2.0;
            }
        }
    }
}

void run_dbscan(double* x, double* y, int n, double eps, int minPts, int* cluster_ids, int cluster_limit) {
    for (int i = 0; i < n; i++) {
        cluster_ids[i] = -1;
    }

    int cluster_index = 0;
    int* neighbors = (int*)malloc(n * sizeof(int));
    int* seed_set = (int*)malloc(n * sizeof(int));
    bool* in_seed_set = (bool*)malloc(n * sizeof(bool));

    for (int i = 0; i < n; i++) {
        if (cluster_ids[i] != -1 || cluster_index > cluster_limit) continue;

        int neighbor_count = 0;
        for (int j = 0; j < n; j++) {
            double dx = x[i] - x[j];
            double dy = y[i] - y[j];
            if (sqrt(dx*dx + dy*dy) <= eps) {
                neighbors[neighbor_count++] = j;
            }
        }

        if (neighbor_count < minPts) {
            cluster_ids[i] = 0; 
        } else {
            cluster_index++;
            cluster_ids[i] = cluster_index;

            for (int k = 0; k < n; k++) {
                in_seed_set[k] = false;
            }

            int seed_count = 0;
            for (int j = 0; j < neighbor_count; j++) {
                int n_idx = neighbors[j];
                if (n_idx != i) {
                    seed_set[seed_count++] = n_idx;
                    in_seed_set[n_idx] = true;
                }
            }

            int current_seed_idx = 0;
            while (current_seed_idx < seed_count) {
                int current_p = seed_set[current_seed_idx++];

                if (cluster_ids[current_p] == 0) {
                    cluster_ids[current_p] = cluster_index;
                }
                if (cluster_ids[current_p] != -1) {
                    continue;
                }

                cluster_ids[current_p] = cluster_index;

                int inner_neighbor_count = 0;
                for (int j = 0; j < n; j++) {
                    double dx = x[current_p] - x[j];
                    double dy = y[current_p] - y[j];
                    if (sqrt(dx*dx + dy*dy) <= eps) {
                        neighbors[inner_neighbor_count++] = j;
                    }
                }

                if (inner_neighbor_count >= minPts) {
                    for (int j = 0; j < inner_neighbor_count; j++) {
                        int n_idx = neighbors[j];
                        if (!in_seed_set[n_idx] && cluster_ids[n_idx] == -1) {
                            seed_set[seed_count++] = n_idx;
                            in_seed_set[n_idx] = true;
                        }
                    }
                }
            }
        }
    }
    
    free(neighbors);
    free(seed_set);
    free(in_seed_set);
}

void get_cluster_lines(double* x, double* y, int* cluster_ids, int n, double eps, double* out_x, double* out_y, int* out_cids, int* total_lines) {
    int count = 0;
    double eps_sq = eps * eps;
    
    for (int i = 0; i < n; i++) {
        int cid = cluster_ids[i];
        if (cid <= 0) continue;
        
        for (int j = i + 1; j < n; j++) {
            if (cluster_ids[j] == cid) {
                double dx = x[i] - x[j];
                double dy = y[i] - y[j];
                
                if (dx*dx + dy*dy <= eps_sq) {
                    out_x[count*2] = x[i];
                    out_x[count*2 + 1] = x[j];
                    out_y[count*2] = y[i];
                    out_y[count*2 + 1] = y[j];
                    out_cids[count] = cid;
                    count++;
                }
            }
        }
    }
    *total_lines = count;
}