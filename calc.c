#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

typedef struct QuadtreeNode {
    double x, y, hw, hh;
    int capacity;
    int count;
    int* particles;
    struct QuadtreeNode* nw;
    struct QuadtreeNode* ne;
    struct QuadtreeNode* sw;
    struct QuadtreeNode* se;
} QuadtreeNode;

QuadtreeNode* create_node(double x, double y, double hw, double hh, int capacity) {
    QuadtreeNode* node = (QuadtreeNode*)malloc(sizeof(QuadtreeNode));
    node->x = x;
    node->y = y;
    node->hw = hw;
    node->hh = hh;
    node->capacity = capacity;
    node->count = 0;
    node->particles = (int*)malloc(capacity * sizeof(int));
    node->nw = NULL;
    node->ne = NULL;
    node->sw = NULL;
    node->se = NULL;
    return node;
}

void subdivide(QuadtreeNode* node) {
    double hw = node->hw / 2.0;
    double hh = node->hh / 2.0;
    node->nw = create_node(node->x - hw, node->y + hh, hw, hh, node->capacity);
    node->ne = create_node(node->x + hw, node->y + hh, hw, hh, node->capacity);
    node->sw = create_node(node->x - hw, node->y - hh, hw, hh, node->capacity);
    node->se = create_node(node->x + hw, node->y - hh, hw, hh, node->capacity);
}

bool insert(QuadtreeNode* node, int p_idx, double px, double py) {
    if (px < node->x - node->hw || px > node->x + node->hw ||
        py < node->y - node->hh || py > node->y + node->hh) {
        return false;
    }

    if (node->count < node->capacity && node->nw == NULL) {
        node->particles[node->count++] = p_idx;
        return true;
    }

    if (node->nw == NULL) {
        subdivide(node);
    }

    if (insert(node->nw, p_idx, px, py)) return true;
    if (insert(node->ne, p_idx, px, py)) return true;
    if (insert(node->sw, p_idx, px, py)) return true;
    if (insert(node->se, p_idx, px, py)) return true;

    return false;
}

void query(QuadtreeNode* node, double qx, double qy, double qhw, double qhh, int* found, int* found_count) {
    if (node->x + node->hw < qx - qhw || node->x - node->hw > qx + qhw ||
        node->y + node->hh < qy - qhh || node->y - node->hh > qy + qhh) {
        return;
    }

    for (int i = 0; i < node->count; i++) {
        found[(*found_count)++] = node->particles[i];
    }

    if (node->nw == NULL) {
        return;
    }

    query(node->nw, qx, qy, qhw, qhh, found, found_count);
    query(node->ne, qx, qy, qhw, qhh, found, found_count);
    query(node->sw, qx, qy, qhw, qhh, found, found_count);
    query(node->se, qx, qy, qhw, qhh, found, found_count);
}

void free_tree(QuadtreeNode* node) {
    if (node == NULL) return;
    free(node->particles);
    free_tree(node->nw);
    free_tree(node->ne);
    free_tree(node->sw);
    free_tree(node->se);
    free(node);
}

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

    QuadtreeNode* root = create_node(0.0, 0.0, 10.0, 10.0, 4);
    for (int i = 0; i < n; i++) {
        insert(root, i, x[i], y[i]);
    }

    int* candidates = (int*)malloc(n * sizeof(int));
    
    for (int i = 0; i < n; i++) {
        int candidate_count = 0;
        query(root, x[i], y[i], radius * 2.0, radius * 2.0, candidates, &candidate_count);

        for (int k = 0; k < candidate_count; k++) {
            int j = candidates[k];
            if (j <= i) continue;

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

    free(candidates);
    free_tree(root);
}

void run_dbscan(double* x, double* y, int n, double eps, int minPts, int* cluster_ids, int cluster_limit) {
    for (int i = 0; i < n; i++) {
        cluster_ids[i] = -1;
    }

    int cluster_idx = 0;
    int* neighbors = (int*)malloc(n * sizeof(int));
    int* seed_set = (int*)malloc(n * sizeof(int));
    bool* in_seed_set = (bool*)malloc(n * sizeof(bool));

    for (int i = 0; i < n; i++) {
        if (cluster_ids[i] != -1) continue;

        if (cluster_idx >= cluster_limit) {
            cluster_ids[i] = 0;
            continue;
        }

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
            cluster_idx++;
            cluster_ids[i] = cluster_idx;

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
                    cluster_ids[current_p] = cluster_idx;
                }
                if (cluster_ids[current_p] != -1) {
                    continue;
                }

                cluster_ids[current_p] = cluster_idx;

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
