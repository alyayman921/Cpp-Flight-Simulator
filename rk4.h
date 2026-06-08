#include <stdlib.h>
#include <stdio.h>
#include <math.h>

struct rkdata {
    int i;
    float dt;
    float tfinal;
    int N_steps;
    float *time_v;
};

struct rkdata data;

void rkupdateData(struct rkdata *data, float dt, float tfinal) {
    data->dt = dt;
    data->tfinal = tfinal;
    data->N_steps = (int)(data->tfinal / data->dt);
    data->time_v = (float*)malloc(data->N_steps * sizeof(float));
    for (int i = 0; i < data->N_steps; i++) {
        data->time_v[i] = i * dt;
    }
}

// For system of ODEs (2 equations)
typedef struct {
    float t;
    float y1, y2;
    float **results;  // 2D array: [N_steps][2]
} rkanswer_system;

rkanswer_system rk4_system(float y1_0, float y2_0, void (*f)(float, float*, float*)) {
    rkanswer_system d;
    d.t = 0;
    d.y1 = y1_0;
    d.y2 = y2_0;
    
    // Allocate results array
    d.results = (float**)malloc(data.N_steps * sizeof(float*));
    if (d.results == NULL) {
        d.results = NULL;
        return d;
    }
    
    for (int i = 0; i < data.N_steps; i++) {
        d.results[i] = (float*)malloc(2 * sizeof(float));
        if (d.results[i] == NULL) {
            // Clean up on failure
            for (int j = 0; j < i; j++) free(d.results[j]);
            free(d.results);
            d.results = NULL;
            return d;
        }
    }
    
    d.results[0][0] = d.y1;
    d.results[0][1] = d.y2;
    
    float k1[2], k2[2], k3[2], k4[2];
    float y_temp[2], dy[2];
    
    for (int i = 1; i < data.N_steps; i++) {
        d.t = data.time_v[i-1];
        
        // k1
        f(d.t, (float[]){d.y1, d.y2}, dy);
        k1[0] = dy[0];
        k1[1] = dy[1];
        
        // k2
        y_temp[0] = d.y1 + (data.dt/2.0f) * k1[0];
        y_temp[1] = d.y2 + (data.dt/2.0f) * k1[1];
        f(d.t + data.dt/2.0f, y_temp, dy);
        k2[0] = dy[0];
        k2[1] = dy[1];
        
        // k3
        y_temp[0] = d.y1 + (data.dt/2.0f) * k2[0];
        y_temp[1] = d.y2 + (data.dt/2.0f) * k2[1];
        f(d.t + data.dt/2.0f, y_temp, dy);
        k3[0] = dy[0];
        k3[1] = dy[1];
        
        // k4
        y_temp[0] = d.y1 + data.dt * k3[0];
        y_temp[1] = d.y2 + data.dt * k3[1];
        f(d.t + data.dt, y_temp, dy);
        k4[0] = dy[0];
        k4[1] = dy[1];
        
        // Update
        d.y1 = d.y1 + (data.dt/6.0f) * (k1[0] + 2.0f*k2[0] + 2.0f*k3[0] + k4[0]);
        d.y2 = d.y2 + (data.dt/6.0f) * (k1[1] + 2.0f*k2[1] + 2.0f*k3[1] + k4[1]);
        
        d.results[i][0] = d.y1;
        d.results[i][1] = d.y2;
    }
    
    return d;
}

void ode_system(float t, float *y, float *dy) {
    dy[0] = sin(t) + cos(y[0]) + cos(y[1]);
    dy[1] = cos(t) + sin(y[1]) + cos(y[0]); 
}

void free_results(rkanswer_system *results) {
    if (results->results != NULL) {
        for (int i = 0; i < data.N_steps; i++) {
            if (results->results[i] != NULL)
                free(results->results[i]);
        }
        free(results->results);
        results->results = NULL;
    }
}

