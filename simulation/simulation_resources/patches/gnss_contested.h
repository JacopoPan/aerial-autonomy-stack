// AAS contested GNSS: one model for the simulated receivers of:
//  - PX4 (gz_bridge, from Gazebo's NavSat sensor)
//  - ArduPilot (SIM_GPS from the vehicle state sent by the ArduPilot Gazebo plugin)
//  - Betaflight (the SITL's virtual GPS, from the vehicle state sent by the aeroloop Gazebo plugin)
//
// With GNSS=contested, simulation.yml.erb starts each SITL with two environment variables:
// AAS_GNSS_CONTESTED: the values of its gnss_contested hash, in this order
//  - onset_s: seconds of nominal GNSS, from the receiver's first update, so the autopilot initializes with a good fix
//  - min_satellites: fewest satellites in use, below 4 there is no fix (min_satellites = max_satellites = 0 is a denied GNSS)
//  - max_satellites: most satellites in use, also at onset_s
//  - sigma_m: position error (m, standard deviation per axis) at HDOP 1, x HDOP horizontally, x VDOP vertically
//  - tau_s: correlation time (s) of the position error: it drifts, instead of jumping at every fix
//  - vel_sigma_m_s: velocity error (m/s, standard deviation per axis) at HDOP 1, x HDOP horizontally, x VDOP vertically
// AAS_GNSS_SEED: the drone ID, used as random seed (different errors for each drone, the same in every run)
//
// Without AAS_GNSS_CONTESTED (GNSS=nominal or none), the model in this file changes nothing of how the PX4/ArduPilot/Betaflight SITLs work
#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define AAS_GNSS_MIN_FIX_SATELLITES 4 // Fewest satellites needed for a fix
#define AAS_GNSS_STEP_MIN_S 2.0 // The satellites in use change at random times, between this...
#define AAS_GNSS_STEP_MAX_S 6.0 // ...and this many seconds apart
#define AAS_GNSS_STEP_SATELLITES 2 // Each change adds or removes up to this many satellites, at random (only adds without a fix)
#define AAS_GNSS_HDOP_NUMERATOR 6.0 // HDOP = this / (satellites - 2): 3.0 with 4 satellites, 1.5 with 6, 0.86 with 9 (heuristic)
#define AAS_GNSS_VDOP_PER_HDOP 1.5 // VDOP = this x HDOP: vertical is less accurate than horizontal
#define AAS_GNSS_JUMP_KEEP 0.5 // When the satellites change, the position error jumps: it keeps this fraction, the rest is redrawn

typedef struct {
    // Set by aas_gnss_update(), satellites < 0 when nominal: keep the simulator's own GNSS
    int satellites;
    double error_ned_m[3];
    double velocity_error_ned_m_s[3];
    double hdop, vdop, eph_m, epv_m, sacc_m_s;
    // Parameters and state
    int initialized, contested, started, min_sats, max_sats, sats, lost;
    double onset_s, sigma_m, tau_s, vel_sigma_m_s, first_s, time_s, next_step_s;
    struct drand48_data rng;
} aas_gnss_t;

static inline double aas_gnss_uniform(aas_gnss_t *g, double low, double high)
{
    double u = 0.0;
    drand48_r(&g->rng, &u); // glibc, in [0, 1)
    return low + (high - low) * u;
}

static inline double aas_gnss_normal(aas_gnss_t *g) // Box-Muller
{
    const double u = 1.0 - aas_gnss_uniform(g, 0.0, 1.0); // In (0, 1]
    return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * aas_gnss_uniform(g, 0.0, 1.0));
}

static inline void aas_gnss_init(aas_gnss_t *g, double time_s)
{
    const char *contested = getenv("AAS_GNSS_CONTESTED");
    const char *seed = getenv("AAS_GNSS_SEED");
    g->initialized = 1;
    g->first_s = time_s;
    srand48_r(seed ? atol(seed) : 0L, &g->rng);
    if (contested) {
        g->contested = sscanf(contested, "%lf %d %d %lf %lf %lf", &g->onset_s, &g->min_sats, &g->max_sats, &g->sigma_m, &g->tau_s, &g->vel_sigma_m_s) == 6
                       && g->min_sats <= g->max_sats && g->tau_s > 0.0;
        if (!g->contested) {
            printf("AAS GNSS: AAS_GNSS_CONTESTED='%s' is not '<onset_s> <min satellites> <max satellites> <sigma m> <tau s> <vel_sigma m/s>', nominal\n", contested);
        }
    }
}

// Updates the model at time_s (s), then sets the receiver's outputs
static inline void aas_gnss_update(aas_gnss_t *g, double time_s)
{
    if (!g->initialized) {
        aas_gnss_init(g, time_s);
    }
    g->satellites = -1;
    g->hdop = g->vdop = g->eph_m = g->epv_m = g->sacc_m_s = 0.0;
    if (!g->contested || time_s - g->first_s < g->onset_s) {
        return;
    }
    if (!g->started) {
        printf("AAS GNSS: contested\n");
        g->started = 1;
        g->sats = g->max_sats;
        g->time_s = time_s;
        g->next_step_s = time_s + aas_gnss_uniform(g, AAS_GNSS_STEP_MIN_S, AAS_GNSS_STEP_MAX_S);
    }
    double dt = time_s - g->time_s;
    dt = (dt < 0.0) ? 0.0 : dt;
    g->time_s = time_s;

    int changed = 0;
    if (time_s >= g->next_step_s) {
        int step = (int)floor(aas_gnss_uniform(g, -AAS_GNSS_STEP_SATELLITES, AAS_GNSS_STEP_SATELLITES + 1.0));
        if (g->sats < AAS_GNSS_MIN_FIX_SATELLITES && step < 0) {
            step = -step;
        }
        int sats = g->sats + step;
        sats = (sats < g->min_sats) ? g->min_sats : ((sats > g->max_sats) ? g->max_sats : sats);
        changed = (sats != g->sats);
        g->sats = sats;
        g->next_step_s = time_s + aas_gnss_uniform(g, AAS_GNSS_STEP_MIN_S, AAS_GNSS_STEP_MAX_S);
    }
    g->satellites = g->sats;
    if (g->sats < AAS_GNSS_MIN_FIX_SATELLITES) {
        g->lost = 1;
        return;
    }

    g->hdop = AAS_GNSS_HDOP_NUMERATOR / (double)(g->sats - 2);
    g->vdop = AAS_GNSS_VDOP_PER_HDOP * g->hdop;
    const double dop[3] = {g->hdop, g->hdop, g->vdop};
    const double decay = exp(-dt / g->tau_s);
    for (int i = 0; i < 3; i++) {
        const double sigma = g->sigma_m * dop[i];
        if (g->lost) {
            g->error_ned_m[i] = sigma * aas_gnss_normal(g);
        } else {
            g->error_ned_m[i] = decay * g->error_ned_m[i] + sqrt(1.0 - decay * decay) * sigma * aas_gnss_normal(g);
            if (changed) {
                g->error_ned_m[i] = AAS_GNSS_JUMP_KEEP * g->error_ned_m[i] + sqrt(1.0 - AAS_GNSS_JUMP_KEEP * AAS_GNSS_JUMP_KEEP) * sigma * aas_gnss_normal(g);
            }
        }
        g->velocity_error_ned_m_s[i] = g->vel_sigma_m_s * dop[i] * aas_gnss_normal(g);
    }
    g->lost = 0;
    g->eph_m = g->sigma_m * g->hdop;
    g->epv_m = g->sigma_m * g->vdop;
    g->sacc_m_s = g->vel_sigma_m_s * g->hdop;
}
