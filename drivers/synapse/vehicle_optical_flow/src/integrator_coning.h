/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * C port of PX4 IntegratorConing. All float, no double.
 */

#ifndef INTEGRATOR_CONING_H
#define INTEGRATOR_CONING_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define IC_DT_MIN 1e-6f
#define IC_DT_MAX 4294.967296f /* UINT32_MAX * 1e-6f */

struct integrator_coning {
	float alpha[3];
	float last_val[3];
	float beta[3];
	float last_delta_alpha[3];
	float last_alpha[3];
	float integral_dt;
	uint8_t integrated_samples;
	uint8_t reset_samples_min;
};

static inline void integrator_coning_init(struct integrator_coning *ic)
{
	memset(ic, 0, sizeof(*ic));
	ic->reset_samples_min = 1;
}

static inline void integrator_coning_reset(struct integrator_coning *ic)
{
	memset(ic->alpha, 0, sizeof(ic->alpha));
	memset(ic->beta, 0, sizeof(ic->beta));
	memset(ic->last_alpha, 0, sizeof(ic->last_alpha));
	memset(ic->last_delta_alpha, 0, sizeof(ic->last_delta_alpha));
	ic->integral_dt = 0.0f;
	ic->integrated_samples = 0;
}

static inline float integrator_coning_integral_dt(const struct integrator_coning *ic)
{
	return ic->integral_dt;
}

static inline bool integrator_coning_integral_ready(const struct integrator_coning *ic)
{
	return ic->integrated_samples >= ic->reset_samples_min;
}

static inline void integrator_coning_put(struct integrator_coning *ic,
					 const float val[3], float dt)
{
	if (dt <= IC_DT_MIN || (ic->integral_dt + dt) >= IC_DT_MAX) {
		integrator_coning_reset(ic);
		ic->last_val[0] = val[0];
		ic->last_val[1] = val[1];
		ic->last_val[2] = val[2];
		return;
	}

	/* trapezoidal integration */
	float delta_alpha[3];

	for (int i = 0; i < 3; i++) {
		delta_alpha[i] = (val[i] + ic->last_val[i]) * dt * 0.5f;
	}

	ic->integrated_samples++;
	ic->integral_dt += dt;
	ic->last_val[0] = val[0];
	ic->last_val[1] = val[1];
	ic->last_val[2] = val[2];

	/*
	 * Coning correction: beta += ((last_alpha + last_delta_alpha/6) x delta_alpha) * 0.5
	 * Reference: Strapdown Inertial Navigation Integration Algorithm Design Part 1
	 * https://arc.aiaa.org/doi/pdf/10.2514/2.4228
	 */
	float term[3];

	for (int i = 0; i < 3; i++) {
		term[i] = ic->last_alpha[i] + ic->last_delta_alpha[i] * (1.0f / 6.0f);
	}

	/* cross product: term x delta_alpha */
	float cross[3];

	cross[0] = term[1] * delta_alpha[2] - term[2] * delta_alpha[1];
	cross[1] = term[2] * delta_alpha[0] - term[0] * delta_alpha[2];
	cross[2] = term[0] * delta_alpha[1] - term[1] * delta_alpha[0];

	for (int i = 0; i < 3; i++) {
		ic->beta[i] += cross[i] * 0.5f;
	}

	ic->last_delta_alpha[0] = delta_alpha[0];
	ic->last_delta_alpha[1] = delta_alpha[1];
	ic->last_delta_alpha[2] = delta_alpha[2];

	ic->last_alpha[0] = ic->alpha[0];
	ic->last_alpha[1] = ic->alpha[1];
	ic->last_alpha[2] = ic->alpha[2];

	/* accumulate */
	for (int i = 0; i < 3; i++) {
		ic->alpha[i] += delta_alpha[i];
	}
}

/**
 * Reset and retrieve integrated value with coning corrections applied.
 * Returns true if integral was ready, false otherwise.
 */
static inline bool integrator_coning_reset_get(struct integrator_coning *ic,
					       float integral[3],
					       uint32_t *integral_dt_us)
{
	if (!integrator_coning_integral_ready(ic)) {
		return false;
	}

	/* apply coning corrections */
	for (int i = 0; i < 3; i++) {
		integral[i] = ic->alpha[i] + ic->beta[i];
	}

	*integral_dt_us = (uint32_t)(ic->integral_dt * 1e6f + 0.5f);

	integrator_coning_reset(ic);
	return true;
}

#endif /* INTEGRATOR_CONING_H */
