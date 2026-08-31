#include "thrust_allocator.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define TA_N                 8U
#define TA_DOF               6U
#define TA_TMAX              500.0f
#define TA_EFFORT_WEIGHT     1.0e-6f
#define TA_RHO               100.0f
#define TA_MAX_ITER          100U
#define TA_ABS_TOL           1.0e-5f
#define TA_REL_TOL           1.0e-5f
#define TA_AXIS_EPS          1.0e-12f
#define TA_CHOLESKY_EPS      1.0e-8f

/*
 * Fixed layout copied from the supplied C++ main().
 * fan1..fan4: horizontal thrusters
 * fan5..fan8: vertical thrusters
 */
static const float s_position[TA_N][3] =
{
    { 0.23016f, -0.18359f, -0.040532f},
    { 0.23016f,  0.18359f, -0.040532f},
    {-0.25591f, -0.18359f, -0.040532f},
    {-0.25591f,  0.18359f, -0.040532f},
    { 0.25239f, -0.26163f,  0.0f},
    { 0.25239f,  0.26163f,  0.0f},
    {-0.27087f, -0.26163f,  0.0f},
    {-0.27087f,  0.26163f,  0.0f}
};

static const float s_axis_raw[TA_N][3] =
{
    {-0.70711f, -0.70711f, 0.0f},
    {-0.70711f,  0.70711f, 0.0f},
    { 0.70711f, -0.70711f, 0.0f},
    { 0.70711f,  0.70711f, 0.0f},
    { 0.0f,      0.0f,     1.0f},
    { 0.0f,      0.0f,     1.0f},
    { 0.0f,      0.0f,     1.0f},
    { 0.0f,      0.0f,     1.0f}
};

typedef struct
{
    uint8_t initialized;

    /* B maps normalized thruster command k to body wrench. */
    float B[TA_DOF][TA_N];

    /* q = G * desired. */
    float G[TA_N][TA_DOF];

    /* Cholesky factor of P + rho*I. */
    float L[TA_N][TA_N];

    /* ADMM warm-start state. */
    float x[TA_N];
    float z[TA_N];
    float u[TA_N];
} TA_State;

static TA_State s_ta;

static float ta_clip(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void ta_cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static float ta_inf_norm(const float v[TA_N])
{
    uint32_t i;
    float m = 0.0f;

    for (i = 0U; i < TA_N; ++i)
    {
        const float a = fabsf(v[i]);
        if (a > m) m = a;
    }
    return m;
}

/* Dense Cholesky factorization A = L * L^T. */
static int ta_cholesky(const float A[TA_N][TA_N], float L[TA_N][TA_N])
{
    uint32_t i, j, k;

    memset(L, 0, sizeof(float) * TA_N * TA_N);

    for (i = 0U; i < TA_N; ++i)
    {
        for (j = 0U; j <= i; ++j)
        {
            float sum = A[i][j];

            for (k = 0U; k < j; ++k)
                sum -= L[i][k] * L[j][k];

            if (i == j)
            {
                if ((!isfinite(sum)) || (sum <= TA_CHOLESKY_EPS))
                    return 0;
                L[i][j] = sqrtf(sum);
            }
            else
            {
                if (fabsf(L[j][j]) <= TA_CHOLESKY_EPS)
                    return 0;
                L[i][j] = sum / L[j][j];
            }
        }
    }

    return 1;
}

static void ta_cholesky_solve(const float L[TA_N][TA_N],
                              const float b[TA_N],
                              float x[TA_N])
{
    float y[TA_N];
    int32_t i;
    uint32_t j;

    for (i = 0; i < (int32_t)TA_N; ++i)
    {
        float sum = b[i];
        for (j = 0U; j < (uint32_t)i; ++j)
            sum -= L[i][j] * y[j];
        y[i] = sum / L[i][i];
    }

    for (i = (int32_t)TA_N - 1; i >= 0; --i)
    {
        float sum = y[i];
        for (j = (uint32_t)i + 1U; j < TA_N; ++j)
            sum -= L[j][i] * x[j];
        x[i] = sum / L[i][i];
    }
}

int ThrustAllocator_Init(void)
{
    uint32_t i, j, d;
    float P[TA_N][TA_N];
    float K[TA_N][TA_N];

    memset(&s_ta, 0, sizeof(s_ta));

    /* Build allocation matrix B. */
    for (i = 0U; i < TA_N; ++i)
    {
        float axis[3];
        float force[3];
        float torque[3];
        const float norm2 = s_axis_raw[i][0] * s_axis_raw[i][0]
                          + s_axis_raw[i][1] * s_axis_raw[i][1]
                          + s_axis_raw[i][2] * s_axis_raw[i][2];

        if ((!isfinite(norm2)) || (norm2 <= TA_AXIS_EPS))
            return 0;

        {
            const float inv_norm = 1.0f / sqrtf(norm2);
            axis[0] = s_axis_raw[i][0] * inv_norm;
            axis[1] = s_axis_raw[i][1] * inv_norm;
            axis[2] = s_axis_raw[i][2] * inv_norm;
        }

        force[0] = TA_TMAX * axis[0];
        force[1] = TA_TMAX * axis[1];
        force[2] = TA_TMAX * axis[2];

        ta_cross3(s_position[i], force, torque);

        s_ta.B[0][i] = force[0];
        s_ta.B[1][i] = force[1];
        s_ta.B[2][i] = force[2];
        s_ta.B[3][i] = torque[0];
        s_ta.B[4][i] = torque[1];
        s_ta.B[5][i] = torque[2];
    }

    /* W = I, so G = -2*B^T. */
    for (i = 0U; i < TA_N; ++i)
    {
        for (d = 0U; d < TA_DOF; ++d)
            s_ta.G[i][d] = -2.0f * s_ta.B[d][i];
    }

    /* P = 2*(B^T*B + effort_weight*I). */
    for (i = 0U; i < TA_N; ++i)
    {
        for (j = 0U; j < TA_N; ++j)
        {
            float sum = 0.0f;

            for (d = 0U; d < TA_DOF; ++d)
                sum += s_ta.B[d][i] * s_ta.B[d][j];

            if (i == j)
                sum += TA_EFFORT_WEIGHT;

            P[i][j] = 2.0f * sum;
            K[i][j] = P[i][j];
        }
    }

    for (i = 0U; i < TA_N; ++i)
        K[i][i] += TA_RHO;

    if (!ta_cholesky(K, s_ta.L))
        return 0;

    s_ta.initialized = 1U;
    return 1;
}

static int ta_solve_normalized(const float desired[TA_DOF], float kout[TA_N])
{
    uint32_t i, d, iter;
    float q[TA_N];
    float rhs[TA_N];
    float z_old[TA_N];
    float r[TA_N];
    float s[TA_N];

    if ((!s_ta.initialized) || (desired == 0) || (kout == 0))
        return 0;

    for (d = 0U; d < TA_DOF; ++d)
    {
        if (!isfinite(desired[d]))
            return 0;
    }

    for (i = 0U; i < TA_N; ++i)
    {
        float v = 0.0f;
        for (d = 0U; d < TA_DOF; ++d)
            v += s_ta.G[i][d] * desired[d];
        q[i] = v;
    }

    for (iter = 0U; iter < TA_MAX_ITER; ++iter)
    {
        float eps_pri;
        float eps_dual;
        float r_norm;
        float s_norm;
        float x_norm;
        float z_norm;
        float yu_norm;

        memcpy(z_old, s_ta.z, sizeof(z_old));

        for (i = 0U; i < TA_N; ++i)
            rhs[i] = -q[i] + TA_RHO * (s_ta.z[i] - s_ta.u[i]);

        ta_cholesky_solve(s_ta.L, rhs, s_ta.x);

        for (i = 0U; i < TA_N; ++i)
            s_ta.z[i] = ta_clip(s_ta.x[i] + s_ta.u[i], -1.0f, 1.0f);

        for (i = 0U; i < TA_N; ++i)
            s_ta.u[i] += s_ta.x[i] - s_ta.z[i];

        for (i = 0U; i < TA_N; ++i)
        {
            r[i] = s_ta.x[i] - s_ta.z[i];
            s[i] = TA_RHO * (s_ta.z[i] - z_old[i]);
        }

        r_norm = ta_inf_norm(r);
        s_norm = ta_inf_norm(s);
        x_norm = ta_inf_norm(s_ta.x);
        z_norm = ta_inf_norm(s_ta.z);

        for (i = 0U; i < TA_N; ++i)
            rhs[i] = TA_RHO * s_ta.u[i];
        yu_norm = ta_inf_norm(rhs);

        eps_pri = TA_ABS_TOL
                + TA_REL_TOL * ((x_norm > z_norm) ? x_norm : z_norm);
        eps_dual = TA_ABS_TOL + TA_REL_TOL * yu_norm;

        if ((r_norm <= eps_pri) && (s_norm <= eps_dual))
        {
            memcpy(kout, s_ta.z, sizeof(float) * TA_N);
            return 1;
        }
    }

    /* Bounded best iterate is still returned. */
    memcpy(kout, s_ta.z, sizeof(float) * TA_N);
    return 1;
}

int ThrustAllocator_Solve(float Fx,
                          float Fy,
                          float Fz,
                          float Mx,
                          float My,
                          float Mz,
                          float thrust_out[THRUST_ALLOCATOR_THRUSTER_COUNT])
{
    uint32_t i;
    float k[TA_N] = {0.0f};
    const float desired[TA_DOF] = {Fx, Fy, Fz, Mx, My, Mz};

    if (thrust_out == 0)
        return 0;

    for (i = 0U; i < TA_N; ++i)
        thrust_out[i] = 0.0f;

    if (!ta_solve_normalized(desired, k))
        return 0;

    for (i = 0U; i < TA_N; ++i)
        thrust_out[i] = k[i] * TA_TMAX;

    return 1;
}
