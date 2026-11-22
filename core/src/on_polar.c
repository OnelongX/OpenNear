#include "on_polar.h"
#include "on_frame.h"
#include <math.h>
#include <string.h>

/*
 * 5G NR Beta-Expansion Polar Reliability Ordering (beta = 2^(1/4))
 * Produces deterministic frozen/info bit mask for N = 128, K = 48.
 */
static bool     s_polar_inited = false;
static bool     s_is_info[ON_POLAR_N];
static uint8_t  s_info_indices[ON_POLAR_K];

static void polar_init_tables(void) {
    if (s_polar_inited) return;

    double pw[ON_POLAR_N];
    uint8_t order[ON_POLAR_N];
    const double beta = 1.189207115002721; /* 2^(1/4) */

    for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
        double w = 0.0;
        double bpow = 1.0;
        for (uint32_t bit = 0; bit < ON_POLAR_LOG2_N; ++bit) {
            if ((i >> bit) & 1u) {
                w += bpow;
            }
            bpow *= beta;
        }
        pw[i] = w;
        order[i] = (uint8_t)i;
        s_is_info[i] = false;
    }

    /* Sort indices ascending by polarization weight (insertion sort, N=128) */
    for (uint32_t i = 1; i < ON_POLAR_N; ++i) {
        uint8_t key = order[i];
        double kw = pw[key];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && (pw[order[j]] > kw || (pw[order[j]] == kw && order[j] > key))) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }

    /* Select the top K most reliable bit channels */
    for (uint32_t idx = ON_POLAR_N - ON_POLAR_K; idx < ON_POLAR_N; ++idx) {
        s_is_info[order[idx]] = true;
    }

    /* Store info positions in natural bit-decoding order */
    uint32_t k = 0;
    for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
        if (s_is_info[i]) {
            s_info_indices[k++] = (uint8_t)i;
        }
    }
    s_polar_inited = true;
}

/* Arikan butterfly transform x = u * G_N (natural order) */
static void polar_transform(uint8_t u[ON_POLAR_N]) {
    for (uint32_t step = 1u; step < ON_POLAR_N; step <<= 1u) {
        for (uint32_t base = 0u; base < ON_POLAR_N; base += (step << 1u)) {
            for (uint32_t j = 0u; j < step; ++j) {
                u[base + j] ^= u[base + j + step];
            }
        }
    }
}

on_err_t on_polar_encode_block(const uint8_t msg[ON_POLAR_MSG_BYTES],
                               uint8_t coded_bits[ON_POLAR_N]) {
    if (!msg || !coded_bits) {
        return ON_ERR_INVALID_ARG;
    }
    polar_init_tables();

    uint8_t payload_crc[ON_POLAR_MSG_BYTES + 1u];
    memcpy(payload_crc, msg, ON_POLAR_MSG_BYTES);
    payload_crc[ON_POLAR_MSG_BYTES] = on_crc8(msg, ON_POLAR_MSG_BYTES);

    memset(coded_bits, 0, ON_POLAR_N);
    for (uint32_t k = 0; k < ON_POLAR_K; ++k) {
        uint8_t byte_val = payload_crc[k >> 3u];
        uint8_t bit_val  = (uint8_t)((byte_val >> (7u - (k & 7u))) & 1u);
        coded_bits[s_info_indices[k]] = bit_val;
    }

    polar_transform(coded_bits);
    return ON_OK;
}

/* SCL decoder helper math */
static float llr_f(float a, float b) {
    float sa = (a >= 0.0f) ? 1.0f : -1.0f;
    float sb = (b >= 0.0f) ? 1.0f : -1.0f;
    float aa = fabsf(a);
    float ab = fabsf(b);
    return sa * sb * (aa < ab ? aa : ab);
}

static float llr_g(float a, float b, uint8_t u_left) {
    return u_left ? (b - a) : (b + a);
}

static float pm_penalty(float llr_val, uint8_t u_hat) {
    float abs_l = fabsf(llr_val);
    float soft = (abs_l > 20.0f) ? 0.0f : log1pf(expf(-abs_l));
    bool disagree = ((u_hat == 0u) && (llr_val < 0.0f)) ||
                    ((u_hat == 1u) && (llr_val > 0.0f));
    return disagree ? (abs_l + soft) : soft;
}

/*
 * Compact Flat Binary-Tree Offset Helper:
 * Stage st (0..7) has exact length (1 << st), stored at offset ((1 << st) - 1).
 * Total elements across all 8 stages = 1 + 2 + 4 + 8 + 16 + 32 + 64 + 128 = 255 (< 256),
 * shrinking path memory by 4x compared to [8][128] 2D arrays!
 */
#define POLAR_TREE_NODES  256u
#define STAGE_OFF(st)     ((1u << (uint32_t)(st)) - 1u)

typedef struct {
    float   pm;
    float   llr[POLAR_TREE_NODES];
    uint8_t beta_l[POLAR_TREE_NODES];
    uint8_t beta_r[POLAR_TREE_NODES];
    uint8_t u_bits[ON_POLAR_N];
} scl_path_t;

/* Compute LLRs down to stage 0 for bit index phi */
static void scl_update_llrs(scl_path_t *p, uint32_t phi) {
    uint32_t highest_stage = ON_POLAR_LOG2_N;
    if (phi > 0u) {
        uint32_t s = 0u;
        while (s < ON_POLAR_LOG2_N && (((phi >> s) & 1u) == 0u)) {
            s++;
        }
        highest_stage = s + 1u;
        if (highest_stage > ON_POLAR_LOG2_N) {
            highest_stage = ON_POLAR_LOG2_N;
        }
    }

    for (int32_t st = (int32_t)highest_stage; st >= 1; --st) {
        uint32_t half = 1u << (uint32_t)(st - 1);
        bool right_branch = (((phi >> (uint32_t)(st - 1)) & 1u) != 0u);
        const float *src = &p->llr[STAGE_OFF(st)];
        float       *dst = &p->llr[STAGE_OFF(st - 1)];
        const uint8_t *bl = &p->beta_l[STAGE_OFF(st - 1)];
        for (uint32_t j = 0u; j < half; ++j) {
            float a = src[j];
            float b = src[j + half];
            dst[j] = right_branch ? llr_g(a, b, bl[j]) : llr_f(a, b);
        }
    }
}

/* Propagate partial sums upward after deciding bit phi */
static void scl_update_betas(scl_path_t *p, uint32_t phi, uint8_t u_hat) {
    p->u_bits[phi] = u_hat;
    if ((phi & 1u) == 0u) {
        p->beta_l[STAGE_OFF(0)] = u_hat;
        return;
    }

    p->beta_r[STAGE_OFF(0)] = u_hat;
    uint32_t st = 0u;
    uint32_t cur_phi = phi;
    while (st < ON_POLAR_LOG2_N && ((cur_phi & 1u) == 1u)) {
        uint32_t half = 1u << st;
        bool next_is_right = (((cur_phi >> 1u) & 1u) != 0u);
        const uint8_t *bl = &p->beta_l[STAGE_OFF(st)];
        const uint8_t *br = &p->beta_r[STAGE_OFF(st)];
        uint8_t *dst = next_is_right ? &p->beta_r[STAGE_OFF(st + 1u)]
                                     : &p->beta_l[STAGE_OFF(st + 1u)];
        for (uint32_t j = 0u; j < half; ++j) {
            dst[j]        = (uint8_t)(bl[j] ^ br[j]);
            dst[j + half] = br[j];
        }
        cur_phi >>= 1u;
        st++;
    }
}

typedef struct {
    uint32_t parent_idx;
    uint8_t  bit;
    float    pm;
} scl_cand_t;

on_err_t on_polar_decode_block(const float llr[ON_POLAR_N],
                               uint8_t msg_out[ON_POLAR_MSG_BYTES]) {
    if (!llr || !msg_out) {
        return ON_ERR_INVALID_ARG;
    }
    polar_init_tables();

    /* Static workspace kept small & deterministic (~36 KB on stack/BSS -> use local/static carefully) */
    scl_path_t paths[ON_POLAR_LIST_SIZE];
    scl_path_t next_paths[ON_POLAR_LIST_SIZE];
    uint32_t num_paths = 1u;

    memset(&paths[0], 0, sizeof(paths[0]));
    paths[0].pm = 0.0f;
    for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
        paths[0].llr[STAGE_OFF(ON_POLAR_LOG2_N) + i] = llr[i];
    }

    for (uint32_t phi = 0; phi < ON_POLAR_N; ++phi) {
        for (uint32_t p = 0; p < num_paths; ++p) {
            scl_update_llrs(&paths[p], phi);
        }

        if (!s_is_info[phi]) {
            /* Frozen bit: always 0 */
            for (uint32_t p = 0; p < num_paths; ++p) {
                paths[p].pm += pm_penalty(paths[p].llr[STAGE_OFF(0)], 0u);
                scl_update_betas(&paths[p], phi, 0u);
            }
        } else {
            /* Information bit: fork every active path into 0 and 1 */
            scl_cand_t cands[ON_POLAR_LIST_SIZE * 2u];
            uint32_t nc = 0u;
            for (uint32_t p = 0; p < num_paths; ++p) {
                float leaf = paths[p].llr[STAGE_OFF(0)];
                cands[nc].parent_idx = p;
                cands[nc].bit = 0u;
                cands[nc].pm = paths[p].pm + pm_penalty(leaf, 0u);
                nc++;

                cands[nc].parent_idx = p;
                cands[nc].bit = 1u;
                cands[nc].pm = paths[p].pm + pm_penalty(leaf, 1u);
                nc++;
            }

            /* Sort candidates ascending by PM */
            for (uint32_t i = 1; i < nc; ++i) {
                scl_cand_t key = cands[i];
                int32_t j = (int32_t)i - 1;
                while (j >= 0 && cands[j].pm > key.pm) {
                    cands[j + 1] = cands[j];
                    j--;
                }
                cands[j + 1] = key;
            }

            uint32_t keep = (nc < ON_POLAR_LIST_SIZE) ? nc : ON_POLAR_LIST_SIZE;
            for (uint32_t k = 0; k < keep; ++k) {
                next_paths[k] = paths[cands[k].parent_idx];
                next_paths[k].pm = cands[k].pm;
                scl_update_betas(&next_paths[k], phi, cands[k].bit);
            }
            for (uint32_t k = 0; k < keep; ++k) {
                paths[k] = next_paths[k];
            }
            num_paths = keep;
        }
    }

    /* Sort final surviving paths by ascending PM */
    for (uint32_t i = 1; i < num_paths; ++i) {
        scl_path_t key = paths[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && paths[j].pm > key.pm) {
            paths[j + 1] = paths[j];
            j--;
        }
        paths[j + 1] = key;
    }

    /* Pick the best path passing CRC-8 verification */
    for (uint32_t p = 0; p < num_paths; ++p) {
        uint8_t payload_crc[ON_POLAR_MSG_BYTES + 1u];
        memset(payload_crc, 0, sizeof(payload_crc));
        for (uint32_t k = 0; k < ON_POLAR_K; ++k) {
            if (paths[p].u_bits[s_info_indices[k]]) {
                payload_crc[k >> 3u] |= (uint8_t)(1u << (7u - (k & 7u)));
            }
        }
        if (on_crc8(payload_crc, ON_POLAR_MSG_BYTES) == payload_crc[ON_POLAR_MSG_BYTES]) {
            memcpy(msg_out, payload_crc, ON_POLAR_MSG_BYTES);
            return ON_OK;
        }
    }

    return ON_ERR_DECODE_FAIL;
}

/* ========================================================================== */
/* v3.0 Fixed-Point (Q4.4 int16_t, zero-float) Embedded & FPGA-HLS CA-SCL     */
/* ========================================================================== */
static int16_t sat_i16(int32_t v) {
    if (v >  1023) return  1023;
    if (v < -1024) return -1024;
    return (int16_t)v;
}

static int16_t llr_f_q4(int16_t a, int16_t b) {
    int16_t aa = (a < 0) ? (int16_t)(-a) : a;
    int16_t ab = (b < 0) ? (int16_t)(-b) : b;
    int16_t m  = (aa < ab) ? aa : ab;
    bool neg = ((a < 0) != (b < 0));
    return neg ? (int16_t)(-m) : m;
}

static int16_t llr_g_q4(int16_t a, int16_t b, uint8_t u_left) {
    return sat_i16(u_left ? ((int32_t)b - (int32_t)a) : ((int32_t)b + (int32_t)a));
}

static uint32_t pm_penalty_q4(int16_t llr_val, uint8_t u_hat) {
    bool disagree = ((u_hat == 0u) && (llr_val < 0)) ||
                    ((u_hat == 1u) && (llr_val > 0));
    if (!disagree) return 0u;
    return (uint32_t)((llr_val < 0) ? -llr_val : llr_val);
}

typedef struct {
    uint32_t pm;
    int16_t  llr[POLAR_TREE_NODES];
    uint8_t  beta_l[POLAR_TREE_NODES];
    uint8_t  beta_r[POLAR_TREE_NODES];
    uint8_t  u_bits[ON_POLAR_N];
} scl_path_q4_t; /* Only 1,156 bytes per path -> 8 paths fit in 9.2 KB on stack (100% reentrant!) */

static void scl_q4_update_llrs(scl_path_q4_t *p, uint32_t phi) {
    uint32_t highest_stage = ON_POLAR_LOG2_N;
    if (phi > 0u) {
        uint32_t s = 0u;
        while (s < ON_POLAR_LOG2_N && (((phi >> s) & 1u) == 0u)) s++;
        highest_stage = s + 1u;
        if (highest_stage > ON_POLAR_LOG2_N) highest_stage = ON_POLAR_LOG2_N;
    }
    for (int32_t st = (int32_t)highest_stage; st >= 1; --st) {
        uint32_t half = 1u << (uint32_t)(st - 1);
        bool right_branch = (((phi >> (uint32_t)(st - 1)) & 1u) != 0u);
        const int16_t *src = &p->llr[STAGE_OFF(st)];
        int16_t       *dst = &p->llr[STAGE_OFF(st - 1)];
        const uint8_t *bl  = &p->beta_l[STAGE_OFF(st - 1)];
        for (uint32_t j = 0u; j < half; ++j) {
            int16_t a = src[j];
            int16_t b = src[j + half];
            dst[j] = right_branch ? llr_g_q4(a, b, bl[j]) : llr_f_q4(a, b);
        }
    }
}

static void scl_q4_update_betas(scl_path_q4_t *p, uint32_t phi, uint8_t u_hat) {
    p->u_bits[phi] = u_hat;
    if ((phi & 1u) == 0u) {
        p->beta_l[STAGE_OFF(0)] = u_hat;
        return;
    }
    p->beta_r[STAGE_OFF(0)] = u_hat;
    uint32_t st = 0u;
    uint32_t cur_phi = phi;
    while (st < ON_POLAR_LOG2_N && ((cur_phi & 1u) == 1u)) {
        uint32_t half = 1u << st;
        bool next_is_right = (((cur_phi >> 1u) & 1u) != 0u);
        const uint8_t *bl = &p->beta_l[STAGE_OFF(st)];
        const uint8_t *br = &p->beta_r[STAGE_OFF(st)];
        uint8_t *dst = next_is_right ? &p->beta_r[STAGE_OFF(st + 1u)]
                                     : &p->beta_l[STAGE_OFF(st + 1u)];
        for (uint32_t j = 0u; j < half; ++j) {
            dst[j]        = (uint8_t)(bl[j] ^ br[j]);
            dst[j + half] = br[j];
        }
        cur_phi >>= 1u;
        st++;
    }
}

typedef struct {
    uint32_t parent_idx;
    uint8_t  bit;
    uint32_t pm;
} scl_cand_q4_t;

on_err_t on_polar_decode_block_fixed(const int16_t llr_q4[ON_POLAR_N],
                                     uint8_t msg_out[ON_POLAR_MSG_BYTES]) {
    if (!llr_q4 || !msg_out) return ON_ERR_INVALID_ARG;
    polar_init_tables();

    /* Reentrant stack allocation with compact 256-node flat tree */
    scl_path_q4_t paths[ON_POLAR_LIST_SIZE];
    scl_path_q4_t next_paths[ON_POLAR_LIST_SIZE];
    uint32_t num_paths = 1u;

    memset(&paths[0], 0, sizeof(paths[0]));
    for (uint32_t i = 0u; i < ON_POLAR_N; ++i) {
        paths[0].llr[STAGE_OFF(ON_POLAR_LOG2_N) + i] = sat_i16(llr_q4[i]);
    }

    for (uint32_t phi = 0u; phi < ON_POLAR_N; ++phi) {
        for (uint32_t p = 0u; p < num_paths; ++p) {
            scl_q4_update_llrs(&paths[p], phi);
        }
        if (!s_is_info[phi]) {
            for (uint32_t p = 0u; p < num_paths; ++p) {
                paths[p].pm += pm_penalty_q4(paths[p].llr[STAGE_OFF(0)], 0u);
                scl_q4_update_betas(&paths[p], phi, 0u);
            }
        } else {
            scl_cand_q4_t cands[ON_POLAR_LIST_SIZE * 2u];
            uint32_t nc = 0u;
            for (uint32_t p = 0u; p < num_paths; ++p) {
                int16_t leaf = paths[p].llr[STAGE_OFF(0)];
                cands[nc].parent_idx = p;
                cands[nc].bit = 0u;
                cands[nc].pm  = paths[p].pm + pm_penalty_q4(leaf, 0u);
                nc++;
                cands[nc].parent_idx = p;
                cands[nc].bit = 1u;
                cands[nc].pm  = paths[p].pm + pm_penalty_q4(leaf, 1u);
                nc++;
            }
            for (uint32_t i = 1u; i < nc; ++i) {
                scl_cand_q4_t key = cands[i];
                int32_t j = (int32_t)i - 1;
                while (j >= 0 && cands[j].pm > key.pm) {
                    cands[j + 1] = cands[j];
                    j--;
                }
                cands[j + 1] = key;
            }
            uint32_t keep = (nc < ON_POLAR_LIST_SIZE) ? nc : ON_POLAR_LIST_SIZE;
            for (uint32_t k = 0u; k < keep; ++k) {
                next_paths[k] = paths[cands[k].parent_idx];
                next_paths[k].pm = cands[k].pm;
                scl_q4_update_betas(&next_paths[k], phi, cands[k].bit);
            }
            for (uint32_t k = 0u; k < keep; ++k) paths[k] = next_paths[k];
            num_paths = keep;
        }
    }

    for (uint32_t i = 1u; i < num_paths; ++i) {
        scl_path_q4_t key = paths[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && paths[j].pm > key.pm) {
            paths[j + 1] = paths[j];
            j--;
        }
        paths[j + 1] = key;
    }

    for (uint32_t p = 0u; p < num_paths; ++p) {
        uint8_t payload_crc[ON_POLAR_MSG_BYTES + 1u];
        memset(payload_crc, 0, sizeof(payload_crc));
        for (uint32_t k = 0u; k < ON_POLAR_K; ++k) {
            if (paths[p].u_bits[s_info_indices[k]]) {
                payload_crc[k >> 3u] |= (uint8_t)(1u << (7u - (k & 7u)));
            }
        }
        if (on_crc8(payload_crc, ON_POLAR_MSG_BYTES) == payload_crc[ON_POLAR_MSG_BYTES]) {
            memcpy(msg_out, payload_crc, ON_POLAR_MSG_BYTES);
            return ON_OK;
        }
    }
    return ON_ERR_DECODE_FAIL;
}
