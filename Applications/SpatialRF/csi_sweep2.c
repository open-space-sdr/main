// csi_sweep.c
//
// Fast pipelined LO sweep phase-scatter visualization for 4-channel CSI SDR.
// Telemetry measures time spent in:
//   - CSI block acquisition (read)
//   - LO programming (LO)
//   - FFT execution (FFT)
//   - processing + selection + stamping (proc)
//
// Speedups included:
//   1) Single-pixel stamping (instead of disks)
//   2) Phase differences via complex products => 2 atan2f/bin (not 4)
//   3) Aggressive ring wait: short spin, then small usleep
//   4) Cap points per LO using top-K selection (min-heap)
//   5) Read-path optimization: one GET_RING_INFO + one CONSUME per block
//
// NEW (requested): render at lower logical resolution (CANVAS_W/H) and scale up
// to a larger SDL window (WIN_WIDTH/HEIGHT) using nearest-neighbor. This makes
// "single pixels" appear larger, like big dots, without expensive disk stamping.
//
// Build:
//   gcc csi_sweep.c -O3 -o csi_sweep -lfftw3f -lSDL2 -lm -lpthread
//
// Run:
//   ./csi_sweep

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#include <sys/mman.h>
#include <sys/ioctl.h>

#include <pthread.h>

#include <SDL2/SDL.h>
#include <fftw3.h>

#include "../csi/fpga_csi.h"

// ------------------------------------------------------------
// Configuration
// ------------------------------------------------------------

#define DEVICE_PATH      "/dev/csi_stream0"

#define FFT_SIZE         4096
#define CHANNELS         4
#define BYTES_PER_IQ     2
#define BYTES_PER_FRAME  (CHANNELS * BYTES_PER_IQ)
#define BLOCK_BYTES      (FFT_SIZE * BYTES_PER_FRAME)

// Logical render resolution (small => "fatter" points when scaled up)
#define CANVAS_W         256
#define CANVAS_H         256

// Actual window size on screen
#define WIN_WIDTH        1920
#define WIN_HEIGHT       1024

// Sweep parameters (MHz)
#define LO_START_MHZ     4900.0
#define LO_END_MHZ       6000.0
#define LO_STEP_MHZ      18.0
#define FS_MHZ           18.0

// Antenna Geometry
#define SPEED_OF_LIGHT   299792458.0f
#define ANT_DIST_VERT    0.045f       // 4.5 cm (Index 0 to 2)
#define ANT_DIST_HORIZ   0.078f       // 7.8 cm (Index 1 to 3)

#define CALIB_PHASE_V    (3.14159f)  
#define CALIB_PHASE_H    (3.14159f)

// If ring backlog grows, drop older data to keep latency bounded.
#define MAX_QUEUED_BLOCKS   2

// Aggressive ring-wait tuning:
#define WAIT_SPIN_ITERS     2000   // spin iterations before sleeping
#define WAIT_SLEEP_US       20     // short sleep after spin phase

// Visualization tuning
#define ENABLE_DECAY        1
#define DECAY_FACTOR        0.50f
#define POINT_GAIN          10000.0f

// Cap points per LO deterministically (top-K by magnitude metric)
#define TOPK_PER_LO         512

// Telemetry
#define TELEMETRY_PRINT_EVERY_N_FRAMES  1   // set to 10 for less spam

bool  mirror_display = false;
#define BTN_SIZE 100
#define BTN_MARGIN 30

// ------------------------------------------------------------

static void die(const char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline double ns_to_ms(uint64_t ns) { return (double)ns / 1e6; }

static inline uint32_t ring_used_bytes(uint32_t head, uint32_t tail, uint64_t ring_size)
{
    return (head >= tail) ? (head - tail)
                          : ((uint32_t)ring_size - (tail - head));
}

static void consume_bytes(int fd, uint32_t nbytes)
{
    if (!nbytes) return;
    if (ioctl(fd, CSI_IOC_CONSUME_BYTES, &nbytes) < 0)
        die("CSI_IOC_CONSUME_BYTES");
}

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline void cpu_relax(void)
{
#if defined(__aarch64__) || defined(__arm__)
    __asm__ __volatile__("yield" ::: "memory");
#elif defined(__x86_64__) || defined(__i386__)
    __asm__ __volatile__("pause" ::: "memory");
#else
    __asm__ __volatile__("" ::: "memory");
#endif
}

static void hsv_to_rgb(float h, float s, float v, float *r, float *g, float *b)
{
    if (s <= 0.0f) { *r = *g = *b = v; return; }
    h = fmodf(h, 1.0f); if (h < 0) h += 1.0f;

    float hf = h * 6.0f;
    int i = (int)hf;
    float f = hf - i;

    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));

    switch (i % 6)
    {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    default:*r = v; *g = p; *b = q; break;
    }
}

static void decay_accum(uint16_t *acc)
{
    size_t n = (size_t)CANVAS_W * CANVAS_H * 3;
    for (size_t i = 0; i < n; ++i)
        acc[i] = (uint16_t)((float)acc[i] * DECAY_FACTOR);
}

static void accum_to_pixels(const uint16_t *acc, uint32_t *pix)
{
    for (size_t i = 0; i < (size_t)CANVAS_W * CANVAS_H; ++i)
    {
        uint32_t r = acc[i*3+0] >> 6;
        uint32_t g = acc[i*3+1] >> 6;
        uint32_t b = acc[i*3+2] >> 6;

        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;

        /* Camera underlay support: make "empty" pixels transparent so the
         * video shows through, while stamped points remain opaque.
         */
        uint32_t a = (r | g | b) ? 0xFFu : 0x00u;
        pix[i] = (a<<24) | (r<<16) | (g<<8) | b;
    }
}


// Single pixel add with saturation (fast).
static inline void stamp_pixel_add(uint16_t *acc, int x, int y,
                                   float cr, float cg, float cb, float strength)
{
    if ((unsigned)x >= (unsigned)CANVAS_W)  return;
    if ((unsigned)y >= (unsigned)CANVAS_H)  return;

    size_t idx = ((size_t)y * CANVAS_W + (size_t)x) * 3;

    uint32_t ir = (uint32_t)(cr * strength);
    uint32_t ig = (uint32_t)(cg * strength);
    uint32_t ib = (uint32_t)(cb * strength);

    uint32_t nr = acc[idx+0] + ir;
    uint32_t ng = acc[idx+1] + ig;
    uint32_t nb = acc[idx+2] + ib;

    acc[idx+0] = (nr > 65535u) ? 65535u : (uint16_t)nr;
    acc[idx+1] = (ng > 65535u) ? 65535u : (uint16_t)ng;
    acc[idx+2] = (nb > 65535u) ? 65535u : (uint16_t)nb;
}

// ------------------------------------------------------------
// Read telemetry (per frame)
// ------------------------------------------------------------

typedef struct {
    uint64_t ns_getinfo;
    uint64_t ns_copy;
    uint64_t ns_consume;
    uint64_t calls_getinfo;
    uint64_t calls_consume;
    uint64_t wraps;
} read_telem_t;

// Wait until at least one block is available. If backlog is large, drop oldest blocks.
// Optimized for fewer ioctls:
//   - exactly one GET_RING_INFO per returned block
//   - exactly one CONSUME_BYTES per returned block
static void ring_wait_and_get_one_block_fast(int fd, const void *ring, uint64_t ring_size,
                                             uint8_t *dst, read_telem_t *rt)
{
    int spins = WAIT_SPIN_ITERS;

    while (1)
    {
        uint64_t t0 = now_ns();
        struct csi_ring_info ri;
        if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0)
            die("CSI_IOC_GET_RING_INFO");
        uint64_t t1 = now_ns();

        rt->ns_getinfo += (t1 - t0);
        rt->calls_getinfo++;

        uint32_t used = ring_used_bytes(ri.head, ri.tail, ring_size);
        used -= (used % BYTES_PER_FRAME);

        uint32_t max_keep = (uint32_t)(MAX_QUEUED_BLOCKS * BLOCK_BYTES);
        if (used > max_keep)
        {
            uint32_t drop = used - max_keep;
            drop -= (drop % BYTES_PER_FRAME);
            if (drop)
            {
                uint64_t tc0 = now_ns();
                consume_bytes(fd, drop);
                uint64_t tc1 = now_ns();
                rt->ns_consume += (tc1 - tc0);
                rt->calls_consume++;
            }
            spins = WAIT_SPIN_ITERS;
            continue;
        }

        if (used >= BLOCK_BYTES)
        {
            uint32_t tail = ri.tail;

            uint32_t n1 = BLOCK_BYTES;
            if ((uint64_t)tail + (uint64_t)n1 > ring_size)
            {
                n1 = (uint32_t)ring_size - tail;
                rt->wraps++;
            }

            uint64_t tm0 = now_ns();
            memcpy(dst, (const uint8_t*)ring + tail, n1);
            uint32_t rem = BLOCK_BYTES - n1;
            if (rem) memcpy(dst + n1, (const uint8_t*)ring, rem);
            uint64_t tm1 = now_ns();
            rt->ns_copy += (tm1 - tm0);

            uint64_t tc0 = now_ns();
            consume_bytes(fd, BLOCK_BYTES);
            uint64_t tc1 = now_ns();
            rt->ns_consume += (tc1 - tc0);
            rt->calls_consume++;

            return;
        }

        if (spins-- > 0) cpu_relax();
        else usleep(WAIT_SLEEP_US);
    }
}

static void cs8_to_fftw_ch(const int8_t *src, int ch, fftwf_complex *dst)
{
    for (int n = 0; n < FFT_SIZE; ++n)
    {
        size_t base = (size_t)n * BYTES_PER_FRAME + (size_t)ch * BYTES_PER_IQ;
        dst[n][0] = (float)src[base + 0] / 127.0f;
        dst[n][1] = (float)src[base + 1] / 127.0f;
    }
}

// ------------------------------------------------------------
// JTAG LO programming
// ------------------------------------------------------------

// ------------------------------------------------------------
// JTAG LO programming
// ------------------------------------------------------------

static uint16_t max2851_lna_band_reg2_for_freq(double mhz)
{
    /* Main2: LNA_BAND[1:0] at D[6:5]; keep reserved bits at default. */
    if (mhz < 5200.0) return 0x180;  /* 00: 4.9–5.2 GHz */
    if (mhz < 5500.0) return 0x1A0;  /* 01: 5.2–5.5 GHz (default) */
    if (mhz < 5800.0) return 0x1C0;  /* 10: 5.5–5.8 GHz */
    return 0x1E0;                    /* 11: 5.8–5.9 GHz */
}

static int program_set_freq(int fd, double freq_mhz)
{
    double ratio = freq_mhz / 80.0;
    int idiv = (int)floor(ratio);
    int fdiv = (int)llround((ratio - idiv) * (double)(1u << 20));

    struct csi_jtag_reg r;
    r.addr = 0x43;

    r.value = (uint16_t)((15u<<10) | (1u<<9) | (idiv & 0x7f));
    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;

    r.value = (uint16_t)((16u<<10) | ((fdiv >> 10) & 0x3ff));
    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;

    r.value = (uint16_t)((17u<<10) | (fdiv & 0x3ff));
    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;

    uint16_t reg2 = max2851_lna_band_reg2_for_freq(freq_mhz);
    r.value = (uint16_t)((2u << 10) | (reg2 & 0x3FF));
    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;

    return 0;
}

int jtag_read_u16(int fd, uint8_t addr, uint16_t *out_value)
{
    if (!out_value) {
        errno = EINVAL;
        return -1;
    }
    struct csi_jtag_reg r;
    memset(&r, 0, sizeof(r));
    r.addr = addr;
    if (ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) != 0) return -1;
    *out_value = r.value;
    return 0;
}


static int adjust_rx_gain(int fd, int adjust)
{
    struct csi_jtag_reg r;
    r.addr = 0x6A;

    jtag_read_u16(fd, r.addr, &r.value);

    r.value += adjust;

    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;
    return 0;
}

// ------------------------------------------------------------
// Telemetry structures (worker -> UI)
// ------------------------------------------------------------

typedef struct {
    uint64_t frame_idx;

    double t_read_ms;
    double t_lo_ms;
    double t_fft_ms;
    double t_proc_ms;
    double t_frame_ms;

    // Read breakdown
    double t_read_getinfo_ms;
    double t_read_copy_ms;
    double t_read_consume_ms;
    uint64_t read_calls_getinfo;
    uint64_t read_calls_consume;
    uint64_t read_wraps;

    uint32_t blocks;
    uint32_t points;

    /* Normalized RF activity estimate in [0,1]. Higher => more/stronger points. */
    float rf_activity;

    double fps;
} telemetry_t;

// ------------------------------------------------------------
// Top-K min-heap selection (by score=v_raw)
// ------------------------------------------------------------

typedef struct {
    float v;
    int   k;
} heap_item_t;

static inline void heap_swap(heap_item_t *a, heap_item_t *b)
{
    heap_item_t t = *a; *a = *b; *b = t;
}

static inline void heap_sift_up(heap_item_t *h, int idx)
{
    while (idx > 0)
    {
        int p = (idx - 1) >> 1;
        if (h[p].v <= h[idx].v) break;
        heap_swap(&h[p], &h[idx]);
        idx = p;
    }
}

static inline void heap_sift_down(heap_item_t *h, int n, int idx)
{
    while (1)
    {
        int l = idx * 2 + 1;
        int r = l + 1;
        int m = idx;

        if (l < n && h[l].v < h[m].v) m = l;
        if (r < n && h[r].v < h[m].v) m = r;
        if (m == idx) break;
        heap_swap(&h[m], &h[idx]);
        idx = m;
    }
}

static inline int heap_push_topk(heap_item_t *h, int size, int cap, float v, int k)
{
    if (size < cap)
    {
        h[size].v = v;
        h[size].k = k;
        heap_sift_up(h, size);
        return size + 1;
    }
    if (v <= h[0].v) return size;
    h[0].v = v;
    h[0].k = k;
    heap_sift_down(h, cap, 0);
    return size;
}

// ------------------------------------------------------------
// Worker thread
// ------------------------------------------------------------

typedef struct {
    int fd;
    void *ring;
    uint64_t ring_size;

    fftwf_complex *fft_in;
    fftwf_complex *fft_out[CHANNELS];
    fftwf_plan plan[CHANNELS];

    uint16_t *front, *back;
    pthread_mutex_t mtx;
    volatile int quit;

    int   mirror_display;      // 0 or 1
    float point_gain;

    telemetry_t telem;
} ctx_t;

 // WORKER THREAD
static void *worker(void *arg)
{
    ctx_t *c = (ctx_t*)arg;

    uint8_t *blk = (uint8_t*)malloc(BLOCK_BYTES);
    if (!blk) die("malloc blk");

    float Vraw[FFT_SIZE];

    const int half = FFT_SIZE / 2;
    const float TWO_PI = 2.0f * (float)M_PI;

    // Precompute sweep list (MHz)
    const int nsteps = (int)ceil((LO_END_MHZ - LO_START_MHZ) / LO_STEP_MHZ);
    double *lo_list = (double*)malloc((size_t)nsteps * sizeof(double));
    if (!lo_list) die("malloc lo_list");
    for (int i = 0; i < nsteps; ++i)
        lo_list[i] = LO_START_MHZ + (double)i * LO_STEP_MHZ;

    heap_item_t topk[TOPK_PER_LO];

    uint64_t frame_idx = 0;
    float vmax = 1e-9f;
    float vmax_next = 1e-9f;
    
    while (!c->quit)
    {
        uint64_t t_frame0 = now_ns();

        uint64_t ns_read = 0, ns_lo = 0, ns_fft = 0, ns_proc = 0;
        uint32_t blocks = 0, points = 0;
        float activity_sum = 0.0f;

        read_telem_t rt = (read_telem_t){0};

#if ENABLE_DECAY
        pthread_mutex_lock(&c->mtx);
        memcpy(c->back, c->front, (size_t)CANVAS_W * CANVAS_H * 3 * sizeof(uint16_t));
        pthread_mutex_unlock(&c->mtx);
        decay_accum(c->back);
#else
        memset(c->back, 0, (size_t)CANVAS_W * CANVAS_H * 3 * sizeof(uint16_t));
#endif

        // Pipeline: program LO0 before reading first block
        double lo_curr = lo_list[0];
        uint64_t t0 = now_ns();
        (void)program_set_freq(c->fd, lo_curr);
        ns_lo += (now_ns() - t0);
    
        vmax_next = 1e-9f;
        for (int idx = 0; idx < nsteps && !c->quit; ++idx)
        {
            lo_curr = lo_list[idx];
            double lo_next = (idx + 1 < nsteps) ? lo_list[idx + 1] : lo_list[0];

            // Acquire one CSI block
            uint64_t tr0 = now_ns();
            ring_wait_and_get_one_block_fast(c->fd, c->ring, c->ring_size, blk, &rt);
            ns_read += (now_ns() - tr0);

            // Program next LO (pipelined)
            uint64_t tl0 = now_ns();
            (void)program_set_freq(c->fd, lo_next);
            ns_lo += (now_ns() - tl0);

            blocks++;

            // FFT each channel
            uint64_t tf0 = now_ns();
            for (int ch = 0; ch < CHANNELS; ++ch)
            {
                cs8_to_fftw_ch((const int8_t*)blk, ch, c->fft_in);
                fftwf_execute(c->plan[ch]);
            }
            ns_fft += (now_ns() - tf0);

            // Processing
            uint64_t tp0 = now_ns();

            // Pass 1: compute Vraw
            for (int k = 0; k < FFT_SIZE; ++k)
            {
                int i = (k + half) % FFT_SIZE;

                float re0 = c->fft_out[0][i][0], im0 = c->fft_out[0][i][1];
                float re1 = c->fft_out[1][i][0], im1 = c->fft_out[1][i][1];
                float re2 = c->fft_out[2][i][0], im2 = c->fft_out[2][i][1];
                float re3 = c->fft_out[3][i][0], im3 = c->fft_out[3][i][1];

                float s0 = re0*re0 + im0*im0;
                float s1 = re1*re1 + im1*im1;
                float s2 = re2*re2 + im2*im2;
                float s3 = re3*re3 + im3*im3;

                float v = logf(1.0f + (s0 + s1 + s2 + s3));
                Vraw[k] = v;
                if (v > vmax_next) vmax_next = v;
            }

	// Pass 2: Cell-Averaging CFAR + Top-K Select
            int hsz = 0;
            
            // CFAR Tuning Parameters
            const int CFAR_WIN = 32;        // Half-width of the noise estimation window
            const int CFAR_GUARD = 4;       // Guard cells around the peak to exclude signal energy
            const float CFAR_THRESH = 1.2f; // Local SNR requirement (1.2 in log-scale is ~5.2 dB)

            const int num_noise_cells = (CFAR_WIN - CFAR_GUARD) * 2;
            
            // Prime the sliding window sum
            float window_sum = 0.0f;
            for (int k = 0; k < CFAR_WIN * 2 + 1; ++k) {
                window_sum += Vraw[k];
            }

            // Sweep through the baseband (ignoring the extreme outer edges to prevent wrap-around)
            for (int k = CFAR_WIN; k < FFT_SIZE - CFAR_WIN; ++k)
            {
                // Fast sliding window update: add leading edge, subtract trailing edge
                if (k > CFAR_WIN) {
                    window_sum += Vraw[k + CFAR_WIN] - Vraw[k - CFAR_WIN - 1];
                }

                // Sum the guard cells (to exclude the peak's own energy from the noise estimate)
                float guard_sum = 0.0f;
                for (int g = -CFAR_GUARD; g <= CFAR_GUARD; ++g) {
                    guard_sum += Vraw[k + g];
                }

                // Calculate the average local noise floor
                float noise_floor = (window_sum - guard_sum) / (float)num_noise_cells;

                // GATE: Only push to Top-K if the signal strongly exceeds the local noise floor
                // Because Vraw is logarithmic (log(1+S)), subtraction is equivalent to an SNR ratio!
                if (Vraw[k] > noise_floor + CFAR_THRESH) {
                    hsz = heap_push_topk(topk, hsz, TOPK_PER_LO, Vraw[k], k);
                }
            }
            // ---------------------------------------------------------
            // PRECOMPUTE CONSTANTS FOR THIS LO BLOCK
            // ---------------------------------------------------------
            // 1. Color: Hue is roughly constant for the 18MHz block.
            //    Calculate base RGB once, scale by 'v' in loop.
            float r_base, g_base, b_base;
            {
                float hue = (float)((lo_curr - LO_START_MHZ) / (LO_END_MHZ - LO_START_MHZ));
                // Clamp hue strictly or hsv_to_rgb might wrap oddly if slightly <0
                if(hue < 0.0f) hue = 0.0f; 
                if(hue > 1.0f) hue = 1.0f;
                hsv_to_rgb(hue, 1.0f, 1.0f, &r_base, &g_base, &b_base);
            }

            // 2. Geometry: Wavenumber k is roughly constant.
            //    Calculate inverse scaling factor once.
            float inv_norm_scale;
            {
                float rf_center_hz = (float)(lo_curr * 1.0e6); // Center of block
                float wave_k = (TWO_PI * rf_center_hz) / SPEED_OF_LIGHT;
                // Use ANT_DIST_VERT for both axes to achieve the anamorphic stretch
                float scale = wave_k * ANT_DIST_VERT; 
                inv_norm_scale = 0.5f / scale; // Includes the 0.5 factor for (0.5 + 0.5*sin)
            }
            // ---------------------------------------------------------

            // Stamp selected bins
            for (int t = 0; t < hsz; ++t)
            {
                int k = topk[t].k;
                int i = (k + half) % FFT_SIZE;

                float v = Vraw[k] / vmax;
                // Branchless clamp for v (speedup)
                if (v < 0.0f) v = 0.0f;
                if (v > 1.0f) v = 1.0f;

                float re0 = c->fft_out[0][i][0], im0 = c->fft_out[0][i][1];
                float re1 = c->fft_out[1][i][0], im1 = c->fft_out[1][i][1];
                float re2 = c->fft_out[2][i][0], im2 = c->fft_out[2][i][1];
                float re3 = c->fft_out[3][i][0], im3 = c->fft_out[3][i][1];

                // A = arg(X0 * conj(X2)) - Vertical
                float creA = re0*re2 + im0*im2;
                float cimA = im0*re2 - re0*im2;
                float A = -atan2f(cimA, creA); // Bottleneck 1

                // Fast Calibration Wrap (branch prediction friendly)
                A -= CALIB_PHASE_V;
                if (A <= -M_PI) A += TWO_PI;
                else if (A > M_PI) A -= TWO_PI;

                // B = arg(X1 * conj(X3)) - Horizontal
                float creB = re1*re3 + im1*im3;
                float cimB = im1*re3 - re1*im3;
                float B = -atan2f(cimB, creB); // Bottleneck 2

                B -= CALIB_PHASE_H;
                if (B <= -M_PI) B += TWO_PI;
                else if (B > M_PI) B -= TWO_PI;

                // Fast Geometry Mapping (Multiplication instead of Division)
                // ty = 0.5 + (A * inv_norm_scale)
                float ty = 0.5f + A * inv_norm_scale;
                float tx = 0.5f + B * inv_norm_scale;

                if (c->mirror_display) {
                        tx = 1.0f - tx; // Simple Left-Right Mirror
                }

                // Clamp coordinates
                if (tx < 0.0f) tx = 0.0f; else if (tx > 1.0f) tx = 1.0f;
                if (ty < 0.0f) ty = 0.0f; else if (ty > 1.0f) ty = 1.0f;

                int x = (int)(tx * (float)(CANVAS_W - 1));
                int y = (int)(ty * (float)(CANVAS_H - 1));
                y = (CANVAS_H - 1) - y;
                
                // Fast Color Scale
                float str = POINT_GAIN * v;
                stamp_pixel_add(c->back, x, y, r_base, g_base, b_base, str);

                points++;
                activity_sum += v;
            }

            vmax = vmax_next;
            ns_proc += (now_ns() - tp0);
        }

        // Publish
        pthread_mutex_lock(&c->mtx);
        uint16_t *tmp = c->front;
        c->front = c->back;
        c->back  = tmp;

        uint64_t t_frame1 = now_ns();
        double t_frame_ms = ns_to_ms(t_frame1 - t_frame0);
        double fps = (t_frame_ms > 1e-9) ? (1000.0 / t_frame_ms) : 0.0;

        c->telem.frame_idx = frame_idx;
        c->telem.t_read_ms  = ns_to_ms(ns_read);
        c->telem.t_lo_ms    = ns_to_ms(ns_lo);
        c->telem.t_fft_ms   = ns_to_ms(ns_fft);
        c->telem.t_proc_ms  = ns_to_ms(ns_proc);
        c->telem.t_frame_ms = t_frame_ms;
        c->telem.blocks = blocks;
        c->telem.points = points;
        
        float denom = (float)nsteps * (float)TOPK_PER_LO;
        float act = (denom > 1e-9f) ? (activity_sum / denom) : 0.0f;
        c->telem.rf_activity = clampf(act, 0.0f, 1.0f);
        c->telem.fps = fps;

        pthread_mutex_unlock(&c->mtx);

        frame_idx++;

        if ((frame_idx % TELEMETRY_PRINT_EVERY_N_FRAMES) == 0)
        {
            fprintf(stdout,
                "[frame %llu] total=%.1f ms (%.2f fps)  points=%u  rf=%.2f\n",
                (unsigned long long)frame_idx,
                c->telem.t_frame_ms, c->telem.fps,
                c->telem.points, c->telem.rf_activity);
            fflush(stdout);
        }
    }

    free(lo_list);
    free(blk);
    return NULL;
}
// ------------------------------------------------------------
// Camera underlay (Raspberry Pi 5 + OV5647)
// ------------------------------------------------------------
//
// Implementation approach:
//   - Spawn libcamera-vid and read raw YUV420 (I420) frames from stdout.
//   - Convert to ARGB8888 in a background thread.
//   - Render the camera texture first, then alpha-blend the scatter texture on top.
//
// Notes:
//   - Requires libcamera-vid to be installed and the camera enabled.
//   - If libcamera-vid is unavailable, run without --camera.
//
// Example:
//   ./csi_sweep --camera
//   ./csi_sweep --camera --cam_w 640 --cam_h 480 --cam_fps 30
//

typedef struct {
    int enable;

    int w;
    int h;
    int fps;

    /* Optional override command. If NULL, a default libcamera-vid command is used. */
    const char *cmd_override;

    FILE *pipe;

    pthread_t th;
    pthread_mutex_t mtx;
    volatile int quit;

    /* Latest frame (ARGB8888) */
    uint32_t *argb;
    size_t argb_bytes;

    volatile int have_frame;
} cam_t;

static inline uint8_t clip_u8(int x)
{
    if (x < 0) return 0;
    if (x > 255) return 255;
    return (uint8_t)x;
}

/* Convert I420 (Y plane, then U plane, then V plane) to ARGB8888. */
static void i420_to_argb(const uint8_t *yuv, int w, int h, uint32_t *dst_argb)
{
    const uint8_t *Y = yuv;
    const uint8_t *U = Y + (size_t)w * h;
    const uint8_t *V = U + (size_t)(w * h) / 4;

    for (int yy = 0; yy < h; ++yy)
    {
        const uint8_t *yrow = Y + (size_t)yy * w;
        const uint8_t *urow = U + (size_t)(yy / 2) * (w / 2);
        const uint8_t *vrow = V + (size_t)(yy / 2) * (w / 2);

        for (int xx = 0; xx < w; ++xx)
        {
            int y = (int)yrow[xx];
            int u = (int)urow[xx / 2] - 128;
            int v = (int)vrow[xx / 2] - 128;

            /* ITU-R BT.601 full-range-ish integer approximation */
            int c = y - 16;
            int d = u;
            int e = v;

            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;

            uint32_t R = clip_u8(r);
            uint32_t G = clip_u8(g);
            uint32_t B = clip_u8(b);

            dst_argb[(size_t)yy * w + (size_t)xx] = 0xFF000000u | (R<<16) | (G<<8) | B;
        }
    }
}

/* Read exactly n bytes from f (blocking). Return 0 on success, -1 on EOF/error. */
static int fread_exact(FILE *f, void *buf, size_t n)
{
    uint8_t *p = (uint8_t*)buf;
    size_t got = 0;
    while (got < n)
    {
        size_t r = fread(p + got, 1, n - got, f);
        if (r == 0)
        {
            if (feof(f)) return -1;
            if (ferror(f)) return -1;
        }
        got += r;
    }
    return 0;
}

static void *camera_thread(void *arg)
{
    cam_t *cam = (cam_t*)arg;

    const size_t frame_bytes = (size_t)cam->w * cam->h * 3 / 2;
    uint8_t *yuv = (uint8_t*)malloc(frame_bytes);
    if (!yuv) return NULL;

    char cmd[512];
    if (cam->cmd_override && cam->cmd_override[0])
    {
        snprintf(cmd, sizeof(cmd), "%s", cam->cmd_override);
    }
    else
    {
        /* Raw I420 frames to stdout, no preview window. */
        snprintf(cmd, sizeof(cmd),
                 "rpicam-vid -n --timeout 0 --width %d --height %d --framerate %d --codec yuv420 -o -",
                 cam->w, cam->h, cam->fps);
    }

    cam->pipe = popen(cmd, "r");
    if (!cam->pipe)
    {
        fprintf(stderr, "camera: popen failed for cmd: %s\n", cmd);
        free(yuv);
        return NULL;
    }

    while (!cam->quit)
    {
        if (fread_exact(cam->pipe, yuv, frame_bytes) < 0)
            break;

        pthread_mutex_lock(&cam->mtx);
        i420_to_argb(yuv, cam->w, cam->h, cam->argb);
        cam->have_frame = 1;
        pthread_mutex_unlock(&cam->mtx);
    }

    if (cam->pipe) pclose(cam->pipe);
    cam->pipe = NULL;
    free(yuv);
    return NULL;
}

static void camera_init_defaults(cam_t *cam)
{
    memset(cam, 0, sizeof(*cam));
    cam->enable = 0;
    cam->w = 640;
    cam->h = 480;
    cam->fps = 30;
}

static int camera_start(cam_t *cam)
{
    if (!cam->enable) return 0;

    pthread_mutex_init(&cam->mtx, NULL);

    cam->argb_bytes = (size_t)cam->w * cam->h * sizeof(uint32_t);
    cam->argb = (uint32_t*)malloc(cam->argb_bytes);
    if (!cam->argb)
    {
        fprintf(stderr, "camera: malloc argb failed\n");
        return -1;
    }
    memset(cam->argb, 0, cam->argb_bytes);

    cam->quit = 0;
    cam->have_frame = 0;

    if (pthread_create(&cam->th, NULL, camera_thread, cam) != 0)
    {
        fprintf(stderr, "camera: pthread_create failed\n");
        free(cam->argb);
        cam->argb = NULL;
        return -1;
    }
    return 0;
}

static void camera_stop(cam_t *cam)
{
    if (!cam->enable) return;

    cam->quit = 1;
    pthread_join(cam->th, NULL);

    pthread_mutex_destroy(&cam->mtx);

    free(cam->argb);
    cam->argb = NULL;
}


static void draw_ui_buttons(SDL_Renderer *ren, int win_w, int win_h, 
                            float current_gain, int is_swapped)
{
    int x = win_w - BTN_SIZE - BTN_MARGIN;
    int y_start = (win_h - (3 * BTN_SIZE + 2 * BTN_MARGIN)) / 2; // Vertically centered

    SDL_Rect rects[3];
    for(int i=0; i<3; i++) {
        rects[i].x = x;
        rects[i].y = y_start + i * (BTN_SIZE + BTN_MARGIN);
        rects[i].w = BTN_SIZE;
        rects[i].h = BTN_SIZE;
        
        // Background: Semi-transparent gray
        SDL_SetRenderDrawColor(ren, 50, 50, 50, 200);
        SDL_RenderFillRect(ren, &rects[i]);
        
        // Border: White
        SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
        SDL_RenderDrawRect(ren, &rects[i]);
    }

    // Draw Symbols (Simple lines)
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    int cx = x + BTN_SIZE/2;

    // 1. UP Arrow
    int y1 = rects[0].y + BTN_SIZE/2;
    SDL_RenderDrawLine(ren, cx, rects[0].y + 5, cx - 15, y1 + 5);
    SDL_RenderDrawLine(ren, cx, rects[0].y + 5, cx + 15, y1 + 5);

    // 2. DOWN Arrow
    int y2 = rects[1].y + BTN_SIZE/3;
    SDL_RenderDrawLine(ren, cx, rects[1].y + BTN_SIZE - 25, cx - 15, y2 - 5);
    SDL_RenderDrawLine(ren, cx, rects[1].y + BTN_SIZE - 25, cx + 15, y2 - 5);

    // 3. SWAP (Arrows <->)
    int y3 = rects[2].y + BTN_SIZE/2;
    SDL_RenderDrawLine(ren, x + 15, y3, x + BTN_SIZE - 15, y3); // Horizontal
    SDL_RenderDrawLine(ren, x + 15, y3, x + 25, y3 - 15);       // Left tip
    SDL_RenderDrawLine(ren, x + 15, y3, x + 25, y3 + 15);
    SDL_RenderDrawLine(ren, x + BTN_SIZE - 15, y3, x + BTN_SIZE - 25, y3 - 15); // Right tip
    SDL_RenderDrawLine(ren, x + BTN_SIZE - 15, y3, x + BTN_SIZE - 25, y3 + 15);
    
}

// ------------------------------------------------------------
// Main / SDL
// ------------------------------------------------------------

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [--camera] [--cam_w W] [--cam_h H] [--cam_fps FPS] [--cam_cmd 'CMD']\n"
        "  --camera           Enable camera underlay via libcamera-vid\n"
        "  --cam_w/--cam_h    Camera capture resolution (default 640x480)\n"
        "  --cam_fps          Camera framerate (default 30)\n"
        "  --cam_cmd          Override capture command (must output raw I420 frames)\n",
        argv0);
}

int main(int argc, char **argv)
{
    cam_t cam;
    camera_init_defaults(&cam);

    for (int ai = 1; ai < argc; ++ai)
    {
        if (!strcmp(argv[ai], "--camera"))
        {
            cam.enable = 1;
        }
        else if (!strcmp(argv[ai], "--cam_w") && ai + 1 < argc)
        {
            cam.w = atoi(argv[++ai]);
        }
        else if (!strcmp(argv[ai], "--cam_h") && ai + 1 < argc)
        {
            cam.h = atoi(argv[++ai]);
        }
        else if (!strcmp(argv[ai], "--cam_fps") && ai + 1 < argc)
        {
            cam.fps = atoi(argv[++ai]);
        }
        else if (!strcmp(argv[ai], "--cam_cmd") && ai + 1 < argc)
        {
            cam.cmd_override = argv[++ai];
            cam.enable = 1;
        }
        else if (!strcmp(argv[ai], "--help") || !strcmp(argv[ai], "-h"))
        {
            usage(argv[0]);
            return 0;
        }
        else
        {
            fprintf(stderr, "Unknown arg: %s\n", argv[ai]);
            usage(argv[0]);
            return 1;
        }
    }


    int fd = open(DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (fd < 0) die("open");

    ioctl(fd, CSI_IOC_JTAG_SETUP);

    struct csi_ring_info ri;
    if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0) die("CSI_IOC_GET_RING_INFO");
    if (!ri.ring_size) { fprintf(stderr, "ring_size=0\n"); return 1; }

    long page = sysconf(_SC_PAGESIZE);
    size_t map_len = (size_t)((ri.ring_size + page - 1) & ~((uint64_t)page - 1));

    void *ring = mmap(NULL, map_len, PROT_READ, MAP_SHARED, fd, 0);
    if (ring == MAP_FAILED) die("mmap");

    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    // Nearest-neighbor scaling: makes CANVAS pixels appear as big blocks in the window.
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");

    SDL_Window *win = SDL_CreateWindow("Telemetry: starting...",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_WIDTH, WIN_HEIGHT, SDL_WINDOW_RESIZABLE);
    if (!win) die("SDL_CreateWindow");

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (!ren) die("SDL_CreateRenderer");

    /* Camera underlay texture (optional) */
    SDL_Texture *cam_tex = NULL;
    if (cam.enable)
    {
        if (camera_start(&cam) != 0)
        {
            fprintf(stderr, "camera: failed to start; continuing without camera underlay\n");
            cam.enable = 0;
        }
        else
        {
            cam_tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                SDL_TEXTUREACCESS_STREAMING, cam.w, cam.h);
            if (!cam_tex)
            {
                fprintf(stderr, "camera: SDL_CreateTexture failed: %s\n", SDL_GetError());
                camera_stop(&cam);
                cam.enable = 0;
            }
        }
    }


    // Texture is logical canvas size; we scale it to the window each frame.
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, CANVAS_W, CANVAS_H);
    if (!tex) die("SDL_CreateTexture");
    SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_ADD); /* additive over camera underlay */

    uint16_t *front = (uint16_t*)calloc((size_t)CANVAS_W * CANVAS_H * 3, sizeof(uint16_t));
    uint16_t *back  = (uint16_t*)calloc((size_t)CANVAS_W * CANVAS_H * 3, sizeof(uint16_t));
    uint32_t *pix   = (uint32_t*)malloc((size_t)CANVAS_W * CANVAS_H * sizeof(uint32_t));
    if (!front || !back || !pix) die("alloc buffers");

    fftwf_complex *in = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    if (!in) die("fftwf_malloc in");

    fftwf_complex *out[CHANNELS];
    fftwf_plan plan[CHANNELS];
    for (int ch = 0; ch < CHANNELS; ++ch)
    {
        out[ch] = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
        if (!out[ch]) die("fftwf_malloc out");
        plan[ch] = fftwf_plan_dft_1d(FFT_SIZE, in, out[ch], FFTW_FORWARD, FFTW_MEASURE);
        if (!plan[ch]) die("fftw plan");
    }

    ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.fd = fd;
    ctx.ring = ring;
    ctx.ring_size = ri.ring_size;
    ctx.fft_in = in;
    for (int ch = 0; ch < CHANNELS; ++ch) { ctx.fft_out[ch] = out[ch]; ctx.plan[ch] = plan[ch]; }
    ctx.front = front;
    ctx.back  = back;
    pthread_mutex_init(&ctx.mtx, NULL);

    pthread_t th;
    if (pthread_create(&th, NULL, worker, &ctx) != 0) die("pthread_create");

    bool quit = false;
    telemetry_t telem_last = (telemetry_t){0};

    /* Dynamic camera fade based on RF activity.
     * Goal: when RF activity spikes, dim the camera so scatter is more visible.
     */
    float rf_act_ema = 0.0f;
    float cam_gain_f = 200.0f; /* 0..255 */

    const float CAM_GAIN_BASE = 200.0f;  /* baseline brightness (try 160..220) */
    const float CAM_GAIN_MIN  = 70.0f;   /* minimum brightness under heavy RF */
    const float CAM_EMA_ALPHA = 0.10f;   /* RF activity smoothing (0.05..0.2) */
    const float GAIN_ALPHA    = 0.15f;   /* camera gain smoothing */
    const float FADE_GAMMA    = 1.2f;    /* nonlinear fade response */

    SDL_Rect dst = {0, 0, WIN_WIDTH, WIN_HEIGHT};

    ctx.mirror_display = 0;
    while (!quit)
    {
int win_w, win_h;
        SDL_GetRendererOutputSize(ren, &win_w, &win_h); // Get size for button placement

        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_QUIT) quit = true;
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) quit = true;

            // --- BUTTON CLICKS ---
            if (e.type == SDL_MOUSEBUTTONDOWN) {
                int mx = e.button.x;
                int my = e.button.y;
                
                // Re-calculate button positions to match hit test
                int bx = win_w - BTN_SIZE - BTN_MARGIN;
                int by_start = (win_h - (3 * BTN_SIZE + 2 * BTN_MARGIN)) / 2;
                
                // Check X bounds (same for all buttons)
                if (mx >= bx && mx <= bx + BTN_SIZE) {
                    // Button 1: UP
                    if (my >= by_start && my <= by_start + BTN_SIZE) {
                        adjust_rx_gain(fd, +2);
                    }
                    // Button 2: DOWN
                    else if (my >= by_start + BTN_SIZE + BTN_MARGIN && 
                             my <= by_start + 2*BTN_SIZE + BTN_MARGIN) {
                        adjust_rx_gain(fd, -2);                    }
                    // Button 3: SWAP
                    else if (my >= by_start + 2*(BTN_SIZE + BTN_MARGIN) && 
                             my <= by_start + 3*BTN_SIZE + 2*BTN_MARGIN) {
                        ctx.mirror_display = !ctx.mirror_display;
                        printf("Swap: %d\n", ctx.mirror_display);
                    }
                }
            }
        }

	// --- Dynamic sizing ---
        int cur_w, cur_h;
        SDL_GetRendererOutputSize(ren, &cur_w, &cur_h);
        SDL_Rect dst = { 0, 0, cur_w, cur_h };
        // ---------------------------

        pthread_mutex_lock(&ctx.mtx);
        accum_to_pixels(ctx.front, pix);
// --- seam visualization (wrap boundary) ---
// Force the wrap seams (x=W/2 and y=H/2) to light gray so they don't look like
// "missing data" black cross-hairs.
{
    const int sx = (CANVAS_W-1) / 2;
    const int sy = CANVAS_H / 2;
    const uint32_t gray = 0xFF303030u; // ARGB: light gray (tweak as desired)

    // Vertical seam
    for (int y = CANVAS_H/2 - 25; y < CANVAS_H/2+25; ++y)
        pix[(size_t)y * CANVAS_W + (size_t)sx] = gray;

    // Horizontal seam
    for (int x = CANVAS_W/2 - 25; x < CANVAS_W/2+25; ++x)
        pix[(size_t)sy * CANVAS_W + (size_t)x] = gray;
}




        telem_last = ctx.telem;
        pthread_mutex_unlock(&ctx.mtx);

        /* Update RF activity EMA and compute a dimming gain for the camera.
         * telem_last.rf_activity is already normalized to [0,1].
         */
        rf_act_ema = (1.0f - CAM_EMA_ALPHA) * rf_act_ema + CAM_EMA_ALPHA * telem_last.rf_activity;
        rf_act_ema = clampf(rf_act_ema, 0.0f, 1.0f);

        float fade = powf(rf_act_ema, FADE_GAMMA);
        float target_gain = CAM_GAIN_BASE - (CAM_GAIN_BASE - CAM_GAIN_MIN) * fade;
        target_gain = clampf(target_gain, CAM_GAIN_MIN, 255.0f);
        cam_gain_f += GAIN_ALPHA * (target_gain - cam_gain_f);
        cam_gain_f = clampf(cam_gain_f, CAM_GAIN_MIN, 255.0f);

        char title[256];
        snprintf(title, sizeof(title),
                 "fps=%.2f total=%.0fms read=%.0f(gi=%.0f cp=%.0f cs=%.0f) LO=%.0f FFT=%.0f proc=%.0f blocks=%u points=%u  rf=%.2f cam_gain=%.0f  canvas=%dx%d window=%dx%d",
                 telem_last.fps,
                 telem_last.t_frame_ms,
                 telem_last.t_read_ms,
                 telem_last.t_read_getinfo_ms,
                 telem_last.t_read_copy_ms,
                 telem_last.t_read_consume_ms,
                 telem_last.t_lo_ms,
                 telem_last.t_fft_ms,
                 telem_last.t_proc_ms,
                 telem_last.blocks,
                 telem_last.points,
                 telem_last.rf_activity,
                 cam_gain_f,
                 CANVAS_W, CANVAS_H, WIN_WIDTH, WIN_HEIGHT);
        SDL_SetWindowTitle(win, title);

        SDL_UpdateTexture(tex, NULL, pix, CANVAS_W * (int)sizeof(uint32_t));
        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        /* Underlay: camera */
        if (cam.enable && cam_tex)
        {
            int have = 0;
            pthread_mutex_lock(&cam.mtx);
            have = cam.have_frame;
            if (have)
                SDL_UpdateTexture(cam_tex, NULL, cam.argb, cam.w * (int)sizeof(uint32_t));
            pthread_mutex_unlock(&cam.mtx);

            if (have)
            {
                /* Dim camera dynamically based on RF activity (scatter visibility). */
                uint8_t g = (uint8_t)cam_gain_f;
                SDL_SetTextureColorMod(cam_tex, g, g, g);
                SDL_RenderCopy(ren, cam_tex, NULL, &dst);
            }
        }

	SDL_Rect src;
        src.h = 200;           // The "points tall" user requested
        src.w = CANVAS_W;           // Keep aspect ratio 1:1 (square crop)
        src.x = (CANVAS_W - src.w) / 2; // Center horizontally
        src.y = (CANVAS_H - src.h) / 2; // Center vertically

        // 2. Render specifically this crop to the full window (dst)
        SDL_RenderCopy(ren, tex, &src, &dst);

	/* Overlay: UI Buttons */
        draw_ui_buttons(ren, win_w, win_h, ctx.point_gain, ctx.mirror_display);
        SDL_RenderPresent(ren);

        SDL_Delay(10);
    }

    ctx.quit = 1;
    pthread_join(th, NULL);

    pthread_mutex_destroy(&ctx.mtx);

    for (int ch = 0; ch < CHANNELS; ++ch)
    {
        fftwf_destroy_plan(plan[ch]);
        fftwf_free(out[ch]);
    }
    fftwf_free(in);

    free(front);
    free(back);
        if (cam_tex) SDL_DestroyTexture(cam_tex);
    camera_stop(&cam);

free(pix);

    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();

    munmap(ring, map_len);
    close(fd);

    return 0;
}
