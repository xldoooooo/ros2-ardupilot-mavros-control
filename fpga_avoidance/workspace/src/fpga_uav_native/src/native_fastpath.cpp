#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <cerrno>
#include <ctime>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#if defined(_WIN32)
#define FPGA_NATIVE_EXPORT extern "C" __declspec(dllexport)
#else
#define FPGA_NATIVE_EXPORT extern "C" __attribute__((visibility("default")))
#endif

namespace {
constexpr double EPS = 1.0e-9;

inline int flat_index(int z, int r, int c, int rows, int cols) {
    return (z * rows + r) * cols + c;
}

inline bool inside(int z, int r, int c, int layers, int rows, int cols) {
    return z >= 0 && z < layers && r >= 0 && r < rows && c >= 0 && c < cols;
}

inline double diag_heuristic(int z, int r, int c, int gz, int gr, int gc) {
    int a = std::abs(z - gz);
    int b = std::abs(r - gr);
    int d = std::abs(c - gc);
    const int lo = std::min(a, std::min(b, d));
    const int hi = std::max(a, std::max(b, d));
    const int mid = a + b + d - lo - hi;
    return static_cast<double>(hi - mid)
        + static_cast<double>(mid - lo) * std::sqrt(2.0)
        + static_cast<double>(lo) * std::sqrt(3.0);
}

struct OpenNode {
    double f;
    double g;
    int idx;
};

struct OpenCmp {
    bool operator()(const OpenNode& a, const OpenNode& b) const {
        if (a.f != b.f) return a.f > b.f;
        if (a.g != b.g) return a.g > b.g;
        return a.idx > b.idx;
    }
};

struct AStarWorkspace {
    std::vector<double> g;
    std::vector<int32_t> parent;
    std::vector<uint32_t> seen;
    std::vector<uint32_t> closed;
    uint32_t generation = 0;

    void ensure(std::size_t n) {
        if (g.size() == n) return;
        g.resize(n);
        parent.resize(n);
        seen.assign(n, 0);
        closed.assign(n, 0);
        generation = 0;
    }

    uint32_t next_generation() {
        ++generation;
        if (generation == 0) {
            std::fill(seen.begin(), seen.end(), 0);
            std::fill(closed.begin(), closed.end(), 0);
            generation = 1;
        }
        return generation;
    }
};

thread_local AStarWorkspace astar_ws;

inline long round_ties_to_even(double x) {
    // nearbyint obeys FE_TONEAREST by default, matching numpy.rint's ties-to-even
    // semantics on the supported Linux/x86 and Linux/ARM targets.
    return static_cast<long>(std::nearbyint(x));
}

inline void fnv1a_mix_u64_le(uint64_t& h, uint64_t value) {
    constexpr uint64_t prime = 1099511628211ULL;
    for (int k=0;k<8;++k) {
        h ^= static_cast<uint8_t>((value >> (8*k)) & 0xFFu);
        h *= prime;
    }
}

inline int bit_length_u64(uint64_t value) {
    int bits = 0;
    while (value) { ++bits; value >>= 1; }
    return bits;
}

inline int64_t round_shift_signed_i64(int64_t value, int shift) {
    if (shift <= 0) return value;
    const int64_t bias = static_cast<int64_t>(1) << (shift - 1);
    return value >= 0 ? ((value + bias) >> shift) : -(((-value) + bias) >> shift);
}

inline void append_u16_le(std::vector<uint8_t>& out, uint16_t value) {
    out.push_back(static_cast<uint8_t>(value & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
}
inline void append_u32_le(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>(value & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
}
inline void append_i32_le(std::vector<uint8_t>& out, int32_t value) {
    append_u32_le(out, static_cast<uint32_t>(value));
}
inline void append_i16_le(std::vector<uint8_t>& out, int16_t value) {
    append_u16_le(out, static_cast<uint16_t>(value));
}

} // namespace

FPGA_NATIVE_EXPORT int fpga_native_fastpath_version() {
    return 3301;
}

// Host Opt V5.2: build the exact existing Active-N UAV wire payload in C++.
// mode=0 -> FULL compact payload (BFP8/BFP16/sparse-BFP16/Q9.22 selection),
// mode=1 -> H-only Q9.22 payload for UAV-H-CACHE.  The firmware protocol and
// byte layout are unchanged; this only removes Python/NumPy packing overhead.
FPGA_NATIVE_EXPORT int fpga_uav_compact_payload_active(
    const double* h,
    const double* j,
    int n,
    int mode,
    uint8_t* out,
    int out_capacity
) {
    if (!h || !j || !out || n < 2 || n > 256 || out_capacity <= 0) return -1;
    constexpr double SCALE = 4194304.0;
    constexpr int64_t QMIN = static_cast<int64_t>(-2147483647LL) - 1LL;
    constexpr int64_t QMAX = static_cast<int64_t>(2147483647LL);
    constexpr uint8_t FORMAT_Q9_22 = 1;
    constexpr uint8_t FORMAT_UAV_BFP8_J = 4;
    constexpr uint8_t FORMAT_UAV_BFP16_J = 5;
    constexpr uint8_t FORMAT_UAV_Q9_22 = 6;
    constexpr uint8_t FORMAT_UAV_SPARSE_BFP16_J = 7;
    constexpr uint8_t FLAG_UPPER_TRIANGLE = 1;

    const int upper_count = n * (n - 1) / 2;
    std::vector<int64_t> hq(static_cast<std::size_t>(n));
    std::vector<int64_t> raw(static_cast<std::size_t>(upper_count));
    for (int i = 0; i < n; ++i) {
        const int64_t q = static_cast<int64_t>(round_ties_to_even(h[i] * SCALE));
        if (q < QMIN || q > QMAX) return -2;
        hq[static_cast<std::size_t>(i)] = q;
    }
    int pos = 0;
    for (int i = 0; i < n; ++i) {
        for (int k = i + 1; k < n; ++k) {
            const int64_t q = static_cast<int64_t>(round_ties_to_even(j[static_cast<std::size_t>(i) * n + k] * SCALE));
            if (q < QMIN || q > QMAX) return -2;
            raw[static_cast<std::size_t>(pos++)] = q;
        }
    }
    const uint8_t size_hw = 0; // encode_size_code(256)
    const uint8_t size_n = static_cast<uint8_t>(n == 256 ? 0 : n);

    auto append_header = [&](std::vector<uint8_t>& bytes, uint8_t fmt, uint8_t flags) {
        bytes.push_back(size_hw); bytes.push_back(size_n); bytes.push_back(fmt); bytes.push_back(flags);
    };
    auto append_h = [&](std::vector<uint8_t>& bytes) {
        for (const int64_t q : hq) append_i32_le(bytes, static_cast<int32_t>(q));
    };

    if (mode == 1) {
        std::vector<uint8_t> bytes;
        bytes.reserve(static_cast<std::size_t>(4 + 4 * n));
        append_header(bytes, FORMAT_Q9_22, 0);
        append_h(bytes);
        if (static_cast<int>(bytes.size()) > out_capacity) return -3;
        std::memcpy(out, bytes.data(), bytes.size());
        return static_cast<int>(bytes.size());
    }
    if (mode != 0) return -4;

    struct BfpCandidate {
        bool valid = false;
        int bits = 0;
        int shift = 0;
        std::vector<int64_t> encoded;
        std::vector<uint8_t> bytes;
    };
    auto make_bfp = [&](int bits) -> BfpCandidate {
        BfpCandidate c; c.bits = bits;
        const int64_t limit = (static_cast<int64_t>(1) << (bits - 1)) - 1;
        uint64_t max_abs = 0;
        for (const int64_t value : raw) {
            const uint64_t a = value >= 0 ? static_cast<uint64_t>(value) : static_cast<uint64_t>(-value);
            max_abs = std::max(max_abs, a);
        }
        int shift = std::max(0, bit_length_u64(max_abs) - (bits - 1));
        c.encoded.resize(raw.size());
        auto encode_all = [&]() -> int64_t {
            int64_t m = 0;
            for (std::size_t i = 0; i < raw.size(); ++i) {
                const int64_t e = round_shift_signed_i64(raw[i], shift);
                c.encoded[i] = e;
                const int64_t a = e >= 0 ? e : -e;
                m = std::max(m, a);
            }
            return m;
        };
        int64_t encoded_max = encode_all();
        while (shift < 31 && encoded_max > limit) { ++shift; encoded_max = encode_all(); }
        if (encoded_max > limit) return c;
        int64_t max_error = 0;
        for (std::size_t i = 0; i < raw.size(); ++i) {
            if (raw[i] != 0 && c.encoded[i] == 0) return c;
            const int64_t restored = c.encoded[i] << shift;
            const int64_t err = raw[i] >= restored ? raw[i] - restored : restored - raw[i];
            max_error = std::max(max_error, err);
        }
        const double allowed = bits == 8 ? 1.0e-5 : 5.0e-5;
        if (static_cast<double>(max_error) / SCALE > allowed) return c;
        c.shift = shift;
        const uint8_t fmt = bits == 8 ? FORMAT_UAV_BFP8_J : FORMAT_UAV_BFP16_J;
        const uint8_t flags = static_cast<uint8_t>(FLAG_UPPER_TRIANGLE | ((shift & 0x1F) << 1));
        c.bytes.reserve(static_cast<std::size_t>(4 + 4 * n + (bits == 8 ? upper_count + 3 : 2 * (upper_count + 1))));
        append_header(c.bytes, fmt, flags); append_h(c.bytes);
        const int group = bits == 8 ? 4 : 2;
        const int padded = ((upper_count + group - 1) / group) * group;
        if (bits == 8) {
            for (int i = 0; i < padded; ++i) {
                const int64_t e = i < upper_count ? c.encoded[static_cast<std::size_t>(i)] : 0;
                c.bytes.push_back(static_cast<uint8_t>(static_cast<int8_t>(e)));
            }
        } else {
            for (int i = 0; i < padded; ++i) {
                const int64_t e = i < upper_count ? c.encoded[static_cast<std::size_t>(i)] : 0;
                append_i16_le(c.bytes, static_cast<int16_t>(e));
            }
        }
        c.valid = true;
        return c;
    };

    BfpCandidate b8 = make_bfp(8);
    BfpCandidate b16 = make_bfp(16);
    std::vector<uint8_t> sparse16;
    if (b16.valid) {
        std::vector<int> nz;
        nz.reserve(b16.encoded.size());
        for (int i = 0; i < upper_count; ++i)
            if (b16.encoded[static_cast<std::size_t>(i)] != 0) nz.push_back(i);
        if (nz.size() <= 0xFFFFu) {
            sparse16.reserve(static_cast<std::size_t>(4 + 4 * n + 4 + 4 * nz.size()));
            const uint8_t flags = static_cast<uint8_t>(FLAG_UPPER_TRIANGLE | ((b16.shift & 0x1F) << 1));
            append_header(sparse16, FORMAT_UAV_SPARSE_BFP16_J, flags); append_h(sparse16);
            append_u16_le(sparse16, static_cast<uint16_t>(nz.size())); append_u16_le(sparse16, 0);
            for (const int idx : nz) {
                const uint16_t value = static_cast<uint16_t>(static_cast<int16_t>(b16.encoded[static_cast<std::size_t>(idx)]));
                const uint32_t record = static_cast<uint32_t>(idx) | (static_cast<uint32_t>(value) << 16);
                append_u32_le(sparse16, record);
            }
            if (sparse16.size() >= b16.bytes.size()) sparse16.clear();
        }
    }

    const std::vector<uint8_t>* best = nullptr;
    if (b8.valid) best = &b8.bytes;
    if (b16.valid && (!best || b16.bytes.size() < best->size())) best = &b16.bytes;
    if (!sparse16.empty() && (!best || sparse16.size() < best->size())) best = &sparse16;

    std::vector<uint8_t> q9;
    if (!best) {
        q9.reserve(static_cast<std::size_t>(4 + 4 * n + 4 * upper_count));
        append_header(q9, FORMAT_UAV_Q9_22, FLAG_UPPER_TRIANGLE); append_h(q9);
        for (const int64_t q : raw) append_i32_le(q9, static_cast<int32_t>(q));
        best = &q9;
    }
    if (static_cast<int>(best->size()) > out_capacity) return -3;
    std::memcpy(out, best->data(), best->size());
    return static_cast<int>(best->size());
}



FPGA_NATIVE_EXPORT int fpga_udp_recv_native(
    int fd,
    void* out_buf,
    int out_cap,
    int spin_us,
    int poll_us,
    int deadline_us,
    int* out_phase,
    int* out_elapsed_us,
    int* out_spin_elapsed_us
) {
#if defined(_WIN32)
    (void)fd; (void)out_buf; (void)out_cap; (void)spin_us; (void)poll_us;
    (void)deadline_us; (void)out_phase; (void)out_elapsed_us; (void)out_spin_elapsed_us;
    return -1000;
#else
    if (fd < 0 || !out_buf || out_cap <= 0 || deadline_us <= 0) return -EINVAL;
    spin_us = std::max(0, spin_us);
    poll_us = std::max(50, poll_us);
    deadline_us = std::max(1, deadline_us);
    if (out_phase) *out_phase = 0;
    if (out_elapsed_us) *out_elapsed_us = 0;
    if (out_spin_elapsed_us) *out_spin_elapsed_us = 0;

    auto now_us = []() -> int64_t {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<int64_t>(ts.tv_sec) * 1000000LL + static_cast<int64_t>(ts.tv_nsec / 1000);
    };

    const int64_t started = now_us();
    const int64_t spin_deadline = started + static_cast<int64_t>(spin_us);
    const int64_t absolute_deadline = started + static_cast<int64_t>(deadline_us);

    // Native userspace spin.  The whole loop executes outside the Python GIL
    // because ctypes.CDLL releases it around the call.  MSG_DONTWAIT avoids
    // changing the Python socket's timeout/blocking state.
    while (spin_us > 0 && now_us() < spin_deadline) {
        const ssize_t n = ::recv(fd, out_buf, static_cast<size_t>(out_cap), MSG_DONTWAIT);
        if (n >= 0) {
            const int64_t elapsed = now_us() - started;
            if (out_phase) *out_phase = 1;
            if (out_elapsed_us) *out_elapsed_us = static_cast<int>(std::min<int64_t>(elapsed, 0x7fffffff));
            if (out_spin_elapsed_us) *out_spin_elapsed_us = static_cast<int>(std::min<int64_t>(elapsed, 0x7fffffff));
            return static_cast<int>(n);
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return -errno;
    }
    if (out_spin_elapsed_us) {
        const int64_t used = std::max<int64_t>(0, std::min<int64_t>(now_us(), spin_deadline) - started);
        *out_spin_elapsed_us = static_cast<int>(std::min<int64_t>(used, 0x7fffffff));
    }

    // Blocking phase uses ppoll with the same short quantum as the Python path,
    // but avoids Python-level timeout-state changes and repeated interpreter
    // wakeups.  ppoll returns immediately when the datagram is ready.
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    while (true) {
        const int64_t now = now_us();
        if (now >= absolute_deadline) break;
        const int64_t remaining = absolute_deadline - now;
        const int64_t quantum = std::min<int64_t>(remaining, poll_us);
        timespec ts{};
        ts.tv_sec = static_cast<time_t>(quantum / 1000000LL);
        ts.tv_nsec = static_cast<long>((quantum % 1000000LL) * 1000LL);
        pfd.revents = 0;
        const int rc = ::ppoll(&pfd, 1, &ts, nullptr);
        if (rc < 0) {
            if (errno == EINTR) continue;
            return -errno;
        }
        if (rc == 0) continue;
        if (pfd.revents & (POLLIN | POLLPRI)) {
            const ssize_t n = ::recv(fd, out_buf, static_cast<size_t>(out_cap), MSG_DONTWAIT);
            if (n >= 0) {
                const int64_t elapsed = now_us() - started;
                if (out_phase) *out_phase = 2;
                if (out_elapsed_us) *out_elapsed_us = static_cast<int>(std::min<int64_t>(elapsed, 0x7fffffff));
                return static_cast<int>(n);
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            return -errno;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return -EIO;
    }
    if (out_elapsed_us) {
        const int64_t elapsed = now_us() - started;
        *out_elapsed_us = static_cast<int>(std::min<int64_t>(elapsed, 0x7fffffff));
    }
    return 0;
#endif
}

FPGA_NATIVE_EXPORT int fpga_voxelize_inflate3d(
    const double* points_xyz,
    int64_t point_count,
    int layers,
    int rows,
    int cols,
    double x_min,
    double y_min,
    double z_min,
    double resolution_xy,
    double resolution_z,
    double radius_m,
    uint8_t* raw_out,
    uint8_t* inflated_out
) {
    if (!raw_out || !inflated_out || layers <= 0 || rows <= 0 || cols <= 0 ||
        resolution_xy <= 0.0 || resolution_z <= 0.0 || point_count < 0) {
        return -1;
    }
    const std::size_t total = static_cast<std::size_t>(layers) * rows * cols;
    std::memset(raw_out, 0, total * sizeof(uint8_t));
    std::memset(inflated_out, 0, total * sizeof(uint8_t));

    if (points_xyz && point_count > 0) {
        for (int64_t i = 0; i < point_count; ++i) {
            const double x = points_xyz[3 * i + 0];
            const double y = points_xyz[3 * i + 1];
            const double z = points_xyz[3 * i + 2];
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            const int c = static_cast<int>(std::floor((x - x_min) / resolution_xy));
            const int r = static_cast<int>(std::floor((y - y_min) / resolution_xy));
            const int l = static_cast<int>(std::floor((z - z_min) / resolution_z));
            if (inside(l, r, c, layers, rows, cols)) {
                raw_out[flat_index(l, r, c, rows, cols)] = 1;
            }
        }
    }

    if (radius_m <= 0.0) {
        std::memcpy(inflated_out, raw_out, total * sizeof(uint8_t));
        return 0;
    }

    const int max_dz = static_cast<int>(std::ceil(radius_m / resolution_z));
    const int max_dr = static_cast<int>(std::ceil(radius_m / resolution_xy));
    const double radius2 = radius_m * radius_m + 1.0e-15;
    std::vector<std::tuple<int,int,int>> offsets;
    offsets.reserve(static_cast<std::size_t>((2*max_dz+1) * (2*max_dr+1) * (2*max_dr+1)));
    for (int dz = -max_dz; dz <= max_dz; ++dz) {
        const double z2 = std::pow(dz * resolution_z, 2.0);
        for (int dr = -max_dr; dr <= max_dr; ++dr) {
            const double zr2 = z2 + std::pow(dr * resolution_xy, 2.0);
            for (int dc = -max_dr; dc <= max_dr; ++dc) {
                if (zr2 + std::pow(dc * resolution_xy, 2.0) <= radius2) {
                    offsets.emplace_back(dz, dr, dc);
                }
            }
        }
    }

    // Sparse-centre dilation: identical set semantics to the NumPy shifted-OR
    // implementation, but work scales with occupied voxels rather than grid volume.
    for (int z = 0; z < layers; ++z) {
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (!raw_out[flat_index(z, r, c, rows, cols)]) continue;
                for (const auto& off : offsets) {
                    int dz, dr, dc;
                    std::tie(dz, dr, dc) = off;
                    const int nz = z + dz;
                    const int nr = r + dr;
                    const int nc = c + dc;
                    if (inside(nz, nr, nc, layers, rows, cols)) {
                        inflated_out[flat_index(nz, nr, nc, rows, cols)] = 1;
                    }
                }
            }
        }
    }
    return 0;
}




FPGA_NATIVE_EXPORT int fpga_clearance_limited3d(
    const uint8_t* occupancy,
    int layers,
    int rows,
    int cols,
    int cap_voxels,
    float* out_clearance
) {
    // V2 hybrid exact capped L1 transform. Sparse maps are fastest when we
    // dilate only occupied centres; dense maps favour the separable 1-D
    // Manhattan transform. Both paths produce exactly the same capped
    // 6-neighbour distance field.
    if (!occupancy || !out_clearance || layers <= 0 || rows <= 0 || cols <= 0) return -1;
    const int cap = std::max(1, cap_voxels);
    const int total = layers * rows * cols;
    std::fill(out_clearance, out_clearance + total, static_cast<float>(cap));

    std::vector<int32_t> occupied;
    occupied.reserve(static_cast<std::size_t>(total / 8 + 1));
    for (int idx = 0; idx < total; ++idx) {
        if (occupancy[idx]) occupied.push_back(idx);
    }
    if (occupied.empty()) return 0;

    // Host Opt V5.5: exact production-cap specialization.  For cap==2 the
    // capped Manhattan field has only three relevant values: occupied=0,
    // direct 6-neighbour=1, and every cell at distance >=2 remains the
    // initialized value 2.  The older generic radius-2 dilation visited all
    // Manhattan offsets with d<=2 even though d==2 writes the value already
    // present in the output.  This path is therefore exactly equivalent but
    // performs only six neighbour relaxations per occupied voxel.
    if (cap == 1) {
        for (const int idx : occupied) out_clearance[idx] = 0.0f;
        return 0;
    }
    if (cap == 2) {
        const int plane = rows * cols;
        for (const int idx : occupied) {
            out_clearance[idx] = 0.0f;
            const int z = idx / plane;
            const int rem = idx - z * plane;
            const int r = rem / cols;
            const int c = rem - r * cols;
            auto mark1 = [out_clearance](int nidx) {
                if (out_clearance[nidx] > 1.0f) out_clearance[nidx] = 1.0f;
            };
            if (z > 0) mark1(idx - plane);
            if (z + 1 < layers) mark1(idx + plane);
            if (r > 0) mark1(idx - cols);
            if (r + 1 < rows) mark1(idx + cols);
            if (c > 0) mark1(idx - 1);
            if (c + 1 < cols) mark1(idx + 1);
        }
        // A neighbouring occupied cell may have been temporarily marked 1 by
        // a previous obstacle. Reassert exact occupied distance 0.
        for (const int idx : occupied) out_clearance[idx] = 0.0f;
        return 0;
    }

    // Obstacle-centred dilation wins for the sparse/moderate inflated clouds
    // typical of UAV local maps. Keep the threshold conservative; above it the
    // fixed-cost separable transform avoids obstacle_count * neighbourhood work.
    if (cap <= 8 && occupied.size() * 5u < static_cast<std::size_t>(total)) {
        struct Off { int dz, dr, dc, d; };
        std::vector<Off> offsets;
        offsets.reserve(static_cast<std::size_t>((2*cap+1)*(2*cap+1)*(2*cap+1)));
        for (int dz=-cap; dz<=cap; ++dz) {
            for (int dr=-cap; dr<=cap; ++dr) {
                for (int dc=-cap; dc<=cap; ++dc) {
                    const int d=std::abs(dz)+std::abs(dr)+std::abs(dc);
                    if (d<=cap) offsets.push_back({dz,dr,dc,d});
                }
            }
        }
        const int plane = rows * cols;
        for (const int idx : occupied) {
            const int z = idx / plane;
            const int rem = idx - z * plane;
            const int r = rem / cols;
            const int c = rem - r * cols;
            for (const Off& off: offsets) {
                const int nz=z+off.dz, nr=r+off.dr, nc=c+off.dc;
                if (!inside(nz,nr,nc,layers,rows,cols)) continue;
                float& cell=out_clearance[flat_index(nz,nr,nc,rows,cols)];
                if (static_cast<float>(off.d)<cell) cell=static_cast<float>(off.d);
            }
        }
        return 0;
    }

    for (const int idx : occupied) out_clearance[idx] = 0.0f;
    auto relax_line = [cap](float* base, int count, int stride) {
        for (int i = 1; i < count; ++i) {
            float& cur = base[i * stride];
            const float candidate = std::min(static_cast<float>(cap), base[(i - 1) * stride] + 1.0f);
            if (candidate < cur) cur = candidate;
        }
        for (int i = count - 2; i >= 0; --i) {
            float& cur = base[i * stride];
            const float candidate = std::min(static_cast<float>(cap), base[(i + 1) * stride] + 1.0f);
            if (candidate < cur) cur = candidate;
        }
    };
    for (int z = 0; z < layers; ++z)
        for (int r = 0; r < rows; ++r)
            relax_line(out_clearance + flat_index(z, r, 0, rows, cols), cols, 1);
    for (int z = 0; z < layers; ++z)
        for (int c = 0; c < cols; ++c)
            relax_line(out_clearance + flat_index(z, 0, c, rows, cols), rows, cols);
    const int plane = rows * cols;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            relax_line(out_clearance + flat_index(0, r, c, rows, cols), layers, plane);
    return 0;
}

FPGA_NATIVE_EXPORT int fpga_chain_blocked_count_xyz3d(
    const double* points_xyz,
    int point_count,
    const uint8_t* occupancy,
    int layers,
    int rows,
    int cols,
    double x_min,
    double y_min,
    double z_min,
    double resolution_xy,
    double resolution_z,
    int samples
) {
    // Exact native equivalent of Python _chain_blocked_count().  Deliberately
    // uses a fixed sample count (no span-adaptive oversampling) so safety/result
    // semantics remain bit-for-bit aligned with the existing planner branch.
    if (!points_xyz || !occupancy || point_count < 0 || layers <= 0 || rows <= 0 || cols <= 0 ||
        resolution_xy <= 0.0 || resolution_z <= 0.0) return -1;
    if (point_count < 2) return 0;
    const int steps = std::max(2, samples);
    int blocked_count = 0;
    for (int i = 0; i < point_count - 1; ++i) {
        const double* a = points_xyz + 3*i;
        const double* b = points_xyz + 3*(i+1);
        bool blocked = false;
        for (int k = 1; k < steps; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(steps);
            const double x = a[0] + (b[0] - a[0]) * t;
            const double y = a[1] + (b[1] - a[1]) * t;
            const double z = a[2] + (b[2] - a[2]) * t;
            const int c = static_cast<int>(std::floor((x - x_min) / resolution_xy));
            const int r = static_cast<int>(std::floor((y - y_min) / resolution_xy));
            const int l = static_cast<int>(std::floor((z - z_min) / resolution_z));
            if (!inside(l, r, c, layers, rows, cols) || occupancy[flat_index(l, r, c, rows, cols)]) {
                blocked = true;
                break;
            }
        }
        if (blocked) ++blocked_count;
    }
    return blocked_count;
}

FPGA_NATIVE_EXPORT int fpga_candidates3d(
    const double* spine_xyz,
    int layer_count,
    const uint8_t* occupancy,
    const float* clearance,
    int layers,
    int rows,
    int cols,
    double x_min,
    double y_min,
    double z_min,
    double resolution_xy,
    double resolution_z,
    int radius,
    int max_count,
    double safe_clearance_voxels,
    const double* goal_bias_xyz,
    int32_t* out_counts,
    int32_t* out_cells_zyx,
    double* out_positions_xyz,
    double* out_obstacle_costs,
    double* out_reference_costs
) {
    if (!spine_xyz || !occupancy || !clearance || !out_counts || !out_cells_zyx ||
        !out_positions_xyz || !out_obstacle_costs || !out_reference_costs ||
        layer_count < 0 || layers <= 0 || rows <= 0 || cols <= 0 ||
        resolution_xy <= 0.0 || resolution_z <= 0.0 || radius < 1 || max_count < 1) {
        return -1;
    }

    struct CandidateItem {
        int z;
        int r;
        int c;
        int ordinal;
        double x;
        double y;
        double zz;
        double dist;
        double clearance;
        double capped_clearance;
        double goal_proj;
    };

    for (int layer_idx = 0; layer_idx < layer_count; ++layer_idx) {
        const double sx = spine_xyz[3 * layer_idx + 0];
        const double sy = spine_xyz[3 * layer_idx + 1];
        const double sz = spine_xyz[3 * layer_idx + 2];
        const int center_c = static_cast<int>(std::floor((sx - x_min) / resolution_xy));
        const int center_r = static_cast<int>(std::floor((sy - y_min) / resolution_xy));
        const int center_z = static_cast<int>(std::floor((sz - z_min) / resolution_z));
        const int z0 = std::max(0, center_z - radius);
        const int z1 = std::min(layers, center_z + radius + 1);
        const int r0 = std::max(0, center_r - radius);
        const int r1 = std::min(rows, center_r + radius + 1);
        const int c0 = std::max(0, center_c - radius);
        const int c1 = std::min(cols, center_c + radius + 1);
        if (z1 <= z0 || r1 <= r0 || c1 <= c0) {
            out_counts[layer_idx] = 0;
            continue;
        }

        std::vector<CandidateItem> items;
        items.reserve(static_cast<std::size_t>((z1-z0)*(r1-r0)*(c1-c0)));
        int ordinal = 0;
        int baseline_index = -1;
        double baseline_dist = std::numeric_limits<double>::infinity();
        for (int z = z0; z < z1; ++z) {
            for (int r = r0; r < r1; ++r) {
                for (int c = c0; c < c1; ++c) {
                    if (occupancy[flat_index(z, r, c, rows, cols)]) continue;
                    const double x = x_min + (static_cast<double>(c) + 0.5) * resolution_xy;
                    const double y = y_min + (static_cast<double>(r) + 0.5) * resolution_xy;
                    const double zz = z_min + (static_cast<double>(z) + 0.5) * resolution_z;
                    const double dx = x - sx;
                    const double dy = y - sy;
                    const double dz = zz - sz;
                    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    const double clr = static_cast<double>(clearance[flat_index(z, r, c, rows, cols)]);
                    const double capped = std::min(clr, safe_clearance_voxels);
                    const double proj = goal_bias_xyz
                        ? x * goal_bias_xyz[0] + y * goal_bias_xyz[1] + zz * goal_bias_xyz[2]
                        : 0.0;
                    CandidateItem item{z,r,c,ordinal++,x,y,zz,dist,clr,capped,proj};
                    const int this_index = static_cast<int>(items.size());
                    items.push_back(item);
                    if (z == center_z && r == center_r && c == center_c) {
                        baseline_index = this_index;
                    } else if (baseline_index < 0 && dist < baseline_dist) {
                        // np.argmin keeps the first occurrence.  Track the same
                        // C-order free-voxel scan as np.argwhere.
                        baseline_dist = dist;
                    }
                }
            }
        }
        if (items.empty()) {
            out_counts[layer_idx] = 0;
            continue;
        }
        if (baseline_index < 0) {
            baseline_dist = std::numeric_limits<double>::infinity();
            for (int i = 0; i < static_cast<int>(items.size()); ++i) {
                if (items[i].dist < baseline_dist) {
                    baseline_dist = items[i].dist;
                    baseline_index = i;
                }
            }
        }

        std::vector<int> order(items.size());
        for (int i = 0; i < static_cast<int>(order.size()); ++i) order[i] = i;
        std::stable_sort(order.begin(), order.end(), [&](int ia, int ib) {
            const CandidateItem& a = items[ia];
            const CandidateItem& b = items[ib];
            // np.lexsort((-goal_proj, -min(clear,safe), dist_spine)):
            // distance ascending, capped clearance descending, projection descending.
            if (a.dist < b.dist) return true;
            if (a.dist > b.dist) return false;
            if (a.capped_clearance > b.capped_clearance) return true;
            if (a.capped_clearance < b.capped_clearance) return false;
            if (a.goal_proj > b.goal_proj) return true;
            if (a.goal_proj < b.goal_proj) return false;
            return a.ordinal < b.ordinal;
        });

        std::vector<int> selected;
        selected.reserve(static_cast<std::size_t>(max_count));
        selected.push_back(baseline_index);
        for (int idx : order) {
            if (idx == baseline_index) continue;
            selected.push_back(idx);
            if (static_cast<int>(selected.size()) >= max_count) break;
        }
        const int count = std::min(max_count, static_cast<int>(selected.size()));
        out_counts[layer_idx] = count;
        for (int j = 0; j < count; ++j) {
            const CandidateItem& item = items[selected[j]];
            const int base = layer_idx * max_count + j;
            out_cells_zyx[3*base + 0] = item.z;
            out_cells_zyx[3*base + 1] = item.r;
            out_cells_zyx[3*base + 2] = item.c;
            out_positions_xyz[3*base + 0] = item.x;
            out_positions_xyz[3*base + 1] = item.y;
            out_positions_xyz[3*base + 2] = item.zz;
            const double gap = std::max(0.0, safe_clearance_voxels - item.clearance);
            out_obstacle_costs[base] = gap * gap;
            const double dx = item.x - sx;
            const double dy = item.y - sy;
            const double dz = item.zz - sz;
            out_reference_costs[base] = dx*dx + dy*dy + dz*dz;
        }
    }
    return layer_count;
}

FPGA_NATIVE_EXPORT int fpga_astar26(
    const uint8_t* occupancy,
    const double* bias,
    int layers,
    int rows,
    int cols,
    int sz,
    int sr,
    int sc,
    int gz,
    int gr,
    int gc,
    int32_t* out_path_zyx,
    int out_capacity_cells
) {
    if (!occupancy || !out_path_zyx || layers <= 0 || rows <= 0 || cols <= 0 || out_capacity_cells <= 0) {
        return -1;
    }
    if (!inside(sz, sr, sc, layers, rows, cols) || !inside(gz, gr, gc, layers, rows, cols)) return 0;

    const int size = layers * rows * cols;
    astar_ws.ensure(static_cast<std::size_t>(size));
    const uint32_t gen = astar_ws.next_generation();
    auto& g_score = astar_ws.g;
    auto& parent = astar_ws.parent;
    auto& seen = astar_ws.seen;
    auto& closed = astar_ws.closed;

    const int start_idx = flat_index(sz, sr, sc, rows, cols);
    const int goal_idx = flat_index(gz, gr, gc, rows, cols);
    g_score[start_idx] = 0.0;
    parent[start_idx] = -1;
    seen[start_idx] = gen;

    std::priority_queue<OpenNode, std::vector<OpenNode>, OpenCmp> heap;
    heap.push({diag_heuristic(sz, sr, sc, gz, gr, gc), 0.0, start_idx});
    int current = -1;

    while (!heap.empty()) {
        const OpenNode node = heap.top();
        heap.pop();
        current = node.idx;
        if (current == goal_idx) break;
        if (seen[current] != gen || node.g > g_score[current] + EPS) continue;
        closed[current] = gen;

        const int z = current / (rows * cols);
        const int rem = current - z * rows * cols;
        const int r = rem / cols;
        const int c = rem - r * cols;

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    if (dz == 0 && dr == 0 && dc == 0) continue;
                    const int nz = z + dz;
                    const int nr = r + dr;
                    const int nc = c + dc;
                    if (!inside(nz, nr, nc, layers, rows, cols)) continue;
                    const int nxt = flat_index(nz, nr, nc, rows, cols);
                    if (closed[nxt] == gen || occupancy[nxt]) continue;
                    const double move_cost = std::sqrt(static_cast<double>(dz*dz + dr*dr + dc*dc));
                    const double extra = bias ? bias[nxt] : 0.0;
                    const double tentative = node.g + move_cost + extra;
                    const double old = (seen[nxt] == gen) ? g_score[nxt] : std::numeric_limits<double>::infinity();
                    if (tentative + EPS >= old) continue;
                    seen[nxt] = gen;
                    g_score[nxt] = tentative;
                    parent[nxt] = current;
                    const double h = diag_heuristic(nz, nr, nc, gz, gr, gc);
                    heap.push({tentative + h, tentative, nxt});
                }
            }
        }
    }

    if (current != goal_idx) return 0;
    std::vector<int> reverse_path;
    reverse_path.reserve(128);
    int node = goal_idx;
    while (node != -1) {
        reverse_path.push_back(node);
        node = parent[node];
    }
    const int length = static_cast<int>(reverse_path.size());
    if (length > out_capacity_cells) return -length;
    for (int i = 0; i < length; ++i) {
        const int idx = reverse_path[length - 1 - i];
        const int z = idx / (rows * cols);
        const int rem = idx - z * rows * cols;
        const int r = rem / cols;
        const int c = rem - r * cols;
        out_path_zyx[3*i + 0] = z;
        out_path_zyx[3*i + 1] = r;
        out_path_zyx[3*i + 2] = c;
    }
    return length;
}

FPGA_NATIVE_EXPORT int64_t fpga_blocked_pairs_output_size(
    const int32_t* layer_offsets,
    int layer_count
) {
    if (!layer_offsets || layer_count < 2) return 0;
    int64_t total = 0;
    for (int l = 0; l < layer_count - 1; ++l) {
        const int64_t m = static_cast<int64_t>(layer_offsets[l+1] - layer_offsets[l]);
        const int64_t n = static_cast<int64_t>(layer_offsets[l+2] - layer_offsets[l+1]);
        if (m < 0 || n < 0) return -1;
        total += m * n;
    }
    return total;
}

FPGA_NATIVE_EXPORT int64_t fpga_blocked_pairs3d(
    const int32_t* cells_zyx,
    const int32_t* layer_offsets,
    int layer_count,
    const uint8_t* occupancy,
    int layers,
    int rows,
    int cols,
    int samples,
    uint8_t* out_blocked,
    int64_t out_capacity
) {
    if (!cells_zyx || !layer_offsets || !occupancy || !out_blocked || layer_count < 2 ||
        layers <= 0 || rows <= 0 || cols <= 0 || samples < 2) return -1;
    const int64_t required = fpga_blocked_pairs_output_size(layer_offsets, layer_count);
    if (required < 0 || out_capacity < required) return -required;
    int64_t out_pos = 0;
    const int neighbour[7][3] = {
        {0,0,0}, {0,0,1}, {0,0,-1}, {0,1,0}, {0,-1,0}, {1,0,0}, {-1,0,0}
    };

    for (int l = 0; l < layer_count - 1; ++l) {
        const int left0 = layer_offsets[l];
        const int left1 = layer_offsets[l+1];
        const int right0 = layer_offsets[l+1];
        const int right1 = layer_offsets[l+2];
        int max_span = 1;
        for (int i = left0; i < left1; ++i) {
            const int32_t* a = cells_zyx + 3*i;
            for (int j = right0; j < right1; ++j) {
                const int32_t* b = cells_zyx + 3*j;
                max_span = std::max(max_span, std::abs(static_cast<int>(b[0]-a[0])));
                max_span = std::max(max_span, std::abs(static_cast<int>(b[1]-a[1])));
                max_span = std::max(max_span, std::abs(static_cast<int>(b[2]-a[2])));
            }
        }
        const int steps = std::max(samples, max_span * 2 + 1);
        for (int i = left0; i < left1; ++i) {
            const int32_t* a = cells_zyx + 3*i;
            for (int j = right0; j < right1; ++j) {
                const int32_t* b = cells_zyx + 3*j;
                bool blocked = false;
                for (int k = 1; k < steps && !blocked; ++k) {
                    const double t = static_cast<double>(k) / static_cast<double>(steps);
                    const long cz = round_ties_to_even(a[0] + (b[0] - a[0]) * t);
                    const long cr = round_ties_to_even(a[1] + (b[1] - a[1]) * t);
                    const long cc = round_ties_to_even(a[2] + (b[2] - a[2]) * t);
                    if (!inside(static_cast<int>(cz), static_cast<int>(cr), static_cast<int>(cc), layers, rows, cols)) {
                        blocked = true;
                        break;
                    }
                    for (int q = 0; q < 7; ++q) {
                        const int nz = static_cast<int>(cz) + neighbour[q][0];
                        const int nr = static_cast<int>(cr) + neighbour[q][1];
                        const int nc = static_cast<int>(cc) + neighbour[q][2];
                        if (inside(nz, nr, nc, layers, rows, cols) && occupancy[flat_index(nz,nr,nc,rows,cols)]) {
                            blocked = true;
                            break;
                        }
                    }
                }
                out_blocked[out_pos++] = blocked ? 1 : 0;
            }
        }
    }
    return required;
}


// Host Opt V5.3: exact batch checker for integer voxel-cell line segments.
// This reproduces _segment_relative_samples/_segment_clear_cells exactly:
// 2*max(|delta|)+1 ties-to-even rounded samples, including both endpoints.
// It accelerates rolling-spine suffix validation/shortcutting without changing
// any geometry or safety semantics.
FPGA_NATIVE_EXPORT int fpga_segments_blocked_cells3d(
    const int32_t* a_zyx,
    const int32_t* b_zyx,
    int segment_count,
    const uint8_t* occupancy,
    int layers,
    int rows,
    int cols,
    uint8_t* out_blocked
) {
    if (!a_zyx || !b_zyx || !occupancy || !out_blocked || segment_count < 0 ||
        layers <= 0 || rows <= 0 || cols <= 0) return -1;
    for (int i = 0; i < segment_count; ++i) {
        const int32_t* a = a_zyx + 3*i;
        const int32_t* b = b_zyx + 3*i;
        const int dz = static_cast<int>(b[0] - a[0]);
        const int dr = static_cast<int>(b[1] - a[1]);
        const int dc = static_cast<int>(b[2] - a[2]);
        const int span = std::max({std::abs(dz), std::abs(dr), std::abs(dc), 1});
        const int sample_count = span * 2 + 1;
        bool blocked = false;
        const double step = sample_count <= 1 ? 0.0 : 1.0 / static_cast<double>(sample_count - 1);
        for (int k = 0; k < sample_count; ++k) {
            // numpy.linspace(0,1,n) forms interior samples as k*step; using
            // k/(n-1) differs by one ULP for some half-way cases and can
            // change np.rint ties-to-even voxel selection.
            const double t = (k == sample_count - 1) ? 1.0 : static_cast<double>(k) * step;
            const int z = static_cast<int>(a[0]) + static_cast<int>(round_ties_to_even(t * static_cast<double>(dz)));
            const int r = static_cast<int>(a[1]) + static_cast<int>(round_ties_to_even(t * static_cast<double>(dr)));
            const int c = static_cast<int>(a[2]) + static_cast<int>(round_ties_to_even(t * static_cast<double>(dc)));
            if (!inside(z,r,c,layers,rows,cols) || occupancy[flat_index(z,r,c,rows,cols)]) {
                blocked = true;
                break;
            }
        }
        out_blocked[i] = blocked ? 1 : 0;
    }
    return segment_count;
}

FPGA_NATIVE_EXPORT int fpga_segments_blocked_xyz3d(
    const double* a_xyz,
    const double* b_xyz,
    int segment_count,
    const uint8_t* occupancy,
    int layers,
    int rows,
    int cols,
    double x_min,
    double y_min,
    double z_min,
    double resolution_xy,
    double resolution_z,
    int samples,
    uint8_t* out_blocked
) {
    if (!a_xyz || !b_xyz || !occupancy || !out_blocked || segment_count < 0 ||
        layers <= 0 || rows <= 0 || cols <= 0 || resolution_xy <= 0.0 || resolution_z <= 0.0) return -1;
    for (int i = 0; i < segment_count; ++i) {
        const double* a = a_xyz + 3*i;
        const double* b = b_xyz + 3*i;
        const int az = static_cast<int>(std::floor((a[2] - z_min) / resolution_z));
        const int ar = static_cast<int>(std::floor((a[1] - y_min) / resolution_xy));
        const int ac = static_cast<int>(std::floor((a[0] - x_min) / resolution_xy));
        const int bz = static_cast<int>(std::floor((b[2] - z_min) / resolution_z));
        const int br = static_cast<int>(std::floor((b[1] - y_min) / resolution_xy));
        const int bc = static_cast<int>(std::floor((b[0] - x_min) / resolution_xy));
        const int span = std::max({std::abs(bz-az), std::abs(br-ar), std::abs(bc-ac), 1});
        const int steps = std::max({2, samples, span*2+1});
        bool blocked = false;
        for (int k = 1; k < steps; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(steps);
            const double x = a[0] + (b[0]-a[0]) * t;
            const double y = a[1] + (b[1]-a[1]) * t;
            const double z = a[2] + (b[2]-a[2]) * t;
            const int c = static_cast<int>(std::floor((x - x_min) / resolution_xy));
            const int r = static_cast<int>(std::floor((y - y_min) / resolution_xy));
            const int l = static_cast<int>(std::floor((z - z_min) / resolution_z));
            if (!inside(l,r,c,layers,rows,cols) || occupancy[flat_index(l,r,c,rows,cols)]) {
                blocked = true;
                break;
            }
        }
        out_blocked[i] = blocked ? 1 : 0;
    }
    return segment_count;
}



// Host Opt V5.4: dense trajectory generation in C++.
// This reproduces the Python cubic-Hermite / linear fallback sampling used by
// _build_dense.  It targets the long-tail retime cases where trajectory_dt can
// expand to seconds and Python must otherwise generate hundreds/thousands of
// 20-ms samples per segment.  Geometry/dynamics semantics are unchanged.
FPGA_NATIVE_EXPORT int64_t fpga_dense_trajectory_output_size(
    int waypoint_count,
    double dt,
    double dt_sample
) {
    if (waypoint_count <= 0 || dt_sample <= 0.0) return -1;
    if (waypoint_count == 1) return 1;
    const double denom = std::max(dt, 1.0e-12);
    const double step = dt_sample / denom;
    if (!(step > 0.0) || !std::isfinite(step)) return -1;
    int64_t interior = 0;
    double t = step;
    // Same loop/threshold as Python.  Guard pathological values defensively.
    while (t < 1.0 - 1.0e-9) {
        ++interior;
        if (interior > 10000000) return -1;
        t += step;
    }
    const int64_t per_segment = interior + 1; // interior samples + exact endpoint
    return 1 + static_cast<int64_t>(waypoint_count - 1) * per_segment;
}

FPGA_NATIVE_EXPORT int fpga_dense_trajectory3d(
    const double* points_xyz,
    const double* waypoint_vel_xyz,
    int waypoint_count,
    double dt,
    double dt_sample,
    double max_vel,
    int linear_only,
    double* out_dense_xyz,
    double* out_dense_vel_xyz,
    double* out_dense_acc_xyz,
    int64_t capacity_points,
    double* out_max_speed
) {
    if (!points_xyz || !waypoint_vel_xyz || !out_dense_xyz || !out_dense_vel_xyz ||
        !out_dense_acc_xyz || !out_max_speed || waypoint_count <= 0 || dt_sample <= 0.0) return -1;
    const int64_t required = fpga_dense_trajectory_output_size(waypoint_count, dt, dt_sample);
    if (required <= 0 || capacity_points < required) return -2;
    if (waypoint_count == 1) {
        for (int d=0; d<3; ++d) {
            out_dense_xyz[d] = points_xyz[d];
            out_dense_vel_xyz[d] = waypoint_vel_xyz[d];
            out_dense_acc_xyz[d] = 0.0;
        }
        *out_max_speed = 0.0;
        return 1;
    }
    const double safe_dt = std::max(dt, 1.0e-12);
    const double step = dt_sample / safe_dt;

    auto build_cubic = [&]() -> int64_t {
        int64_t pos = 0;
        for (int d=0; d<3; ++d) out_dense_xyz[3*pos+d] = points_xyz[d];
        ++pos;
        for (int i=0; i<waypoint_count-1; ++i) {
            const double* p0 = points_xyz + 3*i;
            const double* p1 = points_xyz + 3*(i+1);
            const double* v0 = waypoint_vel_xyz + 3*i;
            const double* v1 = waypoint_vel_xyz + 3*(i+1);
            double t = step;
            while (t < 1.0 - 1.0e-9) {
                const double t2=t*t, t3=t2*t;
                const double h00=2.0*t3-3.0*t2+1.0;
                const double h10=t3-2.0*t2+t;
                const double h01=-2.0*t3+3.0*t2;
                const double h11=t3-t2;
                for (int d=0; d<3; ++d) {
                    out_dense_xyz[3*pos+d] = h00*p0[d] + h10*dt*v0[d] + h01*p1[d] + h11*dt*v1[d];
                }
                ++pos;
                t += step;
            }
            for (int d=0; d<3; ++d) out_dense_xyz[3*pos+d] = p1[d];
            ++pos;
        }
        return pos;
    };

    auto build_linear = [&]() -> int64_t {
        int64_t pos = 0;
        for (int d=0; d<3; ++d) out_dense_xyz[3*pos+d] = points_xyz[d];
        ++pos;
        for (int i=0; i<waypoint_count-1; ++i) {
            const double* p0 = points_xyz + 3*i;
            const double* p1 = points_xyz + 3*(i+1);
            double t = step;
            while (t < 1.0 - 1.0e-9) {
                for (int d=0; d<3; ++d) out_dense_xyz[3*pos+d] = p0[d] + (p1[d]-p0[d])*t;
                ++pos;
                t += step;
            }
            for (int d=0; d<3; ++d) out_dense_xyz[3*pos+d] = p1[d];
            ++pos;
        }
        return pos;
    };

    auto fill_interp_vel = [&]() -> int64_t {
        int64_t pos=0;
        for (int i=0; i<waypoint_count-1; ++i) {
            const double* v0=waypoint_vel_xyz+3*i;
            const double* v1=waypoint_vel_xyz+3*(i+1);
            double t=0.0;
            while (t < 1.0 - 1.0e-9) {
                for (int d=0; d<3; ++d) out_dense_vel_xyz[3*pos+d]=(1.0-t)*v0[d]+t*v1[d];
                ++pos;
                t += step;
            }
        }
        const double* vlast=waypoint_vel_xyz+3*(waypoint_count-1);
        for (int d=0; d<3; ++d) out_dense_vel_xyz[3*pos+d]=vlast[d];
        ++pos;
        return pos;
    };

    auto fill_linear_vel = [&]() -> int64_t {
        int64_t pos=0;
        for (int i=0; i<waypoint_count-1; ++i) {
            const double* p0=points_xyz+3*i;
            const double* p1=points_xyz+3*(i+1);
            double v[3]={(p1[0]-p0[0])/safe_dt,(p1[1]-p0[1])/safe_dt,(p1[2]-p0[2])/safe_dt};
            // Python _dense_linear emits one velocity for samples[0], then one
            // for every appended interior/endpoint sample of this segment.
            if (i==0) {
                for (int d=0; d<3; ++d) out_dense_vel_xyz[3*pos+d]=v[d];
                ++pos;
            }
            double t=step;
            while (t < 1.0 - 1.0e-9) {
                for (int d=0; d<3; ++d) out_dense_vel_xyz[3*pos+d]=v[d];
                ++pos;
                t += step;
            }
            for (int d=0; d<3; ++d) out_dense_vel_xyz[3*pos+d]=v[d];
            ++pos;
        }
        return pos;
    };

    auto max_step_speed = [&](int64_t count)->double {
        double vmax=0.0;
        for (int64_t i=1;i<count;++i) {
            const double dx=out_dense_xyz[3*i]-out_dense_xyz[3*(i-1)];
            const double dy=out_dense_xyz[3*i+1]-out_dense_xyz[3*(i-1)+1];
            const double dz=out_dense_xyz[3*i+2]-out_dense_xyz[3*(i-1)+2];
            vmax=std::max(vmax,std::sqrt(dx*dx+dy*dy+dz*dz)/std::max(dt_sample,1.0e-12));
        }
        return vmax;
    };

    int64_t count = linear_only ? build_linear() : build_cubic();
    if (count != required) return -3;
    double vmax=max_step_speed(count);
    bool use_linear = linear_only != 0;
    if (!use_linear && vmax > max_vel*1.05 + 1.0e-6) {
        count=build_linear();
        vmax=max_step_speed(count);
        use_linear=true;
    }
    const int64_t vcount = use_linear ? fill_linear_vel() : fill_interp_vel();
    if (vcount != count) return -4;

    // Python _central_accelerations(dense, dense_vel, dt_sample).
    for (int64_t i=0;i<count;++i) {
        for (int d=0;d<3;++d) {
            double value=0.0;
            if (count > 1) {
                if (i==0) value=(out_dense_vel_xyz[3+d]-out_dense_vel_xyz[d])/dt_sample;
                else if (i==count-1) value=(out_dense_vel_xyz[3*(count-2)+d]-out_dense_vel_xyz[3*(count-1)+d])/dt_sample;
                else value=(out_dense_vel_xyz[3*(i+1)+d]-out_dense_vel_xyz[3*(i-1)+d])/(2.0*dt_sample);
            }
            out_dense_acc_xyz[3*i+d]=value;
        }
    }
    *out_max_speed=vmax;
    return static_cast<int>(count);
}

// Host Opt V4: build the complete active-N kinodynamic QUBO in C++.
// This is a host-only acceleration. It reproduces the Python reference model
// and returns QUBO linear/off-diagonal terms plus the final Ising h/J arrays.
FPGA_NATIVE_EXPORT int fpga_kino_qubo_active(
    const int32_t* layer_offsets, int free_layers,
    const double* candidate_xyz,
    const double* obstacle_costs,
    const double* reference_costs, int has_reference,
    const uint8_t* blocked_pairs_flat,
    const uint8_t* endpoint_first_blocked,
    const uint8_t* endpoint_last_blocked,
    const double* start_xyz, const double* goal_xyz,
    const double* start_vel_xyz, const double* start_acc_xyz,
    double dt,
    double w_vel, double w_acc, double w_obs, double w_spine, double w_len,
    double w_start_vel, double w_start_acc, double w_end_vel,
    double segment_penalty, double a_mult,
    double* out_linear, double* out_matrix,
    double* out_h, double* out_j,
    double* out_info, uint64_t* out_hashes
) {
    if (!layer_offsets || !candidate_xyz || !obstacle_costs || !start_xyz || !goal_xyz ||
        !start_vel_xyz || !start_acc_xyz || !out_linear || !out_matrix || !out_h || !out_j ||
        !out_info || !out_hashes || free_layers < 1 || dt <= 0.0) return -1;
    const int n = layer_offsets[free_layers];
    if (n <= 0 || n > 256) return -2;
    const int total_layers = free_layers + 2;
    std::fill(out_linear, out_linear + n, 0.0);
    std::fill(out_matrix, out_matrix + static_cast<std::size_t>(n)*n, 0.0);

    auto layer_is_fixed = [&](int layer) -> bool {
        return layer == 0 || layer == total_layers - 1;
    };
    auto layer_begin = [&](int layer) -> int { return layer_offsets[layer - 1]; };
    auto layer_end = [&](int layer) -> int { return layer_offsets[layer]; };
    auto fixed_ptr = [&](int layer) -> const double* {
        return layer == 0 ? start_xyz : goal_xyz;
    };
    auto cand_ptr = [&](int idx) -> const double* { return candidate_xyz + 3*idx; };
    auto dot3 = [](const double* a, const double* b) -> double {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    };
    auto add_bilinear = [&](int u, int v, double coeff) {
        if (std::abs(coeff) < 1.0e-12) return;
        out_matrix[static_cast<std::size_t>(u)*n + v] += coeff;
        out_matrix[static_cast<std::size_t>(v)*n + u] += coeff;
    };
    auto add_dot = [&](int la, int lb, double weight) {
        if (std::abs(weight) < 1.0e-12) return;
        const bool fa = layer_is_fixed(la);
        const bool fb = layer_is_fixed(lb);
        if (fa && fb) return;
        if (!fa && !fb && la == lb) {
            const int b = layer_begin(la), e = layer_end(la);
            for (int i=b;i<e;++i) out_linear[i] += weight * dot3(cand_ptr(i), cand_ptr(i));
            return;
        }
        if (fa && !fb) {
            const double* f = fixed_ptr(la);
            const int b = layer_begin(lb), e = layer_end(lb);
            for (int i=b;i<e;++i) out_linear[i] += weight * dot3(cand_ptr(i), f);
            return;
        }
        if (!fa && fb) {
            const double* f = fixed_ptr(lb);
            const int b = layer_begin(la), e = layer_end(la);
            for (int i=b;i<e;++i) out_linear[i] += weight * dot3(cand_ptr(i), f);
            return;
        }
        const int ab = layer_begin(la), ae = layer_end(la);
        const int bb = layer_begin(lb), be = layer_end(lb);
        for (int i=ab;i<ae;++i) {
            const double* a = cand_ptr(i);
            for (int j=bb;j<be;++j) add_bilinear(i,j,weight*dot3(a,cand_ptr(j)));
        }
    };

    const double dt2 = dt*dt;
    const double dt4 = dt2*dt2;
    for (int layer=0; layer<total_layers-1; ++layer) {
        const double w = w_vel/dt2;
        add_dot(layer,layer,w);
        add_dot(layer+1,layer+1,w);
        add_dot(layer,layer+1,-2.0*w);
    }
    for (int layer=0; layer<total_layers-2; ++layer) {
        const double w = w_acc/dt4;
        add_dot(layer,layer,w);
        add_dot(layer+1,layer+1,4.0*w);
        add_dot(layer+2,layer+2,w);
        add_dot(layer,layer+1,-4.0*w);
        add_dot(layer,layer+2,2.0*w);
        add_dot(layer+1,layer+2,-4.0*w);
    }

    if (w_start_vel > 0.0 && free_layers >= 1) {
        add_dot(1,1,w_start_vel);
        const double target[3] = {
            start_xyz[0] + start_vel_xyz[0]*dt,
            start_xyz[1] + start_vel_xyz[1]*dt,
            start_xyz[2] + start_vel_xyz[2]*dt,
        };
        for (int i=layer_begin(1); i<layer_end(1); ++i)
            out_linear[i] += -2.0*w_start_vel*dot3(cand_ptr(i),target);
    }
    if (w_start_acc > 0.0 && free_layers >= 2) {
        const double w = w_start_acc/dt4;
        add_dot(2,2,w);
        add_dot(1,1,4.0*w);
        add_dot(2,1,-4.0*w);
        add_dot(2,0,2.0*w);
        add_dot(1,0,-4.0*w);
        for (int i=layer_begin(1); i<layer_end(1); ++i)
            out_linear[i] += 4.0*w*dot3(cand_ptr(i),start_acc_xyz)*dt2;
        for (int i=layer_begin(2); i<layer_end(2); ++i)
            out_linear[i] += -2.0*w*dot3(cand_ptr(i),start_acc_xyz)*dt2;
    }
    if (w_end_vel > 0.0 && free_layers >= 1) {
        add_dot(total_layers-1,total_layers-1,w_end_vel);
        add_dot(total_layers-2,total_layers-2,w_end_vel);
        add_dot(total_layers-2,total_layers-1,-2.0*w_end_vel);
    }

    for (int i=0;i<n;++i) {
        const double c = obstacle_costs[i];
        if (c > 0.0) out_linear[i] += w_obs*c;
        if (has_reference && reference_costs) {
            const double r = reference_costs[i];
            if (r > 0.0) out_linear[i] += w_spine*r;
        }
    }

    if (w_len > 0.0) {
        // start -> first free layer
        for (int i=layer_begin(1); i<layer_end(1); ++i) {
            const double* q=cand_ptr(i);
            const double dx=q[0]-start_xyz[0],dy=q[1]-start_xyz[1],dz=q[2]-start_xyz[2];
            out_linear[i] += w_len*std::sqrt(dx*dx+dy*dy+dz*dz);
        }
        // interior variable-variable edges
        for (int layer=1; layer<total_layers-2; ++layer) {
            for (int i=layer_begin(layer); i<layer_end(layer); ++i) {
                const double* a=cand_ptr(i);
                for (int j=layer_begin(layer+1); j<layer_end(layer+1); ++j) {
                    const double* b=cand_ptr(j);
                    const double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];
                    add_bilinear(i,j,w_len*std::sqrt(dx*dx+dy*dy+dz*dz));
                }
            }
        }
        // last free layer -> goal
        for (int i=layer_begin(total_layers-2); i<layer_end(total_layers-2); ++i) {
            const double* q=cand_ptr(i);
            const double dx=q[0]-goal_xyz[0],dy=q[1]-goal_xyz[1],dz=q[2]-goal_xyz[2];
            out_linear[i] += w_len*std::sqrt(dx*dx+dy*dy+dz*dz);
        }
    }

    double objective_scale=1.0;
    for (int i=0;i<n;++i) objective_scale=std::max(objective_scale,std::abs(out_linear[i]));
    for (std::size_t k=0;k<static_cast<std::size_t>(n)*n;++k)
        objective_scale=std::max(objective_scale,std::abs(out_matrix[k]));
    if (objective_scale != 1.0) {
        for (int i=0;i<n;++i) out_linear[i] /= objective_scale;
        for (std::size_t k=0;k<static_cast<std::size_t>(n)*n;++k) out_matrix[k] /= objective_scale;
    }
    double max_touch=0.0;
    for (int i=0;i<n;++i) {
        double touch=std::abs(out_linear[i]);
        const double* row=out_matrix+static_cast<std::size_t>(i)*n;
        for (int j=0;j<n;++j) touch += std::abs(row[j]);
        max_touch=std::max(max_touch,touch);
    }
    const double one_hot=std::max(a_mult*max_touch+1.0,10.0);
    for (int layer=1;layer<=free_layers;++layer) {
        const int b=layer_begin(layer),e=layer_end(layer);
        for (int i=b;i<e;++i) out_linear[i]-=one_hot;
        for (int i=b;i<e;++i) for (int j=i+1;j<e;++j) add_bilinear(i,j,2.0*one_hot);
    }
    const double seg=std::min(std::max(0.0,segment_penalty),one_hot*0.5);
    const int first_count=layer_end(1)-layer_begin(1);
    const int last_layer=free_layers;
    const int last_count=layer_end(last_layer)-layer_begin(last_layer);
    if (endpoint_first_blocked) {
        for (int k=0;k<first_count;++k) if (endpoint_first_blocked[k]) out_linear[layer_begin(1)+k]+=seg;
    }
    if (endpoint_last_blocked) {
        for (int k=0;k<last_count;++k) if (endpoint_last_blocked[k]) out_linear[layer_begin(last_layer)+k]+=seg;
    }
    if (blocked_pairs_flat) {
        std::size_t pos=0;
        for (int layer=1;layer<free_layers;++layer) {
            const int b1=layer_begin(layer),e1=layer_end(layer);
            const int b2=layer_begin(layer+1),e2=layer_end(layer+1);
            for (int i=b1;i<e1;++i) for (int j=b2;j<e2;++j,++pos)
                if (blocked_pairs_flat[pos]) add_bilinear(i,j,seg);
        }
    }

    double max_ising=0.0;
    for (int i=0;i<n;++i) {
        double row_sum=0.0;
        for (int j=0;j<n;++j) row_sum += out_matrix[static_cast<std::size_t>(i)*n+j];
        out_h[i]=-(out_linear[i]+row_sum);
        max_ising=std::max(max_ising,std::abs(out_h[i]));
    }
    for (int i=0;i<n;++i) for (int j=0;j<n;++j) {
        const double v=-0.5*out_matrix[static_cast<std::size_t>(i)*n+j];
        out_j[static_cast<std::size_t>(i)*n+j]=v;
        max_ising=std::max(max_ising,std::abs(v));
    }
    double bfp_factor=1.0;
    if (max_ising > 0.6) {
        bfp_factor=0.5/max_ising;
        for (int i=0;i<n;++i) out_h[i]*=bfp_factor;
        for (std::size_t k=0;k<static_cast<std::size_t>(n)*n;++k) out_j[k]*=bfp_factor;
    }
    out_info[0]=objective_scale;
    out_info[1]=one_hot;
    out_info[2]=seg;
    out_info[3]=bfp_factor;

    // Host Opt V5: cache fingerprints are produced while h/J are already hot
    // in C++ cache.  They describe the exact active-N Q9.22 logical problem,
    // not a particular BFP/sparse wire representation.  Equality therefore
    // safely implies that the FPGA cache can be reused without rebuilding a
    // full compact payload on the Python side.
    constexpr uint64_t fnv_offset = 14695981039346656037ULL;
    uint64_t full_hash=fnv_offset, j_hash=fnv_offset;
    fnv1a_mix_u64_le(full_hash,0x0000000051323239ULL); // domain tag "Q229"/Q9.22
    fnv1a_mix_u64_le(j_hash,0x0000000051323239ULL);
    fnv1a_mix_u64_le(full_hash,static_cast<uint64_t>(n));
    fnv1a_mix_u64_le(j_hash,static_cast<uint64_t>(n));
    constexpr double qscale = 4194304.0;
    for (int i=0;i<n;++i) {
        const int64_t q=static_cast<int64_t>(round_ties_to_even(out_h[i]*qscale));
        fnv1a_mix_u64_le(full_hash,static_cast<uint64_t>(q));
    }
    for (int i=0;i<n;++i) for (int j=i+1;j<n;++j) {
        const int64_t q=static_cast<int64_t>(round_ties_to_even(out_j[static_cast<std::size_t>(i)*n+j]*qscale));
        fnv1a_mix_u64_le(full_hash,static_cast<uint64_t>(q));
        fnv1a_mix_u64_le(j_hash,static_cast<uint64_t>(q));
    }
    out_hashes[0]=full_hash;
    out_hashes[1]=j_hash;
    return n;
}
