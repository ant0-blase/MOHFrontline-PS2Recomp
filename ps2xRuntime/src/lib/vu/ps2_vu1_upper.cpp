#include <atomic>
#include <cstdio>
#include "runtime/ps2_vu1.h"
#include "ps2_vu1_detail.h"

#include <cmath>
#include <cstring>

namespace
{
    // Clamp-opcode census. A guard-band clamp before FTOI4 needs both a MAX
    // (lower bound) and a MINI (upper bound); an earlier histogram suggested MAX
    // never fires, which would leave negative coordinates free to wrap.
    std::atomic<uint64_t> g_clampCount[6] = {};
    std::atomic<uint32_t> g_clampPcNear[6] = {};

    // Bounds actually used by the clamp pair at 0x13E8 (MINIi, against the I
    // register) and 0x13F0 (MAX). If these operands are wrong the clamp is
    // ineffective even though it executes.
    inline void noteClampBounds(uint32_t pc, float iReg, const float *vs, const float *vt)
    {
        if (pc != 0x13E8u && pc != 0x13F0u)
            return;
        static std::atomic<uint32_t> s_seen{0};
        const uint32_t k = s_seen.fetch_add(1, std::memory_order_relaxed);
        if (k < 8u)
            std::fprintf(stderr,
                         "[MOH:clamp-bounds-v1] k=%u pc=0x%04x I=%.4f "
                         "vs=(%.3f %.3f %.3f %.3f) vt=(%.3f %.3f %.3f %.3f)\n",
                         k, pc, (double)iReg,
                         (double)vs[0], (double)vs[1], (double)vs[2], (double)vs[3],
                         (double)vt[0], (double)vt[1], (double)vt[2], (double)vt[3]);
    }

    inline void noteClamp(uint32_t which, uint32_t pc)
    {
        const uint64_t c = g_clampCount[which].fetch_add(1, std::memory_order_relaxed);
        if (pc >= 0x1380u && pc <= 0x1480u)
            g_clampPcNear[which].store(pc, std::memory_order_relaxed);
        if ((c % 20000u) == 19999u)
        {
            static const char *n[6] = {"MAXbc", "MAXi", "MAX", "MINIbc", "MINIi", "MINI"};
            std::fprintf(stderr, "[MOH:clamp-census-v1]");
            for (uint32_t i = 0u; i < 6u; ++i)
                std::fprintf(stderr, " %s=%llu/pc0x%04x", n[i],
                             (unsigned long long)g_clampCount[i].load(std::memory_order_relaxed),
                             g_clampPcNear[i].load(std::memory_order_relaxed));
            std::fprintf(stderr, "\n");
        }
    }
}

namespace
{
    // VU FTOI saturates: values beyond the 32-bit signed range clamp to
    // 0x7FFFFFFF / 0x80000000. A plain C++ cast is undefined behaviour there and
    // on x86 yields 0x80000000 for *either* sign, so a large positive coordinate
    // came out negative. Counted so the frequency is visible.
    std::atomic<uint64_t> g_ftoiSatPos{0};
    std::atomic<uint64_t> g_ftoiSatNeg{0};

    inline int32_t vuFtoiSaturate(float v)
    {
        if (!(v == v))
            return 0;
        if (v >= 2147483647.0f)
        {
            const uint64_t c = g_ftoiSatPos.fetch_add(1, std::memory_order_relaxed);
            if ((c % 10000u) == 0u)
                std::fprintf(stderr, "[MOH:ftoi-sat-v1] pos=%llu neg=%llu\n",
                             (unsigned long long)c + 1u,
                             (unsigned long long)g_ftoiSatNeg.load(std::memory_order_relaxed));
            return 0x7FFFFFFF;
        }
        if (v <= -2147483648.0f)
        {
            g_ftoiSatNeg.fetch_add(1, std::memory_order_relaxed);
            return static_cast<int32_t>(0x80000000);
        }
        return static_cast<int32_t>(v);
    }
}

// ============================================================================
// Upper instructions (FMAC pipeline)
// ============================================================================
void VU1Interpreter::execUpper(uint32_t instr)
{
    uint8_t dest = DEST(instr);
    uint8_t ft = FT(instr);
    uint8_t fs = FS(instr);
    uint8_t fd = FD(instr);
    uint8_t op = instr & 0x3F;

    float *vd = m_state.vf[fd];
    const float *vs = m_state.vf[fs];
    const float *vt = m_state.vf[ft];
    float result[4];

    // Upper opcode decoding (bits 5:0 of upper word)
    switch (op)
    {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03: // ADDbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] + bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07: // SUBbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] - bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x08:
    case 0x09:
    case 0x0A:
    case 0x0B: // MADDbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] + vs[c] * bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x0C:
    case 0x0D:
    case 0x0E:
    case 0x0F: // MSUBbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] - vs[c] * bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x10:
    case 0x11:
    case 0x12:
    case 0x13: // MAXbc
        noteClamp(0u, m_state.pc);
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] > bc) ? vs[c] : bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17: // MINIbc
        noteClamp(3u, m_state.pc);
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] < bc) ? vs[c] : bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B: // MULbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] * bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x1C: // MULq
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] * m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x1D: // MAXi
        noteClamp(1u, m_state.pc);
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] > m_state.i) ? vs[c] : m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x1E: // MULi
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] * m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x1F: // MINIi
        noteClamp(4u, m_state.pc);
        noteClampBounds(m_state.pc, m_state.i, vs, vt);
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] < m_state.i) ? vs[c] : m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x20: // ADDq
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] + m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x21: // MADDq
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] + vs[c] * m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x22: // ADDi
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] + m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x23: // MADDi
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] + vs[c] * m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x24: // SUBq
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] - m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x25: // MSUBq
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] - vs[c] * m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x26: // SUBi
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] - m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x27: // MSUBi
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] - vs[c] * m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x28: // ADD
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] + vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x29: // MADD
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] + vs[c] * vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2A: // MUL
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] * vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2B: // MAX
        noteClamp(2u, m_state.pc);
        noteClampBounds(m_state.pc, m_state.i, vs, vt);
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] > vt[c]) ? vs[c] : vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2C: // SUB
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] - vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2D: // MSUB
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] - vs[c] * vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2E: // OPMSUB
        result[0] = m_state.acc[0] - vs[1] * vt[2];
        result[1] = m_state.acc[1] - vs[2] * vt[0];
        result[2] = m_state.acc[2] - vs[0] * vt[1];
        result[3] = 0.0f;
        applyDest(vd, result, dest);
        return;
    case 0x2F: // MINI
        noteClamp(5u, m_state.pc);
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] < vt[c]) ? vs[c] : vt[c];
        applyDest(vd, result, dest);
        return;

    // Upper special group (low op 0x3C..0x3F).
    // Like lower1 special, the real selector is not just bits 5:0.  Dobie decodes:
    //   op = (instr & 0x3) | ((instr >> 4) & 0x7C)
    // Several instructions in this group also use FT as the destination, not FD.
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
    {
        const uint8_t specialOp = static_cast<uint8_t>((instr & 0x3u) | ((instr >> 4) & 0x7Cu));
        float *vtDest = m_state.vf[ft];

        switch (specialOp)
        {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03: // ADDAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] + bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07: // SUBAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] - bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x08:
        case 0x09:
        case 0x0A:
        case 0x0B: // MADDAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] + vs[c] * bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x0C:
        case 0x0D:
        case 0x0E:
        case 0x0F: // MSUBAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] - vs[c] * bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x10: // ITOF0
            for (int c = 0; c < 4; c++)
            {
                int32_t iv;
                std::memcpy(&iv, &vs[c], 4);
                result[c] = static_cast<float>(iv);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x11: // ITOF4
            for (int c = 0; c < 4; c++)
            {
                int32_t iv;
                std::memcpy(&iv, &vs[c], 4);
                result[c] = static_cast<float>(iv) / 16.0f;
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x12: // ITOF12
            for (int c = 0; c < 4; c++)
            {
                int32_t iv;
                std::memcpy(&iv, &vs[c], 4);
                result[c] = static_cast<float>(iv) / 4096.0f;
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x13: // ITOF15
            for (int c = 0; c < 4; c++)
            {
                int32_t iv;
                std::memcpy(&iv, &vs[c], 4);
                result[c] = static_cast<float>(iv) / 32768.0f;
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x14: // FTOI0
            for (int c = 0; c < 4; c++)
            {
                int32_t iv = vuFtoiSaturate(vs[c]);
                std::memcpy(&result[c], &iv, 4);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x15: // FTOI4
            // Correct wrap criterion. The GS vertex XY field is 16 bits of 12.4 fixed
            // point, so a coordinate wraps exactly when |value * 16| >= 65536, i.e.
            // |value| >= 4096. Measuring on-screen span instead conflates wrapped
            // triangles with legitimately fullscreen blits, which invalidated several
            // earlier population measurements.
            {
                static std::atomic<uint64_t> s_conv{0}, s_wrapX{0}, s_wrapY{0}, s_wrapAny{0};
                // Per-site breakdown. The GIF XYZ2 field is 16 bits for X and Y but 24 for
                // Z, so a >= 4096 threshold is only meaningful at sites that produce XY.
                // Counting all sites together mixes real XY wraps with legal large Z values,
                // which would inflate the metric.
                {
                    static std::atomic<uint32_t> s_sPc[8] = {};
                    static std::atomic<uint64_t> s_sTot[8] = {};
                    static std::atomic<uint64_t> s_sBig[8] = {};
                    static std::atomic<uint32_t> s_sN{0};
                    const uint32_t pcs = m_state.pc;
                    const uint32_t nn2 = s_sN.load(std::memory_order_acquire);
                    int slot3 = -1;
                    for (uint32_t i = 0u; i < nn2 && i < 8u; ++i)
                        if (s_sPc[i].load(std::memory_order_relaxed) == pcs) { slot3 = (int)i; break; }
                    if (slot3 < 0 && nn2 < 8u)
                    {
                        s_sPc[nn2].store(pcs, std::memory_order_relaxed);
                        s_sN.store(nn2 + 1u, std::memory_order_release);
                        slot3 = (int)nn2;
                    }
                    if (slot3 >= 0)
                    {
                        const uint64_t t2 = s_sTot[slot3].fetch_add(1, std::memory_order_relaxed);
                        if (std::fabs(vs[0]) >= 4096.0f || std::fabs(vs[1]) >= 4096.0f)
                            s_sBig[slot3].fetch_add(1, std::memory_order_relaxed);
                        if ((t2 % 400000u) == 399999u)
                        {
                            std::fprintf(stderr, "[MOH:wrap-per-site-v1]");
                            for (uint32_t i = 0u; i < s_sN.load(std::memory_order_acquire) && i < 8u; ++i)
                                std::fprintf(stderr, " 0x%04x=%llu/%llu",
                                             s_sPc[i].load(std::memory_order_relaxed),
                                             (unsigned long long)s_sBig[i].load(std::memory_order_relaxed),
                                             (unsigned long long)s_sTot[i].load(std::memory_order_relaxed));
                            std::fprintf(stderr, "\n");
                        }
                    }
                }
                // The decisive cross-tab. At 0x1440 the lower slot is
                // "ISW.w vi10,0x103(vi13)", which stores the vertex's w word; bit 15 of
                // that word is the GS ADC flag ("kick but do not draw"). The microprogram
                // reaches 0x1390 -> "IOR vi10,vi10,vi11" (vi11 = 0x8000) whenever the
                // FCOR trivial-reject chain or the FCAND at 0x12d8 says the triangle
                // touches the frustum. So a wrapped coordinate is harmless on console
                // *provided* ADC is set. Cross-tabulating wrap against ADC separates the
                // two remaining hypotheses: ADC set on wraps means the flag is computed
                // right and is lost further down; ADC clear means the clip flags are wrong.
                // Wrapping turned out to be a red herring: 100% of wrapped vertices
                // already carry ADC = 1, so the GS never draws them. The primitives that
                // do get drawn sit at e.g. x = 3945 with the screen at [1728,2368] --
                // outside the frustum but inside the 16-bit field. Cross-tabulate
                // "outside the frustum" against ADC, per site, to find where a vertex
                // escapes the clip decision. Screen centre is 2048/2048, half-extent
                // 320/224.
                {
                    static std::atomic<uint32_t> s_pc[8] = {};
                    static std::atomic<uint64_t> s_c[8][2] = {};
                    static std::atomic<uint32_t> s_n{0};
                    const uint32_t pcs = m_state.pc;
                    const uint32_t nn = s_n.load(std::memory_order_acquire);
                    int sl = -1;
                    for (uint32_t i = 0u; i < nn && i < 8u; ++i)
                        if (s_pc[i].load(std::memory_order_relaxed) == pcs) { sl = (int)i; break; }
                    if (sl < 0 && nn < 8u)
                    {
                        s_pc[nn].store(pcs, std::memory_order_relaxed);
                        s_n.store(nn + 1u, std::memory_order_release);
                        sl = (int)nn;
                    }
                    const bool off = std::fabs(vs[0] - 2048.0f) > 336.0f ||
                                     std::fabs(vs[1] - 2048.0f) > 240.0f;
                    if (sl >= 0 && off)
                    {
                        const bool adc = (m_state.vi[10] & 0x8000u) != 0u;
                        const uint64_t c = s_c[sl][adc ? 1u : 0u]
                                               .fetch_add(1, std::memory_order_relaxed);
                        if ((c % 50000u) == 49999u)
                        {
                            std::fprintf(stderr, "[MOH:off-adc-v1] hors-ecran ADC0/ADC1 par site:");
                            for (uint32_t i = 0u; i < s_n.load(std::memory_order_acquire) && i < 8u; ++i)
                                std::fprintf(stderr, " 0x%04x=%llu/%llu",
                                             s_pc[i].load(std::memory_order_relaxed),
                                             (unsigned long long)s_c[i][0].load(std::memory_order_relaxed),
                                             (unsigned long long)s_c[i][1].load(std::memory_order_relaxed));
                            std::fprintf(stderr, "\n");
                        }
                    }
                }
                if (m_state.pc == 0x1440u)
                {
                    static std::atomic<uint64_t> s_t[4] = {};
                    const bool wrapped = std::fabs(vs[0]) >= 4096.0f || std::fabs(vs[1]) >= 4096.0f;
                    const bool adc = (m_state.vi[10] & 0x8000u) != 0u;
                    const uint64_t c = s_t[(wrapped ? 2u : 0u) | (adc ? 1u : 0u)]
                                           .fetch_add(1, std::memory_order_relaxed);
                    if ((c % 20000u) == 19999u)
                        std::fprintf(stderr,
                                     "[MOH:wrap-adc-v1] sain/ADC0=%llu sain/ADC1=%llu "
                                     "enroule/ADC0=%llu enroule/ADC1=%llu\n",
                                     (unsigned long long)s_t[0].load(std::memory_order_relaxed),
                                     (unsigned long long)s_t[1].load(std::memory_order_relaxed),
                                     (unsigned long long)s_t[2].load(std::memory_order_relaxed),
                                     (unsigned long long)s_t[3].load(std::memory_order_relaxed));
                }
                // Do the wrapping vertices cluster in a few XGKICK batches, or are they
                // spread across all of them? Clustering would mean one object is misplaced;
                // spread would mean the whole submission is off. The vertex stream is the
                // only remaining candidate, so this discriminates within it.
                {
                    static thread_local uint64_t lastKick = 0;
                    static thread_local uint32_t inBatch = 0, wrapsInBatch = 0;
                    const uint64_t kick = mohvu::g_xgkickSeq.load(std::memory_order_relaxed);
                    if (kick != lastKick)
                    {
                        if (inBatch)
                        {
                            static std::atomic<uint64_t> s_b[5] = {};
                            const uint32_t frac = wrapsInBatch == 0 ? 0u
                                                 : (wrapsInBatch * 100u / inBatch) < 5u ? 1u
                                                 : (wrapsInBatch * 100u / inBatch) < 25u ? 2u
                                                 : (wrapsInBatch * 100u / inBatch) < 75u ? 3u : 4u;
                            const uint64_t c = s_b[frac].fetch_add(1, std::memory_order_relaxed);
                            if ((c % 20000u) == 19999u)
                                std::fprintf(stderr,
                                             "[MOH:wrap-batch-v1] lots: 0%%=%llu <5%%=%llu <25%%=%llu <75%%=%llu >=75%%=%llu\n",
                                             (unsigned long long)s_b[0].load(std::memory_order_relaxed),
                                             (unsigned long long)s_b[1].load(std::memory_order_relaxed),
                                             (unsigned long long)s_b[2].load(std::memory_order_relaxed),
                                             (unsigned long long)s_b[3].load(std::memory_order_relaxed),
                                             (unsigned long long)s_b[4].load(std::memory_order_relaxed));
                        }
                        lastKick = kick; inBatch = 0; wrapsInBatch = 0;
                    }
                    ++inBatch;
                    if (std::fabs(vs[0]) >= 4096.0f || std::fabs(vs[1]) >= 4096.0f) ++wrapsInBatch;
                }
                const bool wx = std::fabs(vs[0]) >= 4096.0f;
                const bool wy = std::fabs(vs[1]) >= 4096.0f;
                const uint64_t n = s_conv.fetch_add(1, std::memory_order_relaxed);
                if (wx) s_wrapX.fetch_add(1, std::memory_order_relaxed);
                if (wy) s_wrapY.fetch_add(1, std::memory_order_relaxed);
                if (wx || wy) s_wrapAny.fetch_add(1, std::memory_order_relaxed);
                {
                    // Tag by (bank, variant) pair, not variant index alone: the index is
                    // per-bank, so index 1 of bank 1 and index 1 of bank 2 are different
                    // programs. Conflating them made a site difference look like a variant
                    // difference.
                    const uint32_t bk = (m_state.pc >> 11) & 7u;
                    const uint32_t vv = mohvu::g_bankVariant[bk].load(std::memory_order_relaxed) & 3u;
                    const uint32_t slot2 = ((bk & 3u) << 2) | vv;
                    static std::atomic<uint64_t> s_cv[16] = {};
                    static std::atomic<uint64_t> s_wv[16] = {};
                    s_cv[slot2].fetch_add(1, std::memory_order_relaxed);
                    if (wx || wy) s_wv[slot2].fetch_add(1, std::memory_order_relaxed);
                    {
                        static std::atomic<uint64_t> s_pr{0};
                        if ((s_pr.fetch_add(1, std::memory_order_relaxed) % 400000u) == 399999u)
                        {
                            std::fprintf(stderr, "[MOH:wrap-bank-variant-v1]");
                            for (uint32_t i = 0u; i < 16u; ++i)
                            {
                                const uint64_t c3 = s_cv[i].load(std::memory_order_relaxed);
                                if (!c3) continue;
                                std::fprintf(stderr, " b%u/v%u=%llu/%llu", i >> 2, i & 3u,
                                             (unsigned long long)s_wv[i].load(std::memory_order_relaxed),
                                             (unsigned long long)c3);
                            }
                            std::fprintf(stderr, "\n");
                        }
                    }
                    static std::atomic<uint64_t> s_vr{0};
                    if ((s_vr.fetch_add(1, std::memory_order_relaxed) % 400000u) == 399999u)
                        std::fprintf(stderr,
                                     "[MOH:wrap-by-variant-v1] v0=%llu/%llu v1=%llu/%llu v2=%llu/%llu v3=%llu/%llu\n",
                                     (unsigned long long)mohvu::g_wrapByVariant[0].load(std::memory_order_relaxed),
                                     (unsigned long long)mohvu::g_convByVariant[0].load(std::memory_order_relaxed),
                                     (unsigned long long)mohvu::g_wrapByVariant[1].load(std::memory_order_relaxed),
                                     (unsigned long long)mohvu::g_convByVariant[1].load(std::memory_order_relaxed),
                                     (unsigned long long)mohvu::g_wrapByVariant[2].load(std::memory_order_relaxed),
                                     (unsigned long long)mohvu::g_convByVariant[2].load(std::memory_order_relaxed),
                                     (unsigned long long)mohvu::g_wrapByVariant[3].load(std::memory_order_relaxed),
                                     (unsigned long long)mohvu::g_convByVariant[3].load(std::memory_order_relaxed));
                }
                if ((wx || wy) && !mohvu::g_histDumped.load(std::memory_order_relaxed))
                    mohvu::t_histDumpRequested = true;
                if ((wx || wy) && mohvu::t_currentProgram)
                    mohvu::t_currentProgram->wrapCount.fetch_add(1, std::memory_order_relaxed);
                // Magnitude of the wrapped coordinates. Legitimate off-screen geometry
                // sits modestly beyond 4096; corrupt or uninitialised data is orders of
                // magnitude larger. This is the fork the span criterion could never settle.
                if (wx || wy)
                {
                    static std::atomic<uint64_t> s_mag[6] = {};
                    const float m = std::max(std::fabs(vs[0]), std::fabs(vs[1]));
                    const uint32_t b = m < 8192.0f     ? 0u
                                       : m < 32768.0f  ? 1u
                                       : m < 131072.0f ? 2u
                                       : m < 1048576.0f ? 3u
                                       : m < 1.0e8f    ? 4u : 5u;
                    const uint64_t mc = s_mag[b].fetch_add(1, std::memory_order_relaxed);
                    if ((mc % 20000u) == 19999u)
                        std::fprintf(stderr,
                                     "[MOH:wrap-mag-v1] <8k=%llu <32k=%llu <128k=%llu <1M=%llu <100M=%llu >=100M=%llu\n",
                                     (unsigned long long)s_mag[0].load(std::memory_order_relaxed),
                                     (unsigned long long)s_mag[1].load(std::memory_order_relaxed),
                                     (unsigned long long)s_mag[2].load(std::memory_order_relaxed),
                                     (unsigned long long)s_mag[3].load(std::memory_order_relaxed),
                                     (unsigned long long)s_mag[4].load(std::memory_order_relaxed),
                                     (unsigned long long)s_mag[5].load(std::memory_order_relaxed));
                }
                // How many of a triangle's three vertices wrap? A rolling window of three
                // conversions at this site approximates per-primitive grouping. If most
                // triangles have all three wrapping, the trivial reject should have removed
                // them and is failing; if one or two, the geometry genuinely needs clipping.
                {
                    // Contingency table: did any WAITQ execute during this triangle's three
                    // conversions, and did the triangle wrap? If wrapped triangles show no
                    // interpolation while clean ones do, clipping is being skipped for them.
                    {
                        static thread_local uint64_t lastTick = 0;
                        static thread_local int wrapAcc = 0;
                        static thread_local int waitAcc = 0;
                        static thread_local uint32_t tpos = 0;
                        const uint64_t tick = mohvu::g_waitqTicks.load(std::memory_order_relaxed);
                        if (tick != lastTick) { waitAcc = 1; lastTick = tick; }
                        if (wx || wy) wrapAcc = 1;
                        ++tpos;
                        if ((tpos % 3u) == 0u)
                        {
                            static std::atomic<uint64_t> s_tab[2][2] = {};
                            const uint64_t c2 = s_tab[wrapAcc][waitAcc].fetch_add(1, std::memory_order_relaxed);
                            if ((c2 % 100000u) == 99999u)
                                std::fprintf(stderr,
                                             "[MOH:clip-corr-v1] clean/noQ=%llu clean/Q=%llu wrap/noQ=%llu wrap/Q=%llu\n",
                                             (unsigned long long)s_tab[0][0].load(std::memory_order_relaxed),
                                             (unsigned long long)s_tab[0][1].load(std::memory_order_relaxed),
                                             (unsigned long long)s_tab[1][0].load(std::memory_order_relaxed),
                                             (unsigned long long)s_tab[1][1].load(std::memory_order_relaxed));
                            wrapAcc = 0; waitAcc = 0;
                        }
                    }
                    static thread_local int win[3] = {0, 0, 0};
                    static thread_local uint32_t wpos = 0u;
                    win[wpos % 3u] = (wx || wy) ? 1 : 0;
                    ++wpos;
                    if (wpos >= 3u && (wpos % 3u) == 0u)
                    {
                        const int k = win[0] + win[1] + win[2];
                        static std::atomic<uint64_t> s_grp[4] = {};
                        const uint64_t g = s_grp[k].fetch_add(1, std::memory_order_relaxed);
                        if (k > 0 && (g % 20000u) == 19999u)
                            std::fprintf(stderr,
                                         "[MOH:wrap-per-tri-v1] 0wrap=%llu 1wrap=%llu 2wrap=%llu 3wrap=%llu\n",
                                         (unsigned long long)s_grp[0].load(std::memory_order_relaxed),
                                         (unsigned long long)s_grp[1].load(std::memory_order_relaxed),
                                         (unsigned long long)s_grp[2].load(std::memory_order_relaxed),
                                         (unsigned long long)s_grp[3].load(std::memory_order_relaxed));
                    }
                }
                if ((n % 400000u) == 399999u)
                    std::fprintf(stderr,
                                 "[MOH:ftoi4-wrap-v1] conversions=%llu wrapX=%llu wrapY=%llu wrapAny=%llu\n",
                                 (unsigned long long)(n + 1),
                                 (unsigned long long)s_wrapX.load(std::memory_order_relaxed),
                                 (unsigned long long)s_wrapY.load(std::memory_order_relaxed),
                                 (unsigned long long)s_wrapAny.load(std::memory_order_relaxed));
            }
            {
                // [MOH diagnostic, env-gated, default off] Clamp X and Y into the GS
                // coordinate space instead of letting them wrap the 16-bit vertex field.
                // 90 % of wrapped coordinates sit 1-8 screen widths out, i.e. ordinary
                // off-screen geometry, and the guard band is only +/-2.7 screen widths.
                // This tests end to end whether guard-band clipping is the missing
                // behaviour. It is a probe, not a fix: a real repair belongs wherever the
                // game clips, not here.
                static const bool clampXY = [] {
                    const char *v = std::getenv("PS2_MOH_DIAG_CLAMP_XY");
                    return v && !std::strcmp(v, "1");
                }();
                float in[4] = {vs[0], vs[1], vs[2], vs[3]};
                if (clampXY)
                {
                    for (int c = 0; c < 2; ++c)
                        in[c] = std::min(4095.0f, std::max(0.0f, in[c]));
                }
                for (int c = 0; c < 4; c++)
                {
                    int32_t iv = vuFtoiSaturate(in[c] * 16.0f);
                    std::memcpy(&result[c], &iv, 4);
                }
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x16: // FTOI12
            for (int c = 0; c < 4; c++)
            {
                int32_t iv = vuFtoiSaturate(vs[c] * 4096.0f);
                std::memcpy(&result[c], &iv, 4);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x17: // FTOI15
            for (int c = 0; c < 4; c++)
            {
                int32_t iv = vuFtoiSaturate(vs[c] * 32768.0f);
                std::memcpy(&result[c], &iv, 4);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x18:
        case 0x19:
        case 0x1A:
        case 0x1B: // MULAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] * bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x1C: // MULAq
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] * m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x1D: // ABS
            for (int c = 0; c < 4; c++)
                result[c] = std::fabs(vs[c]);
            applyDest(vtDest, result, dest);
            return;
        case 0x1E: // MULAi
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] * m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x1F: // CLIP
        {
            float w = std::fabs(vt[3]);
            uint32_t flags = 0;
            if (vs[0] > +w) flags |= 0x01;
            if (vs[0] < -w) flags |= 0x02;
            if (vs[1] > +w) flags |= 0x04;
            if (vs[1] < -w) flags |= 0x08;
            if (vs[2] > +w) flags |= 0x10;
            if (vs[2] < -w) flags |= 0x20;
            // [MOH fix] The VU clipping flag register is 24 bits wide (four 6-bit
            // groups). Shifting without masking let bits accumulate above bit 23,
            // and FCOR tests `(CF | imm24) == 0xFFFFFF`, so any stale high bit made
            // that comparison fail forever: the per-triangle trivial-reject tests
            // stopped rejecting once a few CLIPs had run, and near-plane crossing
            // triangles were emitted to the GS with negative Q. FCAND/FCEQ masked
            // implicitly and hid the defect.
            // [MOH diagnostic, env-gated, default off] Revert the CF 24-bit mask applied
            // early in this session. That fix changed flag-test behaviour substantially
            // (FCOR true went 21 446 -> 133 620), and FCAND/FCOR is what selects between the
            // fast path and the clip route — so the fix itself could have increased the
            // fraction of triangles taking the unclipped path. Testing my own change.
            static const bool noCfMask = [] {
                const char *v = std::getenv("PS2_MOH_DIAG_NO_CF_MASK");
                return v && !std::strcmp(v, "1");
            }();
            m_state.clip = noCfMask ? ((m_state.clip << 6) | flags)
                                    : (((m_state.clip << 6) | flags) & 0xFFFFFFu);
            {
                mohvu::ClipRecord &rec =
                    mohvu::t_clipRing[mohvu::t_clipRingPos & 3u];
                rec.x = vs[0]; rec.y = vs[1]; rec.z = vs[2]; rec.w = vt[3];
                std::memcpy(&rec.wBits, &vt[3], sizeof(rec.wBits));
                rec.flags = flags;
                rec.pc = m_state.pc;
                ++mohvu::t_clipRingPos;
            }
            if (vt[3] < 0.0f)
            {
                mohvu::t_clipsSinceNegW = 0u;
                mohvu::t_lastNegWFlags = flags;
                mohvu::t_lastNegW = vt[3];
            }
            else if (mohvu::t_clipsSinceNegW < 1000u)
            {
                ++mohvu::t_clipsSinceNegW;
            }
            if (mohvu::ProgramStats *ps = mohvu::t_currentProgram)
            {
                ps->clips.fetch_add(1, std::memory_order_relaxed);
                if (flags != 0u)
                    ps->clipsWithFlags.fetch_add(1, std::memory_order_relaxed);
                if (vt[3] < 0.0f)
                    ps->clipsNegW.fetch_add(1, std::memory_order_relaxed);
                if (vt[3] < 0.0f && flags == 0u)
                    ps->clipsNegWUnflagged.fetch_add(1, std::memory_order_relaxed);
                if (flags & 0x01u) ps->clipBit0.fetch_add(1, std::memory_order_relaxed);
                if (flags & 0x02u) ps->clipBit1.fetch_add(1, std::memory_order_relaxed);
                if (flags & 0x04u) ps->clipBit2.fetch_add(1, std::memory_order_relaxed);
                if (flags & 0x08u) ps->clipBit3.fetch_add(1, std::memory_order_relaxed);
                if (flags & 0x10u) ps->clipBit4.fetch_add(1, std::memory_order_relaxed);
                if (flags & 0x20u) ps->clipBit5.fetch_add(1, std::memory_order_relaxed);

                // One-shot snapshot of the whole register file at the first CLIP
                // of a behind-camera vertex. The transform matrix is still live
                // in vf at this point; it is identified by its fourth column,
                // which must read (0,0,1,2) for the measured W = Z + 2.
                if (vt[3] < 0.0f)
                {
                    static std::atomic<bool> s_dumped{false};
                    bool expected = false;
                    if (s_dumped.compare_exchange_strong(expected, true))
                    {
                        std::fprintf(stderr,
                                     "[MOH:vu1-regfile-v1] pc=0x%04x CF=%06x\n",
                                     m_state.pc, m_state.clip & 0xFFFFFFu);
                        for (int r = 0; r < 32; ++r)
                        {
                            std::fprintf(stderr,
                                         "[MOH:vu1-regfile-v1] vf%02d = %14.6f %14.6f %14.6f %14.6f\n",
                                         r, m_state.vf[r][0], m_state.vf[r][1],
                                         m_state.vf[r][2], m_state.vf[r][3]);
                        }
                    }
                }
            }
            return;
        }
        case 0x20: // ADDAq
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] + m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x21: // MADDAq
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] + vs[c] * m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x22: // ADDAi
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] + m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x23: // MADDAi
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] + vs[c] * m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x24: // SUBAq
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] - m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x25: // MSUBAq
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] - vs[c] * m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x26: // SUBAi
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] - m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x27: // MSUBAi
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] - vs[c] * m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x28: // ADDA
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] + vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x29: // MADDA
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] + vs[c] * vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x2A: // MULA
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] * vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x2C: // SUBA
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] - vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x2D: // MSUBA
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] - vs[c] * vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x2E: // OPMULA
            result[0] = vs[1] * vt[2];
            result[1] = vs[2] * vt[0];
            result[2] = vs[0] * vt[1];
            result[3] = 0.0f;
            applyDestAcc(result, dest);
            return;
        case 0x2F:
        case 0x30: // NOP
            return;
        default:
            return;
        }
    }

    case 0x30:
    case 0x31:
    case 0x32:
    case 0x33:
    default:
        return;
    }
}
