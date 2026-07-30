#include <cmath>
#include "runtime/ps2_vu1.h"
#include "runtime/ps2_memory.h"
#include "ps2_vu1_detail.h"

#include <cstring>

VU1Interpreter::VU1Interpreter()
{
    reset();
}

void VU1Interpreter::reset()
{
    std::memset(&m_state, 0, sizeof(m_state));
    m_state.vf[0][3] = 1.0f; // VF0.w = 1.0
    m_state.q = 1.0f;
}

float VU1Interpreter::broadcast(const float *vf, uint8_t bc)
{
    return vf[bc & 3];
}

// [MOH fix] VU MAC flag computation.
//
// The MAC flag register holds, per lane, the zero/sign/underflow/overflow state
// of the last arithmetic result. It was never written anywhere in this
// interpreter, so it stayed 0 forever while the level-6_1 VU1 microprogram
// tested it 1.69M times per run through FMEQ and 177k times through FMOR.
// Those tests therefore always took the same (wrong) branch, which is why
// vertices behind the near plane were never rejected and reached the GS with a
// negative Q, producing triangles smeared across the whole screen.
//
// Layout (as on hardware): four nibbles, x at bit 3 of each nibble down to w at
// bit 0; Z = bits 3:0, S = bits 7:4, U = bits 11:8, O = bits 15:12. Only lanes
// selected by the destination mask contribute; the others read back as 0.
void VU1Interpreter::updateMacFlags(const float *result, uint8_t dest)
{
    uint32_t mac = 0u;
    for (int lane = 0; lane < 4; ++lane)
    {
        const uint8_t laneMask = static_cast<uint8_t>(0x8u >> lane);
        if ((dest & laneMask) == 0u)
            continue;

        const int shift = 3 - lane;
        uint32_t bits = 0u;
        std::memcpy(&bits, &result[lane], sizeof(bits));
        const uint32_t exponent = (bits >> 23) & 0xFFu;
        const uint32_t mantissa = bits & 0x7FFFFFu;

        if (exponent == 0u && mantissa == 0u)
            mac |= 1u << (0 + shift); // Z
        if ((bits & 0x80000000u) != 0u)
            mac |= 1u << (4 + shift); // S
        if (exponent == 0u && mantissa != 0u)
            mac |= 1u << (8 + shift); // U (denormal: flushed to zero on VU)
        if (exponent == 0xFFu)
            mac |= 1u << (12 + shift); // O (VU saturates instead of Inf/NaN)
    }
    m_state.mac = mac;
}

void VU1Interpreter::applyDest(float *dst, const float *result, uint8_t dest)
{
    updateMacFlags(result, dest);
    if (dest & 0x8)
        dst[0] = result[0]; // x
    if (dest & 0x4)
        dst[1] = result[1]; // y
    if (dest & 0x2)
        dst[2] = result[2]; // z
    if (dest & 0x1)
        dst[3] = result[3]; // w
}

void VU1Interpreter::applyDestAcc(const float *result, uint8_t dest)
{
    applyDest(m_state.acc, result, dest);
}

VU1Interpreter::DecodedInstructionPair VU1Interpreter::decodeInstructionPair(const uint8_t *vuCode, uint32_t pc) const
{
    DecodedInstructionPair decoded;
    std::memcpy(&decoded.lower, vuCode + pc, sizeof(decoded.lower));
    std::memcpy(&decoded.upper, vuCode + pc + sizeof(decoded.lower), sizeof(decoded.upper));

    decoded.iBit = ((decoded.upper >> 31) & 1u) != 0u;
    decoded.eBit = ((decoded.upper >> 30) & 1u) != 0u;
    decoded.lowerBeforeUpper = !decoded.iBit && vuLowerShouldRunBeforeUpper(decoded.upper, decoded.lower);
    return decoded;
}

void VU1Interpreter::rebuildDecodedCodeCache(const uint8_t *vuCode, uint32_t codeSize,
                                             const PS2Memory *memory, uint64_t generation)
{
    const uint32_t pairCount = codeSize / 8u;
    m_decodedCodeCache.resize(pairCount);
    for (uint32_t i = 0; i < pairCount; ++i)
    {
        m_decodedCodeCache[i] = decodeInstructionPair(vuCode, i * 8u);
    }

    m_cachedVuCode = vuCode;
    m_cachedMemory = memory;
    m_cachedCodeSize = codeSize;
    m_cachedCodeGeneration = generation;
    m_decodedCodeCacheValid = true;
}

VU1Interpreter::DecodedInstructionPair VU1Interpreter::getDecodedInstructionPairForPc(const uint8_t *vuCode,
                                                                                      uint32_t codeSize,
                                                                                      PS2Memory *memory,
                                                                                      uint32_t pc)
{
    // Only 8-byte aligned VU instruction pairs can use the decode cache.
    if ((pc & 7u) != 0u)
    {
        return decodeInstructionPair(vuCode, pc);
    }

    const bool trackedVu1Code = vuCode == memory->getVU1Code();
    if (!trackedVu1Code)
    {
        return decodeInstructionPair(vuCode, pc);
    }

    const uint64_t generation = memory->getVU1CodeGeneration();
    const bool rebuild =
        !m_decodedCodeCacheValid ||
        m_cachedVuCode != vuCode ||
        m_cachedMemory != memory ||
        m_cachedCodeSize != codeSize ||
        m_cachedCodeGeneration != generation;

    if (rebuild)
    {
        rebuildDecodedCodeCache(vuCode, codeSize, memory, generation);
    }

    return m_decodedCodeCache[pc / 8u];
}

void VU1Interpreter::execute(uint8_t *vuCode, uint32_t codeSize,
                             uint8_t *vuData, uint32_t dataSize,
                             GS &gs, PS2Memory *memory,
                             uint32_t startPC, uint32_t top, uint32_t itop,
                             uint32_t maxCycles)
{
    m_state.pc = startPC & 0x3FFFu;
    mohvu::t_currentProgram = mohvu::programFor(startPC & 0x3FFFu);

    // Camera-matrix oracle probe: read 0x23F620 straight out of guest memory at
    // the start of the main transform program, regardless of which code wrote
    // it. PCSX2 shows an orthonormal rotation there plus the camera position.
    if ((startPC & 0x3FFFu) == 0u && memory)
    {
        static std::atomic<uint32_t> s_camRun{0u};
        const uint32_t n = s_camRun.fetch_add(1u, std::memory_order_relaxed);
        if (n < 3u || (n % 200u) == 0u)
        {
            std::fprintf(stderr, "[MOH:cam-mem-v1] run=%u rows=", n);
            for (uint32_t i = 0u; i < 16u; ++i)
                std::fprintf(stderr, "%s%08x", i ? "," : "",
                             memory->read32(0x23F620u + i * 4u));
            std::fprintf(stderr, "\n");
            // Same sample point for the object pool at 0x1D4E000. PCSX2 shows
            // guest pointers plus a float position there; garbage here would
            // mean the level data itself decompressed wrong.
            // Decisive check on the layout-rect question, without mid-function
            // instrumentation: if 0x112030 really writes the rect through a null
            // pointer, guest addresses 0x14 and 0x18 hold the coordinates (307.5, 137).
            // Also read 0x332628+0x14/+0x18, the address static analysis predicts.
            {
                const uint32_t n0 = memory->read32(0x14u);
                const uint32_t n1 = memory->read32(0x18u);
                const uint32_t p0 = memory->read32(0x332628u + 0x14u);
                const uint32_t p1 = memory->read32(0x332628u + 0x18u);
                float f0, f1, g0, g1;
                std::memcpy(&f0, &n0, 4); std::memcpy(&f1, &n1, 4);
                std::memcpy(&g0, &p0, 4); std::memcpy(&g1, &p1, 4);
                std::fprintf(stderr,
                             "[MOH:rect-target-v1] run=%u null[0x14]=%.3f null[0x18]=%.3f"
                             " pred[0x33263c]=%.3f pred[0x332640]=%.3f\n",
                             n, (double)f0, (double)f1, (double)g0, (double)g1);
            }

            std::fprintf(stderr, "[MOH:obj-mem-v1] run=%u words=", n);
            for (uint32_t i = 0u; i < 32u; ++i)
                std::fprintf(stderr, "%s%08x", i ? "," : "",
                             memory->read32(0x1D4E000u + i * 4u));
            std::fprintf(stderr, "\n");

            // Pool census. PCSX2 shows every slot carrying geometry pointers at
            // +0x80 and +0x90; count how many of ours do, versus how many still
            // hold the constructor constant 0x3FFFBE75 written by FUN_001402B0.
            {
                // CORRECTED OFFSET. The constructor constant 0x3FFFBE75 sits at
                // objBase+0x80 (FUN_001402B0). In PCSX2 that constant is at
                // 0x1D4E250, so the native object base is 0x1D4E1D0 - confirmed
                // by the sibling base pointers the object stores at +0x214/+0x218
                // (0x01D4DF20 and 0x01D4E480, exactly one stride either side).
                // The geometry pointers therefore live at objBase+0x120, not
                // +0x80. Our own base is 0x1D4DFC0 (our constant is at 0x1D4E040).
                uint32_t withPtr = 0u, ctorConst = 0u, zero = 0u, other = 0u;
                for (uint32_t slot = 0u; slot < 64u; ++slot)
                {
                    const uint32_t w =
                        memory->read32(0x1D4DFC0u + slot * 0x2B0u + 0x120u);
                    if (w >= 0x01000000u && w <= 0x02000000u) ++withPtr;
                    else if (w == 0x3FFFBE75u) ++ctorConst;
                    else if (w == 0u) ++zero;
                    else ++other;
                }
                std::fprintf(stderr,
                             "[MOH:obj-census-v2] off=0x120 run=%u slots=64 ptr=%u ctorConst=%u zero=%u other=%u\n",
                             n, withPtr, ctorConst, zero, other);

                // Anchored geometry comparison. Native slot pointers at +0x120
                // are 0x01DF2AD0, 0x01DF2B90, ... (stride 0xC0), and the data at
                // 0x01DF2AD0 begins 0x00326F28, 0x408CC482, 0x40885E56,
                // 0x3FAE09AA. Dump ours the same way.
                std::fprintf(stderr, "[MOH:geom-cmp-v1] run=%u ptrs=", n);
                for (uint32_t slot = 0u; slot < 4u; ++slot)
                    std::fprintf(stderr, "%s%08x", slot ? "," : "",
                                 memory->read32(0x1D4DFC0u + slot * 0x2B0u + 0x120u));
                const uint32_t g0 = memory->read32(0x1D4DFC0u + 0x120u);
                std::fprintf(stderr, " data@0x%08x=", g0);
                for (uint32_t i = 0u; i < 16u; ++i)
                    std::fprintf(stderr, "%s%08x", i ? "," : "",
                                 memory->read32(g0 + i * 4u));
                std::fprintf(stderr, "\n");

                // Structural invariant of the geometry header, verified on two
                // native objects: the head triple at +0x04 duplicates the
                // translation held in the 4th column of the 3x4 matrix, i.e.
                // w1==w7, w2==w11, w3==w15 bitwise. Count violations here.
                {
                    uint32_t checked = 0u, violations = 0u, badPtr = 0u;
                    for (uint32_t slot = 0u; slot < 64u; ++slot)
                    {
                        const uint32_t g =
                            memory->read32(0x1D4DFC0u + slot * 0x2B0u + 0x120u);
                        if (g < 0x01000000u || g > 0x02000000u) { ++badPtr; continue; }
                        if (memory->read32(g) != 0x00326F28u) { ++badPtr; continue; }
                        ++checked;
                        const bool ok =
                            memory->read32(g + 0x04u) == memory->read32(g + 0x1Cu) &&
                            memory->read32(g + 0x08u) == memory->read32(g + 0x2Cu) &&
                            memory->read32(g + 0x0Cu) == memory->read32(g + 0x3Cu);
                        if (!ok) ++violations;
                    }
                    std::fprintf(stderr,
                                 "[MOH:geom-invariant-v1] run=%u checked=%u violations=%u badPtr=%u\n",
                                 n, checked, violations, badPtr);
                }

                // [MOH diagnostic, env-gated, default off] Restore the invariant
                // by copying the matrix translation into the head triple, then
                // observe whether the rendered frame changes. This answers
                // whether anything actually consumes the head triple. It is a
                // probe, not a fix: with PS2_MOH_DIAG_FIX_HEAD unset nothing is
                // written and behaviour is identical.
                {
                    static const bool fixHead = [] {
                        const char *v = std::getenv("PS2_MOH_DIAG_FIX_HEAD");
                        return v && !std::strcmp(v, "1");
                    }();
                    if (fixHead)
                    {
                        uint32_t patched = 0u;
                        for (uint32_t slot = 0u; slot < 64u; ++slot)
                        {
                            const uint32_t g =
                                memory->read32(0x1D4DFC0u + slot * 0x2B0u + 0x120u);
                            if (g < 0x01000000u || g > 0x02000000u) continue;
                            if (memory->read32(g) != 0x00326F28u) continue;
                            memory->write32(g + 0x04u, memory->read32(g + 0x1Cu));
                            memory->write32(g + 0x08u, memory->read32(g + 0x2Cu));
                            memory->write32(g + 0x0Cu, memory->read32(g + 0x3Cu));
                            ++patched;
                        }
                        if (n < 3u)
                            std::fprintf(stderr,
                                         "[MOH:diag-fix-head] run=%u patched=%u\n",
                                         n, patched);
                    }
                }

                // Is the level-geometry region populated at all? PCSX2 has real
                // float data at 0x01DF2AD0 and 0x01DE5858, targets of the
                // pointers the native slots carry.
                uint32_t nonZero2AD0 = 0u, nonZero5858 = 0u;
                for (uint32_t i = 0u; i < 16u; ++i)
                {
                    if (memory->read32(0x01DF2AD0u + i * 4u) != 0u) ++nonZero2AD0;
                    if (memory->read32(0x01DE5858u + i * 4u) != 0u) ++nonZero5858;
                }
                std::fprintf(stderr,
                             "[MOH:geom-region-v1] run=%u nz@0x1DF2AD0=%u/16 nz@0x1DE5858=%u/16 first=%08x,%08x\n",
                             n, nonZero2AD0, nonZero5858,
                             memory->read32(0x01DF2AD0u), memory->read32(0x01DE5858u));

                // Has the object->model binder at 0x206C98 (and its callers)
                // ever executed? Reported via counters the overrides bump.
                extern std::atomic<uint64_t> g_mohBinderHits;
                extern std::atomic<uint64_t> g_mohBinderCallerHits;
                extern std::atomic<uint64_t> g_moh191688Hits;
                extern std::atomic<uint64_t> g_moh191900Hits;
                extern std::atomic<uint64_t> g_mohLayout1da8f0;
                extern std::atomic<uint64_t> g_mohLayout112030;
                extern std::atomic<uint64_t> g_mohLayout1d8620;
                extern std::atomic<uint64_t> g_mohLayout10a4f8;
                extern std::atomic<uint64_t> g_mohLayout144a88;
                extern std::atomic<uint64_t> g_mohLayout144bb0;
                extern std::atomic<uint64_t> g_moh191800Hits;
                extern std::atomic<uint64_t> g_moh182318Hits;
                std::fprintf(stderr,
                             "[MOH:binder-v1] run=%u binder206c98=%llu callers=%llu f191688=%llu f182318=%llu f191900=%llu f191800=%llu layout1da8f0=%llu layout112030=%llu layout1d8620=%llu 10a4f8=%llu 144a88=%llu 144bb0=%llu\n",
                             n,
                             (unsigned long long)g_mohBinderHits.load(std::memory_order_relaxed),
                             (unsigned long long)g_mohBinderCallerHits.load(std::memory_order_relaxed),
                             (unsigned long long)g_moh191688Hits.load(std::memory_order_relaxed),
                             (unsigned long long)g_moh182318Hits.load(std::memory_order_relaxed),
                             (unsigned long long)g_moh191900Hits.load(std::memory_order_relaxed),
                             (unsigned long long)g_moh191800Hits.load(std::memory_order_relaxed),
                             (unsigned long long)g_mohLayout1da8f0.load(std::memory_order_relaxed),
                             (unsigned long long)g_mohLayout112030.load(std::memory_order_relaxed),
                             (unsigned long long)g_mohLayout1d8620.load(std::memory_order_relaxed),
                             (unsigned long long)g_mohLayout10a4f8.load(std::memory_order_relaxed),
                             (unsigned long long)g_mohLayout144a88.load(std::memory_order_relaxed),
                             (unsigned long long)g_mohLayout144bb0.load(std::memory_order_relaxed));
            }
        }
    }
    if (mohvu::t_currentProgram)
    {
        const uint64_t r =
            mohvu::t_currentProgram->runs.fetch_add(1, std::memory_order_relaxed);
        if (((r + 1u) % 4000u) == 0u)
            mohvu::dumpPrograms();
        if (((r + 1u) % 4000u) == 0u)
            mohvu::dumpXgkickSites();
            mohvu::dumpFlagImm();
    }
    m_state.ebit = false;
    m_state.top = top;
    m_state.itop = itop;
    m_state.branchPending = false;
    m_state.branchTarget = 0;
    m_state.branchDelay = 0;
    m_state.vf[0][0] = 0.0f;
    m_state.vf[0][1] = 0.0f;
    m_state.vf[0][2] = 0.0f;
    m_state.vf[0][3] = 1.0f;
    run(vuCode, codeSize, vuData, dataSize, gs, memory, maxCycles);
}

void VU1Interpreter::resume(uint8_t *vuCode, uint32_t codeSize,
                            uint8_t *vuData, uint32_t dataSize,
                            GS &gs, PS2Memory *memory,
                            uint32_t top, uint32_t itop, uint32_t maxCycles)
{
    m_state.ebit = false;
    m_state.top = top;
    m_state.itop = itop;
    run(vuCode, codeSize, vuData, dataSize, gs, memory, maxCycles);
}

void VU1Interpreter::run(uint8_t *vuCode, uint32_t codeSize,
                         uint8_t *vuData, uint32_t dataSize,
                         GS &gs, PS2Memory *memory, uint32_t maxCycles)
{
    uint32_t retired = 0u;
    for (uint32_t cycle = 0; cycle < maxCycles; ++cycle)
    {
        if (m_state.pc + 8 > codeSize)
            break;
        ++retired;
        // Funnel through the clip routine: where does the flow stop? If 0x0E00 itself is
        // rarely reached the problem is upstream of it, not in its internal branches.
        {
            // Which addresses inside 0x0E00..0x0FFF actually execute? The entry at 0x0E00
            // is never reached, yet the region runs 460 times, so execution enters it
            // somewhere else - most likely its divide tail called via BAL from elsewhere.
            if (m_state.pc >= 0x0E00u && m_state.pc < 0x1000u)
            {
                static std::atomic<uint64_t> s_ins[64] = {};
                static std::atomic<uint64_t> s_tot{0};
                const uint32_t slot = (m_state.pc - 0x0E00u) >> 3;
                if (slot < 64u) s_ins[slot].fetch_add(1, std::memory_order_relaxed);
                const uint64_t t = s_tot.fetch_add(1, std::memory_order_relaxed);
                if ((t % 200u) == 199u)
                {
                    std::fprintf(stderr, "[MOH:dead-hits-v1]");
                    for (uint32_t i = 0u; i < 64u; ++i)
                    {
                        const uint64_t v = s_ins[i].load(std::memory_order_relaxed);
                        if (v) std::fprintf(stderr, " 0x%04x=%llu", 0x0E00u + i * 8u,
                                            (unsigned long long)v);
                    }
                    std::fprintf(stderr, "\n");
                }
            }

            static std::atomic<uint64_t> s_f[5] = {};
            int idx = -1;
            switch (m_state.pc)
            {
            case 0x0E00u: idx = 0; break;   // entry
            case 0x0E68u: idx = 1; break;   // FCAND clip-flag test
            case 0x0E80u: idx = 2; break;   // past the IBEQ, doing the work
            case 0x0EF8u: idx = 3; break;   // BAL to the subroutine
            case 0x0F10u: idx = 4; break;   // the interpolation DIV
            default: break;
            }
            if (idx >= 0)
            {
                const uint64_t c = s_f[idx].fetch_add(1, std::memory_order_relaxed);
                if (idx == 0 && (c == 0u || (c % 5u) == 4u))
                    std::fprintf(stderr,
                                 "[MOH:clip-funnel-v1] entry0E00=%llu fcand0E68=%llu work0E80=%llu bal0EF8=%llu div0F10=%llu\n",
                                 (unsigned long long)s_f[0].load(std::memory_order_relaxed),
                                 (unsigned long long)s_f[1].load(std::memory_order_relaxed),
                                 (unsigned long long)s_f[2].load(std::memory_order_relaxed),
                                 (unsigned long long)s_f[3].load(std::memory_order_relaxed),
                                 (unsigned long long)s_f[4].load(std::memory_order_relaxed));
            }
        }
        // Which high microcode regions actually execute? The game uploads code up to
        // ~0x2110 but every observed entry point is below 0x0948 and no traced route goes
        // past 0x1E50. A never-executed region would be a strong candidate for the missing
        // clipping routine.
        {
            static std::atomic<uint64_t> s_reg[18] = {};
            const uint32_t r = m_state.pc >> 9;   // 0x200-byte buckets
            if (r < 18u)
            {
                const uint64_t c = s_reg[r].fetch_add(1, std::memory_order_relaxed);
                if ((c % 2000000u) == 1999999u)
                {
                    std::fprintf(stderr, "[MOH:pc-regions-v1]");
                    for (uint32_t i = 0u; i < 18u; ++i)
                        std::fprintf(stderr, " 0x%04x=%llu", i * 0x200u,
                                     (unsigned long long)s_reg[i].load(std::memory_order_relaxed));
                    std::fprintf(stderr, "\n");
                }
            }
        }
        if (mohvu::ProgramStats *psRun = mohvu::t_currentProgram)
            psRun->instrCount.fetch_add(1, std::memory_order_relaxed);

        // One-shot disassembly around the saturating FTOI4 at 0x1440: does the
        // microprogram clamp XY (MAX/MINI, upper ops 0x2B/0x2F/0x13/0x17/0x1D/0x1F)
        // before converting to 12.4 fixed point?
        if (m_state.pc == 0x1440u)
        {
            static std::atomic<bool> s_ctxDumped{false};
            bool expected = false;
            if (s_ctxDumped.compare_exchange_strong(expected, true))
            {
                for (uint32_t a = 0x13C0u; a <= 0x1448u && a + 8u <= codeSize; a += 8u)
                {
                    uint32_t lo, hi;
                    std::memcpy(&lo, vuCode + a, 4);
                    std::memcpy(&hi, vuCode + a + 4, 4);
                    const uint32_t upOp = hi & 0x3Fu;
                    const char *name =
                        upOp == 0x2Bu ? "MAX" : upOp == 0x2Fu ? "MINI"
                        : upOp == 0x13u ? "MAXbc" : upOp == 0x17u ? "MINIbc"
                        : upOp == 0x1Du ? "MAXi" : upOp == 0x1Fu ? "MINIi/CLIP"
                        : upOp == 0x15u ? "FTOI4" : "";
                    std::fprintf(stderr,
                                 "[MOH:ftoi4-ctx-v1] pc=0x%04x lower=%08x upper=%08x upOp=0x%02x %s\n",
                                 a, lo, hi, upOp, name);
                }
            }
        }

        // FTOI4 at 0x1440 is `vf20 = FTOI4(vf02)` (fs=2, ft=20, special group
        // writes FT). Trace vf02 across the window so the instruction that gives
        // it the out-of-range X can be identified.
        if (m_state.pc >= 0x13F8u && m_state.pc <= 0x1440u)
        {
            static std::atomic<uint32_t> s_traced{0};
            if (s_traced.load(std::memory_order_relaxed) < 44u)
            {
                s_traced.fetch_add(1, std::memory_order_relaxed);
                std::fprintf(stderr,
                             "[MOH:vf02-trace-v1] pc=0x%04x vf02=(%.3f %.3f %.3f %.3f) Q=%.6f I=%.3f\n",
                             m_state.pc, m_state.vf[2][0], m_state.vf[2][1],
                             m_state.vf[2][2], m_state.vf[2][3],
                             (double)m_state.q, (double)m_state.i);
            }
        }

        // One-shot disassembly of the nearly-dead region 0x0E00..0x0FFF, dumped the first
        // time execution reaches it so the code is known to be loaded. Both pipelines are
        // decoded: an upper NOP can still carry a lower-pipeline DIV, and missing that is
        // what made the 0x12F0 route look arithmetic-free earlier.
        if (m_state.pc >= 0x0E00u && m_state.pc < 0x1000u)
        {
            static std::atomic<bool> s_deadDumped{false};
            bool e0 = false;
            if (s_deadDumped.compare_exchange_strong(e0, true))
            {
                for (uint32_t a = 0x0E00u; a < 0x1000u && a + 8u <= codeSize; a += 8u)
                {
                    uint32_t lo, hi;
                    std::memcpy(&lo, vuCode + a, 4);
                    std::memcpy(&hi, vuCode + a + 4, 4);
                    const uint32_t upOp = hi & 0x3Fu;
                    const uint32_t lop = lo >> 25;
                    const uint32_t lfun = lo & 0x3Fu;
                    const uint32_t lsub = (lo & 3u) | ((lo >> 4) & 0x7Cu);
                    const char *ln = "";
                    if (lop == 0x40u && lfun >= 0x3Cu)
                        ln = (lsub == 0x38u) ? " LOWER-DIV" : (lsub == 0x39u) ? " LOWER-SQRT"
                             : (lsub == 0x3Au) ? " LOWER-RSQRT" : (lsub == 0x3Bu) ? " WAITQ" : "";
                    const char *un = (upOp >= 0x08u && upOp <= 0x0Bu) ? " MADDbc"
                                     : (upOp >= 0x0Cu && upOp <= 0x0Fu) ? " MSUBbc"
                                     : (upOp == 0x21u) ? " MADDq" : (upOp == 0x25u) ? " MSUBq"
                                     : (upOp == 0x1Cu) ? " MULq" : "";
                    // Name the lower op too, so branches and stores are visible and the
                    // routine's shape can be read.
                    const char *lname = "";
                    if (lop == 0x40u && lfun < 0x3Cu)
                        lname = (lfun == 0x30u) ? "IADD" : (lfun == 0x31u) ? "ISUB"
                                : (lfun == 0x32u) ? "IADDI" : (lfun == 0x34u) ? "IAND"
                                : (lfun == 0x35u) ? "IOR" : "int";
                    else if (lop == 0x40u)
                        lname = (lsub == 0x30u) ? "MOVE" : (lsub == 0x34u) ? "LQI"
                                : (lsub == 0x35u) ? "SQI" : (lsub == 0x3Cu) ? "MTIR"
                                : (lsub == 0x3Du) ? "MFIR" : (lsub == 0x3Eu) ? "ILWR"
                                : (lsub == 0x3Fu) ? "ISWR" : "spec";
                    else
                        lname = (lop == 0x00u) ? "LQ" : (lop == 0x01u) ? "SQ"
                                : (lop == 0x04u) ? "ILW" : (lop == 0x05u) ? "ISW"
                                : (lop == 0x08u) ? "IADDIU" : (lop == 0x12u) ? "FCAND"
                                : (lop == 0x13u) ? "FCOR" : (lop == 0x20u) ? "B"
                                : (lop == 0x21u) ? "BAL" : (lop == 0x24u) ? "JR"
                                : (lop == 0x28u) ? "IBEQ" : (lop == 0x29u) ? "IBNE"
                                : (lop == 0x2Cu) ? "IBLTZ" : (lop == 0x2Du) ? "IBGTZ"
                                : (lop == 0x2Eu) ? "IBLEZ" : (lop == 0x2Fu) ? "IBGEZ" : "?";
                    std::fprintf(stderr,
                                 "[MOH:dead-region-v1] pc=0x%04x up=%08x(0x%02x)%s lo=%08x %s%s\n",
                                 a, hi, upOp, un, lo, lname, ln);
                }
            }
        }

        // One-shot static scan of the whole uploaded microcode for branches targeting the
        // dead clip region. A VU branch goes to pc + 8 + imm*8 with an 11-bit signed imm.
        // This finds the dispatch site that should route partially-visible triangles there.
        {
            static std::atomic<bool> s_scanned{false};
            bool e1 = false;
            if (m_state.pc > 0x1000u && s_scanned.compare_exchange_strong(e1, true))
            {
                for (uint32_t a = 0u; a + 8u <= codeSize && a < 0x2200u; a += 8u)
                {
                    uint32_t lo;
                    std::memcpy(&lo, vuCode + a, 4);
                    const uint32_t op = lo >> 25;
                    const bool isBr = (op == 0x20u || op == 0x21u || op == 0x28u ||
                                       op == 0x29u || op == 0x2Cu || op == 0x2Du ||
                                       op == 0x2Eu || op == 0x2Fu);
                    if (!isBr) continue;
                    int32_t imm = (int32_t)(lo & 0x7FFu);
                    if (imm & 0x400) imm -= 0x800;          // sign-extend 11 bits
                    const int32_t tgt = (int32_t)a + 8 + imm * 8;
                    if (tgt >= 0x0E00 && tgt < 0x1000)
                    {
                        const char *n = (op == 0x20u) ? "B" : (op == 0x21u) ? "BAL"
                                        : (op == 0x28u) ? "IBEQ" : (op == 0x29u) ? "IBNE"
                                        : (op == 0x2Cu) ? "IBLTZ" : (op == 0x2Du) ? "IBGTZ"
                                        : (op == 0x2Eu) ? "IBLEZ" : "IBGEZ";
                        std::fprintf(stderr,
                                     "[MOH:clip-dispatch-v1] branch at 0x%04x  %s  -> 0x%04x  lo=%08x\n",
                                     a, n, (unsigned)tgt, lo);
                    }
                }
                std::fprintf(stderr, "[MOH:clip-dispatch-v1] scan complete\n");
            }
        }

        // Census of what our VU1 microcode CONTAINS (not what we execute), to compare
        // like with like against the console: PCSX2 now exposes VU1 micro memory via the
        // read_vu_memory command added to its DebugServer, and shows 16 FTOI4 sites.
        {
            static std::atomic<uint32_t> s_censusN{0};
            const uint32_t cn = s_censusN.fetch_add(1u, std::memory_order_relaxed);
            if (cn == 200u || cn == 40000u)
            {
                uint32_t nFtoi = 0, nClip = 0, nDiv = 0, nWaitq = 0, nMiniMax = 0;
                for (uint32_t a = 0u; a + 8u <= codeSize && a < 0x4000u; a += 8u)
                {
                    uint32_t lo, hi;
                    std::memcpy(&lo, vuCode + a, 4);
                    std::memcpy(&hi, vuCode + a + 4, 4);
                    const uint32_t uo = hi & 0x3Fu, us = (hi & 3u) | ((hi >> 4) & 0x7Cu);
                    if (uo >= 0x3Cu && us == 0x15u) ++nFtoi;
                    if (uo >= 0x3Cu && us == 0x1Fu) ++nClip;
                    if (uo == 0x1Fu || uo == 0x2Bu || uo == 0x1Du || uo == 0x2Fu) ++nMiniMax;
                    if ((lo >> 25) == 0x40u && (lo & 0x3Fu) >= 0x3Cu)
                    {
                        const uint32_t ls = (lo & 3u) | ((lo >> 4) & 0x7Cu);
                        if (ls == 0x38u) ++nDiv;
                        if (ls == 0x3Bu) ++nWaitq;
                    }
                }
                std::fprintf(stderr,
                             "[MOH:micro-census-v1] n=%u FTOI4=%u CLIP=%u DIV=%u WAITQ=%u MINI/MAX=%u\n",
                             cn, nFtoi, nClip, nDiv, nWaitq, nMiniMax);
                // The 16 FTOI4 addresses, so the 8 we execute can be set against the 8 we
                // never reach. And per-4KB hashes, to check the microcode is byte-identical
                // to the console's (which reports d66b7747/cb552e23/8ed6c216/5bd1c383).
                std::fprintf(stderr, "[MOH:micro-census-v1] sites FTOI4:");
                for (uint32_t a = 0u; a + 8u <= codeSize && a < 0x4000u; a += 8u)
                {
                    uint32_t hi;
                    std::memcpy(&hi, vuCode + a + 4, 4);
                    const uint32_t uo = hi & 0x3Fu, us = (hi & 3u) | ((hi >> 4) & 0x7Cu);
                    if (uo >= 0x3Cu && us == 0x15u)
                        std::fprintf(stderr, " 0x%04x", a);
                }
                // Dump block 2 (0x1000..0x1FFF) verbatim: its hash differs from the
                // console's while blocks 0 and 1 match byte-for-byte, and it is the block
                // containing the offending FTOI4 site 0x1440.
                // VU1 data memory at the screen-mapping matrix. The console holds
                // 320 / -224 / -2047.9688 with offsets 2048 / 2048 / 2047.9688 at
                // 0x00A0..0x00D0; ours reconstructs to 32767.5 for Z, exactly 16x larger,
                // which is the factor FTOI4 applies. Dump ours at the same offsets.
                if (vuData && dataSize >= 0x100u)
                {
                    std::fprintf(stderr, "[MOH:vu1-data-dump-v1] matrice ecran :\n");
                    for (uint32_t a = 0x0090u; a <= 0x00E0u; a += 16u)
                    {
                        float q[4];
                        std::memcpy(q, vuData + a, 16);
                        std::fprintf(stderr,
                                     "[MOH:vu1-data-dump-v1]   0x%04x: %14.4f %14.4f %14.4f %14.4f\n",
                                     a, (double)q[0], (double)q[1], (double)q[2], (double)q[3]);
                    }
                }
                std::fprintf(stderr, "[MOH:micro-blk2-v1] ");
                for (uint32_t i = 0x1000u; i < 0x2000u && i < codeSize; ++i)
                    std::fprintf(stderr, "%02x", vuCode[i]);
                std::fprintf(stderr, "\n");
                std::fprintf(stderr, "\n[MOH:micro-census-v1] hashes:");
                for (uint32_t blk = 0u; blk < 4u; ++blk)
                {
                    uint64_t h = 1469598103934665603ull;
                    for (uint32_t i = 0u; i < 0x1000u; ++i)
                    {
                        const uint32_t a = blk * 0x1000u + i;
                        if (a >= codeSize) break;
                        h ^= vuCode[a]; h *= 1099511628211ull;
                    }
                    std::fprintf(stderr, " %08x", (unsigned)(h & 0xFFFFFFFFu));
                }
                std::fprintf(stderr, "\n");
            }
        }

        const DecodedInstructionPair decoded = getDecodedInstructionPairForPc(vuCode, codeSize, memory, m_state.pc);

        // Who writes vf28, and with what? The register was dumped as
        // (2048, 2048, 32767.5, 1) while VU1 data at 0x00D0 holds
        // (2048, 2048, 2047.9688, 1) - Z exactly 16x larger, which is the FTOI4 factor.
        // Record the transition and the instruction responsible.
        const float vf28zBefore = m_state.vf[28][2];

        // Record into the history ring, then honour a dump request raised by FTOI4.
        {
            const uint32_t hp = mohvu::t_histPos % mohvu::kHistN;
            mohvu::t_histPc[hp] = m_state.pc;
            mohvu::t_histUp[hp] = decoded.upper;
            mohvu::t_histLo[hp] = decoded.lower;
            ++mohvu::t_histPos;
            if (mohvu::t_histDumpRequested)
            {
                mohvu::t_histDumpRequested = false;
                bool e2 = false;
                if (mohvu::g_histDumped.compare_exchange_strong(e2, true))
                {
                    std::fprintf(stderr, "[MOH:wrap-history-v1] 64 instructions avant enroulement\n");
                    for (uint32_t i = 0u; i < mohvu::kHistN; ++i)
                    {
                        const uint32_t idx = (mohvu::t_histPos + i) % mohvu::kHistN;
                        if (!mohvu::t_histPc[idx] && !mohvu::t_histUp[idx]) continue;
                        const uint32_t uo = mohvu::t_histUp[idx] & 0x3Fu;
                        const uint32_t lop = mohvu::t_histLo[idx] >> 25;
                        const uint32_t lsub = (mohvu::t_histLo[idx] & 3u) |
                                              ((mohvu::t_histLo[idx] >> 4) & 0x7Cu);
                        const char *tag = "";
                        if (lop == 0x40u && (mohvu::t_histLo[idx] & 0x3Fu) >= 0x3Cu)
                            tag = (lsub == 0x38u) ? " DIV" : (lsub == 0x3Bu) ? " WAITQ"
                                  : (lsub == 0x34u) ? " LQI" : (lsub == 0x35u) ? " SQI" : "";
                        std::fprintf(stderr,
                                     "[MOH:wrap-history-v1] %02u pc=0x%04x up=%08x(0x%02x) lo=%08x%s\n",
                                     i, mohvu::t_histPc[idx], mohvu::t_histUp[idx], uo,
                                     mohvu::t_histLo[idx], tag);
                    }
                }
            }
        }


        // Trace the executed path from the MADDw at 0x13B8 (which writes vf20) to the
        // FTOI4 at 0x1440 that converts it, following branches. The question is whether
        // any instruction on that path multiplies vf20 by Q; the linear dump of the
        // address range could not answer it because it does not follow control flow.
        {
            if (mohvu::t_pathState == 1)
            {
                const uint32_t n = mohvu::t_pathN++;
                const uint32_t upOp = decoded.upper & 0x3Fu;
                const bool usesQ = (upOp == 0x1Cu) || (upOp == 0x20u) || (upOp == 0x21u) ||
                                   (upOp == 0x24u) || (upOp == 0x25u);
                std::fprintf(stderr,
                             "[MOH:vf20-path-v1] n=%02u pc=0x%04x up=%08x upOp=0x%02x lo=%08x%s"
                             " vf20x=%.3f Q=%.6f\n",
                             n, m_state.pc, decoded.upper, upOp, decoded.lower,
                             usesQ ? "  <<< USES Q" : "", m_state.vf[20][0], (double)m_state.q);
                if (m_state.pc == 0x1440u || n >= 40u)
                    mohvu::t_pathState = 2;
            }
        }

        // LOI is controlled by the upper I-bit.  The lower word is the float immediate.
        // DobieStation executes the upper instruction first, then commits lower into I.
        if (decoded.iBit)
        {
            // LOI is special: the upper instruction sees the old I value, then LOI loads I.
            execUpper(decoded.upper);
            std::memcpy(&m_state.i, &decoded.lower, sizeof(decoded.lower));
        }
        else if (decoded.lowerBeforeUpper)
        {
            // VU upper/lower execute as a pair.  If the upper op writes a VF register
            // that the lower op reads or also writes, Dobie runs the lower side first
            // so it observes the old VF value and the upper write has priority.
            execLower(decoded.lower, vuData, dataSize, gs, memory, decoded.upper);
            execUpper(decoded.upper);
        }
        else
        {
            execUpper(decoded.upper);
            execLower(decoded.lower, vuData, dataSize, gs, memory, decoded.upper);
        }

        // Arm the path trace only on a *saturating* case: the previous attempt armed on
        // the first pass through 0x13B8, which processed a zero vertex and therefore
        // could not show what happens to Q for the vertices that actually overflow.
        if (mohvu::t_pathState == 0 && m_state.pc == 0x13B8u &&
            std::fabs(m_state.vf[20][0]) > 10000.0f)
        {
            mohvu::t_pathState = 1;
            mohvu::t_pathN = 0;
            std::fprintf(stderr,
                         "[MOH:vf20-path-v1] ARM sur cas saturant vf20=(%.3f %.3f %.3f %.3f) Q=%.6f\n",
                         m_state.vf[20][0], m_state.vf[20][1], m_state.vf[20][2],
                         m_state.vf[20][3], (double)m_state.q);
        }

        // Retire the pending divide: Q becomes visible once the latency elapses.
        if (mohvu::t_qDelay > 0 && --mohvu::t_qDelay == 0)
            m_state.q = mohvu::t_qPending;

        {
            const float za = m_state.vf[28][2];
            if (za != vf28zBefore)
            {
                static std::atomic<uint32_t> s_w28{0};
                const uint32_t k = s_w28.fetch_add(1u, std::memory_order_relaxed);
                if (k < 10u)
                    std::fprintf(stderr,
                                 "[MOH:vf28-writer-v1] k=%u pc=0x%04x up=%08x lo=%08x"
                                 " vf28.z %.4f -> %.4f  vf28=(%.4f %.4f %.4f %.4f)\n",
                                 k, m_state.pc, decoded.upper, decoded.lower,
                                 (double)vf28zBefore, (double)za,
                                 m_state.vf[28][0], m_state.vf[28][1],
                                 m_state.vf[28][2], m_state.vf[28][3]);
            }
        }

        // [MOH diag] Fast-path arming. 0x13E0 is taken when the FCAND at 0x12d8
        // reports "nothing outside", so a triangle reaching it is treated as
        // fully inside. A vertex with W<0 but |x|,|y|,|z| < |W| gets CLIP
        // flags=0x00 and slips through here - the suspected source of the GS
        // primitives with negative Q. Arm on exactly that, once.
        if (false /* 0x13E0 also reached from the 0x1390 loop tail: not discriminant */ &&
            m_state.pc == 0x13E0u &&
            mohvu::g_chronoState.load(std::memory_order_relaxed) == 0 &&
            mohvu::t_currentProgram &&
            mohvu::t_currentProgram->startPc.load(std::memory_order_relaxed) == 0u)
        {
            bool anyNegW = false;
            for (uint32_t k = 0; k < 3u; ++k)
                if (mohvu::t_clipRing[(mohvu::t_clipRingPos - 1u - k) & 3u].w < 0.0f)
                    anyNegW = true;
            if (anyNegW)
            {
                mohvu::g_chronoState.store(1, std::memory_order_relaxed);
                std::fprintf(stderr,
                    "[MOH:vu1-fastpath-negw-v1] ARM pc=0x13e0 CF=%06x\n",
                    (unsigned)(m_state.clip & 0xFFFFFFu));
                for (uint32_t k = 0; k < 3u; ++k)
                {
                    const mohvu::ClipRecord &rec =
                        mohvu::t_clipRing[(mohvu::t_clipRingPos - 1u - k) & 3u];
                    uint32_t xb, yb, zb;
                    std::memcpy(&xb, &rec.x, 4); std::memcpy(&yb, &rec.y, 4);
                    std::memcpy(&zb, &rec.z, 4);
                    std::fprintf(stderr,
                        "[MOH:vu1-fastpath-negw-v1]   vtx[-%u] X=%.6f Y=%.6f Z=%.6f "
                        "W=%.6f bits=[%08x %08x %08x %08x] flags=0x%02x "
                        "absCmp=[%d %d %d]\n",
                        k, rec.x, rec.y, rec.z, rec.w,
                        (unsigned)xb, (unsigned)yb, (unsigned)zb,
                        (unsigned)rec.wBits, (unsigned)rec.flags,
                        (int)(std::fabs(rec.x) < std::fabs(rec.w)),
                        (int)(std::fabs(rec.y) < std::fabs(rec.w)),
                        (int)(std::fabs(rec.z) < std::fabs(rec.w)));
                }
            }
        }

        if (mohvu::chronoArmed())
        {
            // Inside the clipping subroutines (BAL 0x1540 / BAL 0x1d98) log every
            // instruction with Q and ACC: geometric clipping computes the
            // intersection coefficient with DIV, whose result latency and WAITQ
            // handling are the classic emulation hazard. Elsewhere keep the
            // compact control-flow/store view so the trace still spans a batch.
            const uint32_t lo = decoded.lower;
            const uint32_t opHi = (lo >> 25) & 0x7Fu;
            const bool upperIsNop = (decoded.upper & 0x3Fu) == 0x3Fu ||
                                    decoded.upper == 0x000002FFu;
            // Only arithmetic (non-NOP upper) inside the route: this is where any
            // real clipping math would show up if it exists further along.
            const bool inClipSub = (m_state.pc >= 0x12F0u) && !upperIsNop;
            const bool isBranch = (opHi >= 0x20u && opHi <= 0x2Fu);
            const bool isStore = (opHi == 0x01u) || (opHi == 0x05u) ||
                                 (opHi == 0x40u && ((lo & 0x3Fu) == 0x35u ||
                                                    (lo & 0x3Fu) == 0x37u ||
                                                    (lo & 0x3Fu) == 0x3Fu));
            if ((inClipSub || isBranch || isStore) && mohvu::chronoTakeLine())
            {
                const uint32_t loFunct = lo & 0x3Fu;
                const char *tag = inClipSub ? "SUB" : (isBranch ? "BR " : "ST ");
                std::fprintf(stderr,
                    "[MOH:vu1-clipsub-v1] pc=0x%04x %s up=%08x lo=%08x opHi=%02x "
                    "f=%02x Q=%g ACC=(%g,%g,%g,%g)\n",
                    (unsigned)m_state.pc, tag, (unsigned)decoded.upper,
                    (unsigned)lo, (unsigned)opHi, (unsigned)loFunct,
                    m_state.q, m_state.acc[0], m_state.acc[1],
                    m_state.acc[2], m_state.acc[3]);
            }
        }

        // Enforce VF0 invariant
        m_state.vf[0][0] = 0.0f;
        m_state.vf[0][1] = 0.0f;
        m_state.vf[0][2] = 0.0f;
        m_state.vf[0][3] = 1.0f;
        // Enforce VI0 invariant
        m_state.vi[0] = 0;

        uint32_t nextPC = m_state.pc + 8;
        if (nextPC >= codeSize)
            nextPC = 0;
        m_state.pc = nextPC;

        // VU branch/jump has a delay slot. Branch handlers set a pending target;
        // we execute one sequential instruction before committing the branch.
        if (m_state.branchPending)
        {
            if (m_state.branchDelay == 0)
            {
                m_state.pc = m_state.branchTarget & 0x3FFFu;
                m_state.branchPending = false;
            }
            else
            {
                --m_state.branchDelay;
            }
        }

        if (m_state.ebit)
            break;

        if (decoded.eBit)
            m_state.ebit = true;
    }
    if (retired <= 1u)
    {
        if (mohvu::ProgramStats *psRun = mohvu::t_currentProgram)
            psRun->earlyExit.fetch_add(1, std::memory_order_relaxed);
    }
}
