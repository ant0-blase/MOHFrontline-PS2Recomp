#ifndef PS2_VU1_DETAIL_H
#define PS2_VU1_DETAIL_H

#include <cstdint>
#include <array>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// [MOH diag] Per-microprogram VU1 profile.
//
// Question: near-plane-crossing primitives reach the GS with negative Q while
// CLIP/FCAND/FCOR all execute somewhere. Aggregating per MSCAL entry point shows
// WHICH program emits the scene geometry (XGKICK) and whether that same program
// ever consumes the clip flags it produces. Bounded: fixed slot table, periodic
// dump only.
namespace mohvu
{
    // VU DIV/SQRT/RSQRT latency. On hardware Q is not visible for several cycles
    // after the divide issues, and microprograms exploit that window: they issue the
    // divide for vertex N+1 while still consuming vertex N's Q. An interpreter that
    // updates Q immediately hands the MULq the wrong vertex's Q. Enabled with
    // PS2_MOH_VU_DIV_LATENCY=1 so the two behaviours can be compared.
    // Path trace armed on a saturating vf20, shared between the arming point
    // (post-execution of the MADDw) and the logging point (pre-execution).
    inline thread_local int t_pathState = 0;   // 0 idle, 1 tracing, 2 done
    inline thread_local uint32_t t_pathN = 0;

    // Global WAITQ tick, so the FTOI4 site can tell whether interpolation ran between
    // a triangle's three conversions. Aggregate counts cannot answer that.
    inline std::atomic<uint64_t> g_waitqTicks{0};
    // Monotonic XGKICK counter, so per-batch statistics can be gathered: a wrap that
    // clusters in a few batches means one object is misplaced, whereas an even spread
    // means the whole submission is off.
    inline std::atomic<uint64_t> g_xgkickSeq{0};

    // Resident program variant per 0x800 bank, set at MPG upload. Each bank cycles among
    // only three distinct programs, so tagging measurements by variant is tractable and is
    // what makes per-address statements well-formed.
    inline std::atomic<uint32_t> g_bankVariant[8] = {};
    inline std::atomic<uint64_t> g_wrapByVariant[4] = {};
    inline std::atomic<uint64_t> g_convByVariant[4] = {};

    // Ring of recent instructions, dumped when a wrapping FTOI4 is detected. Tracing
    // forward from the wrap only shows the packet write; the history leading to it is what
    // has never been observed.
    inline constexpr uint32_t kHistN = 64u;
    inline thread_local uint32_t t_histPc[kHistN] = {};
    inline thread_local uint32_t t_histUp[kHistN] = {};
    inline thread_local uint32_t t_histLo[kHistN] = {};
    inline thread_local uint32_t t_histPos = 0u;
    inline thread_local bool t_histDumpRequested = false;
    inline std::atomic<bool> g_histDumped{false};

    inline thread_local float t_qPending = 0.0f;
    inline thread_local int t_qDelay = 0;

    inline bool divLatencyEnabled()
    {
        static const bool on = [] {
            const char *v = std::getenv("PS2_MOH_VU_DIV_LATENCY");
            return v && !std::strcmp(v, "1");
        }();
        return on;
    }

    struct ProgramStats
    {
        std::atomic<uint32_t> startPc{0xFFFFFFFFu};
        std::atomic<uint64_t> clips{0};
        std::atomic<uint64_t> clipsWithFlags{0};
        std::atomic<uint64_t> clipsNegW{0};
        // Vertices behind the near plane (W<0) whose |x|,|y|,|z| are all smaller
        // than |W|: CLIP legitimately reports flags=0x00 for them, so the
        // six-plane trivial reject cannot catch them and the triangle is emitted
        // with a negative Q. Counting them tells whether this is the real source
        // of the pathological GS primitives.
        std::atomic<uint64_t> clipsNegWUnflagged{0};
        // Per-plane tally of CLIP outcomes. The trivial-reject chain ANDs each
        // of these across the three vertices of a triangle, so a plane that is
        // never flagged at all can never reject.
        std::atomic<uint64_t> clipBit0{0}; // x > +w
        std::atomic<uint64_t> clipBit1{0}; // x < -w
        std::atomic<uint64_t> clipBit2{0}; // y > +w
        std::atomic<uint64_t> clipBit3{0}; // y < -w
        std::atomic<uint64_t> clipBit4{0}; // z > +w
        std::atomic<uint64_t> clipBit5{0}; // z < -w
        // Instructions actually retired by this microprogram, and how many
        // activations ended without retiring more than one. A program invoked
        // heavily but retiring almost nothing is one the interpreter is failing
        // to run, not one that legitimately does nothing.
        // Per-program correlation: does the program that produces wrapped coordinates
        // also run divide-stalling interpolation? Address histograms cannot answer this
        // because the microcode banks are rewritten ~86 times per run.
        std::atomic<uint64_t> waitqCount{0};
        std::atomic<uint64_t> wrapCount{0};

        std::atomic<uint64_t> instrCount{0};
        std::atomic<uint64_t> earlyExit{0};
        std::atomic<uint64_t> fcand{0};
        std::atomic<uint64_t> fcor{0};
        std::atomic<uint64_t> fceq{0};
        std::atomic<uint64_t> fcandTrue{0};
        std::atomic<uint64_t> fcorTrue{0};
        std::atomic<uint64_t> xgkick{0};
        std::atomic<uint64_t> runs{0};
    };

    // Set at every XGKICK so the GS-side pathological-draw probe can attribute a
    // bad primitive to the exact VU1 microprogram that emitted it.
    inline std::atomic<uint32_t> g_lastXgkickProgram{0xFFFFFFFFu};

    // Chronology of ONE rejection decision in the scene program: armed on a CLIP
    // whose W is negative (vertex behind the near plane), then every instruction
    // is logged until the emitting XGKICK, so the branch that should have
    // rejected the primitive can be seen taking its decision.
    // How many CLIP instructions have executed since the last one whose W was
    // negative (vertex behind the near plane). CF keeps only the last four CLIP
    // results, so a value < 4 means that vertex's 6-bit group is still inside the
    // 24-bit CF window that a per-triangle FCAND/FCOR is about to consume.
    // Ring of the last four CLIP results, mirroring the 24-bit CF window, so an
    // armed per-triangle test can report the actual X/Y/Z/W (and raw W bits) of
    // the three vertices whose flags it is consuming.
    struct ClipRecord
    {
        float x, y, z, w;
        uint32_t wBits;
        uint32_t flags;
        uint32_t pc;
    };
    inline thread_local ClipRecord t_clipRing[4] = {};
    inline thread_local uint32_t t_clipRingPos = 0u;

    inline thread_local bool t_pendingSurvivor = false;
    inline thread_local uint32_t t_clipsSinceNegW = 999u;
    inline thread_local uint32_t t_lastNegWFlags = 0u;
    inline thread_local float t_lastNegW = 0.0f;
    // Purely host-side correlation id (never written to guest memory).
    inline std::atomic<uint64_t> g_triangleTraceId{0};
    inline std::atomic<uint64_t> g_armedTraceId{0};

    inline std::atomic<int> g_chronoState{0}; // 0 idle, 1 armed, 2 done
    inline std::atomic<int> g_chronoLines{0};
    inline constexpr int kChronoMaxLines = 900;

    // Distinct (pc, opcode, imm24) triples seen for FCAND/FCOR/FCEQ, with a hit
    // count each. The immediates encode exactly which planes the microprogram
    // tests, which is what decides whether a triangle is rejected or emitted.
    struct FlagImm
    {
        uint32_t pc;
        uint32_t imm;
        const char *op;
        std::atomic<uint64_t> hits;
        std::atomic<uint64_t> trueHits;
    };
    inline constexpr int kFlagImmMax = 24;
    inline std::array<FlagImm, kFlagImmMax> g_flagImm{};
    inline std::atomic<int> g_flagImmCount{0};

    inline void noteFlagImm(uint32_t pc, uint32_t imm, const char *op, bool wasTrue)
    {
        const int n = g_flagImmCount.load(std::memory_order_acquire);
        for (int i = 0; i < n; ++i)
        {
            if (g_flagImm[i].pc == pc && g_flagImm[i].imm == imm)
            {
                g_flagImm[i].hits.fetch_add(1, std::memory_order_relaxed);
                if (wasTrue)
                    g_flagImm[i].trueHits.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }
        if (n >= kFlagImmMax)
            return;
        g_flagImm[n].pc = pc;
        g_flagImm[n].imm = imm;
        g_flagImm[n].op = op;
        g_flagImm[n].hits.store(1, std::memory_order_relaxed);
        g_flagImm[n].trueHits.store(wasTrue ? 1u : 0u, std::memory_order_relaxed);
        g_flagImmCount.store(n + 1, std::memory_order_release);
    }

    inline void dumpFlagImm()
    {
        const int n = g_flagImmCount.load(std::memory_order_acquire);
        for (int i = 0; i < n; ++i)
        {
            std::fprintf(stderr,
                         "[MOH:vu1-flagimm-v1] %s pc=0x%04x imm=0x%06x hits=%llu true=%llu\n",
                         g_flagImm[i].op, g_flagImm[i].pc, g_flagImm[i].imm,
                         (unsigned long long)g_flagImm[i].hits.load(std::memory_order_relaxed),
                         (unsigned long long)g_flagImm[i].trueHits.load(std::memory_order_relaxed));
        }
    }
    inline bool chronoArmed()
    {
        return g_chronoState.load(std::memory_order_relaxed) == 1;
    }
    inline bool chronoTakeLine()
    {
        if (g_chronoLines.fetch_add(1, std::memory_order_relaxed) < kChronoMaxLines)
            return true;
        g_chronoState.store(2, std::memory_order_relaxed);
        return false;
    }

    // Map of every XGKICK site executed by the scene program: which micro-PC
    // emits, from which VI register/address, and how often. Needed because the
    // 0x1260..0x12a8 reject block turned out to sit INSIDE the per-vertex loop,
    // so the GIF emission happens at an as-yet-unknown end-of-batch site.
    struct XgkickSite
    {
        std::atomic<uint32_t> pc{0xFFFFFFFFu};
        std::atomic<uint32_t> viReg{0};
        std::atomic<uint32_t> lastAddr{0};
        std::atomic<uint64_t> count{0};
    };
    inline constexpr int kMaxXgkickSites = 16;
    inline XgkickSite g_xgkickSites[kMaxXgkickSites];
    inline std::atomic<int> g_xgkickSiteCount{0};

    inline void noteXgkick(uint32_t pc, uint32_t viReg, uint32_t addr)
    {
        const int used = g_xgkickSiteCount.load(std::memory_order_relaxed);
        for (int i = 0; i < used && i < kMaxXgkickSites; ++i)
        {
            if (g_xgkickSites[i].pc.load(std::memory_order_relaxed) == pc)
            {
                g_xgkickSites[i].viReg.store(viReg, std::memory_order_relaxed);
                g_xgkickSites[i].lastAddr.store(addr, std::memory_order_relaxed);
                g_xgkickSites[i].count.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }
        const int slot = g_xgkickSiteCount.fetch_add(1, std::memory_order_relaxed);
        if (slot >= kMaxXgkickSites)
            return;
        g_xgkickSites[slot].pc.store(pc, std::memory_order_relaxed);
        g_xgkickSites[slot].viReg.store(viReg, std::memory_order_relaxed);
        g_xgkickSites[slot].lastAddr.store(addr, std::memory_order_relaxed);
        g_xgkickSites[slot].count.fetch_add(1, std::memory_order_relaxed);
    }

    inline void dumpXgkickSites()
    {
        const int used = g_xgkickSiteCount.load(std::memory_order_relaxed);
        std::fprintf(stderr, "[MOH:vu1-xgkick-map-v1]");
        for (int i = 0; i < used && i < kMaxXgkickSites; ++i)
            std::fprintf(stderr, " {pc=0x%04x vi=%u addr=0x%04x n=%llu}",
                (unsigned)g_xgkickSites[i].pc.load(std::memory_order_relaxed),
                (unsigned)g_xgkickSites[i].viReg.load(std::memory_order_relaxed),
                (unsigned)g_xgkickSites[i].lastAddr.load(std::memory_order_relaxed),
                (unsigned long long)g_xgkickSites[i].count.load(std::memory_order_relaxed));
        std::fprintf(stderr, "\n");
    }

    inline constexpr int kMaxPrograms = 24;
    inline ProgramStats g_programs[kMaxPrograms];
    inline std::atomic<int> g_programCount{0};
    inline thread_local ProgramStats *t_currentProgram = nullptr;

    inline ProgramStats *programFor(uint32_t startPc)
    {
        const int used = g_programCount.load(std::memory_order_relaxed);
        for (int i = 0; i < used && i < kMaxPrograms; ++i)
            if (g_programs[i].startPc.load(std::memory_order_relaxed) == startPc)
                return &g_programs[i];
        const int slot = g_programCount.fetch_add(1, std::memory_order_relaxed);
        if (slot >= kMaxPrograms)
            return nullptr;
        g_programs[slot].startPc.store(startPc, std::memory_order_relaxed);
        return &g_programs[slot];
    }

    inline void dumpPrograms()
    {
        const int used = g_programCount.load(std::memory_order_relaxed);
        std::fprintf(stderr, "[MOH:vu1-program-profile-v1]");
        for (int i = 0; i < used && i < kMaxPrograms; ++i)
        {
            const ProgramStats &p = g_programs[i];
            std::fprintf(stderr,
                " {pc=0x%04x runs=%llu clip=%llu flagged=%llu negW=%llu fcand=%llu"
                " fcor=%llu fcandT=%llu fcorT=%llu negWUnflagged=%llu"
                " bits=[xp=%llu xm=%llu yp=%llu ym=%llu zp=%llu zm=%llu]"
                " instr=%llu early=%llu waitq=%llu wrap=%llu xgkick=%llu}",
                (unsigned)p.startPc.load(std::memory_order_relaxed),
                (unsigned long long)p.runs.load(std::memory_order_relaxed),
                (unsigned long long)p.clips.load(std::memory_order_relaxed),
                (unsigned long long)p.clipsWithFlags.load(std::memory_order_relaxed),
                (unsigned long long)p.clipsNegW.load(std::memory_order_relaxed),
                (unsigned long long)p.fcand.load(std::memory_order_relaxed),
                (unsigned long long)p.fcor.load(std::memory_order_relaxed),
                (unsigned long long)p.fcandTrue.load(std::memory_order_relaxed),
                (unsigned long long)p.fcorTrue.load(std::memory_order_relaxed),
                (unsigned long long)p.clipsNegWUnflagged.load(std::memory_order_relaxed),
                (unsigned long long)p.clipBit0.load(std::memory_order_relaxed),
                (unsigned long long)p.clipBit1.load(std::memory_order_relaxed),
                (unsigned long long)p.clipBit2.load(std::memory_order_relaxed),
                (unsigned long long)p.clipBit3.load(std::memory_order_relaxed),
                (unsigned long long)p.clipBit4.load(std::memory_order_relaxed),
                (unsigned long long)p.clipBit5.load(std::memory_order_relaxed),
                (unsigned long long)p.instrCount.load(std::memory_order_relaxed),
                (unsigned long long)p.earlyExit.load(std::memory_order_relaxed),
                (unsigned long long)p.waitqCount.load(std::memory_order_relaxed),
                (unsigned long long)p.wrapCount.load(std::memory_order_relaxed),
                (unsigned long long)p.xgkick.load(std::memory_order_relaxed));
        }
        std::fprintf(stderr, "\n");
    }
}

// Instruction field extraction helpers
static inline uint8_t DEST(uint32_t i) { return (uint8_t)((i >> 21) & 0xF); }
static inline uint8_t FT(uint32_t i) { return (uint8_t)((i >> 16) & 0x1F); }
static inline uint8_t FS(uint32_t i) { return (uint8_t)((i >> 11) & 0x1F); }
static inline uint8_t FD(uint32_t i) { return (uint8_t)((i >> 6) & 0x1F); }
static inline uint8_t BC(uint32_t i) { return (uint8_t)(i & 0x3); }

// Lower instruction field helpers
static inline uint8_t LIT(uint32_t i) { return (uint8_t)((i >> 16) & 0x1F); }
static inline uint8_t LIS(uint32_t i) { return (uint8_t)((i >> 11) & 0x1F); }
static inline uint8_t LID(uint32_t i) { return (uint8_t)((i >> 6) & 0x1F); }
static inline uint8_t VIT(uint32_t i) { return (uint8_t)((i >> 16) & 0xF); }
static inline uint8_t VIS(uint32_t i) { return (uint8_t)((i >> 11) & 0xF); }
static inline uint8_t VID(uint32_t i) { return (uint8_t)((i >> 6) & 0xF); }
static inline int16_t IMM11(uint32_t i) { return (int16_t)(int32_t)((int32_t)(i << 21) >> 21); }
static inline int16_t IMM15(uint32_t i)
{
    uint32_t lo11 = i & 0x7FF;
    uint32_t hi4 = (i >> 21) & 0xF;
    uint32_t raw = (hi4 << 11) | lo11;
    return (int16_t)(int32_t)((int32_t)(raw << 17) >> 17);
}


static inline uint8_t vuUpperVfWriteReg(uint32_t upper)
{
    const uint8_t op = upper & 0x3Fu;
    const uint8_t dest = DEST(upper);
    const uint8_t ft = FT(upper);
    const uint8_t fd = FD(upper);

    if (dest == 0u)
        return 0u;

    if (op <= 0x2Fu)
        return fd;

    if (op >= 0x3Cu)
    {
        const uint8_t specialOp = static_cast<uint8_t>((upper & 0x3u) | ((upper >> 4) & 0x7Cu));
        switch (specialOp)
        {
        // Upper special ops that write a VF register use FT as destination.
        case 0x10: // ITOF0
        case 0x11: // ITOF4
        case 0x12: // ITOF12
        case 0x13: // ITOF15
        case 0x14: // FTOI0
        case 0x15: // FTOI4
        case 0x16: // FTOI12
        case 0x17: // FTOI15
        case 0x1D: // ABS
            return ft;
        default:
            return 0u; // ACC/NOP/CLIP/etc.
        }
    }

    return 0u;
}

static inline void vuSetRegBit(uint32_t &mask, uint8_t reg)
{
    if (reg != 0u && reg < 32u)
        mask |= (1u << reg);
}

static inline void vuLowerVfReadWriteMasks(uint32_t lower, uint32_t &readMask, uint32_t &writeMask)
{
    readMask = 0u;
    writeMask = 0u;

    if (lower == 0u || lower == 0x8000033Cu)
        return;

    const uint8_t opHi = static_cast<uint8_t>((lower >> 25) & 0x7Fu);
    const uint8_t it = LIT(lower);
    const uint8_t is = LIS(lower);

    if ((lower & 0x80000000u) != 0u)
    {
        const uint8_t funct = lower & 0x3Fu;
        if (funct >= 0x3Cu && funct <= 0x3Fu)
        {
            const uint8_t specialOp = static_cast<uint8_t>((lower & 0x3u) | ((lower >> 4) & 0x7Cu));
            switch (specialOp)
            {
            case 0x30: // MOVE
            case 0x31: // MR32
                vuSetRegBit(readMask, is);
                vuSetRegBit(writeMask, it);
                return;
            case 0x34: // LQI
            case 0x36: // LQD
                vuSetRegBit(writeMask, it);
                return;
            case 0x35: // SQI
            case 0x37: // SQD
                vuSetRegBit(readMask, is);
                return;
            case 0x38: // DIV
            case 0x3A: // RSQRT
                vuSetRegBit(readMask, is);
                vuSetRegBit(readMask, it);
                return;
            case 0x39: // SQRT
                vuSetRegBit(readMask, it);
                return;
            case 0x3C: // MTIR
            case 0x3E: // ILWR source base is integer, but field source is VF for MTIR only.
                if (specialOp == 0x3C)
                    vuSetRegBit(readMask, is);
                return;
            case 0x3D: // MFIR
            case 0x64: // MFP
                vuSetRegBit(writeMask, it);
                return;
            default:
                return;
            }
        }
        return;
    }

    switch (opHi)
    {
    case 0x00: // LQ
        vuSetRegBit(writeMask, it);
        return;
    case 0x01: // SQ
        vuSetRegBit(readMask, is);
        return;
    default:
        return;
    }
}

static inline bool vuLowerShouldRunBeforeUpper(uint32_t upper, uint32_t lower)
{
    const uint8_t upperWrite = vuUpperVfWriteReg(upper);
    if (upperWrite == 0u)
        return false;

    uint32_t lowerReads = 0u;
    uint32_t lowerWrites = 0u;
    vuLowerVfReadWriteMasks(lower, lowerReads, lowerWrites);

    const uint32_t upperBit = (1u << upperWrite);
    return ((lowerReads | lowerWrites) & upperBit) != 0u;
}

#endif