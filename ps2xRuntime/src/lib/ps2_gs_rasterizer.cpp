#include <atomic>
// [MOH diag] VU1 microprogram attribution for pathological primitives
// (definition lives in the private vu/ps2_vu1_detail.h).
namespace mohvu { extern std::atomic<uint32_t> g_lastXgkickProgram; }
namespace mohgeom
{
    // Ring of the most recent VIF1 chain segments, filled by the DMAC tag walk in
    // ps2_memory.cpp and read when a mixed-Q primitive is drawn, so the guest
    // buffer that carries those primitives can be identified.
    std::atomic<uint32_t> g_vif1SegAddr[8] = {};
    std::atomic<uint32_t> g_vif1SegQwc[8] = {};
    std::atomic<uint32_t> g_vif1SegPos{0};
    // Segment payloads captured at DMA time. Binding the bytes to the event
    // rather than to an address is what makes this reliable: the display-list
    // buffers are heap-allocated and move between runs.
    constexpr uint32_t kSegCap = 704u;
    uint8_t g_vif1SegData[8][kSegCap] = {};
    std::atomic<uint32_t> g_vif1SegLen[8] = {};

    extern thread_local bool t_vu1LaunchActive;
    extern thread_local uint64_t t_vu1LaunchId;
    extern thread_local uint64_t t_vu1LaunchFrame;
    extern thread_local uint64_t t_objectTraceId;
    extern thread_local uint64_t t_vifTransferHash;
    extern thread_local uint32_t t_vifSourcePhys;
    extern thread_local uint32_t t_vifTransferBytes;
    extern thread_local uint32_t t_vifCommandOffset;
    extern thread_local uint32_t t_vu1StartPc;
    extern thread_local uint32_t t_vu1Top;
    extern thread_local uint32_t t_vu1Itop;
    extern thread_local uint32_t t_matrixMatchMask;
    extern thread_local uint32_t t_matrixSlot;
    extern thread_local uint32_t t_matrixAddress;
    extern thread_local uint32_t t_matrixVuOffset;
    extern thread_local uint32_t t_matrixMatchWords;
    extern thread_local uint32_t t_matrixBits[16];
    extern thread_local uint32_t t_ownerHeadWords[32];
    extern thread_local uint32_t t_ownerTailWords[16];

    extern thread_local uint64_t t_xgkickId;
    extern thread_local uint64_t t_xgkickPacketHash;
    extern thread_local uint32_t t_xgkickProgram;
    extern thread_local uint32_t t_xgkickMicroPc;
    extern thread_local uint32_t t_xgkickPacketAddress;
    extern thread_local uint32_t t_xgkickPacketBytes;

    extern thread_local uint32_t t_gifPathId;
    extern thread_local uint32_t t_gifPacketIndex;
    extern thread_local uint32_t t_gifPacketBytes;
    extern thread_local uint32_t t_gifByteOffset;
    extern thread_local uint32_t t_gifTagIndex;
    extern thread_local uint32_t t_gifPrimitiveIndex;
}
#include "runtime/ps2_gs_rasterizer.h"
#include "runtime/ps2_gs_gpu.h"
#include "runtime/ps2_gs_common.h"
#include "runtime/ps2_gs_psmct16.h"
#include "runtime/ps2_gs_psmct32.h"
#include "runtime/ps2_gs_psmt4.h"
#include "runtime/ps2_gs_psmt8.h"
#include "runtime/ps2_gs_memory.h"
#include "ps2_log.h"
#include <atomic>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <unordered_set>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace GSInternal;

namespace
{
    float fabsQ(float q)
    {
        return (std::fabs(q) > 1.0e-8f) ? q : 1.0f;
    }

    u16 Rgba8888ToRgba5551(u32 c)
    {
        uint32_t r = ((c >> 0)  & 0xFF) >> 3;
        uint32_t g = ((c >> 8)  & 0xFF) >> 3;
        uint32_t b = ((c >> 16) & 0xFF) >> 3;
        uint32_t a = ((c >> 24) & 0xFF) >> 7;

        return (r | (g << 5) | (b << 10) | (a << 15));
    }

    u32 Rgba5551ToRgba8888(u16 c)
    {
        u32 r = ((c >> 0)  & 0x1F) << 3;
        u32 g = ((c >> 5)  & 0x1F) << 3;
        u32 b = ((c >> 10) & 0x1F) << 3;
        u32 a = ((c >> 15) & 0x01) << 7;

        return (r | (g << 8) | (b << 16) | (a << 24));
    }

    u32 pack32(u8 r, u8 g, u8 b, u8 a)
    {
        return static_cast<u32>(r) | (g << 8) | (b << 16) | (a << 24);
    }

    uint32_t applyTexa(const GSTexaReg &texa, uint8_t psm, uint32_t texel)
    {
        if (psm == GS_PSM_CT32)
            return texel;

        const uint8_t r = static_cast<uint8_t>(texel & 0xFFu);
        const uint8_t g = static_cast<uint8_t>((texel >> 8) & 0xFFu);
        const uint8_t b = static_cast<uint8_t>((texel >> 16) & 0xFFu);
        const bool rgbZero = r == 0u && g == 0u && b == 0u;
        uint8_t a = static_cast<uint8_t>((texel >> 24) & 0xFFu);

        switch (psm)
        {
        case GS_PSM_CT24:
            a = (texa.aem && rgbZero) ? 0u : texa.ta0;
            break;
        case GS_PSM_CT16:
        case GS_PSM_CT16S:
            if ((a & 0x80u) != 0u)
                a = texa.ta1;
            else
                a = (texa.aem && rgbZero) ? 0u : texa.ta0;
            break;
        default:
            break;
        }

        return (texel & 0x00FFFFFFu) | (static_cast<uint32_t>(a) << 24);
    }

    uint32_t addrPSMCT16Family(uint32_t basePtr, uint32_t width, uint8_t psm, uint32_t x, uint32_t y)
    {
        switch (psm)
        {
        case GS_PSM_CT16:
            return GSPSMCT16::addrPSMCT16(basePtr, width, x, y);
        case GS_PSM_CT16S:
            return GSPSMCT16::addrPSMCT16S(basePtr, width, x, y);
        case GS_PSM_Z16:
            return GSPSMCT16::addrPSMZ16(basePtr, width, x, y);
        case GS_PSM_Z16S:
            return GSPSMCT16::addrPSMZ16S(basePtr, width, x, y);
        default:
            return 0u;
        }
    }

    std::atomic<uint32_t> s_debugPrimitiveCount{0};
    std::atomic<uint32_t> s_debugPixelCount{0};
    std::atomic<uint32_t> s_debugContext1PrimitiveCount{0};
    std::atomic<uint32_t> s_debugFbp150PixelCount{0};
    bool passesAlphaTest(uint64_t testReg, uint8_t alpha)
    {
        if ((testReg & 0x1u) == 0u)
            return true;

        const uint8_t atst = static_cast<uint8_t>((testReg >> 1) & 0x7u);
        const uint8_t aref = static_cast<uint8_t>((testReg >> 4) & 0xFFu);

        switch (atst)
        {
        case 0:
            return false;
        case 1:
            return true;
        case 2:
            return alpha < aref;
        case 3:
            return alpha <= aref;
        case 4:
            return alpha == aref;
        case 5:
            return alpha >= aref;
        case 6:
            return alpha > aref;
        case 7:
            return alpha != aref;
        default:
            return true;
        }
    }

    struct AlphaTestResult
    {
        bool writeFramebuffer;
        bool preserveDestinationAlpha;
    };

    AlphaTestResult classifyAlphaTest(uint64_t testReg, uint8_t alpha)
    {
        const bool pass = passesAlphaTest(testReg, alpha);
        if (pass)
            return {true, false};

        // TEST.AFAIL controls what happens when the alpha comparison fails.
        switch (static_cast<uint8_t>((testReg >> 12) & 0x3u))
        {
        case 1: // FB_ONLY
            return {true, false};
        case 3: // RGB_ONLY
            return {true, true};
        case 0: // KEEP
        case 2: // ZB_ONLY
        default:
            return {false, false};
        }
    }

    struct TextureCombineResult
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    };

    TextureCombineResult combineTexture(const GSTex0Reg &tex,
                                        uint8_t vr,
                                        uint8_t vg,
                                        uint8_t vb,
                                        uint8_t va,
                                        uint8_t tr,
                                        uint8_t tg,
                                        uint8_t tb,
                                        uint8_t ta)
    {
        const bool textureHasAlpha = tex.tcc != 0u;
        TextureCombineResult out{tr, tg, tb, textureHasAlpha ? ta : va};

        switch (tex.tfx)
        {
        case 0: // MODULATE
            out.r = clampU8((tr * vr) >> 7);
            out.g = clampU8((tg * vg) >> 7);
            out.b = clampU8((tb * vb) >> 7);
            out.a = textureHasAlpha ? clampU8((ta * va) >> 7) : va;
            break;
        case 1: // DECAL
            out.r = tr;
            out.g = tg;
            out.b = tb;
            out.a = textureHasAlpha ? ta : va;
            break;
        case 2: // HIGHLIGHT
            out.r = clampU8(((tr * vr) >> 7) + va);
            out.g = clampU8(((tg * vg) >> 7) + va);
            out.b = clampU8(((tb * vb) >> 7) + va);
            out.a = textureHasAlpha ? clampU8(ta + va) : va;
            break;
        case 3: // HIGHLIGHT2
            out.r = clampU8(((tr * vr) >> 7) + va);
            out.g = clampU8(((tg * vg) >> 7) + va);
            out.b = clampU8(((tb * vb) >> 7) + va);
            out.a = textureHasAlpha ? ta : va;
            break;
        default:
            out.r = tr;
            out.g = tg;
            out.b = tb;
            out.a = textureHasAlpha ? ta : va;
            break;
        }

        return out;
    }

    uint32_t swizzleClutIndexCSM1(uint32_t index)
    {
        return (index & 0xE7u) | ((index & 0x08u) << 1u) | ((index & 0x10u) >> 1u);
    }

    // TODO: clut cache
    uint32_t resolveClutIndex(uint8_t index, uint8_t csm, uint8_t csa, uint8_t sourcePsm)
    {
        uint32_t clutIndex = static_cast<uint32_t>(index);

        switch (sourcePsm)
        {
        case GS_PSM_T4:
        case GS_PSM_T4HH:
        case GS_PSM_T4HL:
        {
            clutIndex = (static_cast<uint32_t>(csa) << 4u) | (clutIndex & 0x0Fu);

            if (csm == 0u)
                clutIndex = swizzleClutIndexCSM1(clutIndex);
        }
        break;
        case GS_PSM_T8:
        case GS_PSM_T8H:
            if (csm == 0)
                clutIndex = swizzleClutIndexCSM1(clutIndex);
            break;
        default:
            break;
        }

        return clutIndex;
    }

    bool tex1UsesLinearFilter(uint64_t tex1)
    {
        const uint8_t mmag = static_cast<uint8_t>((tex1 >> 5) & 0x1u);
        const uint8_t mmin = static_cast<uint8_t>((tex1 >> 6) & 0x7u);
        return mmag != 0u || mmin == 1u || (mmin & 0x4u) != 0u;
    }

    uint8_t lerpChannel(uint8_t c00, uint8_t c10, uint8_t c01, uint8_t c11, float fx, float fy)
    {
        const float top = static_cast<float>(c00) + (static_cast<float>(c10) - static_cast<float>(c00)) * fx;
        const float bottom = static_cast<float>(c01) + (static_cast<float>(c11) - static_cast<float>(c01)) * fx;
        return clampU8(static_cast<int>(std::lround(top + (bottom - top) * fy)));
    }
}

void GSRasterizer::drawPrimitive(GS *gs)
{
    // [MOH diagnostic, env-gated, default off] Skip any primitive whose screen
    // span exceeds a threshold, regardless of primitive type or texturing. The
    // earlier probes only covered types 4/5 (strips and fans) and the fullscreen
    // one was gated on tme, so sprites and untextured passes escaped both.
    if (gs)
    {
        static const int spanLimit = [] {
            const char *v = std::getenv("PS2_MOH_DIAG_SPAN_LIMIT");
            return v ? std::atoi(v) : 0;
        }();
        // Targeted variant: skip only near-fullscreen sprites (type 6), which the
        // census shows arrive on PATH3 at exactly 640x448. Keeps the 20 000 long
        // thin PATH1 strips, so it isolates whether the sprites alone do the damage.
        static const bool skipFsSprites = [] {
            const char *v = std::getenv("PS2_MOH_DIAG_SKIP_FSSPRITE");
            return v && !std::strcmp(v, "1");
        }();
        // Skip both full-screen families - sprites (type 6, PATH3) and fans
        // (type 5, PATH2) - while keeping the 20 000 PATH1 strips, so the two
        // populations are finally separated.
        if (skipFsSprites && (gs->m_prim.type == 6u || gs->m_prim.type == 5u))
        {
            const int nv = (gs->m_prim.type == 6u) ? 2 : 3;
            float mnx = gs->m_vtxQueue[0].x, mxx = mnx;
            float mny = gs->m_vtxQueue[0].y, mxy = mny;
            for (int i = 1; i < nv; ++i)
            {
                mnx = std::min(mnx, gs->m_vtxQueue[i].x);
                mxx = std::max(mxx, gs->m_vtxQueue[i].x);
                mny = std::min(mny, gs->m_vtxQueue[i].y);
                mxy = std::max(mxy, gs->m_vtxQueue[i].y);
            }
            if ((mxx - mnx) > 500.0f && (mxy - mny) > 300.0f)
                return;
        }
        if (spanLimit > 0)
        {
            const int nv = (gs->m_prim.type == 6u) ? 2 : 3;
            float mnx = gs->m_vtxQueue[0].x, mxx = mnx;
            float mny = gs->m_vtxQueue[0].y, mxy = mny;
            for (int i = 1; i < nv; ++i)
            {
                mnx = std::min(mnx, gs->m_vtxQueue[i].x);
                mxx = std::max(mxx, gs->m_vtxQueue[i].x);
                mny = std::min(mny, gs->m_vtxQueue[i].y);
                mxy = std::max(mxy, gs->m_vtxQueue[i].y);
            }
            if ((mxx - mnx) > (float)spanLimit || (mxy - mny) > (float)spanLimit)
                return;
        }
    }

    // Characterise the oversized population instead of just filtering it:
    // histogram by primitive type and texturing, and sample a few of each type.
    if (gs)
    {
        const int nv = (gs->m_prim.type == 6u) ? 2 : 3;
        float mnx = gs->m_vtxQueue[0].x, mxx = mnx;
        float mny = gs->m_vtxQueue[0].y, mxy = mny;
        for (int i = 1; i < nv; ++i)
        {
            mnx = std::min(mnx, gs->m_vtxQueue[i].x);
            mxx = std::max(mxx, gs->m_vtxQueue[i].x);
            mny = std::min(mny, gs->m_vtxQueue[i].y);
            mxy = std::max(mxy, gs->m_vtxQueue[i].y);
        }
        const float spanX = mxx - mnx, spanY = mxy - mny;
        // The 300px census is dominated by legitimate fullscreen blits and long thin
        // strips, and its samples are capped at the first few, which has produced wrong
        // population claims before. Sample the *extreme* tail separately: PATH1 (VU1
        // XGKICK) primitives far wider than the 640px screen. These are the candidates
        // for the olive triangles, and every one of them reached here with ADC = 0.
        if (mohgeom::t_gifPathId == 1u && (spanX > 900.0f || spanY > 900.0f))
        {
            static std::atomic<uint64_t> s_n{0};
            const uint64_t k = s_n.fetch_add(1, std::memory_order_relaxed);
            if (k < 12u || (k % 20000u) == 19999u)
                std::fprintf(stderr,
                             "[MOH:huge-path1-v1] k=%llu type=%u span=%.0fx%.0f "
                             "xy0=(%.1f,%.1f) xy1=(%.1f,%.1f) xy2=(%.1f,%.1f) q=(%g,%g,%g)\n",
                             (unsigned long long)k, (unsigned)gs->m_prim.type,
                             (double)spanX, (double)spanY,
                             (double)gs->m_vtxQueue[0].x, (double)gs->m_vtxQueue[0].y,
                             (double)gs->m_vtxQueue[1].x, (double)gs->m_vtxQueue[1].y,
                             (double)gs->m_vtxQueue[2].x, (double)gs->m_vtxQueue[2].y,
                             (double)gs->m_vtxQueue[0].q, (double)gs->m_vtxQueue[1].q,
                             (double)gs->m_vtxQueue[2].q);
        }
        if (spanX > 300.0f || spanY > 300.0f)
        {
            // Which meshes produce the oversized primitives? Different meshes use
            // different textures, so a histogram over TEX0.TBP says whether this is
            // one object or systemic across the scene.
            {
                static std::atomic<uint32_t> s_tbp[16] = {};
                static std::atomic<uint64_t> s_tbpN[16] = {};
                static std::atomic<uint32_t> s_tbpCount{0};
                const uint32_t tbp = gs->activeContext().tex0.tbp0;
                const uint32_t n = s_tbpCount.load(std::memory_order_acquire);
                bool found = false;
                for (uint32_t i = 0u; i < n && i < 16u; ++i)
                    if (s_tbp[i].load(std::memory_order_relaxed) == tbp)
                    { s_tbpN[i].fetch_add(1, std::memory_order_relaxed); found = true; break; }
                if (!found && n < 16u)
                {
                    s_tbp[n].store(tbp, std::memory_order_relaxed);
                    s_tbpN[n].store(1, std::memory_order_relaxed);
                    s_tbpCount.store(n + 1u, std::memory_order_release);
                }
                static std::atomic<uint64_t> s_rep{0};
                if ((s_rep.fetch_add(1, std::memory_order_relaxed) % 4000u) == 3999u)
                {
                    // Are these tbp values also used as frame buffers? If so the
                    // offending strips are sampling render targets, i.e. they belong to
                    // a render-to-texture pass (a water reflection would fit), not to
                    // ordinary level geometry.
                    {
                        static std::atomic<uint32_t> s_fbp[8] = {};
                        static std::atomic<uint32_t> s_fbpN{0};
                        const uint32_t fbp = gs->activeContext().frame.fbp;
                        const uint32_t fn = s_fbpN.load(std::memory_order_acquire);
                        bool f2 = false;
                        for (uint32_t i = 0u; i < fn && i < 8u; ++i)
                            if (s_fbp[i].load(std::memory_order_relaxed) == fbp) { f2 = true; break; }
                        if (!f2 && fn < 8u)
                        {
                            s_fbp[fn].store(fbp, std::memory_order_relaxed);
                            s_fbpN.store(fn + 1u, std::memory_order_release);
                        }
                        std::cerr << "[MOH:oversize-fbp-v1] fbp vus:";
                        for (uint32_t i = 0u; i < s_fbpN.load(std::memory_order_acquire) && i < 8u; ++i)
                            std::cerr << " 0x" << std::hex << s_fbp[i].load(std::memory_order_relaxed);
                        std::cerr << " | tex0 psm=0x" << (uint32_t)gs->activeContext().tex0.psm
                                  << " tw=" << std::dec << (uint32_t)gs->activeContext().tex0.tw
                                  << " th=" << (uint32_t)gs->activeContext().tex0.th << std::endl;
                    }
                    std::cerr << "[MOH:oversize-tbp-v1] tbp:";
                    const uint32_t m = s_tbpCount.load(std::memory_order_acquire);
                    for (uint32_t i = 0u; i < m && i < 16u; ++i)
                        std::cerr << " 0x" << std::hex << s_tbp[i].load(std::memory_order_relaxed)
                                  << "=" << std::dec << s_tbpN[i].load(std::memory_order_relaxed);
                    std::cerr << std::endl;
                }
            }

            static std::atomic<uint64_t> s_byType[8][2] = {};
            const uint32_t t = gs->m_prim.type & 7u;
            const uint32_t tex = gs->m_prim.tme ? 1u : 0u;
            const uint64_t c = s_byType[t][tex].fetch_add(1, std::memory_order_relaxed);
            if ((c % 2000u) == 1999u)
            {
                std::cerr << "[MOH:oversize-census-v1]";
                for (uint32_t tt = 0u; tt < 8u; ++tt)
                    for (uint32_t xx = 0u; xx < 2u; ++xx)
                    {
                        const uint64_t v = s_byType[tt][xx].load(std::memory_order_relaxed);
                        if (v) std::cerr << " t" << tt << (xx ? "tex" : "flat") << "=" << v;
                    }
                std::cerr << std::endl;
            }
            static std::atomic<uint32_t> s_sample[8] = {};
            if (s_sample[t].fetch_add(1u, std::memory_order_relaxed) < 3u)
                std::cerr << "[MOH:oversize-sample-v1] type=" << t
                          << " tme=" << tex
                          << " path=" << mohgeom::t_gifPathId
                          << " span=" << spanX << "x" << spanY
                          << " xy0=(" << gs->m_vtxQueue[0].x << "," << gs->m_vtxQueue[0].y << ")"
                          << " xy1=(" << gs->m_vtxQueue[1].x << "," << gs->m_vtxQueue[1].y << ")"
                          << " xy2=(" << gs->m_vtxQueue[2].x << "," << gs->m_vtxQueue[2].y << ")"
                          << " q=(" << gs->m_vtxQueue[0].q << "," << gs->m_vtxQueue[1].q
                          << "," << gs->m_vtxQueue[2].q << ")"
                          << std::endl;
        }
    }

    // [MOH diagnostic, env-gated, default off] Draw everything untextured, using
    // vertex colours only. Geometry, UVs, camera and projection have all measured
    // healthy, so if the scene becomes recognisable this way the fault is in the
    // texture data or its lookup rather than anywhere upstream. A probe, not a fix.
    {
        static const bool noTex = [] {
            const char *v = std::getenv("PS2_MOH_DIAG_NO_TEXTURE");
            return v && !std::strcmp(v, "1");
        }();
        if (noTex && gs)
            gs->m_prim.tme = 0;
    }

    // Per-primitive Q sign census. A primitive whose three vertices mix Q signs
    // makes the per-pixel s/q division cross zero, which stretches the texture
    // without bound. Counting this at the primitive level - rather than per
    // packet, which conflates many triangles - measures the actual defect.
    if (gs && gs->m_prim.tme)
    {
        static std::atomic<uint64_t> s_prims{0}, s_mixed{0}, s_allNeg{0};
        const float q0 = gs->m_vtxQueue[0].q;
        const float q1 = gs->m_vtxQueue[1].q;
        const float q2 = gs->m_vtxQueue[2].q;
        const bool anyPos = q0 > 0.0f || q1 > 0.0f || q2 > 0.0f;
        const bool anyNeg = q0 < 0.0f || q1 < 0.0f || q2 < 0.0f;
        const uint64_t n = s_prims.fetch_add(1, std::memory_order_relaxed);
        // What fraction of each texture format's primitives is oversized? If nearly all
        // psm=0x14 primitives are, the whole surface is mis-laid-out; if only a minority,
        // the defect is partial and the surface itself is fine.
        {
            static std::atomic<uint64_t> s_psmTotal[64] = {};
            static std::atomic<uint64_t> s_psmBig[64] = {};
            const uint32_t psm = gs->activeContext().tex0.psm & 0x3F;
            const int nv2 = (gs->m_prim.type == 6u) ? 2 : 3;
            float ax = gs->m_vtxQueue[0].x, bx = ax, ay = gs->m_vtxQueue[0].y, by = ay;
            for (int i = 1; i < nv2; ++i)
            {
                ax = std::min(ax, gs->m_vtxQueue[i].x); bx = std::max(bx, gs->m_vtxQueue[i].x);
                ay = std::min(ay, gs->m_vtxQueue[i].y); by = std::max(by, gs->m_vtxQueue[i].y);
            }
            const bool big = (bx - ax) > 300.0f || (by - ay) > 300.0f;
            // Characterise the movie surface (psm=0x13) directly: how many of its
            // primitives are degenerate (the documented failure), normal, or oversized.
            if (psm == 0x13u)
            {
                static std::atomic<uint64_t> s_deg{0}, s_norm{0}, s_big2{0};
                const GSVertex &q0 = gs->m_vtxQueue[0];
                const GSVertex &q1 = gs->m_vtxQueue[1];
                const GSVertex &q2 = gs->m_vtxQueue[2];
                const bool degen = (q0.x == q1.x && q0.y == q1.y &&
                                    q1.x == q2.x && q1.y == q2.y);
                if (degen) s_deg.fetch_add(1, std::memory_order_relaxed);
                else if (big) s_big2.fetch_add(1, std::memory_order_relaxed);
                else s_norm.fetch_add(1, std::memory_order_relaxed);
                static std::atomic<uint64_t> s_r{0};
                const uint64_t rr = s_r.fetch_add(1, std::memory_order_relaxed);
                if ((rr % 200u) == 199u)
                    std::cerr << "[MOH:movie-surface-v1] degenerate="
                              << s_deg.load(std::memory_order_relaxed)
                              << " normal=" << s_norm.load(std::memory_order_relaxed)
                              << " oversized=" << s_big2.load(std::memory_order_relaxed)
                              << std::endl;
                if (rr < 4u)
                    std::cerr << "[MOH:movie-surface-v1] sample xy0=(" << q0.x << "," << q0.y
                              << ") xy1=(" << q1.x << "," << q1.y
                              << ") xy2=(" << q2.x << "," << q2.y << ")"
                              << " degen=" << (degen ? 1 : 0) << " big=" << (big ? 1 : 0)
                              << std::endl;
            }
            const uint64_t c = s_psmTotal[psm].fetch_add(1, std::memory_order_relaxed);
            if (big) s_psmBig[psm].fetch_add(1, std::memory_order_relaxed);
            if ((c % 40000u) == 39999u)
            {
                std::cerr << "[MOH:psm-oversize-v1]";
                for (uint32_t i = 0u; i < 64u; ++i)
                {
                    const uint64_t t = s_psmTotal[i].load(std::memory_order_relaxed);
                    if (!t) continue;
                    const uint64_t b = s_psmBig[i].load(std::memory_order_relaxed);
                    std::cerr << " psm0x" << std::hex << i << std::dec
                              << "=" << b << "/" << t;
                }
                std::cerr << std::endl;
            }
        }
        // Distribution of u = s/q and v = t/q for ordinary textured triangles.
        // 96 % of triangles are small, so a frame made of large blurry shapes can
        // only mean each small triangle is sampling the wrong part of its texture.
        // Correct u,v for tiled world geometry stay within a few tile widths.
        {
            static std::atomic<uint64_t> s_uv[5] = {};
            const float q0 = gs->m_vtxQueue[0].q;
            if (q0 != 0.0f)
            {
                const float u = gs->m_vtxQueue[0].s / q0;
                const float v = gs->m_vtxQueue[0].t / q0;
                const float a = std::max(std::fabs(u), std::fabs(v));
                const uint32_t bkt = a <= 1.001f  ? 0u
                                     : a <= 4.0f   ? 1u
                                     : a <= 16.0f  ? 2u
                                     : a <= 256.0f ? 3u : 4u;
                const uint64_t c = s_uv[bkt].fetch_add(1, std::memory_order_relaxed);
                if ((c % 200000u) == 199999u)
                    std::cerr << "[MOH:uv-hist-v1] <=1=" << s_uv[0].load(std::memory_order_relaxed)
                              << " <=4=" << s_uv[1].load(std::memory_order_relaxed)
                              << " <=16=" << s_uv[2].load(std::memory_order_relaxed)
                              << " <=256=" << s_uv[3].load(std::memory_order_relaxed)
                              << " >256=" << s_uv[4].load(std::memory_order_relaxed)
                              << std::endl;
            }
        }
        // Edge-length histogram for triangle strips and fans. In a correctly
        // assembled strip consecutive vertices are neighbours in the mesh, so
        // edges are short. A window that advances wrongly connects distant
        // vertices and the distribution shifts to very long edges.
        if (gs->m_prim.type == 4u || gs->m_prim.type == 5u)
        {
            static std::atomic<uint64_t> s_bucket[6] = {};
            const float dx01 = gs->m_vtxQueue[0].x - gs->m_vtxQueue[1].x;
            const float dy01 = gs->m_vtxQueue[0].y - gs->m_vtxQueue[1].y;
            const float dx12 = gs->m_vtxQueue[1].x - gs->m_vtxQueue[2].x;
            const float dy12 = gs->m_vtxQueue[1].y - gs->m_vtxQueue[2].y;
            const float dx20 = gs->m_vtxQueue[2].x - gs->m_vtxQueue[0].x;
            const float dy20 = gs->m_vtxQueue[2].y - gs->m_vtxQueue[0].y;
            float m2 = dx01 * dx01 + dy01 * dy01;
            m2 = std::max(m2, dx12 * dx12 + dy12 * dy12);
            m2 = std::max(m2, dx20 * dx20 + dy20 * dy20);
            const float len = std::sqrt(m2);
            const uint32_t bkt = len < 8.0f    ? 0u
                                 : len < 32.0f  ? 1u
                                 : len < 128.0f ? 2u
                                 : len < 512.0f ? 3u
                                 : len < 2048.0f ? 4u : 5u;
            // [MOH diagnostic, env-gated, default off] Skip only the
            // enormous-edge triangles to confirm whether the whole visual defect
            // is this 0.44 % population. A probe, not a fix.
            // Threshold corrected: a triangle only needs roughly 800 px of edge
            // to blanket a 640x448 screen, so the covering primitives sit in the
            // 512-2048 px band (8 142 of them), not only above 2048.
            // Edges >= 512 px: 33 629 triangles, 1.6 % of the total. The previous
            // threshold left the 512..2048 tranche drawing, which is why the frame
            // stayed covered.
            if (bkt >= 4u)
            {
                // Test state under which the oversized primitives are drawn. If Z
                // testing is off for them they pass unconditionally here and on
                // hardware alike, which rules the depth test out as the console's
                // protection.
                static std::atomic<uint32_t> s_t{0};
                const uint32_t kt = s_t.fetch_add(1u, std::memory_order_relaxed);
                if (kt < 8u)
                {
                    const auto &c2 = gs->activeContext();
                    std::cerr << "[MOH:oversize-test-v1] k=" << std::dec << kt
                              << " len=" << len
                              // Blend state: if ABE is clear the full-screen pass
                              // overwrites the scene instead of blending into it,
                              // which would turn a bloom/blur buffer into a smear.
                              << " abe=" << static_cast<uint32_t>(gs->m_prim.abe)
                              << " alpha=0x" << std::hex << c2.alpha
                              << " fbp=0x" << c2.frame.fbp
                              << " tbp=0x" << c2.tex0.tbp0
                              << " tpsm=0x" << (uint32_t)c2.tex0.psm
                              << " fpsm=0x" << (uint32_t)c2.frame.psm
                              << " fbmsk=0x" << c2.frame.fbmsk << std::dec
                              << " tme=" << static_cast<uint32_t>(gs->m_prim.tme)
                              << " ate=" << (c2.test & 1u)
                              // PSMT8 means the sampled render target is indexed:
                              // a wrong CLUT renders it as flat wrong colour, which
                              // is what an olive smear looks like.
                              << " cbp=0x" << std::hex << c2.tex0.cbp
                              << " cpsm=0x" << (uint32_t)c2.tex0.cpsm
                              << " csm=" << std::dec << (uint32_t)c2.tex0.csm
                              << " csa=" << (uint32_t)c2.tex0.csa
                              << " cld=" << (uint32_t)c2.tex0.cld
                              << " tw=" << (uint32_t)c2.tex0.tw
                              << " th=" << (uint32_t)c2.tex0.th
                              << " tcc=" << (uint32_t)c2.tex0.tcc
                              << " tfx=" << (uint32_t)c2.tex0.tfx
                              << std::endl;
                    // First 16 CLUT entries and a few texels. A palette whose
                    // entries are all the same colour paints a flat smear no matter
                    // what the indices are.
                    std::cerr << "[MOH:clut-dump-v1] k=" << std::dec << kt << " clut=";
                    for (uint32_t e = 0u; e < 16u; ++e)
                        std::cerr << (e ? "," : "") << std::hex
                                  << gs->ReadVram(c2.tex0.cpsm, c2.tex0.cbp, 1u, e, 0u);
                    std::cerr << " texels=";
                    for (uint32_t t = 0u; t < 12u; ++t)
                        std::cerr << (t ? "," : "") << std::hex
                                  << gs->ReadVram(c2.tex0.psm, c2.tex0.tbp0,
                                                  (uint32_t)c2.tex0.tbw, t * 64u, 100u);
                    std::cerr << std::dec << std::endl;
                }
                static const bool skipHuge = [] {
                    const char *v = std::getenv("PS2_MOH_DIAG_SKIP_HUGE");
                    return v && !std::strcmp(v, "1");
                }();
                if (skipHuge)
                    return;
            }
            s_bucket[bkt].fetch_add(1, std::memory_order_relaxed);
            // Time-phased dump with deltas: cumulative totals cannot tell which
            // scene generates the oversized primitives, and the frame changes
            // scene partway through a run.
            {
                using Clock = std::chrono::steady_clock;
                static const auto t0 = Clock::now();
                static std::atomic<uint64_t> s_lastMs{0};
                static uint64_t prev[6] = {};
                const uint64_t ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                                        Clock::now() - t0).count();
                uint64_t last = s_lastMs.load(std::memory_order_relaxed);
                if (ms - last >= 2000u &&
                    s_lastMs.compare_exchange_strong(last, ms))
                {
                    uint64_t cur[6];
                    for (uint32_t i = 0u; i < 6u; ++i)
                        cur[i] = s_bucket[i].load(std::memory_order_relaxed);
                    std::cerr << "[MOH:edge-delta-v1] t=" << (ms / 1000u) << "s";
                    static const char *names[6] = {"<8", "<32", "<128", "<512", "<2048", ">=2048"};
                    for (uint32_t i = 0u; i < 6u; ++i)
                        std::cerr << " " << names[i] << "=+" << (cur[i] - prev[i]);
                    std::cerr << std::endl;
                    for (uint32_t i = 0u; i < 6u; ++i) prev[i] = cur[i];
                }
            }
            const uint64_t c = s_bucket[bkt].load(std::memory_order_relaxed);
            if ((c % 100000u) == 99999u)
                std::cerr << "[MOH:edge-hist-v1] <8=" << s_bucket[0].load(std::memory_order_relaxed)
                          << " <32=" << s_bucket[1].load(std::memory_order_relaxed)
                          << " <128=" << s_bucket[2].load(std::memory_order_relaxed)
                          << " <512=" << s_bucket[3].load(std::memory_order_relaxed)
                          << " <2048=" << s_bucket[4].load(std::memory_order_relaxed)
                          << " >=2048=" << s_bucket[5].load(std::memory_order_relaxed)
                          << std::endl;
        }
        // Widened probe: skip any primitive carrying a negative Q at all, mixed
        // or uniformly negative. Env-gated, default off.
        if (anyNeg)
        {
            static const bool skipAnyNeg = [] {
                const char *v = std::getenv("PS2_MOH_DIAG_SKIP_ANYNEGQ");
                return v && !std::strcmp(v, "1");
            }();
            if (skipAnyNeg)
                return;
        }
        if (anyPos && anyNeg)
        {
            // [MOH diagnostic, env-gated, default off] Skip exactly the
            // mixed-sign-Q primitives - nothing else - to see what the frame
            // looks like without the defect. This is a probe, not a fix: with
            // PS2_MOH_DIAG_SKIP_MIXEDQ unset nothing changes.
            static const bool skipMixed = [] {
                const char *v = std::getenv("PS2_MOH_DIAG_SKIP_MIXEDQ");
                return v && !std::strcmp(v, "1");
            }();
            const uint64_t m = s_mixed.fetch_add(1, std::memory_order_relaxed);
            if (skipMixed)
                return;
            // Full histogram by path, not just the first few: the earlier
            // "all PATH2" conclusion rested on a 12-sample cap, and PATH2 only
            // issues about 40 XYZ2 writes in total, which cannot account for
            // over a thousand mixed-sign primitives.
            {
                static std::atomic<uint64_t> s_mixedByPath[4] = {};
                const uint32_t pi = mohgeom::t_gifPathId < 4u ? mohgeom::t_gifPathId : 0u;
                const uint64_t c = s_mixedByPath[pi].fetch_add(1, std::memory_order_relaxed);
                if ((c % 200u) == 199u)
                    std::cerr << "[MOH:mixedq-bypath-v1] p0=" << s_mixedByPath[0].load(std::memory_order_relaxed)
                              << " p1=" << s_mixedByPath[1].load(std::memory_order_relaxed)
                              << " p2=" << s_mixedByPath[2].load(std::memory_order_relaxed)
                              << " p3=" << s_mixedByPath[3].load(std::memory_order_relaxed)
                              << std::endl;
            }
            // t_gifPathId is set by GifArbiter::drain while the packet is being
            // processed, so unlike g_lastXgkickProgram it is not stale.
            // Path1 = VU1 XGKICK, Path2 = VIF1/GIF FIFO, Path3 = EE DMA.
            if (m < 12u)
            {
                std::cerr << "[MOH:mixedq-vif1-v1] m=" << m << " recentSegs=";
                const uint32_t pos = mohgeom::g_vif1SegPos.load(std::memory_order_relaxed);
                uint32_t bestIdx = 0u, bestLen = 0u;
                for (uint32_t i = 0u; i < 8u; ++i)
                {
                    const uint32_t idx = (pos - 1u - i) & 7u;
                    const uint32_t a = mohgeom::g_vif1SegAddr[idx].load(std::memory_order_relaxed);
                    if (!a) continue;
                    const uint32_t len = mohgeom::g_vif1SegLen[idx].load(std::memory_order_relaxed);
                    std::cerr << (i ? " " : "") << "0x" << std::hex << a << "/"
                              << std::dec << mohgeom::g_vif1SegQwc[idx].load(std::memory_order_relaxed);
                    if (len > bestLen) { bestLen = len; bestIdx = idx; }
                }
                std::cerr << std::endl;
                // Search every captured segment for a float in the Q range with
                // the offending sign. Exact coordinates move frame to frame, so
                // the magnitude window is the robust discriminator.
                if (m < 4u)
                {
                    for (uint32_t i = 0u; i < 8u; ++i)
                    {
                        const uint32_t len = mohgeom::g_vif1SegLen[i].load(std::memory_order_relaxed);
                        for (uint32_t off = 0u; off + 4u <= len; off += 4u)
                        {
                            float v;
                            std::memcpy(&v, &mohgeom::g_vif1SegData[i][off], 4);
                            if (v > -0.32f && v < -0.25f)
                                std::cerr << "[MOH:negq-in-buffer-v1] m=" << m
                                          << " slot=" << i
                                          << " addr=0x" << std::hex
                                          << mohgeom::g_vif1SegAddr[i].load(std::memory_order_relaxed)
                                          << " off=0x" << off << std::dec
                                          << " v=" << v << std::endl;
                        }
                    }
                }
                if (bestLen && m < 2u)
                {
                    std::cerr << "[MOH:mixedq-bytes-v1] m=" << m << " addr=0x" << std::hex
                              << mohgeom::g_vif1SegAddr[bestIdx].load(std::memory_order_relaxed)
                              << " len=" << std::dec << bestLen << " data=";
                    for (uint32_t b = 0u; b < bestLen; ++b)
                        std::cerr << std::hex << (mohgeom::g_vif1SegData[bestIdx][b] >> 4)
                                  << (mohgeom::g_vif1SegData[bestIdx][b] & 0xF);
                    std::cerr << std::dec << std::endl;
                }
            }
            if (m < 12u)
                std::cerr << "[MOH:mixedq-path-v1] m=" << m
                          << " path=" << mohgeom::t_gifPathId
                          << " type=" << static_cast<uint32_t>(gs->m_prim.type)
                          << " q=(" << q0 << "," << q1 << "," << q2 << ")"
                          << " fst=" << static_cast<uint32_t>(gs->m_prim.fst)
                          // If these primitives are meant to use UV rather than
                          // ST/Q, the u,v fields will hold plausible texel
                          // coordinates and fst should have been set.
                          << " uv0=(" << gs->m_vtxQueue[0].u << "," << gs->m_vtxQueue[0].v << ")"
                          << " uv1=(" << gs->m_vtxQueue[1].u << "," << gs->m_vtxQueue[1].v << ")"
                          << " uv2=(" << gs->m_vtxQueue[2].u << "," << gs->m_vtxQueue[2].v << ")"
                          << " st0=(" << gs->m_vtxQueue[0].s << "," << gs->m_vtxQueue[0].t << ")"
                          << std::endl;
        }
        else if (anyNeg) s_allNeg.fetch_add(1, std::memory_order_relaxed);
        if ((n % 200000u) == 199999u)
            std::cerr << "[MOH:prim-qsign-v1] textured=" << (n + 1)
                      << " mixedSign=" << s_mixed.load(std::memory_order_relaxed)
                      << " allNegative=" << s_allNeg.load(std::memory_order_relaxed)
                      << std::endl;
    }

    const auto &ctx = gs->activeContext();
    const uint32_t mohPrimitiveIndex =
        mohgeom::t_gifPrimitiveIndex++;
    // [MOH diag] reliable (non-gated) trace: distinct (ctxt,fbp,fbw,psm,scissor) draw configs.
    {
        static std::mutex s_mohMx;
        static std::unordered_set<uint64_t> s_mohSeen;
        const uint64_t key =
            (static_cast<uint64_t>(gs->m_prim.ctxt) << 60) ^
            (static_cast<uint64_t>(ctx.frame.fbp) << 40) ^
            (static_cast<uint64_t>(ctx.frame.fbw) << 32) ^
            (static_cast<uint64_t>(static_cast<uint16_t>(ctx.scissor.x0)) << 24) ^
            (static_cast<uint64_t>(static_cast<uint16_t>(ctx.scissor.x1)) << 12) ^
            (static_cast<uint64_t>(static_cast<uint16_t>(ctx.scissor.y1)));
        std::lock_guard<std::mutex> lk(s_mohMx);
        if (s_mohSeen.size() < 48u && s_mohSeen.insert(key).second)
            std::cerr << "[MOH:prim] type=" << static_cast<uint32_t>(gs->m_prim.type)
                      << " ctxt=" << static_cast<uint32_t>(gs->m_prim.ctxt)
                      << " tme=" << static_cast<uint32_t>(gs->m_prim.tme)
                      << " abe=" << static_cast<uint32_t>(gs->m_prim.abe)
                      << " fbp=" << ctx.frame.fbp
                      << " fbw=" << ctx.frame.fbw
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.frame.psm) << std::dec
                      << " scissor=(" << ctx.scissor.x0 << "," << ctx.scissor.y0
                      << ")-(" << ctx.scissor.x1 << "," << ctx.scissor.y1 << ")"
                      << std::endl;
    }
    {
        // [MOH diag] Bounded probe for pathological geometry. GS XY is 12.4
        // fixed point masked to 16 bits, so vtx.x/y can never exceed ~4095 after
        // the /16: an absolute-range test can never fire. What distinguishes the
        // broken level-6_1 draws is their SPAN - single primitives stretched far
        // wider than the 640x448 target. Log those, with z/q, to separate a
        // projection problem from a placement problem. Read-only.
        float minX = gs->m_vtxQueue[0].x, maxX = minX;
        float minY = gs->m_vtxQueue[0].y, maxY = minY;
        for (int i = 1; i < 3; ++i)
        {
            minX = std::min(minX, gs->m_vtxQueue[i].x);
            maxX = std::max(maxX, gs->m_vtxQueue[i].x);
            minY = std::min(minY, gs->m_vtxQueue[i].y);
            maxY = std::max(maxY, gs->m_vtxQueue[i].y);
        }
        if ((maxX - minX) > 1200.0f || (maxY - minY) > 900.0f)
        {
            static std::atomic<uint32_t> s_mohCorrelationGuard{0u};
            const uint32_t guardIndex =
                s_mohCorrelationGuard.fetch_add(
                    1u,
                    std::memory_order_relaxed);
            if (guardIndex < 32u)
            {
                std::cerr
                    << "[MOH:olive-correlation-guard-v2]"
                    << " n=" << std::dec << guardIndex
                    << " frame=" << mohgeom::t_vu1LaunchFrame
                    << " primitive=" << mohPrimitiveIndex
                    << " path=" << mohgeom::t_gifPathId
                    << " active="
                    << static_cast<uint32_t>(
                           mohgeom::t_vu1LaunchActive)
                    << " startPc=0x" << std::hex
                    << mohgeom::t_vu1StartPc
                    << " xgProgram=0x"
                    << mohgeom::t_xgkickProgram
                    << " launch=" << std::dec
                    << mohgeom::t_vu1LaunchId
                    << " xgkick=" << mohgeom::t_xgkickId
                    << " xgPc=0x" << std::hex
                    << mohgeom::t_xgkickMicroPc
                    << " xgBytes=0x"
                    << mohgeom::t_xgkickPacketBytes
                    << " gifPacket=" << std::dec
                    << mohgeom::t_gifPacketIndex
                    << " gifBytes=0x" << std::hex
                    << mohgeom::t_gifPacketBytes
                    << std::dec << std::endl;
            }

            if (mohgeom::t_gifPathId == 1u &&
                mohgeom::t_vu1LaunchActive &&
                mohgeom::t_vu1StartPc == 0u &&
                mohgeom::t_xgkickProgram == 0u)
            {
                static std::mutex s_mohCorrelationMutex;
                static std::unordered_set<uint64_t>
                    s_mohLoggedXgkicks;
                bool logCorrelation = false;
                {
                    std::lock_guard<std::mutex> lock(
                        s_mohCorrelationMutex);
                    if (s_mohLoggedXgkicks.size() < 16u)
                    {
                        logCorrelation =
                            s_mohLoggedXgkicks.insert(
                                mohgeom::t_xgkickId)
                                .second;
                    }
                }

                if (logCorrelation)
                {
                    std::cerr
                        << "[MOH:olive-object-correlation-v1]"
                        << " frame=" << std::dec
                        << mohgeom::t_vu1LaunchFrame
                        << " primitive=" << mohPrimitiveIndex
                        << " gifPacket=" << mohgeom::t_gifPacketIndex
                        << " gifBytes=0x" << std::hex
                        << mohgeom::t_gifPacketBytes
                        << " gifTag=" << std::dec
                        << mohgeom::t_gifTagIndex
                        << " gifOffset=0x" << std::hex
                        << mohgeom::t_gifByteOffset
                        << " path=" << std::dec
                        << mohgeom::t_gifPathId
                        << " launch=" << mohgeom::t_vu1LaunchId
                        << " vuProg=0x" << std::hex
                        << mohgeom::t_vu1StartPc
                        << " top=0x" << mohgeom::t_vu1Top
                        << " itop=0x" << mohgeom::t_vu1Itop
                        << " xgkick=" << std::dec
                        << mohgeom::t_xgkickId
                        << " xgPc=0x" << std::hex
                        << mohgeom::t_xgkickMicroPc
                        << " xgPacket=0x"
                        << mohgeom::t_xgkickPacketAddress
                        << " xgBytes=0x"
                        << mohgeom::t_xgkickPacketBytes
                        << " xgHash=0x"
                        << mohgeom::t_xgkickPacketHash
                        << " vifSource=0x"
                        << mohgeom::t_vifSourcePhys
                        << " vifBytes=0x"
                        << mohgeom::t_vifTransferBytes
                        << " vifCmdOff=0x"
                        << mohgeom::t_vifCommandOffset
                        << " vifHash=0x"
                        << mohgeom::t_vifTransferHash
                        << " objectTraceId=0x"
                        << mohgeom::t_objectTraceId
                        << " matrixMask=0x"
                        << mohgeom::t_matrixMatchMask
                        << " matrixSlot=" << std::dec
                        << mohgeom::t_matrixSlot
                        << " matrix=0x" << std::hex
                        << mohgeom::t_matrixAddress
                        << " matrixVuOff=0x"
                        << mohgeom::t_matrixVuOffset
                        << " matrixWords=" << std::dec
                        << mohgeom::t_matrixMatchWords
                        << " span=(" << (maxX - minX)
                        << "," << (maxY - minY) << ")"
                        << " matrixBits=[";
                    for (uint32_t i = 0u; i < 16u; ++i)
                    {
                        if (i != 0u)
                            std::cerr << ",";
                        std::cerr << std::hex
                                  << mohgeom::t_matrixBits[i];
                    }
                    std::cerr << "] ownerHead=[";
                    for (uint32_t i = 0u; i < 32u; ++i)
                    {
                        if (i != 0u)
                            std::cerr << ",";
                        std::cerr << std::hex
                                  << mohgeom::t_ownerHeadWords[i];
                    }
                    std::cerr << "] ownerTail=[";
                    for (uint32_t i = 0u; i < 16u; ++i)
                    {
                        if (i != 0u)
                            std::cerr << ",";
                        std::cerr << std::hex
                                  << mohgeom::t_ownerTailWords[i];
                    }
                    std::cerr << "]" << std::dec << std::endl;
                }
            }

            // [MOH diagnostic, env-gated, default off] Skip the full-screen
            // post-process passes to see whether real level geometry is being
            // drawn underneath them. This is a probe, not a fix: with
            // PS2_MOH_DIAG_SKIP_FSQUAD unset the renderer behaves exactly as
            // before.
            {
                static const bool skipFullscreen = [] {
                    const char *v = std::getenv("PS2_MOH_DIAG_SKIP_FSQUAD");
                    return v && !std::strcmp(v, "1");
                }();
                if (skipFullscreen && gs->m_prim.tme &&
                    (maxX - minX) > 800.0f && (maxY - minY) > 800.0f)
                {
                    static std::atomic<uint32_t> s_skipped{0};
                    const uint32_t k = s_skipped.fetch_add(1u, std::memory_order_relaxed);
                    if (k < 8u)
                        std::cerr << "[MOH:diag-skip-fsquad] k=" << k
                                  << " tbp=0x" << std::hex << ctx.tex0.tbp0 << std::dec
                                  << " span=" << (maxX - minX) << "x" << (maxY - minY)
                                  << std::endl;
                    return;
                }
            }

            static std::atomic<uint32_t> s_mohOversize{0};
            const uint32_t m = s_mohOversize.fetch_add(1u, std::memory_order_relaxed);
            if (m < 40u)
            {
                std::cerr << "[MOH:oversize-draw-v1] m=" << m
                          << " vuProg=0x" << std::hex
                          << mohvu::g_lastXgkickProgram.load(std::memory_order_relaxed)
                          << std::dec
                          << " type=" << static_cast<uint32_t>(gs->m_prim.type)
                          << " tme=" << static_cast<uint32_t>(gs->m_prim.tme)
                          << " fst=" << static_cast<uint32_t>(gs->m_prim.fst)
                          << " spanX=" << (maxX - minX) << " spanY=" << (maxY - minY)
                          << std::hex << " fbp=0x" << ctx.frame.fbp
                          << " tex0.tbp=0x" << ctx.tex0.tbp0
                          << " tex0.psm=0x" << static_cast<uint32_t>(ctx.tex0.psm)
                          << std::dec
                          << " of=(" << (ctx.xyoffset.ofx >> 4) << "," << (ctx.xyoffset.ofy >> 4) << ")"
                          << " v0=(" << gs->m_vtxQueue[0].x << "," << gs->m_vtxQueue[0].y
                          << ",z=" << gs->m_vtxQueue[0].z << ",q=" << gs->m_vtxQueue[0].q << ")"
                          << " v1=(" << gs->m_vtxQueue[1].x << "," << gs->m_vtxQueue[1].y
                          << ",z=" << gs->m_vtxQueue[1].z << ")"
                          << " v2=(" << gs->m_vtxQueue[2].x << "," << gs->m_vtxQueue[2].y
                          << ",z=" << gs->m_vtxQueue[2].z << ")"
                          // Texture path: with fst=0 the rasteriser takes S,T,Q
                          // rather than U,V, so the per-vertex triple is what
                          // decides whether the texture smears.
                          << " st0=(" << gs->m_vtxQueue[0].s << "," << gs->m_vtxQueue[0].t
                          << ",q=" << gs->m_vtxQueue[0].q << ")"
                          << " st1=(" << gs->m_vtxQueue[1].s << "," << gs->m_vtxQueue[1].t
                          << ",q=" << gs->m_vtxQueue[1].q << ")"
                          << " st2=(" << gs->m_vtxQueue[2].s << "," << gs->m_vtxQueue[2].t
                          << ",q=" << gs->m_vtxQueue[2].q << ")"
                          << std::endl;
            }
        }
    }
    {
        static std::atomic<uint32_t> s_mohDrawTrace{0};
        const uint32_t n = s_mohDrawTrace.fetch_add(1u, std::memory_order_relaxed);
        if (n < 128u)
        {
            std::cerr << "[MOH:draw] n=" << n
                      << " type=" << static_cast<uint32_t>(gs->m_prim.type)
                      << " ctxt=" << static_cast<uint32_t>(gs->m_prim.ctxt)
                      << " tme=" << static_cast<uint32_t>(gs->m_prim.tme)
                      << " fst=" << static_cast<uint32_t>(gs->m_prim.fst)
                      << " abe=" << static_cast<uint32_t>(gs->m_prim.abe)
                      << " fbp=0x" << std::hex << ctx.frame.fbp
                      << " fbw=0x" << ctx.frame.fbw
                      << " fpsm=0x" << static_cast<uint32_t>(ctx.frame.psm)
                      << " test=0x" << ctx.test
                      << " tex0.tbp=0x" << ctx.tex0.tbp0
                      << " tex0.psm=0x" << static_cast<uint32_t>(ctx.tex0.psm)
                      << " tex0.cbp=0x" << ctx.tex0.cbp
                      << " tex0.cpsm=0x" << static_cast<uint32_t>(ctx.tex0.cpsm)
                      << std::dec
                      << " of=(" << (ctx.xyoffset.ofx >> 4) << "," << (ctx.xyoffset.ofy >> 4) << ")"
                      << " scissor=(" << ctx.scissor.x0 << "," << ctx.scissor.y0
                      << ")-(" << ctx.scissor.x1 << "," << ctx.scissor.y1 << ")"
                      << " v0=(" << gs->m_vtxQueue[0].x << "," << gs->m_vtxQueue[0].y << ")"
                      << " v1=(" << gs->m_vtxQueue[1].x << "," << gs->m_vtxQueue[1].y << ")"
                      << " v2=(" << gs->m_vtxQueue[2].x << "," << gs->m_vtxQueue[2].y << ")"
                      << " rgba0=(" << static_cast<uint32_t>(gs->m_vtxQueue[0].r) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[0].g) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[0].b) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[0].a) << ")"
                      << std::endl;
        }
    }
    // MOH: dedicated movie-draw trace (PSMT8 textured prims) — the shared [MOH:draw]
    // cap gets exhausted by UI sprites long before the movies start.
    if (gs->m_prim.tme && ctx.tex0.psm == 0x13u)
    {
        static std::atomic<uint32_t> s_mohMovieDraw{0};
        const uint32_t n = s_mohMovieDraw.fetch_add(1u, std::memory_order_relaxed);
        if (n < 64u)
        {
            std::cerr << "[MOH:mdraw] n=" << n
                      << " type=" << static_cast<uint32_t>(gs->m_prim.type)
                      << " fst=" << static_cast<uint32_t>(gs->m_prim.fst)
                      << std::hex
                      << " tbp=0x" << ctx.tex0.tbp0
                      << " cbp=0x" << ctx.tex0.cbp
                      << std::dec
                      << " v0=(" << gs->m_vtxQueue[0].x << "," << gs->m_vtxQueue[0].y << ")"
                      << " v1=(" << gs->m_vtxQueue[1].x << "," << gs->m_vtxQueue[1].y << ")"
                      << " v2=(" << gs->m_vtxQueue[2].x << "," << gs->m_vtxQueue[2].y << ")"
                      << std::endl;
        }
    }
    PS2_IF_AGRESSIVE_LOGS({
        const uint32_t primitiveIndex = s_debugPrimitiveCount.fetch_add(1u, std::memory_order_relaxed);
        if (primitiveIndex < 64u)
        {
            std::cout << "[gs:prim] idx=" << primitiveIndex
                      << " type=" << static_cast<uint32_t>(gs->m_prim.type)
                      << " tme=" << static_cast<uint32_t>(gs->m_prim.tme)
                      << " abe=" << static_cast<uint32_t>(gs->m_prim.abe)
                      << " fst=" << static_cast<uint32_t>(gs->m_prim.fst)
                      << " ctxt=" << static_cast<uint32_t>(gs->m_prim.ctxt)
                      << " fbp=" << ctx.frame.fbp
                      << " fbw=" << ctx.frame.fbw
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.frame.psm) << std::dec
                      << " tex0=("
                      << "tbp0=" << ctx.tex0.tbp0
                      << " tbw=" << static_cast<uint32_t>(ctx.tex0.tbw)
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.tex0.psm) << std::dec
                      << " tw=" << static_cast<uint32_t>(ctx.tex0.tw)
                      << " th=" << static_cast<uint32_t>(ctx.tex0.th)
                      << " tcc=" << static_cast<uint32_t>(ctx.tex0.tcc)
                      << " tfx=" << static_cast<uint32_t>(ctx.tex0.tfx)
                      << " cbp=" << ctx.tex0.cbp
                      << " cpsm=0x" << std::hex << static_cast<uint32_t>(ctx.tex0.cpsm) << std::dec
                      << " csm=" << static_cast<uint32_t>(ctx.tex0.csm)
                      << " csa=" << static_cast<uint32_t>(ctx.tex0.csa)
                      << ")"
                      << " texclut=("
                      << "cbw=" << static_cast<uint32_t>(gs->m_texclut.cbw)
                      << " cou=" << static_cast<uint32_t>(gs->m_texclut.cou)
                      << " cov=" << gs->m_texclut.cov
                      << ")"
                      << " ofx=" << (ctx.xyoffset.ofx >> 4)
                      << " ofy=" << (ctx.xyoffset.ofy >> 4)
                      << " scissor=(" << ctx.scissor.x0
                      << "," << ctx.scissor.y0
                      << ")-(" << ctx.scissor.x1
                      << "," << ctx.scissor.y1 << ")"
                      << " test=0x" << std::hex << ctx.test
                      << " alpha=0x" << ctx.alpha
                      << std::dec
                      << " v0=(" << gs->m_vtxQueue[0].x << "," << gs->m_vtxQueue[0].y << ")"
                      << " uv0=(" << (gs->m_vtxQueue[0].u >> 4) << "," << (gs->m_vtxQueue[0].v >> 4) << ")"
                      << " stq0=(" << gs->m_vtxQueue[0].s << "," << gs->m_vtxQueue[0].t << "," << gs->m_vtxQueue[0].q << ")"
                      << " v1=(" << gs->m_vtxQueue[1].x << "," << gs->m_vtxQueue[1].y << ")"
                      << " uv1=(" << (gs->m_vtxQueue[1].u >> 4) << "," << (gs->m_vtxQueue[1].v >> 4) << ")"
                      << " stq1=(" << gs->m_vtxQueue[1].s << "," << gs->m_vtxQueue[1].t << "," << gs->m_vtxQueue[1].q << ")"
                      << " v2=(" << gs->m_vtxQueue[2].x << "," << gs->m_vtxQueue[2].y << ")"
                      << " uv2=(" << (gs->m_vtxQueue[2].u >> 4) << "," << (gs->m_vtxQueue[2].v >> 4) << ")"
                      << " stq2=(" << gs->m_vtxQueue[2].s << "," << gs->m_vtxQueue[2].t << "," << gs->m_vtxQueue[2].q << ")"
                      << " rgba0=(" << static_cast<uint32_t>(gs->m_vtxQueue[0].r) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[0].g) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[0].b) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[0].a) << ")"
                      << " rgba1=(" << static_cast<uint32_t>(gs->m_vtxQueue[1].r) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[1].g) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[1].b) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[1].a) << ")"
                      << " rgba2=(" << static_cast<uint32_t>(gs->m_vtxQueue[2].r) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[2].g) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[2].b) << ","
                      << static_cast<uint32_t>(gs->m_vtxQueue[2].a) << ")"
                      << std::endl;
        }
    });

    PS2_IF_AGRESSIVE_LOGS({
        if ((gs->m_prim.ctxt != 0u || ctx.frame.fbp == 150u) &&
            s_debugContext1PrimitiveCount.fetch_add(1u, std::memory_order_relaxed) < 32u)
        {
            std::cout << "[gs:copy-prim]"
                      << " type=" << static_cast<uint32_t>(gs->m_prim.type)
                      << " tme=" << static_cast<uint32_t>(gs->m_prim.tme)
                      << " abe=" << static_cast<uint32_t>(gs->m_prim.abe)
                      << " fst=" << static_cast<uint32_t>(gs->m_prim.fst)
                      << " ctxt=" << static_cast<uint32_t>(gs->m_prim.ctxt)
                      << " fbp=" << ctx.frame.fbp
                      << " fbw=" << ctx.frame.fbw
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.frame.psm) << std::dec
                      << " tex0=("
                      << "tbp0=" << ctx.tex0.tbp0
                      << " tbw=" << static_cast<uint32_t>(ctx.tex0.tbw)
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.tex0.psm) << std::dec
                      << " tcc=" << static_cast<uint32_t>(ctx.tex0.tcc)
                      << " tfx=" << static_cast<uint32_t>(ctx.tex0.tfx)
                      << " cbp=" << ctx.tex0.cbp
                      << " cpsm=0x" << std::hex << static_cast<uint32_t>(ctx.tex0.cpsm) << std::dec
                      << " csm=" << static_cast<uint32_t>(ctx.tex0.csm)
                      << " csa=" << static_cast<uint32_t>(ctx.tex0.csa)
                      << ")"
                      << " texclut=("
                      << "cbw=" << static_cast<uint32_t>(gs->m_texclut.cbw)
                      << " cou=" << static_cast<uint32_t>(gs->m_texclut.cou)
                      << " cov=" << gs->m_texclut.cov
                      << ")"
                      << " ofx=" << (ctx.xyoffset.ofx >> 4)
                      << " ofy=" << (ctx.xyoffset.ofy >> 4)
                      << " scissor=(" << ctx.scissor.x0
                      << "," << ctx.scissor.y0
                      << ")-(" << ctx.scissor.x1
                      << "," << ctx.scissor.y1 << ")"
                      << " test=0x" << std::hex << ctx.test
                      << " alpha=0x" << ctx.alpha
                      << std::dec << std::endl;
        }
    });

    if (gs->m_hasPreferredDisplaySource && ctx.frame.fbp == gs->m_preferredDisplayDestFbp)
    {
        gs->m_hasPreferredDisplaySource = false;
    }

    // MOH: movie fullscreen fallback. The frontend never writes the movie widget's rect
    // (proven upstream: the layout coord chain 1da8f0->112030 never runs for it), so the
    // PSMT8 movie quads arrive with all vertices collapsed on XYOFFSET (zero area) and the
    // decoded frame stays invisible. When that exact proven-broken case shows up, draw the
    // movie texture as a fullscreen sprite instead. PS2_MOH_MOVIE_FULLSCREEN=0 disables.
    if (gs->m_prim.tme && ctx.tex0.psm == 0x13u &&
        (gs->m_prim.type == GS_PRIM_TRIANGLE || gs->m_prim.type == GS_PRIM_TRISTRIP ||
         gs->m_prim.type == GS_PRIM_TRIFAN))
    {
        static const bool s_mohMovieFullscreen = [] {
            const char *v = std::getenv("PS2_MOH_MOVIE_FULLSCREEN");
            return !v || (std::strcmp(v, "0") != 0);
        }();
        const GSVertex &a = gs->m_vtxQueue[0];
        const GSVertex &b = gs->m_vtxQueue[1];
        const GSVertex &c = gs->m_vtxQueue[2];
        const bool degenerate = (a.x == b.x && a.y == b.y && b.x == c.x && b.y == c.y);
        if (s_mohMovieFullscreen && degenerate)
        {
            static std::atomic<uint32_t> s_log{0};
            if (s_log.fetch_add(1u, std::memory_order_relaxed) < 8u)
            {
                std::cerr << "[MOH:movie-fullscreen] degenerate PSMT8 quad tbp=0x" << std::hex
                          << ctx.tex0.tbp0 << std::dec << " -> fullscreen sprite" << std::endl;
            }
            GSVertex v0 = gs->m_vtxQueue[0];
            GSVertex v1 = gs->m_vtxQueue[0];
            v0.x = static_cast<float>(ctx.xyoffset.ofx >> 4);
            v0.y = static_cast<float>(ctx.xyoffset.ofy >> 4);
            v0.u = 0;
            v0.v = 0;
            v1.x = static_cast<float>((ctx.xyoffset.ofx >> 4) + 640);
            v1.y = static_cast<float>((ctx.xyoffset.ofy >> 4) + 448);
            v1.u = static_cast<uint16_t>(640 << 4);
            v1.v = static_cast<uint16_t>(448 << 4);
            v0.r = v0.g = v0.b = 128;
            v0.a = 128;
            v1.r = v1.g = v1.b = 128;
            v1.a = 128;
            gs->m_vtxQueue[0] = v0;
            gs->m_vtxQueue[1] = v1;
            drawSprite(gs);
            return;
        }
    }

    switch (gs->m_prim.type)
    {
    case GS_PRIM_SPRITE:
        drawSprite(gs);
        break;
    case GS_PRIM_TRIANGLE:
    case GS_PRIM_TRISTRIP:
    case GS_PRIM_TRIFAN:
        drawTriangle(gs);
        break;
    case GS_PRIM_LINE:
    case GS_PRIM_LINESTRIP:
        drawLine(gs);
        break;
    case GS_PRIM_POINT:
    {
        const GSVertex &v = gs->m_vtxQueue[0];
        const auto &ctx = gs->activeContext();
        int px = static_cast<int>(v.x) - (ctx.xyoffset.ofx >> 4);
        int py = static_cast<int>(v.y) - (ctx.xyoffset.ofy >> 4);
        writePixel(gs, px, py, static_cast<u32>(v.z), v.r, v.g, v.b, v.a);
        break;
    }
    default:
        break;
    }
}

void GSRasterizer::writePixel(GS *gs, int x, int y, int z, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    const auto &ctx = gs->activeContext();

    // [MOH diag] reliable trace of the first writePixel calls + their reject cause.
    {
        static std::atomic<uint32_t> s_wp{0};
        const uint32_t n = s_wp.fetch_add(1, std::memory_order_relaxed);
        if (n < 24u)
        {
            const uint32_t inSciss = !(x < ctx.scissor.x0 || x > ctx.scissor.x1 ||
                                       y < ctx.scissor.y0 || y > ctx.scissor.y1);
            std::cerr << "[MOH:wp] n=" << n << " ctxt=" << static_cast<uint32_t>(gs->m_prim.ctxt)
                      << " xy=(" << x << "," << y << ")"
                      << " inSciss=" << inSciss
                      << " zte=" << ((ctx.test >> 16) & 1u)
                      << " ztst=" << ((ctx.test >> 17) & 3u)
                      << " test=0x" << std::hex << static_cast<uint64_t>(ctx.test) << std::dec
                      << " abe=" << static_cast<uint32_t>(gs->m_prim.abe)
                      << " rgba=(" << static_cast<uint32_t>(r) << "," << static_cast<uint32_t>(g)
                      << "," << static_cast<uint32_t>(b) << "," << static_cast<uint32_t>(a) << ")"
                      << std::endl;
        }
    }

    if (x < ctx.scissor.x0 || x > ctx.scissor.x1 ||
        y < ctx.scissor.y0 || y > ctx.scissor.y1)
        return;

    const AlphaTestResult alphaTest = classifyAlphaTest(ctx.test, a);

    if (!alphaTest.writeFramebuffer)
        return;

    u8* vram = gs->m_vram;

    const u32 fbp  = GSInternal::framePageBaseToBlock(ctx.frame.fbp);
    const u32 fbw  = std::max<u32>(ctx.frame.fbw, 1u);
    const u32 fpsm = ctx.frame.psm;
    const u32 fmsk = ctx.frame.fbmsk;
    const u32 zbp = GSInternal::framePageBaseToBlock(ctx.zbuf.zbp);
    const u32 zpsm = ctx.zbuf.psm;

    const bool alphaBlendEnabled = gs->m_prim.abe;
    const bool destinationAlpha  = alphaTest.preserveDestinationAlpha;

    // small optimization, avoid reading the framebuffer for simple draws
    // TODO: only one address lookup for rmw
    const bool frmw = (ctx.frame.fbmsk != 0) || alphaBlendEnabled || destinationAlpha;

    u32 fbrgba = 0;
    if (frmw)
    {
        fbrgba = gs->ReadVram(fpsm, fbp, fbw, x, y);

        if (bitsPerPixel(fpsm) == 16)
        {
            fbrgba = Rgba5551ToRgba8888(fbrgba);
        }
    }

    const bool ztestEnabled = ((ctx.test >> 16) & 1u) != 0u;
    const uint ztest_method = (ctx.test >> 17) & 3u;

    bool zpass = true;
    if (ztestEnabled)
    {
        switch (ztest_method)
        {
        case 0:
            zpass = false;
            break;
        case 1:
            zpass = true;
            break;
        case 2:
            zpass = z >= gs->ReadVram(zpsm, zbp, fbw, x, y);
            break;
        case 3:
            zpass = z > gs->ReadVram(zpsm, zbp, fbw, x, y);
            break;
        default:
            zpass = true;
            break;
        }
    }

    if (!zpass)
    {
        return;
    }

    const u8 srcR = r;
    const u8 srcG = g;
    const u8 srcB = b;

    if (gs->m_prim.abe)
    {
        uint8_t dr = fbrgba & 0xFF;
        uint8_t dg = (fbrgba >> 8) & 0xFF;
        uint8_t db = (fbrgba >> 16) & 0xFF;
        uint8_t da = (fbrgba >> 24) & 0xFF;

        // PABE disables alpha blending when the source alpha MSB is clear.
        if (!(gs->m_pabe && (a & 0x80u) == 0u))
        {
            uint64_t alphaReg = ctx.alpha;
            uint8_t asel = alphaReg & 3;
            uint8_t bsel = (alphaReg >> 2) & 3;
            uint8_t csel = (alphaReg >> 4) & 3;
            uint8_t dsel = (alphaReg >> 6) & 3;
            uint8_t fix = static_cast<uint8_t>((alphaReg >> 32) & 0xFF);

            auto pickRGB = [&](uint8_t sel, int cs, int cd) -> int
            {
                if (sel == 0)
                    return cs;
                if (sel == 1)
                    return cd;
                return 0;
            };
            int cAlpha = (csel == 0) ? a : (csel == 1) ? da
                                                       : fix;

            r = clampU8(((pickRGB(asel, r, dr) - pickRGB(bsel, r, dr)) * cAlpha >> 7) + pickRGB(dsel, r, dr));
            g = clampU8(((pickRGB(asel, g, dg) - pickRGB(bsel, g, dg)) * cAlpha >> 7) + pickRGB(dsel, g, dg));
            b = clampU8(((pickRGB(asel, b, db) - pickRGB(bsel, b, db)) * cAlpha >> 7) + pickRGB(dsel, b, db));
        }
        else
        {
            r = srcR;
            g = srcG;
            b = srcB;
        }
    }

    u32 fbmask = ctx.frame.fbmsk;
    bool zmask = ctx.zbuf.zmask;

    if (!alphaTest.preserveDestinationAlpha &&
        (ctx.fba & 0x1ull) != 0ull &&
        ctx.frame.psm != GS_PSM_CT24)
    {
        a = static_cast<uint8_t>(a | 0x80u);
    }

    u32 pixel = pack32(r, g, b, a);

    if (fbmask != 0)
    {
        pixel = (pixel & ~fbmask) | (fbrgba & fbmask);
    }

    if (alphaTest.preserveDestinationAlpha)
    {
        pixel = (pixel & 0x00FFFFFFu) | (fbrgba & 0xFF000000u);
    }
    
    // format conversion
    if (bitsPerPixel(fpsm) == 16)
    {
        pixel = Rgba8888ToRgba5551(pixel);
    }

    gs->WriteVram(fpsm, fbp, fbw, x, y, pixel);

    if (!zmask)
    {
        gs->WriteVram(zpsm, zbp, fbw, x, y, z);
    }
}

uint32_t GSRasterizer::lookupCLUT(GS *gs,
                                  uint8_t index,
                                  uint32_t cbp,
                                  uint8_t cpsm,
                                  uint8_t csm,
                                  uint8_t csa,
                                  uint8_t sourcePsm)
{
    const uint32_t clutIndex = resolveClutIndex(index, csm, csa, sourcePsm);
    const uint32_t clutWidth = (gs->m_texclut.cbw != 0u) ? static_cast<uint32_t>(gs->m_texclut.cbw) : 1u;
    const uint32_t clutX = static_cast<uint32_t>(gs->m_texclut.cou) + (clutIndex & 0x0Fu);
    const uint32_t clutY = static_cast<uint32_t>(gs->m_texclut.cov) + (clutIndex >> 4);


    switch (cpsm)
    {
    case GS_PSM_CT32:
        return applyTexa(gs->m_texa, cpsm, GSMem::ReadCT32(gs->m_vram, cbp, clutWidth, clutX, clutY));
    case GS_PSM_CT24:
        return applyTexa(gs->m_texa, cpsm, GSMem::ReadCT24(gs->m_vram, cbp, clutWidth, clutX, clutY));
    case GS_PSM_CT16:
        return applyTexa(gs->m_texa, cpsm, GSMem::ReadCT16(gs->m_vram, cbp, clutWidth, clutX, clutY));
    case GS_PSM_CT16S:
        return applyTexa(gs->m_texa, cpsm, GSMem::ReadCT16S(gs->m_vram, cbp, clutWidth, clutX, clutY));
    default:
        break;
    }

    {
        static std::atomic<uint32_t> s_clutMag{0};
        if (s_clutMag.fetch_add(1u) < 16u)
            std::cerr << "[MOH:gs-magenta] lookupCLUT unsupported cpsm=0x" << std::hex << static_cast<int>(cpsm) << std::dec << std::endl;
    }
    return 0xFFFF00FFu;
}

uint32_t GSRasterizer::sampleTexture(GS *gs, float s, float t, float q, uint16_t u, uint16_t v)
{
    const auto &ctx = gs->activeContext();
    const auto &tex = ctx.tex0;

    int texW = 1 << tex.tw;
    int texH = 1 << tex.th;

    float texUf, texVf;
    if (gs->m_prim.fst)
    {
        texUf = static_cast<float>(u) / 16.0f;
        texVf = static_cast<float>(v) / 16.0f;
    }
    else
    {
        const float invQ = 1.0f / fabsQ(q);
        texUf = s * invQ * static_cast<float>(texW);
        texVf = t * invQ * static_cast<float>(texH);
    }

    auto samplePoint = [&](int sampleU, int sampleV) -> uint32_t
    {
        sampleU = clampInt(sampleU, 0, texW - 1);
        sampleV = clampInt(sampleV, 0, texH - 1);

        u32 out = gs->ReadVram(tex.psm, tex.tbp0, tex.tbw, sampleU, sampleV);

        switch (tex.psm)
        {
        case GS_PSM_CT32:
        case GS_PSM_Z32:
        case GS_PSM_CT24:
        case GS_PSM_Z24:
            return applyTexa(gs->m_texa, tex.psm, out);
        case GS_PSM_CT16:
        case GS_PSM_CT16S:
        case GS_PSM_Z16:
        case GS_PSM_Z16S:
            return applyTexa(gs->m_texa, tex.psm, Rgba5551ToRgba8888(out));
        case GS_PSM_T8:
        case GS_PSM_T8H:
        case GS_PSM_T4:
        case GS_PSM_T4HL:
        case GS_PSM_T4HH:
            return lookupCLUT(gs, static_cast<u8>(out), tex.cbp, tex.cpsm, tex.csm, tex.csa, tex.psm);
        }

        {
            static std::atomic<uint32_t> s_texMag{0};
            if (s_texMag.fetch_add(1u) < 16u)
                std::cerr << "[MOH:gs-magenta] samplePoint unsupported tex.psm=0x" << std::hex << static_cast<int>(tex.psm) << std::dec << std::endl;
        }
        return 0xFFFF00FFu;
    };

    if (!tex1UsesLinearFilter(ctx.tex1))
    {
        return samplePoint(static_cast<int>(texUf), static_cast<int>(texVf));
    }

    const float sampleU = texUf - 0.5f;
    const float sampleV = texVf - 0.5f;
    const int u0 = static_cast<int>(std::floor(sampleU));
    const int v0 = static_cast<int>(std::floor(sampleV));
    const int u1 = u0 + 1;
    const int v1 = v0 + 1;
    const float fx = sampleU - static_cast<float>(u0);
    const float fy = sampleV - static_cast<float>(v0);

    const uint32_t c00 = samplePoint(u0, v0);
    const uint32_t c10 = samplePoint(u1, v0);
    const uint32_t c01 = samplePoint(u0, v1);
    const uint32_t c11 = samplePoint(u1, v1);

    const uint8_t r = lerpChannel(static_cast<uint8_t>(c00 & 0xFFu),
                                  static_cast<uint8_t>(c10 & 0xFFu),
                                  static_cast<uint8_t>(c01 & 0xFFu),
                                  static_cast<uint8_t>(c11 & 0xFFu),
                                  fx, fy);
    const uint8_t g = lerpChannel(static_cast<uint8_t>((c00 >> 8) & 0xFFu),
                                  static_cast<uint8_t>((c10 >> 8) & 0xFFu),
                                  static_cast<uint8_t>((c01 >> 8) & 0xFFu),
                                  static_cast<uint8_t>((c11 >> 8) & 0xFFu),
                                  fx, fy);
    const uint8_t b = lerpChannel(static_cast<uint8_t>((c00 >> 16) & 0xFFu),
                                  static_cast<uint8_t>((c10 >> 16) & 0xFFu),
                                  static_cast<uint8_t>((c01 >> 16) & 0xFFu),
                                  static_cast<uint8_t>((c11 >> 16) & 0xFFu),
                                  fx, fy);
    const uint8_t a = lerpChannel(static_cast<uint8_t>((c00 >> 24) & 0xFFu),
                                  static_cast<uint8_t>((c10 >> 24) & 0xFFu),
                                  static_cast<uint8_t>((c01 >> 24) & 0xFFu),
                                  static_cast<uint8_t>((c11 >> 24) & 0xFFu),
                                  fx, fy);

    return static_cast<uint32_t>(r) |
           (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) |
           (static_cast<uint32_t>(a) << 24);
}

void GSRasterizer::drawSprite(GS *gs)
{
    const GSVertex &v0 = gs->m_vtxQueue[0];
    const GSVertex &v1 = gs->m_vtxQueue[1];
    const auto &ctx = gs->activeContext();

    int ofx = ctx.xyoffset.ofx >> 4;
    int ofy = ctx.xyoffset.ofy >> 4;

    int x0 = static_cast<int>(v0.x) - ofx;
    int y0 = static_cast<int>(v0.y) - ofy;
    int x1 = static_cast<int>(v1.x) - ofx;
    int y1 = static_cast<int>(v1.y) - ofy;
    u32 z1 = static_cast<u32>(v1.z);

    if (x0 > x1)
        std::swap(x0, x1);
    if (y0 > y1)
        std::swap(y0, y1);

    const int unclippedX0 = x0;
    const int unclippedY0 = y0;
    const int spanX = std::max(1, x1 - x0);
    const int spanY = std::max(1, y1 - y0);
    const int unclippedX1 = unclippedX0 + spanX - 1;
    const int unclippedY1 = unclippedY0 + spanY - 1;
    const bool intersectsScissor =
        !(unclippedX1 < ctx.scissor.x0 || unclippedX0 > ctx.scissor.x1 ||
          unclippedY1 < ctx.scissor.y0 || unclippedY0 > ctx.scissor.y1);

    {
        static std::atomic<uint32_t> s_mohSpriteTrace{0};
        const uint32_t n = s_mohSpriteTrace.fetch_add(1u, std::memory_order_relaxed);
        if (n < 160u)
        {
            std::cerr << "[MOH:sprite] n=" << n
                      << " ctxt=" << static_cast<uint32_t>(gs->m_prim.ctxt)
                      << " tme=" << static_cast<uint32_t>(gs->m_prim.tme)
                      << " fst=" << static_cast<uint32_t>(gs->m_prim.fst)
                      << " abe=" << static_cast<uint32_t>(gs->m_prim.abe)
                      << " fbp=0x" << std::hex << ctx.frame.fbp
                      << " tex=0x" << ctx.tex0.tbp0
                      << std::dec
                      << " raw=(" << v0.x << "," << v0.y << ")-(" << v1.x << "," << v1.y << ")"
                      << " of=(" << ofx << "," << ofy << ")"
                      << " bbox=(" << unclippedX0 << "," << unclippedY0
                      << ")-(" << unclippedX1 << "," << unclippedY1 << ")"
                      << " scissor=(" << ctx.scissor.x0 << "," << ctx.scissor.y0
                      << ")-(" << ctx.scissor.x1 << "," << ctx.scissor.y1 << ")"
                      << " visible=" << (intersectsScissor ? 1 : 0)
                      << " uv=(" << (v0.u >> 4) << "," << (v0.v >> 4)
                      << ")-(" << (v1.u >> 4) << "," << (v1.v >> 4) << ")"
                      << std::endl;
        }
    }

    // [MOH diag] When the constant off-screen menu sprite (raw v0 == (1548,1903)) is drawn,
    // dump the EE dispatch-PC chain (thread_local, valid since GIF processing is synchronous
    // on GameThread) to identify the recompiled EE function that built/submitted the packet.
    if (static_cast<int>(v0.x) == 1548 && static_cast<int>(v0.y) == 1903)
    {
        extern std::string mohDispatchHistory();
        static std::atomic<uint32_t> s_mohWho{0};
        const uint32_t wn = s_mohWho.fetch_add(1u, std::memory_order_relaxed);
        if (wn < 8u)
        {
            std::cerr << "[MOH:who] n=" << std::dec << wn
                      << " raw=(" << v0.x << "," << v0.y << ")-(" << v1.x << "," << v1.y << ")"
                      << " tme=" << (uint32_t)gs->m_prim.tme
                      << " fst=" << (uint32_t)gs->m_prim.fst
                      << " tbp=0x" << std::hex << ctx.tex0.tbp0
                      << " uv=(" << std::dec << (v0.u >> 4) << "," << (v0.v >> 4) << ")"
                      << " rgba=(" << (int)v1.r << "," << (int)v1.g << "," << (int)v1.b << "," << (int)v1.a << ")"
                      << "\n           ee-dispatch-chain: " << mohDispatchHistory()
                      << std::endl;
        }
    }

    // If the sprite rectangle is fully outside scissor, nothing should render.
    if (!intersectsScissor)
    {
        return;
    }

    const int drawX0 = clampInt(unclippedX0, ctx.scissor.x0, ctx.scissor.x1);
    const int drawY0 = clampInt(unclippedY0, ctx.scissor.y0, ctx.scissor.y1);
    const int drawX1 = clampInt(unclippedX1, ctx.scissor.x0, ctx.scissor.x1);
    const int drawY1 = clampInt(unclippedY1, ctx.scissor.y0, ctx.scissor.y1);

    const uint64_t alphaReg = ctx.alpha;
    const uint8_t alphaMode = static_cast<uint8_t>(alphaReg & 0xFFu);
    const uint8_t alphaFix = static_cast<uint8_t>((alphaReg >> 32) & 0xFFu);
    const bool looksLikeDisplayCopy =
        gs->m_prim.tme &&
        gs->m_prim.abe &&
        gs->m_prim.fst &&
        gs->m_prim.ctxt &&
        ctx.frame.fbp != ctx.tex0.tbp0 &&
        alphaMode == 0x64u &&
        (alphaFix == 0x60u || alphaFix == 0x80u) &&
        unclippedX0 <= 0 &&
        unclippedY0 <= 0 &&
        unclippedX1 >= 639 &&
        unclippedY1 >= 447;
    if (looksLikeDisplayCopy)
    {
        gs->m_preferredDisplaySourceFrame = {ctx.tex0.tbp0, ctx.tex0.tbw, ctx.tex0.psm, 0u};
        gs->m_preferredDisplayDestFbp = ctx.frame.fbp;
        gs->m_hasPreferredDisplaySource = true;
    }

    uint8_t r = v1.r, g = v1.g, b = v1.b, a = v1.a;

    if (gs->m_prim.tme)
    {
        const auto &tex = ctx.tex0;
        int texW = 1 << tex.tw;
        int texH = 1 << tex.th;
        if (texW == 0)
            texW = 1;
        if (texH == 0)
            texH = 1;

        float u0f, v0f, u1f, v1f;
        if (gs->m_prim.fst)
        {
            u0f = static_cast<float>(v0.u >> 4);
            v0f = static_cast<float>(v0.v >> 4);
            u1f = static_cast<float>(v1.u >> 4);
            v1f = static_cast<float>(v1.v >> 4);
        }
        else
        {
            const float q0 = fabsQ(v0.q);
            const float q1 = fabsQ(v1.q);
            u0f = (v0.s / q0) * static_cast<float>(texW);
            v0f = (v0.t / q0) * static_cast<float>(texH);
            u1f = (v1.s / q1) * static_cast<float>(texW);
            v1f = (v1.t / q1) * static_cast<float>(texH);
        }

        float spriteW = static_cast<float>(spanX);
        float spriteH = static_cast<float>(spanY);
        if (spriteW < 1.0f)
            spriteW = 1.0f;
        if (spriteH < 1.0f)
            spriteH = 1.0f;

        for (int y = drawY0; y <= drawY1; ++y)
        {
            float ty = (static_cast<float>(y - unclippedY0) + 0.5f) / spriteH;
            float texVf = v0f + (v1f - v0f) * ty;

            for (int x = drawX0; x <= drawX1; ++x)
            {
                float tx = (static_cast<float>(x - unclippedX0) + 0.5f) / spriteW;
                float texUf = u0f + (u1f - u0f) * tx;
                uint32_t texel = 0xFFFF00FFu;
                if (gs->m_prim.fst)
                {
                    const uint16_t sampleU = static_cast<uint16_t>(clampInt(static_cast<int>(std::lround(texUf * 16.0f)), 0, 0xFFFF));
                    const uint16_t sampleV = static_cast<uint16_t>(clampInt(static_cast<int>(std::lround(texVf * 16.0f)), 0, 0xFFFF));
                    texel = sampleTexture(gs, 0.0f, 0.0f, 1.0f, sampleU, sampleV);
                }
                else
                {
                    texel = sampleTexture(gs,
                                          texUf / static_cast<float>(texW),
                                          texVf / static_cast<float>(texH),
                                          1.0f, 0u, 0u);
                }

                uint8_t tr = static_cast<uint8_t>(texel & 0xFF);
                uint8_t tg = static_cast<uint8_t>((texel >> 8) & 0xFF);
                uint8_t tb = static_cast<uint8_t>((texel >> 16) & 0xFF);
                uint8_t ta = static_cast<uint8_t>((texel >> 24) & 0xFF);

                const TextureCombineResult color = combineTexture(tex, r, g, b, a, tr, tg, tb, ta);
                if (x == drawX0 && y == drawY0)
                {
                    static std::atomic<uint32_t> s_mohTexelTrace{0};
                    const uint32_t tn = s_mohTexelTrace.fetch_add(1u, std::memory_order_relaxed);
                    if (tn < 24u)
                    {
                        std::cerr << "[MOH:texel] spr=" << std::dec << tn
                                  << " tbp=0x" << std::hex << tex.tbp0
                                  << " psm=0x" << static_cast<uint32_t>(tex.psm)
                                  << std::dec << " u0f=" << u0f << " v0f=" << v0f
                                  << " u1f=" << u1f << " v1f=" << v1f
                                  << " texUf=" << texUf << " texVf=" << texVf
                                  << " texel=0x" << std::hex << texel
                                  << " trgba=(" << std::dec << (int)tr << "," << (int)tg << "," << (int)tb << "," << (int)ta << ")"
                                  << " vrgba=(" << (int)r << "," << (int)g << "," << (int)b << "," << (int)a << ")"
                                  << " final=(" << (int)color.r << "," << (int)color.g << "," << (int)color.b << "," << (int)color.a << ")"
                                  << std::endl;
                    }
                }
                writePixel(gs, x, y, z1, color.r, color.g, color.b, color.a);
            }
        }
    }
    else
    {
        for (int y = drawY0; y <= drawY1; ++y)
            for (int x = drawX0; x <= drawX1; ++x)
                writePixel(gs, x, y, z1, r, g, b, a);
    }
}

void GSRasterizer::drawTriangle(GS *gs)
{
    const GSVertex &v0 = gs->m_vtxQueue[0];
    const GSVertex &v1 = gs->m_vtxQueue[1];
    const GSVertex &v2 = gs->m_vtxQueue[2];
    const auto &ctx = gs->activeContext();
    const bool mohFrontendTexture =
        gs->m_prim.tme && ctx.tex0.psm == GS_PSM_T8 &&
        ((ctx.tex0.tbp0 == 0x3800u &&
          (ctx.tex0.cbp == 0x3c60u || ctx.tex0.cbp == 0x3840u)) ||
         (ctx.tex0.tbp0 == 0x3000u &&
          (ctx.tex0.cbp == 0x33c0u || ctx.tex0.cbp == 0x3040u)));
    bool mohFrontendSampleLogged = false;

    int ofx = ctx.xyoffset.ofx >> 4;
    int ofy = ctx.xyoffset.ofy >> 4;

    float fx0 = v0.x - static_cast<float>(ofx);
    float fy0 = v0.y - static_cast<float>(ofy);
    float fx1 = v1.x - static_cast<float>(ofx);
    float fy1 = v1.y - static_cast<float>(ofy);
    float fx2 = v2.x - static_cast<float>(ofx);
    float fy2 = v2.y - static_cast<float>(ofy);

    int minX = static_cast<int>(std::floor(std::min({fx0, fx1, fx2})));
    int maxX = static_cast<int>(std::ceil(std::max({fx0, fx1, fx2})));
    int minY = static_cast<int>(std::floor(std::min({fy0, fy1, fy2})));
    int maxY = static_cast<int>(std::ceil(std::max({fy0, fy1, fy2})));

    minX = clampInt(minX, ctx.scissor.x0, ctx.scissor.x1);
    maxX = clampInt(maxX, ctx.scissor.x0, ctx.scissor.x1);
    minY = clampInt(minY, ctx.scissor.y0, ctx.scissor.y1);
    maxY = clampInt(maxY, ctx.scissor.y0, ctx.scissor.y1);

    float denom = (fy1 - fy2) * (fx0 - fx2) + (fx2 - fx1) * (fy0 - fy2);
    if (std::fabs(denom) < 0.001f)
        return;

    const float winding = (denom < 0.0f) ? -1.0f : 1.0f;
    const float invAbsDenom = 1.0f / std::fabs(denom);
    constexpr float kEdgeEpsilon = 1.0e-4f;

    for (int y = minY; y <= maxY; ++y)
    {
        float py = static_cast<float>(y) + 0.5f;
        for (int x = minX; x <= maxX; ++x)
        {
            float px = static_cast<float>(x) + 0.5f;

            float w0 = (((fy1 - fy2) * (px - fx2) + (fx2 - fx1) * (py - fy2)) * winding) * invAbsDenom;
            float w1 = (((fy2 - fy0) * (px - fx2) + (fx0 - fx2) * (py - fy2)) * winding) * invAbsDenom;
            float w2 = 1.0f - w0 - w1;

            if (w0 < -kEdgeEpsilon || w1 < -kEdgeEpsilon || w2 < -kEdgeEpsilon)
                continue;

            double z = v0.z * w0 + v1.z * w1 + v2.z * w2;

            uint8_t r, g, b, a;
            if (gs->m_prim.iip)
            {
                r = clampU8(static_cast<int>(v0.r * w0 + v1.r * w1 + v2.r * w2));
                g = clampU8(static_cast<int>(v0.g * w0 + v1.g * w1 + v2.g * w2));
                b = clampU8(static_cast<int>(v0.b * w0 + v1.b * w1 + v2.b * w2));
                a = clampU8(static_cast<int>(v0.a * w0 + v1.a * w1 + v2.a * w2));
            }
            else
            {
                r = v2.r;
                g = v2.g;
                b = v2.b;
                a = v2.a;
            }

            float mohSampleS = 0.0f;
            float mohSampleT = 0.0f;
            float mohSampleQ = 1.0f;
            uint16_t mohSampleU = 0u;
            uint16_t mohSampleV = 0u;
            if (gs->m_prim.tme)
            {
                float is, it, iq;
                uint16_t iu, iv;
                if (gs->m_prim.fst)
                {
                    iu = static_cast<uint16_t>(v0.u * w0 + v1.u * w1 + v2.u * w2);
                    iv = static_cast<uint16_t>(v0.v * w0 + v1.v * w1 + v2.v * w2);
                    is = 0.0f;
                    it = 0.0f;
                    iq = 1.0f;
                }
                else
                {
                    const float invQ0 = 1.0f / fabsQ(v0.q);
                    const float invQ1 = 1.0f / fabsQ(v1.q);
                    const float invQ2 = 1.0f / fabsQ(v2.q);
                    const float sOverQ = (v0.s * invQ0) * w0 + (v1.s * invQ1) * w1 + (v2.s * invQ2) * w2;
                    const float tOverQ = (v0.t * invQ0) * w0 + (v1.t * invQ1) * w1 + (v2.t * invQ2) * w2;
                    const float invQ = invQ0 * w0 + invQ1 * w1 + invQ2 * w2;
                    iq = (std::fabs(invQ) > 1.0e-8f) ? (1.0f / invQ) : 1.0f;
                    is = sOverQ * iq;
                    it = tOverQ * iq;
                    iu = 0;
                    iv = 0;
                }

                uint32_t texel = sampleTexture(gs, is, it, iq, iu, iv);
                mohSampleS = is;
                mohSampleT = it;
                mohSampleQ = iq;
                mohSampleU = iu;
                mohSampleV = iv;

                uint8_t tr = static_cast<uint8_t>(texel & 0xFF);
                uint8_t tg = static_cast<uint8_t>((texel >> 8) & 0xFF);
                uint8_t tb = static_cast<uint8_t>((texel >> 16) & 0xFF);
                uint8_t ta = static_cast<uint8_t>((texel >> 24) & 0xFF);

                const auto &tex = ctx.tex0;
                const uint8_t shadeR = r;
                const uint8_t shadeG = g;
                const uint8_t shadeB = b;
                const uint8_t shadeA = a;
                const TextureCombineResult color = combineTexture(tex, shadeR, shadeG, shadeB, shadeA, tr, tg, tb, ta);

                r = color.r;
                g = color.g;
                b = color.b;
                a = color.a;
            }

            bool mohTraceSample = false;
            uint32_t mohTraceIndex = 0u;
            uint32_t mohTraceTexel = 0u;
            uint32_t mohTraceBefore = 0u;
            uint32_t mohTraceSequence = 0u;
            int mohTraceU = 0;
            int mohTraceV = 0;
            if (mohFrontendTexture && !mohFrontendSampleLogged)
            {
                static std::atomic<uint32_t> s_mohFrontendSamples{0u};
                mohTraceSequence = s_mohFrontendSamples.fetch_add(1u, std::memory_order_relaxed);
                mohFrontendSampleLogged = true;
                mohTraceSample = mohTraceSequence < 16u;
                if (mohTraceSample)
                {
                    const auto &tex = ctx.tex0;
                    const int texW = 1 << tex.tw;
                    const int texH = 1 << tex.th;
                    mohTraceU = gs->m_prim.fst
                                    ? clampInt(static_cast<int>(mohSampleU) / 16, 0, texW - 1)
                                    : clampInt(static_cast<int>(mohSampleS * static_cast<float>(texW)), 0, texW - 1);
                    mohTraceV = gs->m_prim.fst
                                    ? clampInt(static_cast<int>(mohSampleV) / 16, 0, texH - 1)
                                    : clampInt(static_cast<int>(mohSampleT * static_cast<float>(texH)), 0, texH - 1);
                    mohTraceIndex = gs->ReadVram(tex.psm, tex.tbp0, tex.tbw, mohTraceU, mohTraceV);
                    mohTraceTexel = sampleTexture(gs,
                                                  mohSampleS,
                                                  mohSampleT,
                                                  mohSampleQ,
                                                  mohSampleU,
                                                  mohSampleV);
                    const uint32_t fbBlock = GSInternal::framePageBaseToBlock(ctx.frame.fbp);
                    mohTraceBefore = gs->ReadVram(ctx.frame.psm,
                                                  fbBlock,
                                                  std::max<uint32_t>(ctx.frame.fbw, 1u),
                                                  x,
                                                  y);
                }
            }

            writePixel(gs, x, y, static_cast<u32>(z), r, g, b, a);

            if (mohTraceSample)
            {
                const uint32_t fbBlock = GSInternal::framePageBaseToBlock(ctx.frame.fbp);
                const uint32_t after = gs->ReadVram(ctx.frame.psm,
                                                    fbBlock,
                                                    std::max<uint32_t>(ctx.frame.fbw, 1u),
                                                    x,
                                                    y);
                std::cerr << "[MOH:menu-gs] n=" << mohTraceSequence
                          << " targetFbp=0x" << std::hex << ctx.frame.fbp
                          << " targetBlock=0x" << fbBlock
                          << " fbw=0x" << ctx.frame.fbw
                          << " fpsm=0x" << static_cast<uint32_t>(ctx.frame.psm)
                          << " tbp=0x" << ctx.tex0.tbp0
                          << " tbw=0x" << static_cast<uint32_t>(ctx.tex0.tbw)
                          << " tpsm=0x" << static_cast<uint32_t>(ctx.tex0.psm)
                          << " cbp=0x" << ctx.tex0.cbp
                          << " cpsm=0x" << static_cast<uint32_t>(ctx.tex0.cpsm)
                          << " index=0x" << mohTraceIndex
                          << " texel=0x" << mohTraceTexel
                          << " before=0x" << mohTraceBefore
                          << " after=0x" << after
                          << " test=0x" << ctx.test
                          << std::dec
                          << " uv=(" << mohTraceU << "," << mohTraceV << ")"
                          << " xy=(" << x << "," << y << ")"
                          << " rgba=(" << static_cast<uint32_t>(r) << ","
                          << static_cast<uint32_t>(g) << ","
                          << static_cast<uint32_t>(b) << ","
                          << static_cast<uint32_t>(a) << ")"
                          << std::endl;
            }
        }
    }
}

void GSRasterizer::drawLine(GS *gs)
{
    const GSVertex &v0 = gs->m_vtxQueue[0];
    const GSVertex &v1 = gs->m_vtxQueue[1];
    const auto &ctx = gs->activeContext();

    int ofx = ctx.xyoffset.ofx >> 4;
    int ofy = ctx.xyoffset.ofy >> 4;

    int x0 = static_cast<int>(v0.x) - ofx;
    int y0 = static_cast<int>(v0.y) - ofy;
    int x1 = static_cast<int>(v1.x) - ofx;
    int y1 = static_cast<int>(v1.y) - ofy;

    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    int totalSteps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    if (totalSteps == 0)
        totalSteps = 1;
    int step = 0;

    for (;;)
    {
        float t = static_cast<float>(step) / static_cast<float>(totalSteps);
        uint8_t r, g, b, a;
        if (gs->m_prim.iip)
        {
            r = clampU8(static_cast<int>(v0.r + (v1.r - v0.r) * t));
            g = clampU8(static_cast<int>(v0.g + (v1.g - v0.g) * t));
            b = clampU8(static_cast<int>(v0.b + (v1.b - v0.b) * t));
            a = clampU8(static_cast<int>(v0.a + (v1.a - v0.a) * t));
        }
        else
        {
            r = v1.r;
            g = v1.g;
            b = v1.b;
            a = v1.a;
        }

        double z = (v0.z + (v1.z - v0.z) * t);

        writePixel(gs, x0, y0, static_cast<u32>(z), r, g, b, a);

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
        ++step;
    }
}
