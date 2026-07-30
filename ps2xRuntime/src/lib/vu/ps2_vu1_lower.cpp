#include <atomic>
#include <cstdio>
#include "runtime/ps2_vu1.h"
#include "runtime/ps2_gif_arbiter.h"
#include "runtime/ps2_gs_gpu.h"
#include "runtime/ps2_memory.h"
#include "ps2_vu1_detail.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace mohgeom
{
    std::atomic<uint64_t> g_xgkickSequence{0u};
    thread_local uint64_t t_xgkickId = 0u;
    thread_local uint64_t t_xgkickPacketHash = 0u;
    thread_local uint32_t t_xgkickProgram = 0xFFFFFFFFu;
    thread_local uint32_t t_xgkickMicroPc = 0xFFFFFFFFu;
    thread_local uint32_t t_xgkickPacketAddress = 0u;
    thread_local uint32_t t_xgkickPacketBytes = 0u;
}

// ============================================================================
// Lower instructions
// ============================================================================
void VU1Interpreter::execLower(uint32_t instr, uint8_t *vuData, uint32_t dataSize, GS &gs, PS2Memory *memory, uint32_t upperInstr)
{
    (void)upperInstr;
    if (instr == 0x00000000 || instr == 0x8000033C) // NOP
        return;

    uint8_t opHi = (instr >> 25) & 0x7F;

    // The lower instruction encoding uses bits 31:25 for the primary opcode
    switch (opHi)
    {
    case 0x00: // LQ (Load Quadword from VU data memory)
    {
        uint8_t it = FT(instr);      // VF destination
        uint8_t is = VIS(instr);    // VI base
        uint8_t dest = (instr >> 21) & 0xF;
        int16_t imm = IMM11(instr);
        uint32_t addr = ((uint32_t)(int32_t)(m_state.vi[is] + imm)) * 16u;
        addr &= (dataSize - 1);
        if (addr + 16 <= dataSize)
        {
            float tmp[4];
            std::memcpy(tmp, vuData + addr, 16);
            applyDest(m_state.vf[it], tmp, dest);
        }
        return;
    }
    case 0x01: // SQ (Store Quadword to VU data memory)
    {
        uint8_t is = FS(instr);      // VF source
        uint8_t it = VIT(instr);     // VI base
        uint8_t dest = (instr >> 21) & 0xF;
        int16_t imm = IMM11(instr);
        uint32_t addr = ((uint32_t)(int32_t)(m_state.vi[it] + imm)) * 16u;
        addr &= (dataSize - 1);
        if (addr + 16 <= dataSize)
        {
            float tmp[4];
            std::memcpy(tmp, vuData + addr, 16);
            if (dest & 0x8)
                tmp[0] = m_state.vf[is][0];
            if (dest & 0x4)
                tmp[1] = m_state.vf[is][1];
            if (dest & 0x2)
                tmp[2] = m_state.vf[is][2];
            if (dest & 0x1)
                tmp[3] = m_state.vf[is][3];
            std::memcpy(vuData + addr, tmp, 16);
        }
        return;
    }
    case 0x04: // ILW (Integer Load Word from VU data memory)
    {
        uint8_t it = VIT(instr);     // VI destination
        uint8_t is = VIS(instr);     // VI base
        uint8_t dest = (instr >> 21) & 0xF;
        int16_t imm = IMM11(instr);
        uint32_t addr = ((uint32_t)(int32_t)(m_state.vi[is] + imm)) * 16u;
        addr &= (dataSize - 1);
        if (addr + 16 <= dataSize)
        {
            int comp = 0;
            if (dest & 0x8)
                comp = 0;
            else if (dest & 0x4)
                comp = 1;
            else if (dest & 0x2)
                comp = 2;
            else
                comp = 3;
            uint32_t v;
            std::memcpy(&v, vuData + addr + comp * 4, 4);
            if (it != 0)
                m_state.vi[it] = (int32_t)(int16_t)(v & 0xFFFF);
        }
        return;
    }
    case 0x05: // ISW (Integer Store Word to VU data memory)
    {
        uint8_t it = VIT(instr);     // VI source
        uint8_t is = VIS(instr);     // VI base
        uint8_t dest = (instr >> 21) & 0xF;
        int16_t imm = IMM11(instr);
        uint32_t addr = ((uint32_t)(int32_t)(m_state.vi[is] + imm)) * 16u;
        addr &= (dataSize - 1);
        if (addr + 16 <= dataSize)
        {
            uint32_t val = (uint32_t)(uint16_t)(m_state.vi[it] & 0xFFFF);
            if (dest & 0x8)
                std::memcpy(vuData + addr + 0, &val, 4);
            if (dest & 0x4)
                std::memcpy(vuData + addr + 4, &val, 4);
            if (dest & 0x2)
                std::memcpy(vuData + addr + 8, &val, 4);
            if (dest & 0x1)
                std::memcpy(vuData + addr + 12, &val, 4);
        }
        return;
    }
    case 0x08: // IADDIU
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        int16_t imm = (int16_t)(instr & 0x7FF) | ((instr >> 10) & 0x7800);
        if (it != 0)
            m_state.vi[it] = (int16_t)(m_state.vi[is] + imm);
        return;
    }
    case 0x09: // ISUBIU
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        int16_t imm = (int16_t)(instr & 0x7FF) | ((instr >> 10) & 0x7800);
        if (it != 0)
            m_state.vi[it] = (int16_t)(m_state.vi[is] - imm);
        return;
    }
    case 0x10: // FCEQ
    {
        uint32_t imm24 = instr & 0xFFFFFF;
        if (mohvu::ProgramStats *ps = mohvu::t_currentProgram)
            ps->fceq.fetch_add(1, std::memory_order_relaxed);
        if (1 != 0)
            m_state.vi[1] = ((m_state.clip & 0xFFFFFF) == imm24) ? 1 : 0;
        mohvu::noteFlagImm(m_state.pc, imm24, "FCEQ", m_state.vi[1] != 0);
        return;
    }
    case 0x11: // FCSET
    {
        m_state.clip = instr & 0xFFFFFF;
        return;
    }
    case 0x12: // FCAND
    {
        uint32_t imm24 = instr & 0xFFFFFF;
        if (mohvu::ProgramStats *ps = mohvu::t_currentProgram)
            ps->fcand.fetch_add(1, std::memory_order_relaxed);

        if (mohvu::ProgramStats *psArm = mohvu::t_currentProgram)
        {
            if (mohvu::t_clipsSinceNegW < 4u &&
                psArm->startPc.load(std::memory_order_relaxed) == 0u &&
                mohvu::g_chronoState.load(std::memory_order_relaxed) == 0 &&
                false /* old arming disabled: clip-route arm below owns the trace */)
            {
                mohvu::g_chronoState.store(1, std::memory_order_relaxed);
                const uint64_t id =
                    mohvu::g_triangleTraceId.fetch_add(1, std::memory_order_relaxed);
                mohvu::g_armedTraceId.store(id, std::memory_order_relaxed);
                const uint32_t cf = m_state.clip & 0xFFFFFFu;
                std::fprintf(stderr,
                    "[MOH:vu1-tri-reject-v1] ARM FCAND id=%llu pc=0x%04x instr=%08x "
                    "CF=%06x groups=[newest=%02x v1=%02x v2=%02x oldest=%02x] imm24=%06x "
                    "vitField=%u viOld=%d negWAgo=%u negW=%g negWFlags=0x%02x\n",
                    (unsigned long long)id, (unsigned)m_state.pc, (unsigned)instr,
                    (unsigned)cf,
                    (unsigned)(cf & 0x3Fu), (unsigned)((cf >> 6) & 0x3Fu),
                    (unsigned)((cf >> 12) & 0x3Fu), (unsigned)((cf >> 18) & 0x3Fu),
                    (unsigned)imm24, (unsigned)VIT(instr),
                    (int)(int16_t)m_state.vi[1], (unsigned)mohvu::t_clipsSinceNegW,
                    mohvu::t_lastNegW, (unsigned)mohvu::t_lastNegWFlags);
                for (uint32_t k = 0; k < 4u; ++k)
                {
                    // Newest first: the CF group at bits 5:0 is the most recent CLIP.
                    const mohvu::ClipRecord &rec =
                        mohvu::t_clipRing[(mohvu::t_clipRingPos - 1u - k) & 3u];
                    std::fprintf(stderr,
                        "[MOH:vu1-tri-reject-v1]   vtx[-%u] pc=0x%04x "
                        "xyz=(%.4f,%.4f,%.4f) W=%.6f Wbits=%08x flags=0x%02x\n",
                        k, (unsigned)rec.pc, rec.x, rec.y, rec.z, rec.w,
                        (unsigned)rec.wBits, (unsigned)rec.flags);
                }
            }
        }
        if (1 != 0)
            m_state.vi[1] = ((m_state.clip & imm24) != 0) ? 1 : 0;
        mohvu::noteFlagImm(m_state.pc, imm24, "FCAND", m_state.vi[1] != 0);
        if (mohvu::ProgramStats *ps = mohvu::t_currentProgram)
            if (m_state.vi[1] != 0)
                ps->fcandTrue.fetch_add(1, std::memory_order_relaxed);
        // The FCAND at 0x12d8 masks every clip bit except the far plane: a
        // non-zero result means "some vertex is outside", and the IBEQ at 0x12e0
        // then falls through into the clipping/subdivision route at 0x12e8.
        // Arm there, on a triangle that still has a W<0 vertex in the CF window:
        // this is the exact near-plane crossing case that must be clipped.
        if (m_state.pc == 0x12D8u && m_state.vi[1] != 0 &&
            mohvu::t_clipsSinceNegW < 4u &&
            mohvu::t_currentProgram &&
            mohvu::t_currentProgram->startPc.load(std::memory_order_relaxed) == 0u &&
            mohvu::g_chronoState.load(std::memory_order_relaxed) == 0)
        {
            mohvu::g_chronoState.store(1, std::memory_order_relaxed);
            const uint32_t cf = m_state.clip & 0xFFFFFFu;
            std::fprintf(stderr,
                "[MOH:vu1-clip-route-v1] ARM pc=0x12d8 CF=%06x groups=[%02x %02x %02x] "
                "imm24=%06x vi01=%d negWAgo=%u negW=%g\n",
                (unsigned)cf, (unsigned)(cf & 0x3Fu), (unsigned)((cf >> 6) & 0x3Fu),
                (unsigned)((cf >> 12) & 0x3Fu), (unsigned)imm24,
                (int)(int16_t)m_state.vi[1], (unsigned)mohvu::t_clipsSinceNegW,
                mohvu::t_lastNegW);
            for (uint32_t k = 0; k < 3u; ++k)
            {
                const mohvu::ClipRecord &rec =
                    mohvu::t_clipRing[(mohvu::t_clipRingPos - 1u - k) & 3u];
                std::fprintf(stderr,
                    "[MOH:vu1-clip-route-v1]   vtx[-%u] xyz=(%.3f,%.3f,%.3f) "
                    "W=%.6f flags=0x%02x\n",
                    k, rec.x, rec.y, rec.z, rec.w, (unsigned)rec.flags);
            }
        }
        if (mohvu::chronoArmed())
            std::fprintf(stderr,
                "[MOH:vu1-reject-chrono-v1]   FCAND imm=%06x clip=%06x -> vi01=%d\n",
                (unsigned)imm24, (unsigned)(m_state.clip & 0xFFFFFFu),
                (int)(int16_t)m_state.vi[1]);
        return;
    }
    case 0x13: // FCOR
    {
        uint32_t imm24 = instr & 0xFFFFFF;
        if (mohvu::ProgramStats *ps = mohvu::t_currentProgram)
            ps->fcor.fetch_add(1, std::memory_order_relaxed);

        if (mohvu::ProgramStats *psArm = mohvu::t_currentProgram)
        {
            if (mohvu::t_clipsSinceNegW < 4u &&
                psArm->startPc.load(std::memory_order_relaxed) == 0u &&
                mohvu::g_chronoState.load(std::memory_order_relaxed) == 0 &&
                false /* old arming disabled: clip-route arm below owns the trace */)
            {
                mohvu::g_chronoState.store(1, std::memory_order_relaxed);
                const uint64_t id =
                    mohvu::g_triangleTraceId.fetch_add(1, std::memory_order_relaxed);
                mohvu::g_armedTraceId.store(id, std::memory_order_relaxed);
                const uint32_t cf = m_state.clip & 0xFFFFFFu;
                std::fprintf(stderr,
                    "[MOH:vu1-tri-reject-v1] ARM FCOR id=%llu pc=0x%04x instr=%08x "
                    "CF=%06x groups=[newest=%02x v1=%02x v2=%02x oldest=%02x] imm24=%06x "
                    "vitField=%u viOld=%d negWAgo=%u negW=%g negWFlags=0x%02x\n",
                    (unsigned long long)id, (unsigned)m_state.pc, (unsigned)instr,
                    (unsigned)cf,
                    (unsigned)(cf & 0x3Fu), (unsigned)((cf >> 6) & 0x3Fu),
                    (unsigned)((cf >> 12) & 0x3Fu), (unsigned)((cf >> 18) & 0x3Fu),
                    (unsigned)imm24, (unsigned)VIT(instr),
                    (int)(int16_t)m_state.vi[1], (unsigned)mohvu::t_clipsSinceNegW,
                    mohvu::t_lastNegW, (unsigned)mohvu::t_lastNegWFlags);
                for (uint32_t k = 0; k < 4u; ++k)
                {
                    // Newest first: the CF group at bits 5:0 is the most recent CLIP.
                    const mohvu::ClipRecord &rec =
                        mohvu::t_clipRing[(mohvu::t_clipRingPos - 1u - k) & 3u];
                    std::fprintf(stderr,
                        "[MOH:vu1-tri-reject-v1]   vtx[-%u] pc=0x%04x "
                        "xyz=(%.4f,%.4f,%.4f) W=%.6f Wbits=%08x flags=0x%02x\n",
                        k, (unsigned)rec.pc, rec.x, rec.y, rec.z, rec.w,
                        (unsigned)rec.wBits, (unsigned)rec.flags);
                }
            }
        }
        if (1 != 0)
            m_state.vi[1] = ((m_state.clip | imm24) == 0xFFFFFF) ? 1 : 0;
        mohvu::noteFlagImm(m_state.pc, imm24, "FCOR", m_state.vi[1] != 0);
        // A per-triangle trivial-reject test that returns 0 lets the triangle
        // through. When the last test of the 0x1260..0x1298 block passes while a
        // W<0 vertex is still inside the CF window, this is exactly a near-plane
        // crossing triangle that survives trivial reject and will be emitted -
        // the pathological case. Mark it so the NEXT arming point captures it.
        // The block runs six trivial-reject tests (one per frustum plane) at
        // 0x1260/70/80/90/98/a0. A triangle whose LAST test still returns 0 has
        // survived all six and falls through to the emission block at 0x12b0.
        // Arm only on that case, with a W<0 vertex still inside the CF window:
        // this is precisely a near-plane crossing triangle that will be emitted.
        // (kept for reference; the arming below now targets the clip path)
        mohvu::t_pendingSurvivor =
            (m_state.vi[1] == 0) && (m_state.pc == 0x12A0u) &&
            (mohvu::t_clipsSinceNegW < 4u);
        if (mohvu::ProgramStats *ps = mohvu::t_currentProgram)
            if (m_state.vi[1] != 0)
                ps->fcorTrue.fetch_add(1, std::memory_order_relaxed);
        if (mohvu::chronoArmed())
            std::fprintf(stderr,
                "[MOH:vu1-reject-chrono-v1]   FCOR imm=%06x clip=%06x -> vi01=%d\n",
                (unsigned)imm24, (unsigned)(m_state.clip & 0xFFFFFFu),
                (int)(int16_t)m_state.vi[1]);
        return;
    }
    case 0x14: // FSEQ
    {
        uint16_t imm12 = instr & 0xFFF;
        if (1 != 0)
            m_state.vi[1] = ((m_state.status & 0xFFF) == imm12) ? 1 : 0;
        return;
    }
    case 0x15: // FSSET
    {
        m_state.status = (instr >> 6) & 0xFC0;
        return;
    }
    case 0x16: // FSAND
    {
        uint16_t imm12 = instr & 0xFFF;
        if (1 != 0)
            m_state.vi[1] = (int32_t)(m_state.status & imm12);
        return;
    }
    case 0x17: // FSOR
    {
        uint16_t imm12 = instr & 0xFFF;
        if (1 != 0)
            m_state.vi[1] = ((m_state.status | imm12) == 0xFFF) ? 1 : 0;
        return;
    }
    case 0x18: // FMAND
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        if (it != 0)
            m_state.vi[it] = (int32_t)(m_state.mac & (uint32_t)(uint16_t)m_state.vi[is]);
            // MAC-flag tests were dead before this session (m_state.mac was never
            // written). If the flags are still subtly wrong, a rejection the game
            // performs through them will not fire. Report the true rate.
            {
                static std::atomic<uint64_t> s_n{0}, s_true{0};
                const uint64_t c = s_n.fetch_add(1, std::memory_order_relaxed);
                if (m_state.vi[it] != 0) s_true.fetch_add(1, std::memory_order_relaxed);
                if ((c % 500000u) == 499999u)
                    std::fprintf(stderr, "[MOH:fmand-v1] n=%llu true=%llu mac=%04x\n",
                                 (unsigned long long)c + 1u,
                                 (unsigned long long)s_true.load(std::memory_order_relaxed),
                                 (unsigned)(m_state.mac & 0xFFFFu));
            }
        return;
    }
    case 0x1A: // FMEQ
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        if (it != 0)
            m_state.vi[it] = ((m_state.mac & 0xFFFF) == (uint32_t)(uint16_t)m_state.vi[is]) ? 1 : 0;
        return;
    }
    case 0x1C: // FMOR
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        if (it != 0)
            m_state.vi[it] = (int32_t)(m_state.mac | (uint32_t)(uint16_t)m_state.vi[is]);
        return;
    }
    case 0x20: // B (unconditional branch)
    {
        int16_t imm = IMM11(instr);
        uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
        m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        return;
    }
    case 0x21: // BAL (Branch and link)
    {
        uint8_t it = VIT(instr);
        int16_t imm = IMM11(instr);
        uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
        if (it != 0)
            m_state.vi[it] = (int32_t)((m_state.pc + 16) / 8);
        m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        return;
    }
    case 0x24: // JR
    {
        uint8_t is = VIS(instr);
        uint32_t target = ((uint32_t)(uint16_t)m_state.vi[is] * 8u) & 0x3FFF;
        m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        return;
    }
    case 0x25: // JALR
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        uint32_t target = ((uint32_t)(uint16_t)m_state.vi[is] * 8u) & 0x3FFF;
        if (it != 0)
            m_state.vi[it] = (int32_t)((m_state.pc + 16) / 8);
        m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        return;
    }
    case 0x28: // IBEQ
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] == (int16_t)m_state.vi[it])
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x29: // IBNE
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] != (int16_t)m_state.vi[it])
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x2C: // IBLTZ
    {
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] < 0)
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x2D: // IBGTZ
    {
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] > 0)
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x2E: // IBLEZ
    {
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] <= 0)
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x2F: // IBGEZ
    {
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] >= 0)
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }

    case 0x40: // Lower1 / lower special. Bit31 set; low 6 bits select integer or special op.
    {
        const uint8_t funct = instr & 0x3Fu;
        const uint8_t vfT = FT(instr);
        const uint8_t vfS = FS(instr);
        const uint8_t viT = VIT(instr);
        const uint8_t viS = VIS(instr);
        const uint8_t viD = VID(instr);
        const uint8_t dest = (instr >> 21) & 0xF;

        auto doXgkick = [&]()
        {
            mohvu::g_xgkickSeq.fetch_add(1, std::memory_order_relaxed);
            if (!vuData || dataSize < 16u)
                return;

            auto wrapOffset = [&](uint32_t off) -> uint32_t
            {
                return off % dataSize;
            };

            auto read64Wrap = [&](uint32_t off) -> uint64_t
            {
                uint8_t bytes[8];
                for (uint32_t i = 0; i < 8u; ++i)
                {
                    bytes[i] = vuData[wrapOffset(off + i)];
                }
                uint64_t value = 0;
                std::memcpy(&value, bytes, sizeof(value));
                return value;
            };

            uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
            addr = wrapOffset(addr);
            uint32_t pktOff = addr;
            uint32_t totalBytes = 0u;
            bool done = false;

            for (int safety = 0; safety < 256 && !done; ++safety)
            {
                uint64_t tagLo = read64Wrap(pktOff);
                uint32_t nloop = (uint32_t)(tagLo & 0x7FFFu);
                uint8_t flg = (uint8_t)((tagLo >> 58) & 0x3u);
                uint32_t nreg = (uint32_t)((tagLo >> 60) & 0xFu);
                if (nreg == 0u)
                    nreg = 16u;
                bool eop = ((tagLo >> 15) & 0x1ull) != 0ull;

                uint32_t pktSize = 16u;
                if (flg == 0u)
                {
                    pktSize += nloop * nreg * 16u;

                    // In PACKED mode the ST descriptor carries Q in bits 64..95.
                    // A packet whose ST descriptors mix Q signs produces a
                    // primitive where Q crosses zero, which is what smears the
                    // texture. Report the first few such packets with the micro-PC
                    // that built them.
                    const uint64_t tagHiLocal = read64Wrap(pktOff + 8u);
                    bool sawPos = false, sawNeg = false;
                    uint32_t stCount = 0u;
                    float firstQ = 0.0f, lastQ = 0.0f;
                    for (uint32_t loop = 0u; loop < nloop && loop < 64u; ++loop)
                    {
                        for (uint32_t r = 0u; r < nreg; ++r)
                        {
                            const uint32_t reg =
                                (uint32_t)((tagHiLocal >> (r * 4u)) & 0xFu);
                            if (reg != 0x02u) continue; // ST
                            const uint32_t qOff =
                                pktOff + 16u + (loop * nreg + r) * 16u + 8u;
                            const uint32_t qBits = (uint32_t)(read64Wrap(qOff) & 0xFFFFFFFFu);
                            float q; std::memcpy(&q, &qBits, 4);
                            if (stCount == 0u) firstQ = q;
                            lastQ = q;
                            ++stCount;
                            if (q > 0.0f) sawPos = true;
                            if (q < 0.0f) sawNeg = true;
                        }
                    }
                    if (sawPos && sawNeg)
                    {
                        static std::atomic<uint32_t> s_mixed{0};
                        const uint32_t k = s_mixed.fetch_add(1u, std::memory_order_relaxed);
                        if (k < 10u)
                        {
                            std::fprintf(stderr,
                                         "[MOH:vu1-mixedq-v1] k=%u pc=0x%04x nloop=%u nreg=%u"
                                         " stCount=%u firstQ=%.6f lastQ=%.6f\n",
                                         k, m_state.pc, nloop, nreg, stCount,
                                         (double)firstQ, (double)lastQ);
                            // Which matrix is live? A screen-space post-process
                            // quad should not be running through the scene
                            // world->clip matrix. Scene matrix rows are
                            // vf25, vf24, vf23, vf22 in that order.
                            for (int r : {25, 24, 23, 22})
                                std::fprintf(stderr,
                                             "[MOH:vu1-mixedq-v1]   vf%02d = %14.6f %14.6f %14.6f %14.6f\n",
                                             r, m_state.vf[r][0], m_state.vf[r][1],
                                             m_state.vf[r][2], m_state.vf[r][3]);
                        }
                    }
                }
                else if (flg == 1u)
                {
                    uint32_t regs = nloop * nreg;
                    pktSize += regs * 8u;
                    if ((regs & 1u) != 0u)
                        pktSize += 8u;
                }
                else if (flg == 2u)
                {
                    pktSize += nloop * 16u;
                }

                if (pktSize == 0u)
                    break;

                totalBytes += pktSize;
                pktOff = wrapOffset(pktOff + pktSize);
                if (eop)
                    done = true;
            }

            if (totalBytes == 0u)
                return;

            auto publishXgkickCorrelation =
                [&](const uint8_t *packetData)
            {
                uint64_t hash = 1469598103934665603ull;
                for (uint32_t i = 0u;
                     packetData && i < totalBytes;
                     ++i)
                {
                    hash ^= packetData[i];
                    hash *= 1099511628211ull;
                }

                mohgeom::t_xgkickId =
                    mohgeom::g_xgkickSequence.fetch_add(
                        1u,
                        std::memory_order_relaxed) +
                    1u;
                mohgeom::t_xgkickPacketHash = hash;
                mohgeom::t_xgkickProgram =
                    mohvu::t_currentProgram
                        ? mohvu::t_currentProgram->startPc.load(
                              std::memory_order_relaxed)
                        : 0xFFFFFFFFu;
                mohgeom::t_xgkickMicroPc = m_state.pc;
                mohgeom::t_xgkickPacketAddress = addr;
                mohgeom::t_xgkickPacketBytes = totalBytes;
            };

            if (addr + totalBytes <= dataSize)
            {
                publishXgkickCorrelation(vuData + addr);
                if (memory)
                    memory->submitGifPacket(GifPathId::Path1, vuData + addr, totalBytes);
                else
                    gs.processGIFPacket(vuData + addr, totalBytes);
            }
            else
            {
                std::vector<uint8_t> wrappedPacket(totalBytes);
                for (uint32_t i = 0; i < totalBytes; ++i)
                {
                    wrappedPacket[i] = vuData[wrapOffset(addr + i)];
                }

                publishXgkickCorrelation(wrappedPacket.data());
                if (memory)
                    memory->submitGifPacket(GifPathId::Path1, wrappedPacket.data(), totalBytes);
                else
                    gs.processGIFPacket(wrappedPacket.data(), totalBytes);
            }
        };

        switch (funct)
        {
        case 0x30: // IADD
            if (viD != 0)
                m_state.vi[viD] = (int16_t)(m_state.vi[viS] + m_state.vi[viT]);
            return;
        case 0x31: // ISUB
            if (viD != 0)
                m_state.vi[viD] = (int16_t)(m_state.vi[viS] - m_state.vi[viT]);
            return;
        case 0x32: // IADDI
        {
            int16_t imm5 = (int16_t)((int32_t)((instr >> 6) & 0x1F) << 27 >> 27);
            if (viT != 0)
                m_state.vi[viT] = (int16_t)(m_state.vi[viS] + imm5);
            return;
        }
        case 0x34: // IAND
            if (viD != 0)
                m_state.vi[viD] = m_state.vi[viS] & m_state.vi[viT];
            return;
        case 0x35: // IOR
            if (viD != 0)
                m_state.vi[viD] = m_state.vi[viS] | m_state.vi[viT];
            return;

        case 0x3C:
        case 0x3D:
        case 0x3E:
        case 0x3F: // Lower1 special. Dobie decodes this as (instr & 3) | ((instr >> 4) & 0x7C).
        {
            const uint8_t funct2 = (uint8_t)((instr & 0x3u) | ((instr >> 4) & 0x7Cu));
            switch (funct2)
            {
            case 0x30: // MOVE
            {
                float tmp[4];
                std::memcpy(tmp, m_state.vf[vfS], 16);
                applyDest(m_state.vf[vfT], tmp, dest);
                return;
            }
            case 0x31: // MR32 (rotate right by 32 bits = shift xyzw -> yzwx)
            {
                float tmp[4] = {m_state.vf[vfS][1], m_state.vf[vfS][2], m_state.vf[vfS][3], m_state.vf[vfS][0]};
                applyDest(m_state.vf[vfT], tmp, dest);
                return;
            }
            case 0x34: // LQI (Load Quadword, post-increment)
            {
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    float tmp[4];
                    std::memcpy(tmp, vuData + addr, 16);
                    applyDest(m_state.vf[vfT], tmp, dest);
                }
                if (viS != 0)
                    m_state.vi[viS] = (int16_t)(m_state.vi[viS] + 1);
                return;
            }
            case 0x35: // SQI (Store Quadword, post-increment)
                // Where does vf20 land? 0x1440 writes it via FTOI4 and its output was
                // reconstructed as screen XY (NDC*320 + 2048). Confirm by recording the
                // VU1-data address it is stored to, which is what XGKICK later sends.
                if (vfS == 20u)
                {
                    static std::atomic<uint64_t> s_st{0};
                    const uint64_t k = s_st.fetch_add(1, std::memory_order_relaxed);
                    if (k < 8u)
                        std::fprintf(stderr,
                                     "[MOH:vf20-store-v1] k=%llu pc=0x%04x SQI vf20 -> vi%02u=0x%04x"
                                     " (octet 0x%04x) dest=%01x\n",
                                     (unsigned long long)k, m_state.pc, viT,
                                     (unsigned)m_state.vi[viT], (unsigned)(m_state.vi[viT] * 16u),
                                     (unsigned)((instr >> 21) & 0xF));
                }
            {
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viT]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    float tmp[4];
                    std::memcpy(tmp, vuData + addr, 16);
                    if (dest & 0x8)
                        tmp[0] = m_state.vf[vfS][0];
                    if (dest & 0x4)
                        tmp[1] = m_state.vf[vfS][1];
                    if (dest & 0x2)
                        tmp[2] = m_state.vf[vfS][2];
                    if (dest & 0x1)
                        tmp[3] = m_state.vf[vfS][3];
                    std::memcpy(vuData + addr, tmp, 16);
                }
                if (viT != 0)
                    m_state.vi[viT] = (int16_t)(m_state.vi[viT] + 1);
                return;
            }
            case 0x36: // LQD (Load Quadword, pre-decrement)
            {
                if (viS != 0)
                    m_state.vi[viS] = (int16_t)(m_state.vi[viS] - 1);
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    float tmp[4];
                    std::memcpy(tmp, vuData + addr, 16);
                    applyDest(m_state.vf[vfT], tmp, dest);
                }
                return;
            }
            case 0x37: // SQD (Store Quadword, pre-decrement)
            {
                if (viT != 0)
                    m_state.vi[viT] = (int16_t)(m_state.vi[viT] - 1);
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viT]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    float tmp[4];
                    std::memcpy(tmp, vuData + addr, 16);
                    if (dest & 0x8)
                        tmp[0] = m_state.vf[vfS][0];
                    if (dest & 0x4)
                        tmp[1] = m_state.vf[vfS][1];
                    if (dest & 0x2)
                        tmp[2] = m_state.vf[vfS][2];
                    if (dest & 0x1)
                        tmp[3] = m_state.vf[vfS][3];
                    std::memcpy(vuData + addr, tmp, 16);
                }
                return;
            }
            case 0x38: // DIV
                // Address-independent instruction census. The microcode banks at
                // 0x0800-0x1800 are overwritten ~86 times per run, so any per-address
                // measurement is meaningless; counting instruction executions is not.
                // WAITQ is characteristic of code that stalls for a divide, i.e. the
                // interpolation a clip needs.
                {
                    // Where do the divides execute? Counted at execution, so unlike the
                    // one-shot route trace this is not a snapshot of one bank. The early
                    // conclusion that the 0x12F0 route has no arithmetic came from a single
                    // trace of a range that is rewritten 85 times per run.
                    {
                        static std::atomic<uint64_t> s_divPc[9] = {};
                        const uint32_t r = m_state.pc >> 9;
                        if (r < 9u)
                        {
                            const uint64_t dc = s_divPc[r].fetch_add(1, std::memory_order_relaxed);
                            if ((dc % 100000u) == 99999u)
                            {
                                std::fprintf(stderr, "[MOH:div-by-pc-v1]");
                                for (uint32_t i = 0u; i < 9u; ++i)
                                    std::fprintf(stderr, " 0x%04x=%llu", i * 0x200u,
                                                 (unsigned long long)s_divPc[i].load(std::memory_order_relaxed));
                                std::fprintf(stderr, "\n");
                            }
                        }
                    }
                    static std::atomic<uint64_t> s_div{0};
                    const uint64_t d = s_div.fetch_add(1, std::memory_order_relaxed);
                    if ((d % 200000u) == 199999u)
                        std::fprintf(stderr, "[MOH:lower-census-v1] DIV=%llu\n",
                                     (unsigned long long)(d + 1));
                }
            {
                int fsf = (instr >> 21) & 0x3;
                int ftf = (instr >> 23) & 0x3;
                float num = m_state.vf[vfS][fsf];
                float den = m_state.vf[vfT][ftf];
                const float qv = (den != 0.0f)
                                     ? (num / den)
                                     : ((num >= 0.0f) ? std::numeric_limits<float>::max()
                                                      : -std::numeric_limits<float>::max());
                if (mohvu::divLatencyEnabled())
                {
                    mohvu::t_qPending = qv;   // visible 7 cycles later
                    mohvu::t_qDelay = 7;
                }
                else
                {
                    m_state.q = qv;
                }
                return;
            }
            case 0x39: // SQRT
            {
                int ftf = (instr >> 23) & 0x3;
                float val = m_state.vf[vfT][ftf];
                m_state.q = std::sqrt(std::fabs(val));
                return;
            }
            case 0x3A: // RSQRT
            {
                int fsf = (instr >> 21) & 0x3;
                int ftf = (instr >> 23) & 0x3;
                float num = m_state.vf[vfS][fsf];
                float den = std::sqrt(std::fabs(m_state.vf[vfT][ftf]));
                if (den != 0.0f)
                    m_state.q = num / den;
                else
                    m_state.q = std::numeric_limits<float>::max();
                return;
            }
            case 0x3B: // WAITQ
                {
                    mohvu::g_waitqTicks.fetch_add(1, std::memory_order_relaxed);
                    if (mohvu::ProgramStats *pw = mohvu::t_currentProgram)
                        pw->waitqCount.fetch_add(1, std::memory_order_relaxed);
                    static std::atomic<uint64_t> s_wq{0};
                    const uint64_t w = s_wq.fetch_add(1, std::memory_order_relaxed);
                    if (w == 0u || (w % 20000u) == 19999u)
                        std::fprintf(stderr, "[MOH:lower-census-v1] WAITQ=%llu\n",
                                     (unsigned long long)(w + 1));
                }
                // Stall until the pending divide lands.
                if (mohvu::divLatencyEnabled() && mohvu::t_qDelay > 0)
                {
                    m_state.q = mohvu::t_qPending;
                    mohvu::t_qDelay = 0;
                }
                return;
            case 0x3C: // MTIR (Move To Integer Register)
            {
                int comp = 0;
                if (dest & 0x8)
                    comp = 0;
                else if (dest & 0x4)
                    comp = 1;
                else if (dest & 0x2)
                    comp = 2;
                else
                    comp = 3;
                uint32_t fval;
                std::memcpy(&fval, &m_state.vf[vfS][comp], 4);
                if (viT != 0)
                    m_state.vi[viT] = (int32_t)(int16_t)(fval & 0xFFFF);
                return;
            }
            case 0x3D: // MFIR (Move From Integer Register)
            {
                float result[4];
                int32_t val = (int32_t)(int16_t)(m_state.vi[viS] & 0xFFFF);
                std::memcpy(&result[0], &val, 4);
                result[1] = result[0];
                result[2] = result[0];
                result[3] = result[0];
                applyDest(m_state.vf[vfT], result, dest);
                return;
            }
            case 0x3E: // ILWR - integer load word from address in VI[is]
            {
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    int comp = 0;
                    if (dest & 0x8)
                        comp = 0;
                    else if (dest & 0x4)
                        comp = 1;
                    else if (dest & 0x2)
                        comp = 2;
                    else
                        comp = 3;
                    uint32_t v;
                    std::memcpy(&v, vuData + addr + comp * 4, 4);
                    if (viT != 0)
                        m_state.vi[viT] = (int32_t)(int16_t)(v & 0xFFFF);
                }
                return;
            }
            case 0x3F: // ISWR - integer store word to address in VI[is]
            {
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    uint32_t val = (uint32_t)(uint16_t)(m_state.vi[viT] & 0xFFFF);
                    if (dest & 0x8)
                        std::memcpy(vuData + addr + 0, &val, 4);
                    if (dest & 0x4)
                        std::memcpy(vuData + addr + 4, &val, 4);
                    if (dest & 0x2)
                        std::memcpy(vuData + addr + 8, &val, 4);
                    if (dest & 0x1)
                        std::memcpy(vuData + addr + 12, &val, 4);
                }
                return;
            }
            case 0x40: // RNEXT
                return;
            case 0x41: // RGET
                return;
            case 0x42: // RINIT
                return;
            case 0x43: // RXOR
                return;
            case 0x64: // MFP (Move From P register)
            {
                float result[4] = {m_state.p, m_state.p, m_state.p, m_state.p};
                applyDest(m_state.vf[vfT], result, dest);
                return;
            }
            case 0x68: // XTOP - move current VIF1 TOP into VI register
            {
                if (viT != 0)
                    m_state.vi[viT] = (int32_t)(m_state.top & 0x3FFu);
                return;
            }
            case 0x69: // XITOP - move current VIF1 ITOP into VI register
            {
                if (viT != 0)
                    m_state.vi[viT] = (int32_t)(m_state.itop & 0x3FFu);
                return;
            }
            case 0x6C: // XGKICK - send GIF packet from VU1 data memory
                if (mohvu::ProgramStats *ps = mohvu::t_currentProgram)
                    ps->xgkick.fetch_add(1, std::memory_order_relaxed);
                mohvu::noteXgkick(m_state.pc, viS,
                                  (uint32_t)(uint16_t)m_state.vi[viS]);
                if (mohvu::chronoArmed())
                {
                    std::fprintf(stderr,
                        "[MOH:vu1-reject-chrono-v1] <<XGKICK>> pc=0x%04x vi%u addr=0x%04x\n",
                        (unsigned)m_state.pc, (unsigned)viS,
                        (unsigned)(uint16_t)m_state.vi[viS]);
                    mohvu::g_chronoState.store(2, std::memory_order_relaxed);
                }
                mohvu::g_lastXgkickProgram.store(
                    mohvu::t_currentProgram
                        ? mohvu::t_currentProgram->startPc.load(std::memory_order_relaxed)
                        : 0xFFFFFFFFu,
                    std::memory_order_relaxed);
                doXgkick();
                return;
            case 0x70: // ESADD
                return;
            case 0x71: // ERSADD
                return;
            case 0x72: // ELENG
            {
                float s = m_state.vf[vfS][0] * m_state.vf[vfS][0] + m_state.vf[vfS][1] * m_state.vf[vfS][1] + m_state.vf[vfS][2] * m_state.vf[vfS][2];
                m_state.p = std::sqrt(s);
                return;
            }
            case 0x73: // ERLENG
            {
                float s = m_state.vf[vfS][0] * m_state.vf[vfS][0] + m_state.vf[vfS][1] * m_state.vf[vfS][1] + m_state.vf[vfS][2] * m_state.vf[vfS][2];
                float len = std::sqrt(s);
                m_state.p = (len != 0.0f) ? (1.0f / len) : std::numeric_limits<float>::max();
                return;
            }
            case 0x7A: // ERCPR
            {
                int fsf = (instr >> 21) & 0x3;
                float val = m_state.vf[vfS][fsf];
                m_state.p = (val != 0.0f) ? (1.0f / val) : std::numeric_limits<float>::max();
                return;
            }
            case 0x7B: // WAITP
                return;
            case 0x7D: // EATAN / EATANxy / EATANxz placeholder
                return;
            default:
                return;
            }
        }
        default:
            return;
        }
    }
    default:
        break;
    }
}
