// Based on Blackline Interactive implementation
#include "runtime/ps2_memory.h"
#include "ps2_syscalls.h"
#include <atomic>
#include <cstring>

// Host-only correlation state shared with the VU1 and GS diagnostics.  These
// values are live only while a synchronous MSCAL callback is executing.
namespace mohvu
{
    // Defined in vu/ps2_vu1_detail.h (private to the VU translation units). Declared here
    // so the MPG upload path can publish which program variant is resident in each bank.
    extern std::atomic<uint32_t> g_bankVariant[8];
}

namespace mohgeom
{
    std::atomic<uint64_t> g_vu1LaunchSequence{0u};
    thread_local bool t_vu1LaunchActive = false;
    thread_local uint64_t t_vu1LaunchId = 0u;
    thread_local uint64_t t_vu1LaunchFrame = 0u;
    thread_local uint64_t t_objectTraceId = 0u;
    thread_local uint64_t t_vifTransferHash = 0u;
    thread_local uint32_t t_vifSourcePhys = 0xFFFFFFFFu;
    thread_local uint32_t t_vifTransferBytes = 0u;
    thread_local uint32_t t_vifCommandOffset = 0u;
    thread_local uint32_t t_vu1StartPc = 0xFFFFFFFFu;
    thread_local uint32_t t_vu1Top = 0u;
    thread_local uint32_t t_vu1Itop = 0u;
    thread_local uint32_t t_matrixMatchMask = 0u;
    thread_local uint32_t t_matrixSlot = 0xFFFFFFFFu;
    thread_local uint32_t t_matrixAddress = 0u;
    thread_local uint32_t t_matrixVuOffset = 0xFFFFFFFFu;
    thread_local uint32_t t_matrixMatchWords = 0u;
    thread_local uint32_t t_matrixBits[16] = {};
    thread_local uint32_t t_ownerHeadWords[32] = {};
    thread_local uint32_t t_ownerTailWords[16] = {};
}

// Defined in ps2_memory.cpp, installed by PS2Runtime::syncCoreSubsystems.
// Dispatches INTC cause 5 (VIF1 interrupt) synchronously.
extern std::function<void()> g_vif1InterruptCallback;

enum VIFCmd : uint8_t
{
    VIF_NOP = 0x00,
    VIF_STCYCL = 0x01,
    VIF_OFFSET = 0x02,
    VIF_BASE = 0x03,
    VIF_ITOP = 0x04,
    VIF_STMOD = 0x05,
    VIF_MSKPATH3 = 0x06,
    VIF_MARK = 0x07,
    VIF_FLUSHE = 0x10,
    VIF_FLUSH = 0x11,
    VIF_FLUSHA = 0x13,
    VIF_MSCAL = 0x14,
    VIF_MSCALF = 0x15,
    VIF_MSCNT = 0x17,
    VIF_STMASK = 0x20,
    VIF_STROW = 0x30,
    VIF_STCOL = 0x31,
    VIF_MPG = 0x4A,
    VIF_DIRECT = 0x50,
    VIF_DIRECTHL = 0x51,
};

namespace
{
    constexpr uint8_t kGifFmtImage = 2u;

    uint64_t mohHashBytes(const uint8_t *data, uint32_t sizeBytes)
    {
        uint64_t hash = 1469598103934665603ull;
        for (uint32_t i = 0u; data && i < sizeBytes; ++i)
        {
            hash ^= data[i];
            hash *= 1099511628211ull;
        }
        return hash;
    }

    uint32_t gifImageQwcFromTag(const uint8_t *data, uint32_t sizeBytes)
    {
        if (!data || sizeBytes < 16u)
            return 0u;

        uint64_t tagLo = 0u;
        std::memcpy(&tagLo, data, sizeof(tagLo));
        const uint8_t flg = static_cast<uint8_t>((tagLo >> 58) & 0x3u);
        if (flg != kGifFmtImage)
            return 0u;

        return static_cast<uint32_t>(tagLo & 0x7FFFu);
    }
}


void PS2Memory::processVIF0Data(uint32_t srcPhys, uint32_t sizeBytes)
{
    if (!m_rdram || sizeBytes == 0u)
        return;
    if (srcPhys >= PS2_RAM_SIZE)
        return;

    const uint64_t requestedEnd = static_cast<uint64_t>(srcPhys) + static_cast<uint64_t>(sizeBytes);
    if (requestedEnd > static_cast<uint64_t>(PS2_RAM_SIZE))
        sizeBytes = PS2_RAM_SIZE - srcPhys;

    processVIF0Data(m_rdram + srcPhys, sizeBytes);
}

void PS2Memory::processVIF0Data(const uint8_t *data, uint32_t sizeBytes)
{
    if (!data || sizeBytes == 0u)
        return;

    uint32_t pos = 0;
    while (pos + 4 <= sizeBytes)
    {
        uint32_t cmd = 0u;
        std::memcpy(&cmd, data + pos, sizeof(cmd));
        pos += 4u;

        const uint8_t opcode = static_cast<uint8_t>((cmd >> 24) & 0x7Fu);
        const uint16_t imm = static_cast<uint16_t>(cmd & 0xFFFFu);
        const uint8_t num = static_cast<uint8_t>((cmd >> 16) & 0xFFu);
        const bool irq = (cmd & 0x80000000u) != 0u;

        vif0_regs.code = cmd;
        vif0_regs.num = num;
        if (irq)
            vif0_regs.stat |= (1u << 11);

        if (opcode == VIF_NOP)
        {
            continue;
        }
        else if (opcode == VIF_STCYCL)
        {
            vif0_regs.cycle = imm;
            continue;
        }
        else if (opcode == VIF_ITOP)
        {
            vif0_regs.itops = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_STMOD)
        {
            vif0_regs.mode = imm & 3u;
            continue;
        }
        else if (opcode == VIF_MARK)
        {
            vif0_regs.mark = imm;
            vif0_regs.stat |= (1u << 6);
            continue;
        }
        else if (opcode == VIF_FLUSHE || opcode == VIF_FLUSH || opcode == VIF_FLUSHA)
        {
            continue;
        }
        else if (opcode == VIF_STMASK)
        {
            if (pos + 4u > sizeBytes)
                break;
            std::memcpy(&vif0_regs.mask, data + pos, sizeof(vif0_regs.mask));
            pos += 4u;
            continue;
        }
        else if (opcode == VIF_STROW)
        {
            if (pos + 16u > sizeBytes)
                break;
            std::memcpy(vif0_regs.row, data + pos, 16u);
            pos += 16u;
            continue;
        }
        else if (opcode == VIF_STCOL)
        {
            if (pos + 16u > sizeBytes)
                break;
            std::memcpy(vif0_regs.col, data + pos, 16u);
            pos += 16u;
            continue;
        }
        else if (opcode == VIF_MPG)
        {
            const uint32_t destAddr = static_cast<uint32_t>(imm & 0x1FFu) * 8u;
            const uint32_t instructionCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
            const uint32_t mpgBytes = instructionCount * 8u;
            uint32_t copyBytes = 0u;
            if (m_vu0Code && destAddr < PS2_VU0_CODE_SIZE && mpgBytes > 0u)
            {
                copyBytes = mpgBytes;
                if (destAddr + copyBytes > PS2_VU0_CODE_SIZE)
                    copyBytes = PS2_VU0_CODE_SIZE - destAddr;
                if (pos + copyBytes <= sizeBytes)
                    std::memcpy(m_vu0Code + destAddr, data + pos, copyBytes);
            }

            pos += mpgBytes;
            if (pos > sizeBytes)
                break;
            continue;
        }
        else if ((opcode & 0x60u) == 0x60u)
        {
            const uint8_t vn = static_cast<uint8_t>((opcode >> 2) & 0x3u);
            const uint8_t vl = static_cast<uint8_t>(opcode & 0x3u);
            const int components = static_cast<int>(vn) + 1;
            int bitsPerComponent = 32;
            switch (vl)
            {
            case 0: bitsPerComponent = 32; break;
            case 1: bitsPerComponent = 16; break;
            case 2: bitsPerComponent = 8; break;
            case 3: bitsPerComponent = (vn == 3u) ? 4 : 16; break;
            default: break;
            }
            const int bitsPerVector = (vl == 3u && vn == 3u) ? 16 : (components * bitsPerComponent);
            uint32_t bytesPerVector = static_cast<uint32_t>((bitsPerVector + 7) / 8);
            const uint32_t writeVectorCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
            uint32_t cl = vif0_regs.cycle & 0xFFu;
            uint32_t wl = (vif0_regs.cycle >> 8) & 0xFFu;
            if (cl == 0u) cl = 1u;
            if (wl == 0u) wl = 1u;
            uint32_t sourceVectorCount = writeVectorCount;
            if (cl < wl)
            {
                const uint32_t fullBlocks = writeVectorCount / wl;
                uint32_t remainder = writeVectorCount % wl;
                if (remainder > cl)
                    remainder = cl;
                sourceVectorCount = fullBlocks * cl + remainder;
            }
            uint32_t totalBytes = sourceVectorCount * bytesPerVector;
            totalBytes = (totalBytes + 3u) & ~3u;

            if (m_vu0Data && pos + totalBytes <= sizeBytes && vl == 0u)
            {
                uint32_t vuAddr = static_cast<uint32_t>(imm & 0x3FFu);
                if ((imm & 0x8000u) != 0u)
                    vuAddr = (vuAddr + (vif0_regs.tops & 0x3FFu)) & 0x3FFu;
                const uint8_t *srcBase = data + pos;
                uint32_t srcIndex = 0u;
                for (uint32_t writeIndex = 0; writeIndex < writeVectorCount; ++writeIndex)
                {
                    const uint32_t cyclePos = writeIndex % wl;
                    const bool sourceAvailable = (cl >= wl) || (cyclePos < cl);
                    uint32_t destVec = (cl >= wl) ? ((vuAddr + (writeIndex / wl) * cl + cyclePos) & 0x3FFu)
                                                   : ((vuAddr + writeIndex) & 0x3FFu);
                    const uint32_t destOff = destVec * 16u;
                    if (destOff + 16u > PS2_VU0_DATA_SIZE)
                    {
                        if (sourceAvailable && srcIndex < sourceVectorCount)
                            ++srcIndex;
                        continue;
                    }
                    if (!sourceAvailable || srcIndex >= sourceVectorCount)
                        continue;
                    const uint8_t *srcVec = srcBase + srcIndex * bytesPerVector;
                    ++srcIndex;
                    uint32_t lanes[4] = {0u, 0u, 0u, 0u};
                    std::memcpy(lanes, m_vu0Data + destOff, sizeof(lanes));
                    const uint32_t limit = (components > 4) ? 4u : static_cast<uint32_t>(components);
                    for (uint32_t c = 0; c < limit; ++c)
                    {
                        uint32_t scalar = 0u;
                        std::memcpy(&scalar, srcVec + c * 4u, sizeof(scalar));
                        lanes[c] = scalar;
                    }
                    _mm_storeu_si128(reinterpret_cast<__m128i *>(m_vu0Data + destOff), _mm_loadu_si128(reinterpret_cast<const __m128i *>(lanes)));
                }
            }
            pos += totalBytes;
            if (pos > sizeBytes)
                break;
            continue;
        }
        else
        {
            break;
        }
    }
}

void PS2Memory::processVIF1Data(uint32_t srcPhys, uint32_t sizeBytes)
{
    if (sizeBytes == 0u || srcPhys >= PS2_RAM_SIZE)
        return;

    const uint64_t requestedEnd = static_cast<uint64_t>(srcPhys) + static_cast<uint64_t>(sizeBytes);
    if (requestedEnd > static_cast<uint64_t>(PS2_RAM_SIZE))
        sizeBytes = PS2_RAM_SIZE - srcPhys;

    processVIF1Data(m_rdram + srcPhys, sizeBytes);
}

void PS2Memory::processVIF1Data(const uint8_t *data, uint32_t sizeBytes)
{
    if (!data || sizeBytes == 0u)
        return;

    const uintptr_t dataAddress = reinterpret_cast<uintptr_t>(data);
    const uintptr_t rdramAddress = reinterpret_cast<uintptr_t>(m_rdram);
    const uint32_t inputSourcePhys =
        m_rdram &&
                dataAddress >= rdramAddress &&
                dataAddress < rdramAddress + PS2_RAM_SIZE
            ? static_cast<uint32_t>(dataAddress - rdramAddress)
            : 0xFFFFFFFFu;
    const uint64_t inputHash = mohHashBytes(data, sizeBytes);

    uint32_t pos = 0;

    while (pos + 4 <= sizeBytes)
    {
        if (m_vif1PendingPath2ImageQwc != 0u)
        {
            const uint32_t availableQw = (sizeBytes - pos) / 16u;
            if (availableQw == 0u)
            {
                break;
            }

            const uint32_t chunkQw = std::min<uint32_t>(m_vif1PendingPath2ImageQwc, availableQw);
            std::vector<uint8_t> imagePacket(16u + static_cast<size_t>(chunkQw) * 16u, 0u);
            const uint64_t imageTag =
                static_cast<uint64_t>(chunkQw & 0x7FFFu) |
                ((m_vif1PendingPath2ImageQwc == chunkQw) ? (1ull << 15) : 0ull) |
                (static_cast<uint64_t>(kGifFmtImage) << 58);
            std::memcpy(imagePacket.data(), &imageTag, sizeof(imageTag));
            std::memcpy(imagePacket.data() + 16u, data + pos, static_cast<size_t>(chunkQw) * 16u);
            submitGifPacket(GifPathId::Path2,
                            imagePacket.data(),
                            static_cast<uint32_t>(imagePacket.size()),
                            true,
                            m_vif1PendingPath2DirectHl);

            pos += chunkQw * 16u;
            m_vif1PendingPath2ImageQwc -= chunkQw;
            if (m_vif1PendingPath2ImageQwc == 0u)
            {
                m_vif1PendingPath2DirectHl = false;
            }
            continue;
        }

        uint32_t cmd;
        memcpy(&cmd, data + pos, 4);
        pos += 4;

        uint8_t opcode = (cmd >> 24) & 0x7F;
        uint16_t imm = cmd & 0xFFFF;
        uint8_t num = (cmd >> 16) & 0xFF;
        const bool irq = (cmd & 0x80000000u) != 0u;

        // Track most-recent command for VIFn_CODE emulation.
        vif1_regs.code = cmd;
        vif1_regs.num = num;
        if (irq)
        {
            vif1_regs.stat |= (1u << 11); // INT
            // Hardware raises INTC cause 5 here and stalls VIF1 until the ISR
            // writes FBRST.STC. MOH's frame display list places one i-bit ahead
            // of each drawing section whose texture the ISR (0x10BFE8) uploads
            // into a shared VRAM staging page, so the dispatch must happen at
            // this exact point in the stream — not after the packet finishes —
            // or later sections sample the wrong staging contents.
            if (g_vif1InterruptCallback)
            {
                static thread_local uint32_t s_vif1IrqDepth = 0u;
                if (s_vif1IrqDepth < 4u)
                {
                    ++s_vif1IrqDepth;
                    vif1_regs.stat &= ~(1u << 11);
                    g_vif1InterruptCallback();
                    --s_vif1IrqDepth;
                }
            }
        }

        if (opcode == VIF_NOP)
        {
            continue;
        }
        else if (opcode == VIF_STCYCL)
        {
            vif1_regs.cycle = imm;
            continue;
        }
        else if (opcode == VIF_OFFSET)
        {
            // VIF double-buffer setup. OFFSET clears DBF and resets TOPS to BASE.
            // Do not rewrite BASE from the previous TOPS value.
            vif1_regs.ofst = imm & 0x3FFu;
            vif1_regs.tops = vif1_regs.base & 0x3FFu;
            vif1_regs.stat &= ~(1u << 7); // clear DBF
            continue;
        }
        else if (opcode == VIF_BASE)
        {
            // BASE only updates the base register. TOPS changes on OFFSET/MSCAL.
            vif1_regs.base = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_ITOP)
        {
            // ITOP VIFcode writes pending ITOPS; VU XITOP observes it after MSCAL/MSCNT.
            vif1_regs.itops = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_STMOD)
        {
            vif1_regs.mode = imm & 3u;
            continue;
        }
        else if (opcode == VIF_MSKPATH3)
        {
            // VIF command docs: MSKPATH3 uses IMMEDIATE bit 15.
            const bool wasMasked = m_path3Masked;
            m_path3Masked = (imm & 0x8000u) != 0u;
            if (wasMasked && !m_path3Masked)
                flushMaskedPath3Packets();
            continue;
        }
        else if (opcode == VIF_MARK)
        {
            vif1_regs.mark = imm;
            vif1_regs.stat |= (1u << 6); // MRK
            continue;
        }
        else if (opcode == VIF_FLUSHE || opcode == VIF_FLUSH || opcode == VIF_FLUSHA)
        {
            continue;
        }
        else if (opcode == VIF_MSCAL || opcode == VIF_MSCALF)
        {
            uint32_t startPC = (uint32_t)imm * 8u;

            // Values visible to the VU program for this MSCAL.
            // DobieStation semantics: ITOP = ITOPS; TOP = current TOPS;
            // then TOPS/DBF are prepared for the next buffer.
            const uint32_t runTop = vif1_regs.tops & 0x3FFu;
            const uint32_t runItop = vif1_regs.itops & 0x3FFu;
            vif1_regs.top = runTop;
            vif1_regs.itop = runItop;

            const bool dbf = (vif1_regs.stat & (1u << 7)) != 0u;
            if (dbf)
                vif1_regs.tops = vif1_regs.base & 0x3FFu;
            else
                vif1_regs.tops = (vif1_regs.base + vif1_regs.ofst) & 0x3FFu;
            vif1_regs.stat ^= (1u << 7); // toggle DBF

            mohgeom::t_vu1LaunchId =
                mohgeom::g_vu1LaunchSequence.fetch_add(
                    1u,
                    std::memory_order_relaxed) +
                1u;
            mohgeom::t_vu1LaunchFrame =
                ps2_syscalls::GetCurrentVSyncTick();
            mohgeom::t_vifTransferHash = inputHash;
            mohgeom::t_vifSourcePhys = inputSourcePhys;
            mohgeom::t_vifTransferBytes = sizeBytes;
            mohgeom::t_vifCommandOffset = pos - 4u;
            mohgeom::t_vu1StartPc = startPC;
            mohgeom::t_vu1Top = runTop;
            mohgeom::t_vu1Itop = runItop;
            mohgeom::t_matrixMatchMask = 0u;
            mohgeom::t_matrixSlot = 0xFFFFFFFFu;
            mohgeom::t_matrixAddress = 0u;
            mohgeom::t_matrixVuOffset = 0xFFFFFFFFu;
            mohgeom::t_matrixMatchWords = 0u;
            mohgeom::t_objectTraceId = 0u;
            std::memset(
                mohgeom::t_matrixBits,
                0,
                sizeof(mohgeom::t_matrixBits));
            std::memset(
                mohgeom::t_ownerHeadWords,
                0,
                sizeof(mohgeom::t_ownerHeadWords));
            std::memset(
                mohgeom::t_ownerTailWords,
                0,
                sizeof(mohgeom::t_ownerTailWords));

            if (startPC == 0u && m_rdram && m_vu1Data)
            {
                constexpr uint32_t kMatrixBase = 0x01D4E000u;
                constexpr uint32_t kMatrixStride = 0x2B0u;
                constexpr uint32_t kMatrixCount = 28u;
                constexpr uint32_t kMatrixBytes = 64u;
                constexpr uint32_t kAffineBytes = 48u;
                uint64_t fullHashes[kMatrixCount] = {};
                uint64_t affineHashes[kMatrixCount] = {};
                for (uint32_t slot = 0u; slot < kMatrixCount; ++slot)
                {
                    const uint8_t *const matrix =
                        m_rdram + kMatrixBase + slot * kMatrixStride;
                    fullHashes[slot] =
                        mohHashBytes(matrix, kMatrixBytes);
                    affineHashes[slot] =
                        mohHashBytes(matrix, kAffineBytes);
                }

                auto findMatrixMatches =
                    [&](uint32_t compareBytes,
                        const uint64_t *slotHashes) -> bool
                {
                    bool found = false;
                    for (uint32_t vuOffset = 0u;
                         vuOffset + compareBytes <= PS2_VU1_DATA_SIZE;
                         vuOffset += 16u)
                    {
                        const uint8_t *const candidate =
                            m_vu1Data + vuOffset;
                        const uint64_t candidateHash =
                            mohHashBytes(candidate, compareBytes);
                        for (uint32_t slot = 0u;
                             slot < kMatrixCount;
                             ++slot)
                        {
                            const uint32_t matrixAddress =
                                kMatrixBase + slot * kMatrixStride;
                            const uint8_t *const matrix =
                                m_rdram + matrixAddress;
                            if (candidateHash != slotHashes[slot] ||
                                std::memcmp(
                                    candidate,
                                    matrix,
                                    compareBytes) != 0)
                            {
                                continue;
                            }

                            mohgeom::t_matrixMatchMask |= 1u << slot;
                            if (!found)
                            {
                                found = true;
                                mohgeom::t_matrixSlot = slot;
                                mohgeom::t_matrixAddress = matrixAddress;
                                mohgeom::t_matrixVuOffset = vuOffset;
                                mohgeom::t_matrixMatchWords =
                                    compareBytes / 4u;
                            }
                        }
                    }
                    return found;
                };

                bool found =
                    findMatrixMatches(kMatrixBytes, fullHashes);
                if (!found)
                {
                    found =
                        findMatrixMatches(kAffineBytes, affineHashes);
                }

                if (found)
                {
                    const uint32_t matrixAddress =
                        mohgeom::t_matrixAddress;
                    const uint32_t ownerAddress =
                        matrixAddress - 0x40u;
                    std::memcpy(
                        mohgeom::t_matrixBits,
                        m_rdram + matrixAddress,
                        sizeof(mohgeom::t_matrixBits));
                    std::memcpy(
                        mohgeom::t_ownerHeadWords,
                        m_rdram + ownerAddress,
                        sizeof(mohgeom::t_ownerHeadWords));

                    static constexpr uint32_t kTailOffsets[16] = {
                        0x100u, 0x11Cu, 0x120u, 0x124u,
                        0x128u, 0x12Cu, 0x180u, 0x1D8u,
                        0x1DCu, 0x1FCu, 0x200u, 0x204u,
                        0x260u, 0x280u, 0x2A0u, 0x2ACu,
                    };
                    for (uint32_t i = 0u; i < 16u; ++i)
                    {
                        std::memcpy(
                            &mohgeom::t_ownerTailWords[i],
                            m_rdram + ownerAddress + kTailOffsets[i],
                            sizeof(uint32_t));
                    }

                    const uint64_t matrixHash =
                        mohHashBytes(
                            m_rdram + matrixAddress,
                            kMatrixBytes);
                    mohgeom::t_objectTraceId =
                        (static_cast<uint64_t>(ownerAddress) << 32u) ^
                        matrixHash;
                }
                else
                {
                    mohgeom::t_objectTraceId =
                        inputHash ^
                        (static_cast<uint64_t>(runTop) << 48u) ^
                        (static_cast<uint64_t>(runItop) << 32u);
                }
            }

            const bool previousLaunchActive =
                mohgeom::t_vu1LaunchActive;
            mohgeom::t_vu1LaunchActive = true;
            if (m_vu1MscalCallback)
                m_vu1MscalCallback(startPC, runTop, runItop);
            mohgeom::t_vu1LaunchActive = previousLaunchActive;
            continue;
        }
        else if (opcode == VIF_MSCNT)
        {
            const uint32_t runTop = vif1_regs.tops & 0x3FFu;
            const uint32_t runItop = vif1_regs.itops & 0x3FFu;
            vif1_regs.top = runTop;
            vif1_regs.itop = runItop;

            const bool dbf = (vif1_regs.stat & (1u << 7)) != 0u;
            if (dbf)
                vif1_regs.tops = vif1_regs.base & 0x3FFu;
            else
                vif1_regs.tops = (vif1_regs.base + vif1_regs.ofst) & 0x3FFu;
            vif1_regs.stat ^= (1u << 7); // toggle DBF

            mohgeom::t_vu1LaunchId =
                mohgeom::g_vu1LaunchSequence.fetch_add(
                    1u,
                    std::memory_order_relaxed) +
                1u;
            mohgeom::t_vu1LaunchFrame =
                ps2_syscalls::GetCurrentVSyncTick();
            mohgeom::t_vifTransferHash = inputHash;
            mohgeom::t_vifSourcePhys = inputSourcePhys;
            mohgeom::t_vifTransferBytes = sizeBytes;
            mohgeom::t_vifCommandOffset = pos - 4u;
            mohgeom::t_vu1Top = runTop;
            mohgeom::t_vu1Itop = runItop;

            const bool previousLaunchActive =
                mohgeom::t_vu1LaunchActive;
            mohgeom::t_vu1LaunchActive = true;
            if (m_vu1MscntCallback)
                m_vu1MscntCallback(runTop, runItop);
            mohgeom::t_vu1LaunchActive = previousLaunchActive;
            continue;
        }
        else if (opcode == VIF_STMASK)
        {
            if (pos + 4 > sizeBytes)
                break;
            uint32_t maskValue = 0;
            std::memcpy(&maskValue, data + pos, sizeof(maskValue));
            vif1_regs.mask = maskValue;
            pos += 4;
            continue;
        }
        else if (opcode == VIF_STROW)
        {
            if (pos + 16 > sizeBytes)
                break;
            std::memcpy(vif1_regs.row, data + pos, 16);
            pos += 16;
            continue;
        }
        else if (opcode == VIF_STCOL)
        {
            if (pos + 16 > sizeBytes)
                break;
            std::memcpy(vif1_regs.col, data + pos, 16);
            pos += 16;
            continue;
        }
        else if (opcode == VIF_MPG)
        {
            uint32_t destAddr = (uint32_t)imm * 8u;
            // VIF MPG semantics: NUM==0 means 256 instructions (2048 bytes).
            // MPG payload is instruction-packed and should not be QW-aligned.
            const uint32_t instructionCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
            const uint32_t mpgBytes = instructionCount * 8u;
            if (m_vu1Code && destAddr < PS2_VU1_CODE_SIZE && mpgBytes > 0)
            {
                uint32_t copyBytes = mpgBytes;
                if (destAddr + copyBytes > PS2_VU1_CODE_SIZE)
                    copyBytes = PS2_VU1_CODE_SIZE - destAddr;
                if (pos + copyBytes <= sizeBytes)
                {
                    std::memcpy(m_vu1Code + destAddr, data + pos, copyBytes);
                    markVU1CodeModified();
                    // Are the repeated uploads to the same address the SAME bytes, or different
                    // programs? This decides whether per-address measurements in the microcode
                    // are meaningful. Hash each payload and count distinct hashes per bank.
                    {
                        uint64_t h = 1469598103934665603ull;   // FNV-1a
                        for (uint32_t i = 0u; i < copyBytes; ++i)
                        { h ^= m_vu1Code[destAddr + i]; h *= 1099511628211ull; }
                        static std::atomic<uint32_t> s_bDst[8] = {};
                        static std::atomic<uint64_t> s_bH[8][4] = {};
                        static std::atomic<uint32_t> s_bN[8] = {};
                        static std::atomic<uint32_t> s_bCount{0};
                        const uint32_t bank = destAddr >> 11;
                        if (bank < 8u)
                        {
                            s_bDst[bank].store(destAddr, std::memory_order_relaxed);
                            const uint32_t nh = s_bN[bank].load(std::memory_order_acquire);
                            bool seen = false;
                            for (uint32_t i = 0u; i < nh && i < 4u; ++i)
                                if (s_bH[bank][i].load(std::memory_order_relaxed) == h) { seen = true; break; }
                            if (!seen && nh < 4u)
                            {
                                s_bH[bank][nh].store(h, std::memory_order_relaxed);
                                s_bN[bank].store(nh + 1u, std::memory_order_release);
                            }
                        }
                        // Publish which variant is now resident in this bank (index into the
                        // per-bank hash table), so execution-time measurements can be tagged.
                        if (bank < 8u)
                        {
                            const uint32_t nh2 = s_bN[bank].load(std::memory_order_acquire);
                            for (uint32_t i = 0u; i < nh2 && i < 4u; ++i)
                                if (s_bH[bank][i].load(std::memory_order_relaxed) == h)
                                { mohvu::g_bankVariant[bank].store(i, std::memory_order_relaxed); break; }
                        }
                        // Dump the transform/FTOI4 window once per distinct variant of bank 2
                        // (0x1000..0x17FF, which contains 0x1390..0x1448). Three dumps, one per
                        // variant, are what the diff needs.
                        // Dump bank 1 around its FTOI4 sites (0x0950, 0x0990, 0x0A00, 0x0A40),
                        // once per variant. b1/v1 produces zero wraps over 3.4 M conversions
                        // while b1/v0 wraps 0.90 %, so the diff between these two programs in
                        // the same address range is the tightest available comparison.
                        if (bank == 1u)
                        {
                            static std::atomic<uint64_t> s_d1[4] = {};
                            const uint32_t nh4 = s_bN[bank].load(std::memory_order_acquire);
                            for (uint32_t i = 0u; i < nh4 && i < 4u; ++i)
                            {
                                if (s_bH[bank][i].load(std::memory_order_relaxed) != h) continue;
                                uint64_t z1 = 0;
                                if (s_d1[i].compare_exchange_strong(z1, h))
                                {
                                    std::cerr << "[MOH:bank1-dump-v1] === banque1 variante " << i
                                              << " hash=0x" << std::hex << h << std::dec << " ===" << std::endl;
                                    for (uint32_t a = 0x0930u; a <= 0x0A60u; a += 8u)
                                    {
                                        uint32_t l3, h3;
                                        std::memcpy(&l3, m_vu1Code + a, 4);
                                        std::memcpy(&h3, m_vu1Code + a + 4, 4);
                                        const uint32_t uo = h3 & 0x3Fu;
                                        const uint32_t us = (h3 & 3u) | ((h3 >> 4) & 0x7Cu);
                                        const uint32_t lop = l3 >> 25;
                                        const uint32_t ls = (l3 & 3u) | ((l3 >> 4) & 0x7Cu);
                                        const char *un = (uo == 0x1Fu) ? " MINIi" : (uo == 0x2Bu) ? " MAX"
                                                         : (uo == 0x1Du) ? " MAXi" : (uo == 0x2Fu) ? " MINI"
                                                         : (uo == 0x1Cu) ? " MULq"
                                                         : (uo >= 0x3Cu && us == 0x15u) ? " FTOI4"
                                                         : (uo >= 0x3Cu && us == 0x1Fu) ? " CLIP" : "";
                                        const char *ln = (lop == 0x40u && (l3 & 0x3Fu) >= 0x3Cu &&
                                                          ls == 0x38u) ? " DIV"
                                                         : (lop == 0x40u && (l3 & 0x3Fu) >= 0x3Cu &&
                                                            ls == 0x3Bu) ? " WAITQ" : "";
                                        std::cerr << "[MOH:bank1-dump-v1] v" << i << " 0x" << std::hex
                                                  << a << " up=" << h3 << " lo=" << l3 << std::dec
                                                  << un << ln << std::endl;
                                    }
                                }
                                break;
                            }
                        }
                        if (bank == 2u)
                        {
                            static std::atomic<uint64_t> s_dumped[4] = {};
                            const uint32_t nh3 = s_bN[bank].load(std::memory_order_acquire);
                            for (uint32_t i = 0u; i < nh3 && i < 4u; ++i)
                            {
                                if (s_bH[bank][i].load(std::memory_order_relaxed) != h) continue;
                                uint64_t z = 0;
                                if (s_dumped[i].compare_exchange_strong(z, h))
                                {
                                    std::cerr << "[MOH:variant-dump-v1] === variante " << i
                                              << " hash=0x" << std::hex << h << std::dec
                                              << " (0x1380..0x1460) ===" << std::endl;
                                    for (uint32_t a = 0x1380u; a <= 0x1460u; a += 8u)
                                    {
                                        uint32_t lo2, hi2;
                                        std::memcpy(&lo2, m_vu1Code + a, 4);
                                        std::memcpy(&hi2, m_vu1Code + a + 4, 4);
                                        const uint32_t uo = hi2 & 0x3Fu;
                                        const uint32_t us = (hi2 & 3u) | ((hi2 >> 4) & 0x7Cu);
                                        const char *un = (uo == 0x1Fu) ? " MINIi" : (uo == 0x2Bu) ? " MAX"
                                                         : (uo == 0x1Du) ? " MAXi" : (uo == 0x1Cu) ? " MULq"
                                                         : (uo >= 0x08u && uo <= 0x0Bu) ? " MADDbc"
                                                         : (uo >= 0x3Cu && us == 0x15u) ? " FTOI4"
                                                         : (uo >= 0x3Cu && us == 0x1Fu) ? " CLIP" : "";
                                        std::cerr << "[MOH:variant-dump-v1] v" << i
                                                  << " 0x" << std::hex << a << " up=" << hi2
                                                  << " lo=" << lo2 << std::dec << un << std::endl;
                                    }
                                }
                                break;
                            }
                        }
                        const uint32_t rc = s_bCount.fetch_add(1u, std::memory_order_relaxed);
                        if ((rc % 200u) == 199u)
                        {
                            std::cerr << "[MOH:bank-identity-v1] programmes distincts par banque:";
                            for (uint32_t b = 0u; b < 8u; ++b)
                            {
                                const uint32_t nh = s_bN[b].load(std::memory_order_acquire);
                                if (nh) std::cerr << " 0x" << std::hex << (b << 11) << std::dec
                                                  << "=" << nh;
                            }
                            std::cerr << std::endl;
                        }
                    }

                    // Which microprograms does the game upload? Comparing this set against
                    // the entry points actually executed says whether a clipping program
                    // exists that this runtime never starts.
                    {
                        static std::atomic<uint32_t> s_dst[16] = {};
                        static std::atomic<uint32_t> s_len[16] = {};
                        static std::atomic<uint64_t> s_cnt[16] = {};
                        static std::atomic<uint32_t> s_n{0};
                        const uint32_t nn = s_n.load(std::memory_order_acquire);
                        bool found = false;
                        for (uint32_t i = 0u; i < nn && i < 16u; ++i)
                            if (s_dst[i].load(std::memory_order_relaxed) == destAddr &&
                                s_len[i].load(std::memory_order_relaxed) == copyBytes)
                            { s_cnt[i].fetch_add(1, std::memory_order_relaxed); found = true; break; }
                        if (!found && nn < 16u)
                        {
                            s_dst[nn].store(destAddr, std::memory_order_relaxed);
                            s_len[nn].store(copyBytes, std::memory_order_relaxed);
                            s_cnt[nn].store(1, std::memory_order_relaxed);
                            s_n.store(nn + 1u, std::memory_order_release);
                        }
                        static std::atomic<uint64_t> s_rep{0};
                        if ((s_rep.fetch_add(1, std::memory_order_relaxed) % 400u) == 399u)
                        {
                            std::cerr << "[MOH:vu1-mpg-v1] uploads:";
                            for (uint32_t i = 0u; i < s_n.load(std::memory_order_acquire) && i < 16u; ++i)
                                std::cerr << " 0x" << std::hex << s_dst[i].load(std::memory_order_relaxed)
                                          << "+" << s_len[i].load(std::memory_order_relaxed)
                                          << std::dec << "x" << s_cnt[i].load(std::memory_order_relaxed);
                            std::cerr << std::endl;
                        }
                    }
                }
            }
            pos += mpgBytes;
            if (pos > sizeBytes)
                break;
            continue;
        }
        else if (opcode == VIF_DIRECT || opcode == VIF_DIRECTHL)
        {
            uint32_t qwCount = imm;
            if (qwCount == 0)
                qwCount = 65536;
            const uint32_t availableQw = (sizeBytes - pos) / 16u;
            const bool truncated = qwCount > availableQw;
            if (qwCount > availableQw)
                qwCount = availableQw;

            if (qwCount > 0)
            {
                const bool directHl = (opcode == VIF_DIRECTHL);
                // Guest address of the DIRECT payload. PATH2 is where the
                // screen-filling mixed-Q primitives come from, so knowing which
                // guest buffer holds them points at the EE code that builds them.
                {
                    static std::atomic<uint32_t> s_directSeen{0};
                    const uint32_t k = s_directSeen.fetch_add(1u, std::memory_order_relaxed);
                    if (k < 16u)
                    {
                        const uint8_t *base = m_rdram;
                        const ptrdiff_t off = (data + pos) - base;
                        std::cerr << "[MOH:path2-src-v1] k=" << std::dec << k
                                  << " qw=" << qwCount
                                  << " guestAddr=0x" << std::hex
                                  << ((off >= 0 && off < (ptrdiff_t)0x02000000)
                                          ? (uint32_t)off : 0xFFFFFFFFu)
                                  << " hl=" << std::dec << (directHl ? 1 : 0)
                                  << std::endl;
                    }
                }
                submitGifPacket(GifPathId::Path2, data + pos, qwCount * 16, true, directHl);

                const uint32_t imageQw = gifImageQwcFromTag(data + pos, qwCount * 16u);
                if (imageQw != 0u)
                {
                    const uint32_t inlineImageQw = (qwCount > 0u) ? (qwCount - 1u) : 0u;
                    if (imageQw > inlineImageQw)
                    {
                        m_vif1PendingPath2ImageQwc = imageQw - inlineImageQw;
                        m_vif1PendingPath2DirectHl = directHl;
                    }
                }
            }

            pos += qwCount * 16;
            if (truncated)
            {
                pos = sizeBytes;
                break;
            }
            continue;
        }
        else if ((opcode & 0x60) == 0x60)
        {
            uint8_t vn = (opcode >> 2) & 0x3;
            uint8_t vl = opcode & 0x3;
            const bool maskEnable = (opcode & 0x10u) != 0u;
            int components = vn + 1;
            int bitsPerComponent = 32;
            switch (vl)
            {
            case 0:
                bitsPerComponent = 32;
                break;
            case 1:
                bitsPerComponent = 16;
                break;
            case 2:
                bitsPerComponent = 8;
                break;
            case 3:
                bitsPerComponent = (vn == 3) ? 4 : 16;
                break;
            default:
                break;
            }
            int bitsPerVector = (vl == 3 && vn == 3) ? 16 : (components * bitsPerComponent);
            uint32_t bytesPerVector = (bitsPerVector + 7) / 8;
            // UNPACK semantics: NUM is 8-bit and NUM==0 means 256 vectors (writes).
            const uint32_t writeVectorCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);

            // STCYCL controls write cycles for UNPACK.
            uint32_t cl = vif1_regs.cycle & 0xFFu;
            uint32_t wl = (vif1_regs.cycle >> 8) & 0xFFu;
            if (cl == 0u)
                cl = 1u;
            if (wl == 0u)
                wl = 1u;

            uint32_t sourceVectorCount = writeVectorCount;
            if (cl < wl)
            {
                const uint32_t fullBlocks = writeVectorCount / wl;
                uint32_t remainder = writeVectorCount % wl;
                if (remainder > cl)
                    remainder = cl;
                sourceVectorCount = fullBlocks * cl + remainder;
            }

            uint32_t totalBytes = sourceVectorCount * bytesPerVector;
            totalBytes = (totalBytes + 3) & ~3u;

            // Which UNPACK modes does the game actually use? The vertex stream is the
            // only remaining candidate for the divergence, since microcode and constants
            // are proved byte-identical to the console. A mode decoded wrongly here would
            // corrupt vertices without touching anything else.
            {
                static std::atomic<uint32_t> s_mk[64] = {};
                static std::atomic<uint64_t> s_mc[64] = {};
                static std::atomic<uint32_t> s_mn{0};
                const uint32_t key = ((uint32_t)vn << 4) | ((uint32_t)vl << 2) |
                                     (((imm & 0x4000u) != 0u) ? 2u : 0u) |
                                     ((cl >= wl) ? 1u : 0u);
                const uint32_t n0 = s_mn.load(std::memory_order_acquire);
                int slot = -1;
                for (uint32_t i = 0u; i < n0 && i < 64u; ++i)
                    if (s_mk[i].load(std::memory_order_relaxed) == key) { slot = (int)i; break; }
                if (slot < 0 && n0 < 64u)
                { s_mk[n0].store(key, std::memory_order_relaxed); s_mn.store(n0+1u, std::memory_order_release); slot = (int)n0; }
                if (slot >= 0)
                {
                    const uint64_t c = s_mc[slot].fetch_add(1, std::memory_order_relaxed);
                    if ((c % 20000u) == 19999u)
                    {
                        std::cerr << "[MOH:unpack-modes-v1]";
                        for (uint32_t i = 0u; i < s_mn.load(std::memory_order_acquire) && i < 64u; ++i)
                        {
                            const uint32_t k = s_mk[i].load(std::memory_order_relaxed);
                            std::cerr << " V" << ((k>>4)&3)+1 << "-"
                                      << (((k>>2)&3)==0?32:((k>>2)&3)==1?16:((k>>2)&3)==2?8:4)
                                      << ((k&2)?"z":"s") << ((k&1)?"":"S")
                                      << "=" << s_mc[i].load(std::memory_order_relaxed);
                        }
                        std::cerr << std::endl;
                    }
                }
            }
            uint32_t vuAddr = (uint32_t)imm & 0x3FFu;
            if ((imm & 0x8000u) != 0u)
                vuAddr = (vuAddr + (vif1_regs.tops & 0x3FFu)) & 0x3FFu;

            const bool zeroExtend = (imm & 0x4000u) != 0u;

            if (m_vu1Data && totalBytes > 0 && pos + totalBytes <= sizeBytes)
            {
                const uint8_t *srcBase = data + pos;
                uint32_t srcIndex = 0u;
                for (uint32_t writeIndex = 0; writeIndex < writeVectorCount; ++writeIndex)
                {
                    const uint32_t cyclePos = writeIndex % wl;
                    const bool sourceAvailable = (cl >= wl) || (cyclePos < cl);

                    uint32_t destVec = 0;
                    if (cl >= wl)
                    {
                        destVec = (vuAddr + (writeIndex / wl) * cl + cyclePos) & 0x3FFu;
                    }
                    else
                    {
                        destVec = (vuAddr + writeIndex) & 0x3FFu;
                    }

                    uint32_t destOff = destVec * 16u;
                    if (destOff + 16u > PS2_VU1_DATA_SIZE)
                    {
                        if (sourceAvailable && srcIndex < sourceVectorCount)
                            ++srcIndex;
                        continue;
                    }

                    uint32_t lanes[4] = {0u, 0u, 0u, 0u};
                    std::memcpy(lanes, m_vu1Data + destOff, sizeof(lanes));
                    uint32_t decompressed[4] = {lanes[0], lanes[1], lanes[2], lanes[3]};
                    bool decoded = false;

                    const uint8_t *srcVec = nullptr;
                    if (sourceAvailable && srcIndex < sourceVectorCount)
                    {
                        srcVec = srcBase + srcIndex * bytesPerVector;
                        ++srcIndex;
                        decoded = true;
                    }

                    auto extend16 = [&](uint16_t raw) -> uint32_t
                    {
                        if (zeroExtend)
                            return static_cast<uint32_t>(raw);
                        return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(raw)));
                    };

                    auto extend8 = [&](uint8_t raw) -> uint32_t
                    {
                        if (zeroExtend)
                            return static_cast<uint32_t>(raw);
                        return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(raw)));
                    };

                    bool handledFormat = true;
                    if (!decoded)
                    {
                        handledFormat = false;
                    }
                    else if (vl == 0u)
                    {
                        if (components == 1)
                        {
                            uint32_t scalar = 0;
                            std::memcpy(&scalar, srcVec, sizeof(scalar));
                            decompressed[0] = scalar;
                            decompressed[1] = scalar;
                            decompressed[2] = scalar;
                            decompressed[3] = scalar;
                        }
                        else
                        {
                            const uint32_t limit = (components > 4) ? 4u : static_cast<uint32_t>(components);
                            for (uint32_t c = 0; c < limit; ++c)
                            {
                                uint32_t scalar = 0;
                                std::memcpy(&scalar, srcVec + c * 4u, sizeof(scalar));
                                decompressed[c] = scalar;
                            }
                        }
                    }
                    else if (vl == 1u)
                    {
                        if (components == 1)
                        {
                            uint16_t raw = 0;
                            std::memcpy(&raw, srcVec, sizeof(raw));
                            const uint32_t scalar = extend16(raw);
                            decompressed[0] = scalar;
                            decompressed[1] = scalar;
                            decompressed[2] = scalar;
                            decompressed[3] = scalar;
                        }
                        else
                        {
                            const uint32_t limit = (components > 4) ? 4u : static_cast<uint32_t>(components);
                            for (uint32_t c = 0; c < limit; ++c)
                            {
                                uint16_t raw = 0;
                                std::memcpy(&raw, srcVec + c * 2u, sizeof(raw));
                                decompressed[c] = extend16(raw);
                            }
                        }
                    }
                    else if (vl == 2u)
                    {
                        if (components == 1)
                        {
                            const uint32_t scalar = extend8(srcVec[0]);
                            decompressed[0] = scalar;
                            decompressed[1] = scalar;
                            decompressed[2] = scalar;
                            decompressed[3] = scalar;
                        }
                        else
                        {
                            const uint32_t limit = (components > 4) ? 4u : static_cast<uint32_t>(components);
                            for (uint32_t c = 0; c < limit; ++c)
                            {
                                decompressed[c] = extend8(srcVec[c]);
                            }
                        }
                    }
                    else if (vl == 3u && vn == 3u)
                    {
                        // V4-5: packed color-like format in a single 16-bit value.
                        uint16_t packed = 0;
                        std::memcpy(&packed, srcVec, sizeof(packed));
                        decompressed[0] = packed & 0x1Fu;
                        decompressed[1] = (packed >> 5) & 0x1Fu;
                        decompressed[2] = (packed >> 10) & 0x1Fu;
                        decompressed[3] = (packed >> 15) & 0x01u;
                    }
                    else
                    {
                        handledFormat = false;
                    }

                    // Unknown compressed format fallback: preserve legacy raw-copy behavior.
                    if (!handledFormat && decoded && !maskEnable && (vif1_regs.mode == 0u || vif1_regs.mode == 3u))
                    {
                        uint32_t copyBytes = (bytesPerVector < 16u) ? bytesPerVector : 16u;
                        std::memcpy(m_vu1Data + destOff, srcVec, copyBytes);
                        continue;
                    }

                    const bool canAdd = (vl != 3u || vn != 3u);
                    const uint32_t mode = vif1_regs.mode & 3u;
                    const uint32_t colIdx = (cyclePos > 3u) ? 3u : cyclePos;
                    const uint32_t maskCycle = (cyclePos > 3u) ? 3u : cyclePos;

                    for (uint32_t field = 0u; field < 4u; ++field)
                    {
                        uint32_t maskSpec = 0u;
                        if (maskEnable)
                        {
                            const uint32_t shift = ((maskCycle * 4u) + field) * 2u;
                            maskSpec = (vif1_regs.mask >> shift) & 0x3u;
                        }

                        // In fill-write cycles with suspended source reads, treat raw-data selections as row-fill.
                        if (!decoded && maskSpec == 0u)
                            maskSpec = 1u;

                        uint32_t writeVal = lanes[field];
                        if (maskSpec == 0u)
                        {
                            if (handledFormat)
                            {
                                writeVal = decompressed[field];
                                if (canAdd && (mode == 1u || mode == 2u))
                                {
                                    writeVal = writeVal + vif1_regs.row[field];
                                    if (mode == 2u)
                                        vif1_regs.row[field] = writeVal;
                                }
                            }
                        }
                        else if (maskSpec == 1u)
                        {
                            writeVal = vif1_regs.row[field];
                        }
                        else if (maskSpec == 2u)
                        {
                            writeVal = vif1_regs.col[colIdx];
                        }
                        else
                        {
                            continue; // write-protect
                        }

                        lanes[field] = writeVal;
                    }

                    std::memcpy(m_vu1Data + destOff, lanes, sizeof(lanes));
                }
            }
            pos += totalBytes;

            if (pos > sizeBytes)
                break;
            continue;
        }
        else
        {
            continue;
        }
    }
}
