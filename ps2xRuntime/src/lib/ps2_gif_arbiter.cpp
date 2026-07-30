#include "runtime/ps2_gif_arbiter.h"
#include <algorithm>
#include <cstring>

namespace mohgeom
{
    thread_local uint32_t t_gifPathId = 0u;
}

GifArbiter::GifArbiter(ProcessPacketFn processFn)
    : m_processFn(std::move(processFn))
{
}

bool GifArbiter::isImagePacket(const uint8_t *data, uint32_t sizeBytes)
{
    if (!data || sizeBytes < 16u)
        return false;

    uint64_t tagLo = 0;
    std::memcpy(&tagLo, data, sizeof(tagLo));
    const uint8_t flg = static_cast<uint8_t>((tagLo >> 58) & 0x3u);
    return flg == 2u;
}

void GifArbiter::submit(GifPathId pathId, const uint8_t *data, uint32_t sizeBytes, bool path2DirectHl)
{
    if (!data || sizeBytes < 16 || !m_processFn)
        return;

    GifArbiterPacket pkt;
    pkt.pathId = pathId;
    pkt.path2DirectHl = (pathId == GifPathId::Path2) && path2DirectHl;
    pkt.path3Image = (pathId == GifPathId::Path3) && isImagePacket(data, sizeBytes);
    pkt.data.resize(sizeBytes);
    std::memcpy(pkt.data.data(), data, sizeBytes);
    m_queue.push_back(std::move(pkt));
}

void GifArbiter::drain()
{
    if (!m_processFn)
        return;

    // [MOH diagnostic, env-gated, default off] Reordering every PATH1 packet
    // ahead of every PATH2 packet in a drain batch changes the GS register state
    // that later packets inherit - m_curQ in particular. Hardware arbitrates
    // per transfer, not by bulk-sorting a batch. Setting
    // PS2_MOH_DIAG_NO_GIF_SORT=1 keeps submission order so the effect can be
    // measured. Default behaviour is unchanged.
    static const bool noSort = [] {
        const char *v = std::getenv("PS2_MOH_DIAG_NO_GIF_SORT");
        return v && !std::strcmp(v, "1");
    }();
    if (!noSort)
    std::stable_sort(m_queue.begin(), m_queue.end(),
                     [](const GifArbiterPacket &a, const GifArbiterPacket &b)
                     {
                         // DIRECTHL cannot preempt PATH3 IMAGE transfers.
                         if (a.path2DirectHl != b.path2DirectHl || a.path3Image != b.path3Image)
                         {
                             if (a.path3Image && b.path2DirectHl)
                                 return true;
                             if (a.path2DirectHl && b.path3Image)
                                 return false;
                         }
                         return pathPriority(a.pathId) < pathPriority(b.pathId);
                     });

    for (size_t i = 0; i < m_queue.size(); ++i)
    {
        auto &pkt = m_queue[i];
        if (!pkt.data.empty())
        {
            const uint32_t previousPath = mohgeom::t_gifPathId;
            mohgeom::t_gifPathId =
                static_cast<uint32_t>(pkt.pathId);
            m_processFn(pkt.data.data(), static_cast<uint32_t>(pkt.data.size()));
            mohgeom::t_gifPathId = previousPath;
        }
    }
    m_queue.clear();
}

uint8_t GifArbiter::pathPriority(GifPathId id)
{
    return static_cast<uint8_t>(id);
}
