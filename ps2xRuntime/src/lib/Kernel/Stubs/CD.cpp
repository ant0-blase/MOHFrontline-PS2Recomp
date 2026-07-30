#include "Common.h"
#include "CD.h"
#include "MPEG.h"
#include <cstring>

namespace
{
    struct CachedCdImageReader
    {
        std::filesystem::path path;
        std::ifstream file;
        uint64_t sizeBytes = 0;
        bool valid = false;
    };

    CachedCdImageReader g_cdImageReader;
    std::mutex g_cdImageReaderMutex;
    bool g_mohCoreEofReached = false;

    bool shouldTraceMohCdAssetPath(const std::string &path)
    {
        const std::string lower = toLowerAscii(normalizePathSeparators(path));
        return lower.find("start.bmp") != std::string::npos ||
               lower.find("victory.bmp") != std::string::npos ||
               lower.find("defeat.bmp") != std::string::npos ||
               lower.find("return.bmp") != std::string::npos ||
               lower.find("shell.viv") != std::string::npos ||
               lower.find("shell1.asf") != std::string::npos ||
               lower.find("data/movies/") != std::string::npos ||
               lower.find(".mpc") != std::string::npos ||
               (lower.find("mem") != std::string::npos && lower.find("card") != std::string::npos);
    }

    std::string findMohCdAssetPathForLbn(uint32_t lbn)
    {
        for (const auto &[key, entry] : g_cdFilesByKey)
        {
            const uint32_t endLbn = entry.baseLbn + entry.sectors;
            if (lbn >= entry.baseLbn && lbn < endLbn && shouldTraceMohCdAssetPath(key))
            {
                return key;
            }
        }

        return {};
    }

    void logMohCdAssetResolvedOnce(const char *tag, const std::string &path, const CdFileEntry &entry,
                                   const R5900Context *ctx, uint32_t dst = 0)
    {
        if (!shouldTraceMohCdAssetPath(path))
        {
            return;
        }

        static std::unordered_set<std::string> logged;
        const std::string key = std::string(tag) + ":" + cdPathKey(path) + ":" + std::to_string(entry.baseLbn);
        if (!logged.insert(key).second)
        {
            return;
        }

        std::cerr << "[sceCd:MOH-asset] " << tag
                  << " path=\"" << sanitizeForLog(path) << "\""
                  << " lbn=0x" << std::hex << entry.baseLbn
                  << " size=0x" << entry.sizeBytes
                  << " sectors=0x" << entry.sectors;
        if (dst != 0u)
        {
            std::cerr << " dst=0x" << dst;
        }
        if (ctx)
        {
            std::cerr << " pc=0x" << ctx->pc
                      << " ra=0x" << getRegU32(ctx, 31);
        }
        std::cerr << std::dec << std::endl;
    }

    void logMohCdAssetReadOnce(const char *tag, uint32_t lbn, uint32_t sectors, uint32_t dst,
                               const R5900Context *ctx)
    {
        const std::string path = findMohCdAssetPathForLbn(lbn);
        if (path.empty())
        {
            return;
        }

        static std::unordered_set<std::string> logged;
        const std::string key = std::string(tag) + ":" + path + ":" + std::to_string(lbn);
        if (!logged.insert(key).second)
        {
            return;
        }

        std::cerr << "[sceCd:MOH-asset-read] " << tag
                  << " path=\"" << sanitizeForLog(path) << "\""
                  << " lbn=0x" << std::hex << lbn
                  << " sectors=0x" << sectors
                  << " dst=0x" << dst;
        if (ctx)
        {
            std::cerr << " pc=0x" << ctx->pc
                      << " ra=0x" << getRegU32(ctx, 31);
        }
        std::cerr << std::dec << std::endl;
    }

    bool ensureCdImageReaderOpen()
    {
        if (g_cdImageReader.valid && g_cdImageReader.file.is_open())
        {
            return true;
        }

        const std::filesystem::path imagePath = getCdImagePath();
        if (imagePath.empty())
        {
            g_cdImageReader = {};
            return false;
        }

        std::error_code ec;
        const uint64_t sizeBytes = static_cast<uint64_t>(std::filesystem::file_size(imagePath, ec));
        if (ec)
        {
            g_cdImageReader = {};
            return false;
        }

        g_cdImageReader.file.close();
        g_cdImageReader.file.clear();
        g_cdImageReader.file.open(imagePath, std::ios::binary);
        if (!g_cdImageReader.file.is_open())
        {
            g_cdImageReader = {};
            return false;
        }

        g_cdImageReader.path = imagePath;
        g_cdImageReader.sizeBytes = sizeBytes;
        g_cdImageReader.valid = true;
        return true;
    }

    bool tryGetCachedCdImageTotalSectors(uint64_t &totalSectorsOut)
    {
        std::lock_guard<std::mutex> lock(g_cdImageReaderMutex);
        if (!ensureCdImageReaderOpen())
        {
            return false;
        }

        totalSectorsOut = g_cdImageReader.sizeBytes / static_cast<uint64_t>(kCdSectorSize);
        return true;
    }

    bool readCachedCdImageRange(uint64_t offsetBytes, uint8_t *dst, size_t byteCount)
    {
        if (!dst)
        {
            g_lastCdError = -1;
            return false;
        }
        if (byteCount == 0)
        {
            g_lastCdError = 0;
            return true;
        }

        std::memset(dst, 0, byteCount);

        std::lock_guard<std::mutex> lock(g_cdImageReaderMutex);
        if (!ensureCdImageReaderOpen())
        {
            g_lastCdError = -1;
            return false;
        }

        g_cdImageReader.file.clear();
        g_cdImageReader.file.seekg(static_cast<std::streamoff>(offsetBytes), std::ios::beg);
        if (!g_cdImageReader.file.good())
        {
            g_lastCdError = -1;
            return false;
        }

        g_cdImageReader.file.read(reinterpret_cast<char *>(dst), static_cast<std::streamsize>(byteCount));
        g_lastCdError = 0;
        return true;
    }

    bool readCdSectorsCached(uint32_t lbn, uint32_t sectors, uint8_t *dst, size_t byteCount)
    {
        for (const auto &[key, entry] : g_cdFilesByKey)
        {
            const uint32_t endLbn = entry.baseLbn + entry.sectors;
            if (lbn < entry.baseLbn || lbn >= endLbn)
            {
                continue;
            }

            const uint64_t relativeLbn = static_cast<uint64_t>(lbn - entry.baseLbn);
            const uint64_t offset = relativeLbn * kCdSectorSize;
            return readHostRange(entry.hostPath, offset, dst, byteCount);
        }

        uint64_t totalSectors = 0;
        if (tryGetCachedCdImageTotalSectors(totalSectors))
        {
            const uint64_t start = static_cast<uint64_t>(lbn);
            const uint64_t end = start + static_cast<uint64_t>(sectors);
            if (start >= totalSectors || end > totalSectors)
            {
                g_lastCdError = -1;
                return false;
            }

            const uint64_t offset = start * kCdSectorSize;
            return readCachedCdImageRange(offset, dst, byteCount);
        }

        std::cerr << "sceCdRead unresolved LBN 0x" << std::hex << lbn
                  << " sectors=" << std::dec << sectors
                  << " (no mapped file and no configured CD image)" << std::endl;
        g_lastCdError = -1;
        return false;
    }

    bool isCdLbnResolvableCached(uint32_t lbn)
    {
        for (const auto &[key, entry] : g_cdFilesByKey)
        {
            const uint32_t endLbn = entry.baseLbn + entry.sectors;
            if (lbn >= entry.baseLbn && lbn < endLbn)
            {
                return true;
            }
        }

        uint64_t totalSectors = 0;
        if (tryGetCachedCdImageTotalSectors(totalSectors))
        {
            return static_cast<uint64_t>(lbn) < totalSectors;
        }

        return false;
    }
}

namespace ps2_stubs
{
    namespace
    {
        uint32_t g_cdStReadTraceCount = 0u;
    }


    CdDebugSnapshot getCdDebugSnapshot()
    {
        CdDebugSnapshot snapshot{};
        snapshot.initialized = g_cdInitialized;
        snapshot.lastError = g_lastCdError;
        snapshot.mode = g_cdMode;
        snapshot.streamingLbn = g_cdStreamingLbn;
        snapshot.streamingEndLbn = g_cdStreamingEndLbn;
        snapshot.nextPseudoLbn = g_nextPseudoLbn;
        snapshot.imageSizeBytes = g_cdImageSizeBytes;
        snapshot.imageSizeValid = g_cdImageSizeValid;
        snapshot.cdRoot = getCdRootPath();
        snapshot.cdImage = getCdImagePath();
        snapshot.imageSizePath = g_cdImageSizePath;
        snapshot.leafIndexRoot = g_cdLeafIndexRoot;
        snapshot.leafIndexBuilt = g_cdLeafIndexBuilt;
        snapshot.leafIndexCount = g_cdLeafIndex.size();
        snapshot.loosePathIndexCount = g_cdLoosePathIndex.size();

        snapshot.files.reserve(g_cdFilesByKey.size());
        for (const auto &[key, entry] : g_cdFilesByKey)
        {
            CdDebugFileEntry row{};
            row.key = key;
            row.hostPath = entry.hostPath;
            row.sizeBytes = entry.sizeBytes;
            row.baseLbn = entry.baseLbn;
            row.sectors = entry.sectors;
            snapshot.files.push_back(std::move(row));
        }
        std::sort(snapshot.files.begin(), snapshot.files.end(), [](const CdDebugFileEntry &a, const CdDebugFileEntry &b)
        {
            return a.baseLbn < b.baseLbn;
        });
        return snapshot;
    }

    void sceCdRead(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t a0 = getRegU32(ctx, 4); // usually lbn
        const uint32_t a1 = getRegU32(ctx, 5); // usually sector count
        const uint32_t a2 = getRegU32(ctx, 6); // usually destination buffer

        struct CdReadArgs
        {
            uint32_t lbn = 0;
            uint32_t sectors = 0;
            uint32_t buf = 0;
            const char *tag = "";
        };

        auto clampReadBytes = [](uint32_t sectors, uint32_t offset) -> size_t
        {
            const uint64_t requested = static_cast<uint64_t>(sectors) * static_cast<uint64_t>(kCdSectorSize);
            if (requested == 0)
            {
                return 0;
            }

            const uint64_t maxBytes = static_cast<uint64_t>(PS2_RAM_SIZE - offset);
            const uint64_t clamped = std::min<uint64_t>(requested, maxBytes);
            return static_cast<size_t>(clamped);
        };

        auto tryRead = [&](const CdReadArgs &args) -> bool
        {
            const uint32_t offset = args.buf & PS2_RAM_MASK;
            const size_t bytes = clampReadBytes(args.sectors, offset);
            if (bytes == 0)
            {
                return true;
            }

            return readCdSectorsCached(args.lbn, args.sectors, rdram + offset, bytes);
        };

        CdReadArgs selected{a0, a1, a2, "a0/a1/a2"};
        bool ok = tryRead(selected);

        static bool mohFileActive = false;
        static uint32_t mohFileStartLbn = 0;
        static uint32_t mohFileNextLbn = 0;
        static uint32_t mohFileEndLbn = 0;

        auto mohWriteNextLbn = [&](const CdReadArgs &args)
        {
            if (!mohFileActive ||
                getRegU32(ctx, 31) != 0x1f2994u ||
                args.lbn < mohFileStartLbn ||
                args.lbn >= mohFileEndLbn)
            {
                return;
            }

            const uint32_t rawNextLbn = args.lbn + args.sectors;
            if (rawNextLbn <= args.lbn)
            {
                return;
            }
            const uint32_t nextLbn = std::min(rawNextLbn, mohFileEndLbn);

            mohFileNextLbn = nextLbn;

            static uint32_t mohLastHeartbeat = 0;
            if (mohFileActive && (nextLbn >= mohLastHeartbeat + 0x40u || nextLbn + 4u >= mohFileEndLbn))
            {
                mohLastHeartbeat = nextLbn;
                std::cerr << "[sceCdRead:MOH-heartbeat]"
                          << " nextLbn=0x" << std::hex << nextLbn
                          << " end=0x" << mohFileEndLbn
                          << " current=0x" << args.lbn
                          << " ra=0x" << getRegU32(ctx, 31)
                          << std::dec << std::endl;
            }

            const uint32_t state = getRegU32(ctx, 7); // a3
            uint8_t *dst = getMemPtr(rdram, state + 0x10u);
            if (!dst)
            {
                return;
            }

            std::memcpy(dst, &nextLbn, sizeof(nextLbn));

            static uint32_t logCount = 0;
            if (logCount < 6)
            {
                std::cerr << "[sceCdRead:MOH-state-fix]"
                          << " wroteNextLbn=0x" << std::hex << nextLbn
                          << " currentLbn=0x" << args.lbn
                          << " state=0x" << state
                          << " ra=0x" << getRegU32(ctx, 31)
                          << std::dec << std::endl;
                ++logCount;
            }
        };

        if (ok)
        {
            mohWriteNextLbn(selected);
        }

        // MOH Frontline bounded sequential recovery:
        // Si le champ a0 est pollué par des données/code, mais qu'on est encore dans
        // le chargement borné de /MOH2RDVD.ELF, on reprend le prochain LBN attendu.
        if (!ok &&
            mohFileActive &&
            getRegU32(ctx, 31) == 0x1f2994u &&
            a1 > 0u &&
            a1 <= 0x1000u &&
            mohFileNextLbn >= mohFileStartLbn &&
            mohFileNextLbn < mohFileEndLbn &&
            !isCdLbnResolvableCached(a0))
        {
            CdReadArgs recovered{mohFileNextLbn, a1, a2, "moh-bounded-next-lbn"};
            if (tryRead(recovered))
            {
                selected = recovered;
                ok = true;
                mohWriteNextLbn(selected);

                static uint32_t boundedLogCount = 0;
                if (boundedLogCount < 6)
                {
                    std::cerr << "[sceCdRead:MOH-bounded-next]"
                              << " recoveredLbn=0x" << std::hex << recovered.lbn
                              << " originalA0=0x" << a0
                              << " next=0x" << mohFileNextLbn
                              << " end=0x" << mohFileEndLbn
                              << " a3=0x" << getRegU32(ctx, 7)
                              << std::dec << std::endl;
                    ++boundedLogCount;
                }
            }
        }

        if (!ok &&
            mohFileActive &&
            getRegU32(ctx, 31) == 0x1f2994u &&
            a1 > 0u &&
            a1 <= 0x1000u &&
            mohFileNextLbn >= mohFileEndLbn &&
            !isCdLbnResolvableCached(a0))
        {
            mohFileActive = false;
            g_mohCoreEofReached = true;   // completion: break the poll loop below + tell sceCdGetError
            g_lastCdError = 0;

            static uint32_t eofLogCount = 0;
            if (eofLogCount < 3)
            {
                std::cerr << "[sceCdRead:MOH-eof-stop] core fully loaded end=0x"
                          << std::hex << mohFileEndLbn << std::dec
                          << " -> return success to break loader poll" << std::endl;
                ++eofLogCount;
            }
        }

        // Post-core-load completion poll break (targeted, NOT a blind fallback):
        // FUN_001f2918 polls sceCdRead in a {WaitSema; read; beqz v0 -> retry} loop
        // that only exits when v0 != 0 (0x1f2994). Once the core ELF is fully streamed,
        // the loader keeps polling with stale/garbage entry pointers. Return success +
        // zeroed buffer so the loop terminates and the game advances to its next init
        // phase (func_232308 ...) instead of spinning forever.
        if (!ok &&
            g_mohCoreEofReached &&
            getRegU32(ctx, 31) == 0x1f2994u &&
            !isCdLbnResolvableCached(a0))
        {
            const uint32_t offset = a2 & PS2_RAM_MASK;
            const size_t bytes = clampReadBytes(a1, offset);
            if (bytes > 0)
            {
                std::memset(rdram + offset, 0, bytes);
            }
            g_lastCdError = 0;

            static uint32_t pollBreakLog = 0;
            if (pollBreakLog < 3)
            {
                std::cerr << "[sceCdRead:MOH-poll-break] post-eof completion, a0=0x"
                          << std::hex << a0 << std::dec << " -> v0=1 (loop exit)" << std::endl;
                ++pollBreakLog;
            }
            setReturnS32(ctx, 1);
            return;
        }

        // MOH Frontline: a0 peut être un pointeur vers l'entrée fichier /MOH2RDVD.ELF.
        if (!ok && getRegU32(ctx, 31) == 0x1f2994u)
        {
            auto readU32 = [&](uint32_t addr, uint32_t &out) -> bool
            {
                uint8_t *ptr = getMemPtr(rdram, addr);
                if (!ptr)
                {
                    return false;
                }

                std::memcpy(&out, ptr, sizeof(out));
                return true;
            };

            uint32_t entryLbn = 0;
            uint32_t entrySize = 0;
            uint32_t path0 = 0;
            uint32_t path1 = 0;
            uint32_t path2 = 0;
            uint32_t path3 = 0;

            const bool haveEntry =
                readU32(a0 + 0x04u, entryLbn) &&
                readU32(a0 + 0x08u, entrySize) &&
                readU32(a0 + 0x0cu, path0) &&
                readU32(a0 + 0x10u, path1) &&
                readU32(a0 + 0x14u, path2) &&
                readU32(a0 + 0x18u, path3);

            const bool isMohCoreElf =
                path0 == 0x484f4d2fu &&      // "/MOH"
                path1 == 0x56445232u &&      // "2RDV"
                path2 == 0x4c452e44u &&      // "D.EL"
                (path3 & 0xffu) == 0x46u;    // "F"

            if (haveEntry &&
                isMohCoreElf &&
                entrySize > 0u &&
                entryLbn >= 0x10u &&
                a1 > 0u &&
                a1 <= 0x1000u &&
                isCdLbnResolvableCached(entryLbn))
            {
                // a0 points at the already-indexed /MOH2RDVD.ELF file-table entry.
                // Recover the REAL core-ELF start LBN + size from that entry and stream
                // the whole core in (0x407..0x876).
                // NOTE: reverted the earlier DATA/SHELL redirect — it loaded only 1 sector
                // and left the core unloaded, so the loader spun forever on 'unresolved' reads.
                const uint32_t recoveredLbn = entryLbn;     // real core start (0x407)
                const uint32_t recoveredSize = entrySize;   // real core size  (0x237100)

                CdReadArgs recovered{recoveredLbn, a1, a2, "moh-core-elf-entry"};
                if (tryRead(recovered))
                {
                    selected = recovered;
                    ok = true;

                    const uint32_t sectorCount = (recoveredSize + 2047u) / 2048u;
                    mohFileActive = true;
                    mohFileStartLbn = recoveredLbn;
                    mohFileNextLbn = recoveredLbn + a1;
                    mohFileEndLbn = recoveredLbn + sectorCount;
                    g_mohCoreEofReached = false;

                    mohWriteNextLbn(selected);

                    std::cerr << "[sceCdRead:MOH-clean-recovered]"
                              << " lbn=0x" << std::hex << recoveredLbn
                              << " end=0x" << mohFileEndLbn
                              << " originalEntryLbn=0x" << entryLbn
                              << " originalA0=0x" << a0
                              << " a1=0x" << a1
                              << " a2=0x" << a2
                              << " a3=0x" << getRegU32(ctx, 7)
                              << std::dec << std::endl;
                }
            }
        }

if (!ok)
        {
            // Some game-side wrappers use a nonstandard register layout.
            // If primary decode does not resolve to a known LBN, try safe alternatives.
            constexpr uint32_t kMaxReasonableSectors = PS2_RAM_SIZE / kCdSectorSize;
            if (!isCdLbnResolvableCached(selected.lbn))
            {
                const std::array<CdReadArgs, 5> alternatives = {
                    CdReadArgs{a2, a1, a0, "a2/a1/a0"},
                    CdReadArgs{a0, a2, a1, "a0/a2/a1"},
                    CdReadArgs{a1, a0, a2, "a1/a0/a2"},
                    CdReadArgs{a1, a2, a0, "a1/a2/a0"},
                    CdReadArgs{a2, a0, a1, "a2/a0/a1"}};

                for (const CdReadArgs &candidate : alternatives)
                {
                    if (candidate.sectors > kMaxReasonableSectors)
                    {
                        continue;
                    }
                    if (!isCdLbnResolvableCached(candidate.lbn))
                    {
                        continue;
                    }

                    if (tryRead(candidate))
                    {
                        static uint32_t recoverLogCount = 0;
                        if (recoverLogCount < 16)
                        {
                            RUNTIME_LOG("[sceCdRead] recovered with alternate args " << candidate.tag
                                                                                     << " (pc=0x" << std::hex << ctx->pc
                                                                                     << " ra=0x" << getRegU32(ctx, 31)
                                                                                     << " a0=0x" << a0
                                                                                     << " a1=0x" << a1
                                                                                     << " a2=0x" << a2 << std::dec << ")" << std::endl);
                            ++recoverLogCount;
                        }
                        selected = candidate;
                        ok = true;
                        break;
                    }
                }
            }

            if (!ok)
            {
                const uint32_t offset = a2 & PS2_RAM_MASK;
                const size_t bytes = clampReadBytes(a1, offset);
                if (bytes > 0)
                {
                    std::memset(rdram + offset, 0, bytes);
                }

                static uint32_t unresolvedLogCount = 0;
                if (unresolvedLogCount < 32)
                {
                    std::cerr << "[sceCdRead] unresolved request pc=0x" << std::hex << ctx->pc
                              << " ra=0x" << getRegU32(ctx, 31)
                              << " a0=0x" << a0
                              << " a1=0x" << a1
                              << " a2=0x" << a2 << std::dec << std::endl;
                    ++unresolvedLogCount;
                }
            }
        }

        if (ok)
        {
            logMohCdAssetReadOnce("sceCdRead", selected.lbn, selected.sectors, selected.buf, ctx);
            g_cdStreamingLbn = selected.lbn + selected.sectors;
            setReturnS32(ctx, 1); // command accepted/success
            return;
        }

        setReturnS32(ctx, 0);
    }

    void sceCdSync(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0); // 0 = completed/not busy
    }

    void sceCdGetError(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        if (g_mohCoreEofReached && getRegU32(ctx, 31) == 0x1f29c4u)
        {
            setReturnS32(ctx, 0);
            return;
        }

        setReturnS32(ctx, g_lastCdError);
    }

    void sceCdRI(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        TODO_NAMED("sceCdRI", rdram, ctx, runtime);
    }

    void sceCdRM(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        TODO_NAMED("sceCdRM", rdram, ctx, runtime);
    }

    void sceCdApplyNCmd(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdBreak(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdCallback(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceCdChangeThreadPriority(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdDelayThread(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceCdDiskReady(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 2);
    }

    void sceCdGetDiskType(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        // SCECdPS2DVD
        setReturnS32(ctx, 0x14);
    }

    void sceCdGetReadPos(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnU32(ctx, g_cdStreamingLbn);
    }

    void sceCdGetToc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t tocAddr = getRegU32(ctx, 4);
        if (uint8_t *toc = getMemPtr(rdram, tocAddr))
        {
            std::memset(toc, 0, 1024);
        }
        setReturnS32(ctx, 1);
    }

    void sceCdInit(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_cdInitialized = true;
        g_lastCdError = 0;
        setReturnS32(ctx, 1);
    }

    void sceCdInitEeCB(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdIntToPos(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t lsn = getRegU32(ctx, 4);
        uint32_t posAddr = getRegU32(ctx, 5);
        uint8_t *pos = getMemPtr(rdram, posAddr);
        if (!pos)
        {
            setReturnS32(ctx, 0);
            return;
        }

        uint32_t adjusted = lsn + 150;
        const uint32_t minutes = adjusted / (60 * 75);
        adjusted %= (60 * 75);
        const uint32_t seconds = adjusted / 75;
        const uint32_t sectors = adjusted % 75;

        pos[0] = toBcd(minutes);
        pos[1] = toBcd(seconds);
        pos[2] = toBcd(sectors);
        pos[3] = 0;
        setReturnS32(ctx, 1);
    }

    void sceCdMmode(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_cdMode = getRegU32(ctx, 4);
        setReturnS32(ctx, 1);
    }

    void sceCdNcmdDiskReady(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 2);
    }

    void sceCdPause(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdPosToInt(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t posAddr = getRegU32(ctx, 4);
        const uint8_t *pos = getConstMemPtr(rdram, posAddr);
        if (!pos)
        {
            setReturnS32(ctx, -1);
            return;
        }

        const uint32_t minutes = fromBcd(pos[0]);
        const uint32_t seconds = fromBcd(pos[1]);
        const uint32_t sectors = fromBcd(pos[2]);
        const uint32_t absolute = (minutes * 60 * 75) + (seconds * 75) + sectors;
        const int32_t lsn = static_cast<int32_t>(absolute) - 150;
        setReturnS32(ctx, lsn);
    }

    void sceCdReadChain(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t chainAddr = getRegU32(ctx, 4);
        bool ok = true;

        for (int i = 0; i < 64; ++i)
        {
            uint32_t *entry = reinterpret_cast<uint32_t *>(getMemPtr(rdram, chainAddr + (i * 16)));
            if (!entry)
            {
                ok = false;
                break;
            }

            const uint32_t lbn = entry[0];
            const uint32_t sectors = entry[1];
            const uint32_t buf = entry[2];
            if (lbn == 0xFFFFFFFFu || sectors == 0)
            {
                break;
            }

            uint32_t offset = buf & PS2_RAM_MASK;
            size_t bytes = static_cast<size_t>(sectors) * kCdSectorSize;
            const size_t maxBytes = PS2_RAM_SIZE - offset;
            if (bytes > maxBytes)
            {
                bytes = maxBytes;
            }

            if (!readCdSectorsCached(lbn, sectors, rdram + offset, bytes))
            {
                ok = false;
                break;
            }

            g_cdStreamingLbn = lbn + sectors;
        }

        setReturnS32(ctx, ok ? 1 : 0);
    }

    void sceCdReadClock(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t clockAddr = getRegU32(ctx, 4);
        uint8_t *clockData = getMemPtr(rdram, clockAddr);
        if (!clockData)
        {
            setReturnS32(ctx, 0);
            return;
        }

        std::time_t now = std::time(nullptr);
        std::tm localTm{};
#ifdef _WIN32
        localtime_s(&localTm, &now);
#else
        localtime_r(&now, &localTm);
#endif

        // sceCdCLOCK format (BCD fields).
        clockData[0] = 0;
        clockData[1] = toBcd(static_cast<uint32_t>(localTm.tm_sec));
        clockData[2] = toBcd(static_cast<uint32_t>(localTm.tm_min));
        clockData[3] = toBcd(static_cast<uint32_t>(localTm.tm_hour));
        clockData[4] = 0;
        clockData[5] = toBcd(static_cast<uint32_t>(localTm.tm_mday));
        clockData[6] = toBcd(static_cast<uint32_t>(localTm.tm_mon + 1));
        clockData[7] = toBcd(static_cast<uint32_t>((localTm.tm_year + 1900) % 100));
        setReturnS32(ctx, 1);
    }

    void sceCdReadIOPm(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        sceCdRead(rdram, ctx, runtime);
    }

    void sceCdSearchFile(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t fileAddr = getRegU32(ctx, 4);
        uint32_t pathAddr = getRegU32(ctx, 5);
        const std::string path = readPs2CStringBounded(rdram, pathAddr, 260);
        const std::string normalizedPath = normalizeCdPathNoPrefix(path);
        static uint32_t traceCount = 0;
        const uint32_t callerRa = getRegU32(ctx, 31);
        const bool shouldTrace = (traceCount < 128u) || ((traceCount % 512u) == 0u);
        if (shouldTrace)
        {
            RUNTIME_LOG("[sceCdSearchFile] pc=0x" << std::hex << ctx->pc
                                                  << " ra=0x" << callerRa
                                                  << " file=0x" << fileAddr
                                                  << " pathAddr=0x" << pathAddr
                                                  << " path=\"" << sanitizeForLog(path) << "\""
                                                  << std::dec << std::endl);
        }
        ++traceCount;

        if (path.empty())
        {
            static uint32_t emptyPathCount = 0;
            if (emptyPathCount < 64 || (emptyPathCount % 512u) == 0u)
            {
                std::ostringstream preview;
                preview << std::hex;
                for (uint32_t i = 0; i < 16; ++i)
                {
                    const uint8_t byte = *getConstMemPtr(rdram, pathAddr + i);
                    preview << (i == 0 ? "" : " ") << static_cast<uint32_t>(byte);
                }
                std::cerr << "[sceCdSearchFile] empty path at 0x" << std::hex << pathAddr
                          << " preview=" << preview.str()
                          << " ra=0x" << callerRa << std::dec << std::endl;
            }
            ++emptyPathCount;
            g_lastCdError = -1;
            setReturnS32(ctx, 0);
            return;
        }

        if (normalizedPath.empty())
        {
            static uint32_t emptyNormalizedCount = 0;
            if (emptyNormalizedCount < 64u || (emptyNormalizedCount % 512u) == 0u)
            {
                std::cerr << "sceCdSearchFile failed: " << sanitizeForLog(path)
                          << " (normalized path is empty, root: " << getCdRootPath().string() << ")"
                          << std::endl;
            }
            ++emptyNormalizedCount;
            g_lastCdError = -1;
            setReturnS32(ctx, 0);
            return;
        }

        CdFileEntry entry;
        bool found = registerCdFile(path, entry);
        CdFileEntry resolvedEntry = entry;
        std::string resolvedPath;

        if (!found)
        {
            static std::string lastFailedPath;
            static uint32_t samePathFailCount = 0;
            if (path == lastFailedPath)
            {
                ++samePathFailCount;
            }
            else
            {
                lastFailedPath = path;
                samePathFailCount = 1;
            }

            if (samePathFailCount <= 16u || (samePathFailCount % 512u) == 0u)
            {
                std::cerr << "sceCdSearchFile failed: " << sanitizeForLog(path)
                          << " (root: " << getCdRootPath().string()
                          << ", repeat=" << samePathFailCount << ")" << std::endl;
            }
            setReturnS32(ctx, 0);
            return;
        }

        if (!writeCdSearchResult(rdram, fileAddr, path, resolvedEntry))
        {
            g_lastCdError = -1;
            setReturnS32(ctx, 0);
            return;
        }

        g_cdStreamingLbn = resolvedEntry.baseLbn;
        logMohCdAssetResolvedOnce("search", path, resolvedEntry, ctx, fileAddr);
        g_cdStreamingEndLbn = resolvedEntry.baseLbn + resolvedEntry.sectors;
        if (shouldTrace)
        {
            RUNTIME_LOG("[sceCdSearchFile:ok] path=\"" << sanitizeForLog(path)
                                                       << "\" lsn=0x" << std::hex << resolvedEntry.baseLbn
                                                       << " size=0x" << resolvedEntry.sizeBytes
                                                       << " sectors=0x" << resolvedEntry.sectors
                                                       << std::dec << std::endl);
        }
        setReturnS32(ctx, 1);
    }

    void sceCdSeek(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_cdStreamingLbn = getRegU32(ctx, 4);
        g_cdStreamingEndLbn = cdStreamingEndLbnForStart(g_cdStreamingLbn);
        setReturnS32(ctx, 1);
    }

    void sceCdStandby(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdStatus(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, g_cdInitialized ? 6 : 0);
    }

    void sceCdStInit(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdStop(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdStPause(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdStRead(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t requestedSectors = getRegU32(ctx, 4);
        uint32_t sectors = requestedSectors;
        uint32_t buf = getRegU32(ctx, 5);
        uint32_t errAddr = getRegU32(ctx, 7);

        uint32_t offset = buf & PS2_RAM_MASK;
        size_t requestedBytes = static_cast<size_t>(requestedSectors) * kCdSectorSize;
        const size_t maxBytes = PS2_RAM_SIZE - offset;
        if (requestedBytes > maxBytes)
        {
            requestedBytes = maxBytes;
        }

        bool hitStreamEnd = false;
        if (g_cdStreamingEndLbn != 0xFFFFFFFFu)
        {
            if (g_cdStreamingLbn >= g_cdStreamingEndLbn)
            {
                sectors = 0u;
                hitStreamEnd = true;
            }
            else
            {
                const uint32_t remaining = g_cdStreamingEndLbn - g_cdStreamingLbn;
                if (sectors > remaining)
                {
                    sectors = remaining;
                    hitStreamEnd = true;
                }
            }
        }

        size_t bytes = static_cast<size_t>(sectors) * kCdSectorSize;
        if (bytes > maxBytes)
        {
            bytes = maxBytes;
        }

        const uint32_t startLbn = g_cdStreamingLbn;
        const bool ok = (sectors > 0u) && readCdSectorsCached(startLbn, sectors, rdram + offset, bytes);
        if (ok)
        {
            logMohCdAssetReadOnce("sceCdStRead", startLbn, sectors, buf, ctx);
            g_cdStreamingLbn += sectors;
            if (requestedBytes > bytes)
            {
                std::memset(rdram + offset + bytes, 0, requestedBytes - bytes);
            }
            if (hitStreamEnd || g_cdStreamingLbn == g_cdStreamingEndLbn)
            {
                notifyMpegCdStreamEof();
            }
        }
        else
        {
            if (requestedBytes > 0u)
            {
                std::memset(rdram + offset, 0, requestedBytes);
            }
            notifyMpegCdStreamEof();
        }

        if (int32_t *err = reinterpret_cast<int32_t *>(getMemPtr(rdram, errAddr)); err)
        {
            *err = ok ? 0 : g_lastCdError;
        }

        if (g_cdStReadTraceCount < 32u)
        {
            std::cerr << "[sceCdStRead] sectors=" << requestedSectors
                      << " read=" << sectors
                      << " buf=0x" << std::hex << buf
                      << " lbn=0x" << startLbn
                      << " end=0x" << g_cdStreamingEndLbn
                      << std::dec << " ok=" << ok
                      << " bytes=" << bytes << std::endl;
            ++g_cdStReadTraceCount;
        }

        setReturnS32(ctx, ok ? static_cast<int32_t>(sectors) : 0);
    }

    void sceCdStream(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdStResume(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceCdStSeek(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_cdStreamingLbn = getRegU32(ctx, 4);
        logMohCdAssetReadOnce("sceCdStSeek", g_cdStreamingLbn, 0u, 0u, ctx);
        g_cdStreamingEndLbn = cdStreamingEndLbnForStart(g_cdStreamingLbn);
        setReturnS32(ctx, 1);
    }

    void sceCdStSeekF(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_cdStreamingLbn = getRegU32(ctx, 4);
        logMohCdAssetReadOnce("sceCdStSeekF", g_cdStreamingLbn, 0u, 0u, ctx);
        g_cdStreamingEndLbn = cdStreamingEndLbnForStart(g_cdStreamingLbn);
        setReturnS32(ctx, 1);
    }

    void sceCdStStart(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_cdStreamingLbn = getRegU32(ctx, 4);
        logMohCdAssetReadOnce("sceCdStStart", g_cdStreamingLbn, 0u, 0u, ctx);
        g_cdStreamingEndLbn = cdStreamingEndLbnForStart(g_cdStreamingLbn);
        g_cdStReadTraceCount = 0u;

        notifyMpegCdStreamStart();

        std::cerr << "[sceCdStStart] lbn=0x" << std::hex << g_cdStreamingLbn
                  << " endLbn=0x" << g_cdStreamingEndLbn << std::dec << std::endl;
        setReturnS32(ctx, 1);
    }

    void sceCdStStat(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceCdStStop(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        notifyMpegCdStreamEof();
        setReturnS32(ctx, 1);
    }

    void sceCdSyncS(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceCdTrayReq(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        uint32_t statusPtr = getRegU32(ctx, 5);
        if (uint32_t *status = reinterpret_cast<uint32_t *>(getMemPtr(rdram, statusPtr)); status)
        {
            *status = 0;
        }
        setReturnS32(ctx, 1);
    }
}
