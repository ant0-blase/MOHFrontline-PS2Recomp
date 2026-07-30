# MOH: Frontline (NTSC-U, SLUS_203.68 + MOH2RDVD.ELF) — Practical Subsystem Map

> Reverse-engineering aid for the PS2Recomp port. **No original EA source names are claimed** — labels are
> functional inferences from SDK stub names, file/path strings, ELF layout, and generated function addresses.
> Stripped retail binaries → most game functions are `FUN_/sub_xxxxxxxx`. Deeper naming needs Ghidra xref work
> (GUI/GhydraMCP) — not yet done. Generated under `recomp/`; ELFs under `extracted/`. Last updated 2026-06-24.

## 0. Load layout (from `readelf -l`)
- **Launcher `SLUS_203.68`**: 1 LOAD seg @ **0x01C00000**, entry **0x01C00008**. Mostly Sony SDK/CRT + boot glue.
- **Core `MOH2RDVD.ELF`**: 1 LOAD seg @ **0x00104000**, entry **0x00104008** (`entry_0x104008`). The game. Has `.vutext/.vudata` (VU microcode).
- No address overlap → both can co-register in one runtime address space.

## 1. Subsystem map (confidence + evidence)

| Subsystem | Confidence | Key addresses / evidence |
|-----------|-----------|--------------------------|
| **boot / entry / crt0** | HIGH | launcher entry `0x01C00008`; `_InitSys@0x01C154F8`; `supplement_crt0@0x01C153F8`; core entry `0x00104008`. crt0 bootstrap in both ELFs. |
| **launcher → core ELF handoff** | HIGH | `sceSifLoadElf@0x01C14EF0` + string `cdrom0:\MOH2RDVD.ELF;1`; exec path `InitExecPS2@0x01C15608`,`ExecPS2@0x01C156C0`,`LoadExecPS2@0x01C15720`,`ExecOSD@0x01C15790`. |
| **SifLoadElf / SifLoadModule / IOP loading** | HIGH | `_sceSifLoadModule@0x01C14B60`,`_sceSifLoadModuleBuffer@0x01C14550`,`sceSifResetIop@0x01C150E8`,`sceSifRebootIop@0x01C15288`; IOP modules in `extracted/IOP/*.IRX` + `IOPRP243.IMG`. |
| **file I/O wrappers** | HIGH | `sceOpen@0x01C115E8`,`sceClose@0x01C11870`,`sceRead@0x01C11C28`,`sceWrite@0x01C11E98`,`sceLseek@0x01C119F0`,`sceIoctl@0x01C12158`,`sceDopen/Dread/Dclose`,`sceFsInit@0x01C11358`. |
| **PS2 path handling (`cdrom0:`,`cd:`,`data\`,`;1`)** | HIGH (strings) / MED (functions) | launcher uses `cdrom0:\DATA\…;1`; core uses `cd:data\…`. Runtime file layer must map BOTH → `extracted/`. The parsing functions themselves not yet identified. |
| **VIV/BIG archive loading** | MEDIUM (format) / LOW (functions) | strings `cd:data\shell\shell.viv`,`%d_%d_blog.viv`; format ref `moh_research/VIV archive/*.bms`. Loader functions need xref. |
| **shell / menu init** | LOW | strings `shell.viv`,`Shell1.asf`; functions unidentified (stripped) — requires xref of those strings. |
| **frontend / menu update loop** | LOW | inferred to exist; no concrete function yet. |
| **loading screen / loadbar** | LOW–MED | strings `data\loading\loadbar.ssh`,`load%d_%d.ssh`; `.ssh` = EA image (ref `ea_graphics_manager`). |
| **movie / video playback** | MED–HIGH | core `sceMpegInit@0x00212E08`,`sceMpegCreate@0x00212EA8`,`sceMpegGetPicture@0x002130E0`,`_sceMpegFlush@0x002138F0`; strings `data\Movies\*.mpc`. |
| **audio init / update** | LOW–MED | `LIBSD.IRX`,`SNDDRV.IRX`; core `.sounddata` section; `Shell1.asf`. No EE-side `sceSd*` stub seen yet (audio likely IOP-side). |
| **input / pad handling** | MEDIUM | `PADMAN.IRX`,`SIO2MAN.IRX`,`MTAPMAN.IRX`; core `sceMtapInit/sceMtapGetModVersion`. ⚠️ `scePad` ABSENT from runtime `ps2_call_list.h` — verify `scePadRead/scePadPortOpen` handlers exist before menu input works. |
| **memory card / save** | MEDIUM | `MCMAN.IRX`,`MCSERV.IRX`; runtime handler `sceMcInit` present. Save functions unidentified. |
| **rendering: GS / VIF / DMA / VU** | HIGH (SDK) / MED (VU) | GS: `sceGsResetGraph@0x01C186C8`/`@0x00212AC8`,`sceGsSyncV@0x01C18BC8`/`@0x00212CD0`,`sceGsPutDispEnv@0x01C19550`,`sceGsPutDrawEnv@0x01C199C8`. DMA: `sceDmaSend@0x01C17F80`. VU microcode in core `.vutext` (incomplete in runtime). |
| **memory allocator / heap** | HIGH | `malloc_extend_top@0x01C04D28`,`_malloc_r@0x01C04F80`,`_free_r@0x01C094E8`,`_calloc_r@0x01C0B4B8`,`_sbrk_r@0x01C05878`,`sbrk@0x01C0E660`. |
| **level loading** | LOW | no direct address yet; `moh_research` shows level archive formats (`.bif/.cdb/.scr/.aem`) — game-side loaders unidentified. |
| **error / assert / logging** | MEDIUM | `printf@0x01C19B90`,`scePrintf@0x01C0F5E8`,`kprintf@0x01C0F5B0`, Deci2 debug (`sceDeci2*@0x01C15E98+`). |

## 2. Notable game-side functions (core)
- `FUN_0020fdd8` (0x20fdd8), `sub_0020FD58` (0x20fd58): contain **PHMADH** (MMI2 0x11) in halfword loops with `pextlh/pmaxw` → fixed-point SIMD halfword math (geometry/DSP/decompress). 14 sites each.
- `sub_00211F88` (0x211f88): contains **PMADDH** (MMI2 0x10) with `pmthi/pmtlo/lq` → fixed-point MAC accumulating into HI/LO (likely matrix/vector or signal math). 3 sites.
- These 3 are the only functions with untranslated instructions (see §6).

## 3. Confidence legend
HIGH = named SDK stub at a known address, or a literal path string. MEDIUM = IOP module / section / partial evidence. LOW = inferred from data strings only; owning function not yet located (needs Ghidra xref).

## 4. Menu-milestone priority path (boot → menu)
1. **boot → core handoff** [HIGH]: launcher crt0/`_InitSys` → IOP reset + module load (`sceSifResetIop`,`sceSifLoadModule`) → `sceSifLoadElf(MOH2RDVD.ELF)` → jump to core entry `0x00104008`. Runtime must implement SIF/IOP load + map the core ELF into RDRAM @0x00104000.
2. **file I/O + path mapping** [HIGH]: `sceOpen/sceRead/sceLseek` + resolve `cdrom0:`/`cd:` → `extracted/`.
3. **shell.viv loading** [MED]: core opens `cd:data\shell\shell.viv`; VIV parse (validate against `moh_research` VIV templates).
4. **menu init/update** [LOW]: locate via xref of `shell.viv`/`Shell1.asf` strings (Ghidra) — label then prioritize.
5. **input** [MED]: pad init/read (confirm `scePad*` handlers; PADMAN/SIO2MAN).
6. **rendering enough for menu** [HIGH/MED]: GS init (`sceGsResetGraph`/`SetDefDBuff`/`PutDispEnv`/`PutDrawEnv`) + vsync (`sceGsSyncV`) + DMA/GIF for 2D shell + `.sfn` fonts.

## 5. What's temporary vs must-be-real
**Temporary / triage-acceptable:** address-bound stub routing; `runtime->lookupFunction` indirect fallbacks (OK if all targets registered); the 3 MMI2-omitted functions running with missing MAC results.
**Must eventually be real:** PHMADH + PMADDH (and any further MMI/VU ops); VU1 microcode (`.vutext`); all 122 (launcher) / 205 (core) stub handlers + syscalls; full GS rasterizer; VIF/GIF; SPU2/IOP audio; MPEG movie decode; memory card; complete file system + path layer.

## 6. Full-recomp compatibility blockers (toward 100%)
- **MMI2**: `PHMADH`(0x11)×28, `PMADDH`(0x10)×3 emitted as comments → silently skipped (wrong results). Proper fix = add codegen + `PS2_PHMADH`/`PS2_PMADDH` runtime macros (pattern matches existing `PS2_PMAXW`/`PS2_PEXTLH`/`PS2_PADDH`), then regenerate. Not a compile/boot blocker.
- **VU1 microcode** (`.vutext`): runtime VU1 noted incomplete upstream.
- **Indirect dispatch**: 3791/5088 core files use `lookupFunction` — every jump-table/function-pointer target must be registered or those calls fail at runtime.
- **Stub/syscall coverage**: launcher 122 + core 205 stubbed names must all resolve to runtime handlers; untracked libs (launcher 210 / core 394) are recompiled normally (full bodies) so they don't need handlers.
- **GS / audio / MPEG / memcard / VIF-GIF**: partial in runtime; required for full gameplay/AV.
