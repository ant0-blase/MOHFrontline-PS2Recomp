# Technical notes

Findings from porting *Medal of Honor: Frontline* that are worth recording,
either because they cost significant time or because they generalise to other PS2
static-recompilation work.

---

## 1. `FTOI` must saturate

The VU `FTOI0/4/12/15` instructions convert float to fixed-point integer. The
obvious implementation is a C++ cast:

```cpp
int32_t iv = (int32_t)(value * 16.0f);   // wrong
```

On overflow this is undefined behaviour, and on x86 it lowers to `cvttss2si`,
which returns `0x80000000` — so a large positive coordinate comes out *negative*.
PS2 hardware saturates to `INT32_MIN`/`INT32_MAX` instead.

This single line was responsible for the 3D render being unrecoverably broken.
Every vertex whose transformed coordinate exceeded the representable range was
reflected to the opposite side of the screen, producing geometry that looked
plausibly "corrupted" rather than obviously clamped — which is why it survived a
long time as a suspected data problem.

```cpp
static inline int32_t vuFtoiSaturate(float f)
{
    if (!(f > -2147483648.0f)) return INT32_MIN;   // also catches NaN
    if (f >= 2147483648.0f)    return INT32_MAX;
    return (int32_t)f;
}
```

**Rule of thumb:** when every upstream check passes and the output is still
wrong, suspect the last conversion stage.

---

## 2. VU1 microcode banks are rewritten constantly

Frontline uploads VU1 microcode in `0x800`-byte banks and **rewrites each bank
roughly 86 times per run**. Each bank cycles among exactly three distinct
programs (verified by hashing every MPG payload at upload).

The consequence is that *any statistic keyed on a VU1 program-counter value is
meaningless on its own* — it silently aggregates three unrelated programs. Two
confident conclusions had to be withdrawn because of this: that a clipping
routine existed at a given address, and that it was dead code. Both were
artefacts of per-address counting.

To make address-based measurements well-formed:

- Count at instruction *execution*, never by address alone.
- Hash each `0x800` MPG payload at upload time and publish the resident variant
  index per bank.
- Tag every measurement with the `(bank, variant)` pair. Variant indices are
  per-bank: variant 1 of bank 1 and variant 1 of bank 2 are different programs.

---

## 3. Frontline does not clip geometry — it suppresses it

The VU1 microprogram contains no near-plane clipping and no vertex splitting.
Instead it evaluates the clip-flag register and marks primitives that touch the
frustum so the GS kicks them without drawing:

```
0x1260..0x12a8  FCOR  vi01,<mask> ; IBNE vi1,vi0 -> 0x1390   trivial reject, 5 planes
0x12c8          FMAND vi8         ; IBNE vi8,vi7 -> 0x1390
0x12d8          FCAND vi01,0x02fbef ; IBEQ vi1,vi0 -> 0x13a0  fully inside: skip
0x1390          IOR   vi10,vi10,vi11        vi11 = 0x7FFF + 1 = 0x8000
0x1410 / 0x1440 ISW.w vi10,0x103(vi13)      vertex w word: fog + ADC
```

`vi11` is built as `0x7FFF + 1 = 0x8000`, i.e. bit 15 of the vertex's `w` word,
which is **bit 111 of the packed qword — the GS ADC flag**. The five `FCOR`
tests are the standard trivial reject (all three vertices outside the same
plane); the `FCAND` at `0x12d8` masks every clip bit of the last three vertices
except bit 4 (the far plane), so *any* remaining clip bit sets ADC.

Two practical consequences:

- A packed vertex is not just X/Y/Z. In `XYZF2`, `FTOI4` on the fog value is a
  deliberate `<< 4`, placing it exactly on bits 4..11 of the `w` word, with ADC
  at bit 15. A site that looks like a coordinate conversion may be a fog
  conversion.
- Coordinates that overflow the GS 16-bit vertex field are *expected* in this
  design. Hardware wraps them too; they are harmless because ADC is set. Chasing
  the wrap rate is chasing a symptom.

---

## 4. The GS `Q` register latches on `RGBAQ`

In `PACKED` mode the `Q` value used for perspective-correct texturing is latched
when `RGBAQ` is written, not when the vertex is kicked. Reading the live `Q` at
vertex-kick time gives the *next* primitive's value, which shifts texture
coordinates by one primitive — subtle enough to look like a UV-precision problem.

---

## 5. Non-drawing kicks still advance the vertex window

A triangle-strip or fan keeps a sliding window of the last N vertices. Vertices
kicked with `XYZ3`/`XYZF3`, or with ADC set, do not draw — **but they still enter
the window**. Skipping the window update on those kicks desynchronises every
subsequent triangle in the strip. This showed up first as sheared font glyphs in
the front-end, long before it was understood as a general strip-state bug.

---

## 6. VIF1 `i`-bit as an upload pump

Front-end textures are uploaded by an interrupt-driven pump: the VIF1 `i` bit
raises INTC-5, whose handler continues the transfer. Dispatching at the exact
stream position where the `i` bit appears is semantically required — deferring it
to the end of the packet leaves the front-end untextured.

---

## Method

Two habits did most of the useful work:

**Environment-gated probes.** Every diagnostic is compiled in but off by default
and selected by an environment variable (see the table in the README). A
hypothesis can then be A/B tested against the same binary, which removes "did the
build actually change?" from the list of explanations.

**Verified runs.** `tools-moh-verified-run.sh` asserts that the linked binary
contains the marker string of the probe being measured, records the binary's
hash next to the log, and re-checks the hash after the run. This was added after
several measurements turned out to describe a binary that no longer existed.

Two recurring measurement mistakes, both of which produced confident wrong
answers:

- **Capped samples generalised to a population.** `if (k < N && cond)` where `k`
  counts *every* event only ever samples the first `N` events overall, not the
  first `N` matching ones. Count per condition instead.
- **Filtering on heap addresses.** Allocation addresses move between runs, so a
  probe bound to an address observed in a previous run lands on unrelated data.
  Bind probes to the event, not to the address.
