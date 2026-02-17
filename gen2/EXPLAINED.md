# EPC Gen2 Inventory Parameters — A Practical Guide

**Context**: This guide explains the Gen2 protocol parameters that control how a UHF RFID reader finds and communicates with multiple tags. It's written for the SparkFun Simultaneous RFID Tag Reader (M6E Nano / M7E Hecto) using paulvha's extended library, but the Gen2 concepts are universal to all compliant readers and tags.

---

## The Core Problem: Why Can't the Reader Just See All Tags at Once?

UHF RFID is a **shared radio channel**. The reader transmits a command, and every tag in range tries to respond on the same frequency at the same time. If two tags respond simultaneously, their signals collide and the reader can't decode either one. This is the same problem as a room full of people all answering a question at once — you hear noise, not individual answers.

The Gen2 standard solves this with three cooperating mechanisms:

| Mechanism   | What It Controls                                     | Analogy                                                |
| ----------- | ---------------------------------------------------- | ------------------------------------------------------ |
| **Q value** | How many time slots tags spread across               | How many "turns" are available for people to speak     |
| **Session** | How long a tag stays quiet after being read          | How long someone stays silent after answering          |
| **Target**  | Which tags the reader is currently asking to respond | Asking "anyone who hasn't spoken yet, raise your hand" |

These three parameters work together. Changing one without understanding the others leads to confusing behavior. The rest of this guide explains each one, then shows how they combine.

---

## Sessions: How Long a Tag Remembers Being Read

### The Inventoried Flag

Every Gen2-compliant tag maintains **four independent flags**, one per session: S0, S1, S2, and S3. Each flag is a single bit that can be in one of two states: **A** or **B**.

When a tag powers up (enters the RF field), all four flags start in state **A**. When the reader inventories a tag using a particular session, that session's flag flips from A to B. How long it stays in B before resetting back to A is **the defining characteristic of each session**:

```
Tag powers on → all flags initialize to A

    S0: ████ A ████     (resets to A almost immediately)
    S1: ████ A ████     (resets to A within 0.5 – 5 seconds)
    S2: ████ A ████     (stays B for 2 seconds minimum, often 60+)
    S3: ████ A ████     (stays B for 2 seconds minimum, often 60+)

Reader inventories tag using Session S1:

    S0: ████ A ████     (unchanged — reader didn't use S0)
    S1: ████ B ████ ──→ (will reset to A within 0.5 – 5 seconds)
    S2: ████ A ████     (unchanged)
    S3: ████ A ████     (unchanged)
```

The four sessions are independent. Reading a tag with Session S1 only affects its S1 flag. The S0, S2, and S3 flags remain untouched.

### Session Persistence Details

| Session | Flag Persistence After Read                             | Reset Behavior                                                                                                                     | Best For                                                                                                                                                                                        |
| ------- | ------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **S0**  | Effectively zero — resets to A almost instantly         | Immediate. The tag acts like it was never read.                                                                                    | Repeatedly reading the **same** tag (e.g., a single tag on a conveyor belt). Worst choice for multi-tag inventory because the same strong tag keeps responding.                                 |
| **S1**  | 500 ms to 5 seconds (tag-dependent)                     | Automatic. The tag quietly returns to A on its own. You cannot control the exact timing — it's set by the tag's chip manufacturer. | **Multi-tag inventory with moderate populations.** Tags stay quiet long enough for the reader to find others, but return to A fast enough that you can re-scan within seconds.                  |
| **S2**  | Minimum 2 seconds, typically 30–60+, no defined maximum | Automatic but slow. Tags stay in B for a long time. The exact duration is tag-dependent and can be hundreds of seconds.            | Large inventories where you need tags to stay quiet for a long time. Also used in **multi-reader** environments — Reader 1 uses S2, Reader 2 uses S3, and they don't interfere with each other. |
| **S3**  | Same as S2 — minimum 2 seconds, typically 30–60+        | Same as S2.                                                                                                                        | Exists so that **two readers** can independently inventory the same tag population without stepping on each other's session flags. Reader 1 on S2 and Reader 2 on S3 can work simultaneously.   |

### Why S2 and S3 Are Identical

A natural question: if S2 and S3 behave the same way, why do both exist? The answer is **multi-reader coordination**. Imagine two readers covering the same area:

```
Reader A (using Session S2)          Reader B (using Session S3)
        │                                    │
        ▼                                    ▼
   Tag's S2 flag: A → B                Tag's S3 flag: A → B
   (Reader A's work)                   (Reader B's work)

   These flags are independent — Reader A's inventory
   doesn't affect Reader B's session, and vice versa.
```

If both readers used S2, Reader A flipping a tag's S2 flag to B would cause Reader B to miss that tag (since it's also querying S2 and the tag is now in B). With separate sessions, they don't interfere.

For your single-reader exhibit, S2 and S3 are functionally interchangeable. S1 is the right choice because its automatic reset (0.5–5s) matches the exhibit's interaction cadence — scan, display results, user removes pucks, scan again.

### Important: You Don't Control the Persistence Duration

The Gen2 standard defines ranges, not exact values. A tag that claims S1 compliance might reset in 600ms or in 4.5 seconds — it's a chip design choice. You cannot send a command to make S1 last exactly 2 seconds. This means:

- Your scan window (750ms) should comfortably fit within S1's minimum persistence (500ms). It does.
- If you need a second scan immediately after, you might need to wait a moment for S1 flags to decay. With a 3-second LED display between scans, this is not a problem.

---

## Target: Which Tags Should Respond

The **Target** parameter tells the reader which inventory state to query. It answers the question: "Should I ask for tags in state A, state B, or both?"

### Single Target Modes

**Target A**: Only tags whose flag (for the selected session) is currently **A** will respond. Tags in state B stay silent.

**Target B**: Only tags in state **B** will respond. Tags in state A stay silent.

### Dual Target Modes

**Target AB**: The reader first inventories all A tags (flipping each to B). When no more A tags respond, it automatically switches to targeting B (flipping them back to A). Then it switches back to A, and the cycle repeats.

**Target BA**: Same as AB but starts with B first, then A.

### Visualizing Target Behavior

```
Target A only (Session S1):

  Round 1: [Tag1=A] [Tag2=A] [Tag3=A]  →  Reader queries A
           Tag1 responds → A→B
           Tag2 responds → A→B
           Tag3 responds → A→B

  Round 2: [Tag1=B] [Tag2=B] [Tag3=B]  →  Reader queries A
           No tags respond (all in B)

  Round 3: (silence... waiting for S1 flags to decay)

  Round N: [Tag1=A] [Tag2=B] [Tag3=A]  →  Some flags have reset
           Tag1 responds, Tag3 responds

  Problem: Dead time between complete sweeps while waiting for flags to reset.
```

```
Target AB (Session S1):

  Round 1: [Tag1=A] [Tag2=A] [Tag3=A]  →  Reader queries A
           Tag1 responds → A→B
           Tag2 responds → A→B
           Tag3 responds → A→B

  Round 2: [Tag1=B] [Tag2=B] [Tag3=B]  →  Reader queries A → no response
           Reader auto-switches to query B
           Tag1 responds → B→A
           Tag2 responds → B→A
           Tag3 responds → B→A

  Round 3: [Tag1=A] [Tag2=A] [Tag3=A]  →  Reader queries A again
           (cycle repeats with no dead time)

  Result: Continuous back-and-forth sweeps with no waiting.
```

### Target + Session Interaction Summary

Not all combinations make practical sense. Here's a reference matrix:

| Session   | Target A                                                                                                                                                  | Target B                                                                                                  | Target AB / BA                                                                                                                               |
| --------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| **S0**    | Reads same tags repeatedly (flags reset instantly, all tags always in A)                                                                                  | Useless — tags are never in B long enough to query                                                        | Same as Target A — flags reset so fast that the B phase finds nothing                                                                        |
| **S1**    | Good for finding diverse tags. Already-read tags stay in B for up to 5s, so the reader finds others. Periodic dead time while waiting for flags to reset. | Useless at startup — all tags begin in A. Only useful after a Target A pass has flipped some tags to B.   | **Optimal for continuous multi-tag inventory.** A phase finds all tags, B phase reads them again, continuous cycling with no dead time.      |
| **S2/S3** | Reads each tag **once** then silence. Tags stay in B for minutes. Good for large one-shot inventories.                                                    | Reads tags that a previous A pass already inventoried. Used to "re-awaken" tags after a complete A sweep. | Continuous cycling, but with very long flag persistence. Tags stay detected for a long time. Can feel sluggish for interactive applications. |

---

## Q Value: How Many Time Slots Per Inventory Round

### The Slotted ALOHA Anti-Collision Mechanism

When the reader starts an inventory round, it announces a **Q** value. Each tag in the field generates a random number between 0 and 2^Q − 1 and loads it into an internal counter. The protocol then proceeds slot by slot:

```
Q = 2  →  2² = 4 slots

Slot 0: Tags with counter=0 respond
        Reader sends QueryRep → all tags decrement counter
Slot 1: Tags with counter=0 respond (originally had counter=1)
        Reader sends QueryRep → decrement again
Slot 2: Tags with counter=0 respond
        Reader sends QueryRep
Slot 3: Tags with counter=0 respond
        Round complete.
```

If two tags pick the same slot, they collide and neither is successfully read in that slot. They'll be found in a subsequent round (or a later slot, depending on reader implementation).

### Choosing Q for a Known Tag Population

The ideal Q value gives enough slots that collisions are unlikely, but not so many that most slots are empty (wasting time):

| Q     | Slots (2^Q) | Good For          | Empty Slot Overhead                         |
| ----- | ----------- | ----------------- | ------------------------------------------- |
| 0     | 1           | Single tag only   | None, but guaranteed collision with 2+ tags |
| 1     | 2           | 1–2 tags          | Low                                         |
| **2** | **4**       | **2–4 tags**      | **Moderate — good balance for 3 pucks**     |
| 3     | 8           | 4–8 tags          | Higher — many empty slots with only 3 tags  |
| 4     | 16          | 8–16 tags         | Significant empty slot time                 |
| 7     | 128         | Large populations | Very high if population is small            |
| 15    | 32768       | Maximum           | Extreme — each round takes a long time      |

### Collision Probability with 3 Tags

With Q=2 (4 slots) and 3 tags, the probability that all three pick unique slots:

```
P(no collision) = 4/4 × 3/4 × 2/4 = 24/64 = 37.5%
```

So roughly 1 in 3 rounds will be collision-free and read all tags in a single pass. The remaining rounds will have at least one collision — but the colliding tag simply gets found in the next round, where (thanks to Session S1) the already-read tags are quiet.

With Q=3 (8 slots), the collision-free probability jumps to ~66%, but each round takes twice as long because of empty slots. For 3 tags, Q=2 with dynamic adjustment is the sweet spot.

### Static Q vs. Dynamic Q

**Static Q**: The Q value is fixed. You set it once and it never changes. Good when you know exactly how many tags are in the field and the count doesn't change.

**Dynamic Q**: The reader starts at your initial Q value and adjusts it automatically based on what it observes during inventory:

- Many collisions detected → Q is increased (more slots, fewer collisions)
- Many empty slots detected → Q is decreased (fewer wasted slots)

Dynamic Q is generally preferred because it adapts to the actual conditions. Even if you know you have 3 pucks, Dynamic Q handles edge cases: a puck might not respond on one round due to orientation, or a stray tag might enter the field temporarily.

```
Dynamic Q behavior example (starting at Q=2):

Round 1: Q=2, 4 slots, 2 collisions detected  → increase Q to 3
Round 2: Q=3, 8 slots, 0 collisions, 4 empty  → decrease Q to 2
Round 3: Q=2, 4 slots, 0 collisions, 1 empty  → keep Q at 2
(stabilizes around the optimal value)
```

---

## RF Mode: The Physical Layer (M7E Only)

While Sessions, Targets, and Q control the **protocol logic** of inventory, RF Mode controls the **physical radio signal** — how bits are actually encoded in the air. This affects range, speed, and noise immunity.

Three physical-layer parameters combine into an RF Mode:

### Backscatter Link Frequency (BLF)

BLF is the frequency at which the tag modulates its reflected signal back to the reader. Higher BLF means faster data transfer but requires stronger signal (shorter range):

| BLF     | Data Rate     | Range    | Noise Sensitivity |
| ------- | ------------- | -------- | ----------------- |
| 160 kHz | Slowest       | Longest  | Most immune       |
| 250 kHz | Moderate      | Moderate | Moderate          |
| 320 kHz | Moderate-fast | Moderate | Moderate          |
| 640 kHz | Fastest       | Shortest | Most sensitive    |

### Tag Encoding (Miller Modulation)

This determines how the tag encodes each bit in its response. Higher Miller values repeat each bit more times, trading speed for reliability:

| Encoding       | Cycles Per Bit | Speed    | Reliability | Typical Use                         |
| -------------- | -------------- | -------- | ----------- | ----------------------------------- |
| **FM0**        | 1              | Fastest  | Lowest      | Clean RF environments, short range  |
| **Miller M=2** | 2              | Fast     | Good        | General purpose                     |
| **Miller M=4** | 4              | Moderate | Very good   | Noisy environments, moderate range  |
| **Miller M=8** | 8              | Slowest  | Excellent   | Very noisy environments, long range |

### Tari (Type A Reference Interval)

Tari defines the duration of a "0" pulse from the reader to the tag. Shorter Tari means faster reader-to-tag communication but requires cleaner RF conditions:

| Tari    | Reader→Tag Speed | Robustness   |
| ------- | ---------------- | ------------ |
| 6.25 µs | Fastest          | Least robust |
| 7.5 µs  | Fast             | Moderate     |
| 15 µs   | Moderate         | Good         |
| 20 µs   | Moderate-slow    | Very good    |
| 25 µs   | Slowest          | Most robust  |

### Pre-Defined RF Modes on M7E

The M7E doesn't let you set BLF, encoding, and Tari independently. Instead, it offers pre-configured profiles:

| RF Mode Enum                  | BLF     | Encoding | Tari   | Character                           |
| ----------------------------- | ------- | -------- | ------ | ----------------------------------- |
| `TMR_GEN2_RFMODE_160_M8_20`   | 160 kHz | M8       | 20 µs  | Maximum range & robustness, slowest |
| `TMR_GEN2_RFMODE_250_M4_20`   | 250 kHz | M4       | 20 µs  | **Good all-rounder for exhibits**   |
| `TMR_GEN2_RFMODE_320_M2_15`   | 320 kHz | M2       | 15 µs  | Faster, moderate robustness         |
| `TMR_GEN2_RFMODE_320_M2_20`   | 320 kHz | M2       | 20 µs  | Slightly more robust than above     |
| `TMR_GEN2_RFMODE_320_M4_20`   | 320 kHz | M4       | 20 µs  | Good robustness, moderate speed     |
| `TMR_GEN2_RFMODE_640_FM0_7_5` | 640 kHz | FM0      | 7.5 µs | Fastest possible, least robust      |
| `TMR_GEN2_RFMODE_640_M2_7_5`  | 640 kHz | M2       | 7.5 µs | Fast with some robustness           |
| `TMR_GEN2_RFMODE_640_M4_7_5`  | 640 kHz | M4       | 7.5 µs | Fast with good robustness           |

For an exhibit environment with fluorescent lighting, nearby electronics, and tags at close range inside a beaker, `TMR_GEN2_RFMODE_250_M4_20` is a solid choice — Miller M=4 provides good noise immunity, and the moderate BLF/Tari don't waste time at short range.

### M6E vs. M7E Differences

| Parameter    | M6E Nano                              | M7E Hecto                             |
| ------------ | ------------------------------------- | ------------------------------------- |
| BLF          | Fixed at 250 kHz                      | Configurable via RF Mode              |
| Tari         | Fixed at 25 µs                        | Configurable via RF Mode              |
| Tag Encoding | Independently settable (FM0/M2/M4/M8) | Only via pre-defined RF Mode profiles |
| RF Mode      | Not applicable                        | Use `setGen2RFmode()`                 |
| Session      | Configurable                          | Configurable                          |
| Target       | Configurable                          | Configurable                          |
| Q            | Configurable                          | Configurable                          |

---

## How It All Comes Together: A Complete Inventory Cycle

Here's a step-by-step walkthrough of what happens during a single scan cycle with the parameters configured for the Beaker & Pucks exhibit:

**Configuration**: Session S1, Target AB, Dynamic Q (initial=2), RF Mode 250/M4/20

```
1. Reader sends QUERY command:
   "I want tags whose S1 flag = A, using 4 slots (Q=2)"

2. All 3 pucks have S1=A (fresh scan), so all participate:

   Yellow picks slot 2
   Blue picks slot 0
   Green picks slot 3

3. Slot 0: Blue responds → reader ACKs → Blue's S1 flips A→B
   Slot 1: (empty)
   Slot 2: Yellow responds → reader ACKs → Yellow's S1 flips A→B
   Slot 3: Green responds → reader ACKs → Green's S1 flips A→B

4. Round complete. All 3 tags found. All 3 S1 flags now = B.
   Reader tries another A-target round:

5. QUERY (S1=A): No tags respond (all in B).
   Reader detects empty round → switches target to B.

6. QUERY (S1=B): All 3 pucks respond (all in B).
   Yellow → B→A, Blue → B→A, Green → B→A.

7. All flags back to A. Reader switches target back to A.
   Cycle repeats from step 1.

Throughout this process, your check() loop receives tag records:
  → Blue found (step 3)
  → Yellow found (step 3)
  → Green found (step 3)
  → Blue found again (step 6)
  → Yellow found again (step 6)
  → Green found again (step 6)
  → ... repeating every ~50-100ms
```

Your application code deduplicates these into a set of 3 unique EPCs, matches them against known puck EPCs, and lights the corresponding LEDs. The entire process — from `startReading()` to having all 3 pucks identified — typically takes well under 200ms.

---

## Quick Reference: Library Functions

```cpp
// Set the inventory session (which tag flag to use)
bool setGen2Session(TMR_GEN2_Session session);
// TMR_GEN2_SESSION_S0, _S1, _S2, _S3

// Set which flag state to query
bool setGen2Target(TMR_GEN2_Target target);
// TMR_GEN2_TARGET_A, _B, _AB, _BA

// Set the anti-collision Q algorithm
bool setGen2Q(TMR_SR_GEN2_QType Q_state, uint8_t init_value, bool set_init);
// Q_state:   TMR_SR_GEN2_Q_DYNAMIC or TMR_SR_GEN2_Q_STATIC
// init_value: 0–10 (starting Q, only applied if set_init=true)
// set_init:   MUST be true to actually send the initial Q value

// Set RF mode profile (M7E only — returns false on M6E)
bool setGen2RFmode(TMR_GEN2_RFMode mode);
// TMR_GEN2_RFMODE_250_M4_20, etc.

// Set tag encoding directly (M6E only — returns false on M7E)
bool setGen2Encoding(TMR_GEN2_TagEncoding enc);
// TMR_GEN2_FM0, TMR_GEN2_MILLER_M_2, _M_4, _M_8
```

All functions return `true` on success. They send opcode `0x9B` (SET_PROTOCOL_PARAM) to the module and verify the response status word is `0x0000`. Parameters persist until changed or power-cycled — set them once in `setup()`, before calling `startReading()`.
