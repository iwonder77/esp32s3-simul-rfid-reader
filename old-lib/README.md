# SparkFun UHF RFID Reader Library — Comprehensive Technical Analysis

**Project**: Beaker and Molecule Pucks Exhibit — Simultaneous RFID Detection  
**Target Hardware**: SparkFun Simultaneous RFID Tag Reader (ThingMagic M7E Hecto) + ESP32-S3  
**Library Version**: SparkFun_UHF_RFID_Reader (GitHub: sparkfun/Simultaneous_RFID_Tag_Reader)

---

## 1. UART Communication Architecture

### 1.1 The Mercury API Serial Protocol

The M7E Hecto module speaks a binary serial protocol defined by ThingMagic's Mercury API. Every interaction is a request/response pair: the host (your ESP32) sends a **command frame**, and the module replies with a **response frame**. Both frames share the same structure:

```
┌────────┬────────┬────────┬───────────────────┬─────────┐
│ Header │ Length │ OpCode │   Data Payload    │  CRC-16 │
│ 1 byte │ 1 byte │ 1 byte │  0–250 bytes      │ 2 bytes │
│  0xFF  │  LEN   │   OP   │  [d0][d1]...[dN]  │ [Hi][Lo]│
└────────┴────────┴────────┴───────────────────┴─────────┘
```

| Field | Size | Description |
|-------|------|-------------|
| **Header** | 1 byte | Always `0xFF`. Acts as a sync byte. |
| **Length** | 1 byte | Number of bytes in the **Data Payload only** (does NOT include header, length, opcode, or CRC). |
| **OpCode** | 1 byte | Identifies the command/response type (e.g., `0x22` = read multiple tags). |
| **Data** | 0–250 bytes | Command-specific parameters (outgoing) or response data (incoming). |
| **CRC-16** | 2 bytes | ThingMagic-mutated CRC over bytes from Length through end of Data (inclusive). Big-endian. |

**Total frame size** = LEN + 5 (header + length + opcode + 2 CRC bytes), or equivalently LEN + 7 if you count from the perspective of the code which adds 7 to `msg[1]`.

**Key insight**: The `LEN` field counts only the data payload bytes. The library repeatedly uses `msg[1] + 7` to compute the total frame length, accounting for: header(1) + length(1) + opcode(1) + status(2) + CRC(2) = 7 overhead bytes. Note that in command frames, there are no status bytes, so the outgoing total is `msg[1] + 5`.

### 1.2 The Response Frame (Slightly Different)

Response frames include a 2-byte **status word** between the opcode and data:

```
Response layout:
[0]  [1]   [2]     [3]      [4]       [5]...[LEN+4]  [LEN+5] [LEN+6]
 FF   LEN   OP   STATUS_HI STATUS_LO   data...         CRC_HI  CRC_LO
```

A status of `0x0000` means success. Non-zero values indicate module-level errors (e.g., no tags found, invalid parameters, etc.). The library only partially interprets these — it mainly checks for `0x0000`.

### 1.3 The CRC Algorithm

The CRC is *not* standard CCITT CRC-16 despite looking similar. It's a nibble-at-a-time table-driven algorithm using a 16-entry lookup table:

```cpp
uint16_t RFID::calculateCRC(uint8_t *u8Buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc = ((crc << 4) | (u8Buf[i] >> 4))  ^ crctable[crc >> 12]; // High nibble
    crc = ((crc << 4) | (u8Buf[i] & 0x0F)) ^ crctable[crc >> 12]; // Low nibble
  }
  return crc;
}
```

It processes each byte in two 4-bit nibble steps. The CRC is seeded with `0xFFFF` and uses a 16-entry table (not the usual 256-entry table). This is specific to ThingMagic's protocol — you cannot substitute a generic CRC-16 library.

**CRC scope in commands**: Computed over `msg[1]` through `msg[1 + messageLength + 1]` — that is, starting at the Length byte and covering Length + OpCode + Data.  
**CRC scope in responses**: Same scope — Length through end of Data, excluding the header byte and the CRC itself.

### 1.4 The `msg[]` Buffer — Shared TX/RX

The library uses a single `uint8_t msg[255]` array as both the transmit and receive buffer. This is a critical architectural decision:

- **Before sending**: `sendMessage()` loads the opcode and data into `msg[1..N]`, then `sendCommand()` prepends the `0xFF` header and appends CRC.
- **After sending**: `sendCommand()` **overwrites** `msg[]` with the response from the module.
- **After return**: The caller inspects `msg[0]` for status (`ALL_GOOD`, `ERROR_COMMAND_RESPONSE_TIMEOUT`, etc.) and `msg[3..4]` for the module's status word.

**Implication**: You cannot "queue" commands or inspect a previous response after sending a new command. The buffer is always overwritten. During continuous reading, `check()` also writes into this same buffer.

### 1.5 Timeout Handling

The library uses two timeout checkpoints in `sendCommand()`:

1. **Timeout 1 — "No response from module"**: After sending the command, it polls `_nanoSerial->available()` in a loop. If no byte arrives within `timeOut` milliseconds (default 2000ms), it sets `msg[0] = ERROR_COMMAND_RESPONSE_TIMEOUT` and returns.

2. **Timeout 2 — "Incomplete response"**: Once the first byte arrives, it continues reading bytes. If the full frame (as indicated by the Length field at `msg[1]`) hasn't arrived within `timeOut` ms from the *start* of the wait (not from the first byte), it also times out.

Both timeouts share the same `startTime` baseline, which means a slow first byte eats into the time budget for receiving the rest of the frame.

```cpp
uint32_t startTime = millis();
// Timeout 1: wait for any byte
while (_nanoSerial->available() == false) {
  if (millis() - startTime > timeOut) { /* timeout */ }
  delay(1);
}
// Timeout 2: read full frame
while (spot < messageLength) {
  if (millis() - startTime > timeOut) { /* timeout */ }
  if (_nanoSerial->available()) { msg[spot] = _nanoSerial->read(); ... }
}
```

**Potential issue for your project**: During continuous reading, the module sends tag records asynchronously. The `check()` function handles this differently — it reads opportunistically from the serial buffer without blocking. But if you call a synchronous command (like `sendMessage()`) while continuous reading is active, it flushes the serial buffer first (`while (_nanoSerial->available()) _nanoSerial->read();`), discarding any pending tag records. This is noted in the code with a `//TODO this is a bad idea if we are constantly readings tags` comment.

---

## 2. Core Methods Deep Dive

### 2.1 `begin(Stream &serialPort, ThingMagic_Module_t moduleType)`

```cpp
void RFID::begin(Stream &serialPort, ThingMagic_Module_t moduleType) {
  _nanoSerial = &serialPort;
  _moduleType = moduleType;
}
```

This is minimal initialization. It stores a pointer to the serial stream and the module type. It does **not**:

- Configure the serial baud rate (you must call `Serial1.begin(115200)` yourself before calling `begin()`)
- Send any initialization commands to the module
- Verify the module is responding
- Set region, power, protocol, or antenna configuration

All of those are your responsibility in `setup()`. A typical initialization sequence looks like:

```cpp
nano.begin(Serial1, ThingMagic_M7E_HECTO);
nano.getVersion();        // Verify communication
nano.setRegion(REGION_NORTHAMERICA);
nano.setAntennaPort();
nano.setAntennaSearchList();
nano.setTagProtocol();    // Defaults to GEN2 (0x05)
nano.setReadPower(2700);  // 27.00 dBm max
```

The `moduleType` parameter matters for one thing: region mapping. The M6E Nano doesn't support `REGION_NORTHAMERICA` (0x01), only `REGION_NORTHAMERICA2` (0x0D), so `setRegion()` silently remaps it. The M7E Hecto supports the plain `REGION_NORTHAMERICA`.

### 2.2 `sendMessage(opcode, *data, size, timeOut, waitForResponse)`

This is the **command construction** layer. It packs the opcode and data into `msg[]`, then delegates to `sendCommand()`:

```cpp
void RFID::sendMessage(uint8_t opcode, uint8_t *data, uint8_t size,
                        uint16_t timeOut, boolean waitForResponse) {
  msg[1] = size;      // Length field = data payload size
  msg[2] = opcode;    // OpCode field
  for (uint8_t x = 0; x < size; x++)
    msg[3 + x] = data[x];  // Copy data payload starting at byte 3
  sendCommand(timeOut, waitForResponse);
}
```

Note that `msg[0]` (the header) is NOT set here — `sendCommand()` does that. The separation exists so that callers can either use the convenience of `sendMessage()` or manually pack `msg[]` and call `sendCommand()` directly (though no code in the library does the latter).

### 2.3 `sendCommand(timeOut, waitForResponse)`

This is the **transmission and reception** layer. It does everything:

**Transmission phase:**
1. Sets `msg[0] = 0xFF` (header)
2. Computes CRC over `msg[1..messageLength+2]` and appends it
3. Flushes the incoming serial buffer (discards stale data)
4. Sends `msg[0..messageLength+4]` byte-by-byte via `_nanoSerial->write()`

**Reception phase (if `waitForResponse == true`):**
5. Waits for the first byte (Timeout 1)
6. Reads bytes into `msg[]` until the full frame is received (Timeout 2)
7. Validates CRC of the response
8. Checks that the response opcode matches the sent opcode
9. Sets `msg[0]` to a status code:
   - `ALL_GOOD` (0) — valid response received
   - `ERROR_COMMAND_RESPONSE_TIMEOUT` (1) — timed out
   - `ERROR_CORRUPT_RESPONSE` (2) — CRC mismatch
   - `ERROR_WRONG_OPCODE_RESPONSE` (3) — response was for a different command

**If `waitForResponse == false`** (used by `setBaud()` and `stopReading()`):
- Calls `_nanoSerial->flush()` to wait for TX to complete, then returns immediately without reading anything.

**The opcode-matching check is important**: If the module sends an unsolicited message (like a tag record during continuous read) right when you send a command, the response you receive might not match your command. The library detects this and returns `ERROR_WRONG_OPCODE_RESPONSE`, but does not retry or attempt to find the correct response — the mismatched data is just discarded.

### 2.4 `parseResponse()`

This method interprets a **continuous-read tag record** that's already sitting in `msg[]` (placed there by `check()`). It does NOT send any commands — it only interprets data.

```
The full response record layout (for opcode 0x22):
msg[0]  = 0xFF (header)
msg[1]  = message length
msg[2]  = 0x22 (opcode)
msg[3,4]= status word
msg[5..11] = RFU (7 bytes)
msg[12] = RSSI
msg[13] = Antenna ID (upper nibble=TX, lower=RX)
msg[14..16] = Frequency in kHz (3 bytes, big-endian)
msg[17..20] = Timestamp in ms since last keep-alive (4 bytes)
msg[21,22] = Phase of signal (0–180)
msg[23] = Protocol ID
msg[24,25] = Number of bits of embedded tag data [M bytes]
msg[26..26+M-1] = Embedded data (if any)
msg[26+M] = RFU
msg[27+M, 28+M] = EPC bit length [N bits, including PC + CRC]
msg[29+M, 30+M] = Protocol Control (PC) bits
msg[31+M..31+M+N-5] = EPC bytes
... = EPC CRC, then Message CRC
```

It first validates the CRC, then branches on the message content:

| `msg[1]` value | Interpretation | Return code |
|---|---|---|
| `0x00` | Keep-alive (status-dependent) | `RESPONSE_IS_KEEPALIVE`, `RESPONSE_IS_TEMPTHROTTLE`, `RESPONSE_IS_HIGHRETURNLOSS`, or `RESPONSE_IS_UNKNOWN` |
| `0x08` | Unknown short message | `RESPONSE_IS_UNKNOWN` |
| `0x0A` | Temperature report | `RESPONSE_IS_TEMPERATURE` |
| Anything else | Full tag record | `RESPONSE_IS_TAGFOUND` |

After `parseResponse()` returns `RESPONSE_IS_TAGFOUND`, you use the accessor methods (`getTagEPCBytes()`, `getTagRSSI()`, `getTagFreq()`, etc.) to pull fields from the *still-loaded* `msg[]` buffer.

**Critical limitation**: `parseResponse()` only handles opcode `0x22` (READ_TAG_ID_MULTIPLE). If the opcode is anything else, it returns `ERROR_UNKNOWN_OPCODE`. This means it's exclusively for continuous-read mode.

### 2.5 `startReading()` and `stopReading()`

**`startReading()`** kicks off continuous inventory mode. It does two things:

1. Calls `disableReadFilter()` — sends opcode `0x9A` with data `[0x01, 0x0C, 0x00]` to disable the tag singulation filter (so it reads *any* tag, not just a specific one).

2. Sends a hardcoded 16-byte "config blob" via opcode `0x2F` (MULTI_PROTOCOL_TAG_OP):

```cpp
uint8_t configBlob[] = {
  0x00, 0x00,  // Timeout = 0 (continuous)
  0x01,        // Option = 1 (continuous reading mode)
  0x22,        // Sub-opcode = READ_TAG_ID_MULTIPLE
  0x00, 0x00,  // Search flags (only 0x0001 supported)
  0x05,        // Protocol = GEN2
  0x07, 0x22,  // ??? (from reverse engineering)
  0x10, 0x00,  // ??? (metadata flags?)
  0x1B, 0x03,  // ???
  0xE8,        // ???
  0x01, 0xFF   // ???
};
```

The comments in the code acknowledge that much of this blob was reverse-engineered from ThingMagic's Universal Reader Assistant logs, and not all bytes are understood. What we *can* determine:
- Timeout of 0 means "read forever"
- Option byte `0x01` means continuous mode
- Sub-opcode `0x22` means it's wrapping a multi-tag read inside the multi-protocol framework
- Protocol `0x05` is GEN2

**The bytes marked `???` almost certainly contain metadata flags, the Gen2 Q algorithm settings, session configuration, and target parameters** — all of which the library treats as opaque constants. This is the core of your problem: you can't configure anti-collision behavior because these parameters are hardcoded.

**`stopReading()`** sends opcode `0x2F` with a 3-byte blob:

```cpp
uint8_t configBlob[] = {0x00, 0x00, 0x02};
// 0x00, 0x00 = timeout (ignored)
// 0x02 = option: stop continuous reading
```

It sends this with `waitForResponse = false` because the module may still be mid-transmission of a tag record. After calling `stopReading()`, you should wait 1000–2000ms for the module to finish flushing its output before sending new commands.

### 2.6 `readTagEPC()`

Despite the name, this doesn't directly perform a tag read in the RF sense. It calls `readData()` with bank=1 (EPC memory), address=2 (the EPC starts at word 2, after CRC and PC):

```cpp
uint8_t RFID::readTagEPC(uint8_t *epc, uint8_t &epcLength, uint16_t timeOut) {
  uint8_t bank = 0x01;    // EPC memory bank
  uint8_t address = 0x02; // EPC starts at word 2
  return readData(bank, address, epc, epcLength, timeOut);
}
```

`readData()` sends opcode `0x28` (READ_TAG_DATA) — this is a **single-shot synchronous read**. The module performs one inventory round, selects one tag, reads the requested memory bank, and returns the data. This is fundamentally different from continuous reading.

Important: `readTagEPC()` reads the EPC from the tag's memory bank. It does NOT use the EPC that the module reports during an inventory round. The EPC returned in an inventory round (via `startReading()`/`parseResponse()`) comes from the tag's air-interface response, while `readTagEPC()` does a separate memory-read operation.

### 2.7 `check()`

This is the non-blocking polling function for continuous reading. It reads bytes from the serial buffer one at a time and assembles them into `msg[]`:

```cpp
bool RFID::check() {
  while (_nanoSerial->available()) {
    uint8_t incomingData = _nanoSerial->read();
    if (_head == 0 && incomingData != 0xFF) {
      // Ignore: waiting for header byte
    } else {
      msg[_head++] = incomingData;
      _head %= MAX_MSG_SIZE;
      if ((_head > 0) && (_head == msg[1] + 7)) {
        // Complete frame received!
        // Zero remainder of buffer, reset _head
        return true;
      }
    }
  }
  return false;
}
```

**How it works**:
1. It waits for a `0xFF` header byte, discarding anything else.
2. Once the header is found, it accumulates bytes at `_head` index.
3. When `_head` reaches `msg[1] + 7` (the expected total frame length), it declares the message complete.
4. It zeros out the rest of `msg[]`, resets `_head` to 0, and returns `true`.

**If `check()` returns `true`**: A complete message is in `msg[]`. You should then call `parseResponse()` to determine what type of message it is (tag found, keep-alive, temperature, etc.).

**If `check()` returns `false`**: Either no data was available, or a partial frame is still being assembled. Call again on the next loop iteration.

**Typical continuous read loop**:
```cpp
void loop() {
  if (nano.check()) {
    uint8_t responseType = nano.parseResponse();
    if (responseType == RESPONSE_IS_TAGFOUND) {
      uint8_t epcBytes = nano.getTagEPCBytes();
      // Read EPC from msg[31..31+epcBytes-1] (assuming no embedded data)
      // Process tag...
    }
  }
}
```

**The critical thing to understand**: `check()` gives you tags **one at a time**, as the module reports them. If 3 tags are in the field, you'll get 3 separate `RESPONSE_IS_TAGFOUND` returns over successive calls to `check()`. It's up to *your* application code to accumulate these into a set and decide "I've seen all 3 pucks." The library provides no aggregation, deduplication, or inventory-round-boundary detection.

---

## 3. Opcode Implementation Audit

### 3.1 Fully Implemented (Method exists, parameters configurable, response parsed)

| OpCode | Hex | Method(s) | Notes |
|--------|-----|-----------|-------|
| SET_BAUD_RATE | 0x06 | `setBaud()` | Sends without waiting for response |
| WRITE_TAG_DATA | 0x24 | `writeData()`, `writeTagEPC()`, `writeUserData()`, `writeKillPW()`, `writeAccessPW()` | Checks status word, returns SUCCESS/FAIL |
| READ_TAG_DATA | 0x28 | `readData()`, `readTagEPC()`, `readUserData()`, `readKillPW()`, `readAccessPW()`, `readTID()`, `readUID()` | Response correctly parsed with offset for option/metadata bytes |
| KILL_TAG | 0x26 | `killTag()` | Sends password, checks status |
| SET_READ_TX_POWER | 0x92 | `setReadPower()` | Clamps to 2700 max |
| GET_READ_TX_POWER | 0x62 | `getReadPower()` | Response left in msg[] |
| SET_WRITE_TX_POWER | 0x94 | `setWritePower()` | No clamping (unlike setReadPower) |
| GET_WRITE_TX_POWER | 0x64 | `getWritePower()` | Response left in msg[] |
| SET_REGION | 0x97 | `setRegion()` | Region remap for M6E backward compat |
| SET_ANTENNA_PORT | 0x91 | `setAntennaPort()`, `setAntennaSearchList()` | Hardcoded to port 1/1 |
| SET_TAG_PROTOCOL | 0x93 | `setTagProtocol()` | Defaults to GEN2 (0x05), parameter exposed |
| SET_USER_GPIO_OUTPUTS | 0x96 | `pinMode()`, `digitalWrite()` | Overloaded for both pin mode and state |
| GET_USER_GPIO_INPUTS | 0x66 | `digitalRead()` | Parses 3-byte-per-pin response format |
| SET_READER_OPTIONAL_PARAMS | 0x9A | `setReaderConfiguration()`, `enableReadFilter()`, `disableReadFilter()` | Key-value format, 2 options exposed |
| GET_READER_OPTIONAL_PARAMS | 0x6A | `getOptionalParameters()` | Sends but doesn't parse response |

### 3.2 Partially Implemented (Used, but hardcoded or limited)

| OpCode | Hex | Method(s) | Limitation |
|--------|-----|-----------|------------|
| MULTI_PROTOCOL_TAG_OP | 0x2F | `startReading()`, `stopReading()` | **Hardcoded config blob.** Reverse-engineered bytes. No way to set session, target, Q, or other inventory parameters. This is the most important opcode for your use case and the most limiting. |
| VERSION | 0x03 | `getVersion()` | Sends command but doesn't parse the version response — it just sits in `msg[]` for the caller to inspect manually. |
| READ_TAG_ID_MULTIPLE | 0x22 | Used only as a sub-opcode within the `0x2F` blob; `parseResponse()` handles incoming `0x22` records | Never sent directly as a standalone opcode. |

### 3.3 Defined but Unused

| OpCode | Hex | Defined In | Status |
|--------|-----|-----------|--------|
| **SET_PROTOCOL_PARAM** | **0x9B** | Header | **`setProtocolParameters()` declared in .h but body is empty/missing from .cpp!** This is the opcode you would need to set Gen2 session, target, Q, and other critical parameters. |
| **GET_PROTOCOL_PARAM** | **0x6B** | Header | **`getProtocolParameters()` declared in .h with a basic 2-byte data send, but no meaningful implementation.** |
| READ_TAG_ID_SINGLE | 0x21 | Header | Defined, never used. Single-tag inventory command. |
| WRITE_TAG_ID | 0x23 | Header | Defined, never used. Distinct from WRITE_TAG_DATA. |
| CLEAR_TAG_ID_BUFFER | 0x2A | Header | Defined, never used. Would clear the module's internal tag buffer. |
| GET_POWER_MODE | 0x68 | Header | Defined, never used. |

### 3.4 Missing Entirely (Not defined, would be useful)

| Functionality | Expected OpCode | Why You'd Need It |
|---|---|---|
| SET_POWER_MODE | ~0x98 | Sleep/low-power modes for the module |
| Tag select/filter by EPC | Part of 0x2F blob | Target specific tags for read/write operations |
| Lock tag memory | ~0x25 (TMR_SR_OPCODE_LOCK_TAG) | Protect tag data from overwrite |
| Get temperature | Uses 0x22 response type | Module temperature monitoring (responses exist in `parseResponse()` but no command to request it) |
| Gen2 custom commands | Via 0x2F | Vendor-specific tag commands |

### 3.5 The Critical Gap: 0x9B (SET_PROTOCOL_PARAM)

This deserves special emphasis. Looking at the header file:

```cpp
void setProtocolParameters(void);  // Declared...
```

But in the .cpp, there's no implementation with parameters. The method signature takes no arguments, which means even if a body existed, it couldn't set anything useful. The Mercury API uses opcode `0x9B` to configure Gen2-specific parameters including:

- **Session** (S0/S1/S2/S3)
- **Target** (A/B/AB)
- **Q static/dynamic and Q value**
- **Select actions**

This is the single biggest gap in the library for your use case.

---

## 4. Current Multi-Tag Behavior Analysis

### 4.1 What Happens When `startReading()` Is Called

The exact sequence on the wire:

**Step 1**: Disable read filter
```
TX: FF 03 9A 01 0C 00 [CRC]
         ^^          ^^ ^^ ^^
         |           |  |  └─ value: 0=disabled
         |           |  └─ key: 0x0C = read filter
         |           └─ format: key-value
         └─ opcode: SET_READER_OPTIONAL_PARAMS
```

**Step 2**: Start continuous multi-tag read
```
TX: FF 10 2F 00 00 01 22 00 00 05 07 22 10 00 1B 03 E8 01 FF [CRC]
         ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^
         |  └──┘  |  |  └──┘  |  └──────────────────────────┘
         |  timeout| sub-op  proto    unknown blob bytes
         |  = 0   option=1   =GEN2
         opcode   (continuous)
```

The module then enters continuous inventory mode and begins sending tag records asynchronously over UART.

### 4.2 How Default Inventory Rounds Work

When the module performs a Gen2 inventory round with default settings (which is what the hardcoded blob gives you), here's what happens at the RF level:

1. The reader sends a **Query** command, which starts an inventory round. This command includes a **Q** value (determines the number of time slots: 2^Q slots) and a **Session** and **Target** parameter.

2. Each tag in the field picks a random slot number between 0 and 2^Q - 1. Tags that pick slot 0 respond immediately. Others decrement their counter each time the reader sends a **QueryRep**.

3. When a tag responds, the reader sends an **ACK** to get the full EPC. After receiving the EPC, the tag flips its **inventoried flag** (from A→B or B→A) for the specified session.

4. The reader then sends **QueryRep** to move to the next slot. This continues until all 2^Q slots are exhausted.

5. After the round, the reader starts a new round. Tags that already flipped their flag won't respond again — **unless** the session's flag has persistence short enough that it resets before the next round.

With default settings (likely **Session S0**, **Target A**):
- S0 flags persist for only a few milliseconds to seconds.
- The reader does rapid inventory rounds.
- Tags flip A→B on first detection, but the flag resets almost immediately, so they're eligible to be read again.
- **Result**: The same tag gets reported over and over, many times per second. The "strongest" tag (closest, best-positioned) gets reported most frequently simply because it wins more anti-collision slots.

### 4.3 Why "Strongest Tag Wins" Behavior Occurs

It's not that weaker tags are *never* detected — they are. But several factors create a bias:

1. **Capture effect**: When two tags respond in the same slot (a collision), the reader may successfully decode the stronger signal and treat the weaker one as noise. This isn't unique to this library — it's fundamental to UHF RFID.

2. **Session S0 default**: With S0, tags' inventoried flags reset very quickly. This means in every inventory round, all tags compete again. The strongest tag has the highest probability of successful singulation in any given slot.

3. **No filtering/aggregation in the library**: The library reports every tag event individually. If you're looking at the output and seeing mostly one tag's EPC, it's because that tag is winning more rounds, not because the others can't be read at all.

4. **Q value may be too low**: If Q is set to 0 or 1 (meaning 1 or 2 slots per round), there aren't enough slots for 3 tags to avoid collisions. Tags that collide are missed in that round. The strongest tag is least likely to have its response destroyed by a collision.

### 4.4 What Would Need to Change for True Simultaneous Detection

For reliable 3-tag detection, you need to configure the Gen2 protocol parameters, which requires sending commands the library doesn't currently support. Specifically:

**Session S1 or S2** instead of S0:
- S1 flags persist for 500ms–5s (implementation-dependent). S2 persists even longer.
- Once a tag is inventoried and its flag flips A→B, it *stays* in B for a meaningful duration.
- Subsequent rounds targeting A won't see already-inventoried tags, so the reader naturally moves on to find the remaining tags.

**Target A→B with proper round management**:
- Round 1, Target=A: All 3 tags are in state A. Reader inventories them, each flips to B.
- Round 2, Target=A: No tags respond (all in B). Reader knows the round is complete.
- Optionally flip target to B for the next cycle, or wait for session persistence to reset flags.

**Q value of 2 or higher**:
- 2^2 = 4 slots. With 3 tags, probability of a collision-free round is reasonable.
- The module's dynamic Q algorithm should handle this, but if it's set to Q=0 (1 slot), collisions are guaranteed with multiple tags.

**These parameters are set via opcode `0x9B` (SET_PROTOCOL_PARAM)**, which the library declares but does not implement.

---

## 5. Library Limitations Summary

### 5.1 Features the M7E Hecto Supports but the Library Doesn't Expose

| Feature | Impact on Your Project |
|---|---|
| **Gen2 Session selection (S0/S1/S2/S3)** | **Critical.** Cannot configure tag persistence behavior. |
| **Gen2 Target selection (A/B/AB)** | **Critical.** Cannot control inventory round targeting. |
| **Q algorithm configuration** | **High.** Cannot optimize anti-collision for known tag population. |
| **Select command / tag filtering** | Medium. Cannot target specific tag populations. |
| **Tag buffer management** | Medium. Cannot use module's internal multi-tag buffer. |
| **Autonomous mode configuration** | Low. Module can operate independently without host polling. |
| **Detailed status/error codes** | Low. Non-zero status words are not decoded into meaningful messages. |

### 5.2 Hardcoded Values That Should Be Configurable

| Value | Location | What It Affects |
|---|---|---|
| `startReading()` config blob (16 bytes) | `startReading()` | Session, target, Q, metadata flags — all of it |
| Antenna port 1/1 | `setAntennaPort()` | Only single-antenna configs supported |
| GEN2 protocol only | `setTagProtocol()` default | Other protocols (ISO 18000-6B, etc.) listed but untested |
| Read filter key 0x0C | `enableReadFilter()`/`disableReadFilter()` | Only one configuration key exposed |
| Power clamp at 2700 | `setReadPower()` | Write power has no clamp — inconsistent |
| COMMAND_TIME_OUT = 2000ms | Global default | May be too short for some operations, too long for others |

### 5.3 Missing Error Handling

- **No retry logic**: If a command fails (timeout, CRC error, wrong opcode), the library returns an error code but never retries.
- **No serial buffer overflow protection**: The `msg[]` buffer is 255 bytes. If the module sends a corrupt length byte suggesting a huge frame, the read loop wraps via `spot %= MAX_MSG_SIZE` but could still overwrite valid data.
- **`check()` doesn't validate CRC**: It only checks for frame completeness (correct number of bytes). CRC is validated in `parseResponse()`, but if you call `check()` and then access `msg[]` directly without calling `parseResponse()`, you're reading unvalidated data.
- **`sendCommand()` flushes the RX buffer before sending**: This discards any pending tag records during continuous read. If you need to send a command mid-read, you lose data.
- **No handling of module resets**: If the module resets (brown-out, watchdog), the library has no mechanism to detect or recover from this.

### 5.4 Architectural Limitations

1. **Single shared buffer**: `msg[255]` is used for everything. You can't inspect a tag record while sending a command.

2. **Blocking synchronous commands**: `sendCommand()` blocks until a response is received or timeout occurs. During continuous reading, this creates a conflict — you can't send commands without interrupting the read stream.

3. **No callback or event system**: There's no way to register a handler for "tag found" events. You must poll `check()` in your main loop.

4. **No tag deduplication or accumulation**: The library reports each tag event individually. Building a "set of currently present tags" is entirely the caller's responsibility — including managing timeouts for tags that have left the field.

5. **`parseResponse()` only handles opcode 0x22**: If you extend the library to use new opcodes that generate async responses, `parseResponse()` won't know what to do with them.

---

## 6. EPC Gen2 Primer (As It Relates to This Library)

### 6.1 Sessions (S0, S1, S2, S3)

Every Gen2 tag maintains four independent **inventoried flags**, one per session. Each flag can be in state **A** or **B**. The sessions differ in how long the flag persists after being flipped:

| Session | Flag Persistence | Typical Use |
|---------|-----------------|-------------|
| **S0** | Nominally zero — resets almost immediately (spec says "indefinite" while powered, but in practice very short without continuous RF energy) | Fast repeated reads of the same tag. Good for single-tag scenarios. |
| **S1** | 500ms to 5s (implementation-dependent) | **Best for your use case.** Tags stay "inventoried" long enough to let the reader find the next tag, but reset in time for the next read cycle. |
| **S2** | 2s or more (implementation-dependent) | Multi-reader environments. Longer persistence. |
| **S3** | 2s or more (implementation-dependent) | Multi-reader environments. Longest persistence. |

**Why this matters for you**: With S0 (the likely default), when the reader inventories Tag A, Tag A's flag flips A→B but resets to A almost instantly. So in the next round, Tag A competes again alongside Tags B and C. Tag A might keep winning, starving the others. With S1, Tag A stays in B for several seconds after being read, removing it from contention and giving Tags B and C a fair chance.

### 6.2 Target (A, B, AB)

The **Target** parameter tells the reader which tags to inventory in a given round:

- **Target A**: Only tags whose flag (for the selected session) is in state A will respond.
- **Target B**: Only tags in state B will respond.
- **Target AB** (or "dual target"): The reader alternates — first inventories all A tags (flipping them to B), then inventories all B tags (flipping them to A), creating a ping-pong cycle.

**For your use case with S1**:
1. Set Session=S1, Target=A.
2. Round 1: All tags are in A. Reader inventories them all, each flips to B.
3. Round 2: Target is still A, but no tags are in A. Reader gets zero responses → you know you've seen all tags.
4. Wait for S1 persistence to expire (~2–5s), tags return to A. Repeat.

This gives you a clean "inventory round" boundary — when you stop seeing tags, you know you've found them all. This is exactly what you need for the button-press detection: press button → run one inventory cycle → collect all tag EPCs → check which combination is present.

### 6.3 Q Value and Slotted ALOHA

The **Q** parameter controls how many time slots exist in an inventory round: there are **2^Q** slots. Each tag picks a random slot. If two tags pick the same slot, they collide and neither is read in that slot.

| Q Value | Slots | Good For |
|---------|-------|----------|
| 0 | 1 | Single tag only |
| 1 | 2 | 1–2 tags |
| 2 | 4 | 2–4 tags |
| 3 | 8 | 4–8 tags |
| 4 | 16 | Larger populations |

For 3 tags, Q=2 gives 4 slots. The probability that all 3 tags pick unique slots is:

P = (4/4) × (3/4) × (2/4) = 37.5% per round

That means ~63% of rounds will have at least one collision, but the colliding tags will be found in subsequent rounds. With Q=3 (8 slots), the probability jumps to ~65% of all 3 being collision-free. The module's **dynamic Q** algorithm can adjust Q automatically based on observed collisions, but you need to ensure it's configured appropriately.

**For only 3 known tags, Q=2 with session S1 is very reliable**: even if a collision occurs in one round, the successfully-inventoried tags are suppressed in the next round, leaving fewer tags to collide.

### 6.4 Putting It All Together for Your Exhibit

The ideal configuration for detecting all 3 pucks:

```
Session:  S1 (flags persist long enough to suppress already-read tags)
Target:   A  (or AB for continuous ping-pong)
Q:        2  (4 slots is plenty for 3 tags)
```

Workflow on button press:
1. Start an inventory cycle (or use continuous reading with proper session/target).
2. Collect unique EPC values over a short window (e.g., 500ms–1s).
3. After the window, check: did we see exactly 3 specific EPCs?
4. If yes → trigger the appropriate animation.
5. If no → show "incomplete" or "wrong combination" feedback.

With S1 and a 500ms collection window, you should reliably see all 3 tags within the first 2–3 inventory rounds, which at UHF speeds takes well under 100ms.

---

## 7. Extension Points Identification

### 7.1 New Methods Needed

| Method Name | Purpose |
|---|---|
| `setGen2Session(uint8_t session)` | Set the Gen2 session (0=S0, 1=S1, 2=S2, 3=S3) via opcode 0x9B |
| `setGen2Target(uint8_t target)` | Set the target (0=A, 1=B, 2=AB, 3=BA) via opcode 0x9B |
| `setGen2Q(uint8_t q)` | Set the Q value (or configure dynamic Q) via opcode 0x9B |
| `getGen2Session()` | Read back current session via opcode 0x6B |
| `getGen2Target()` | Read back current target via opcode 0x6B |
| `getGen2Q()` | Read back current Q configuration via opcode 0x6B |
| `startReadingWithParams(session, target, q)` | A version of `startReading()` that constructs the config blob dynamically instead of using a hardcoded blob |

### 7.2 Existing Methods That Need Modification

| Method | Required Change |
|---|---|
| `startReading()` | Either modify to accept parameters (session, target, Q) and construct the blob dynamically, or create an alternative method and leave this one for backward compatibility. |
| `parseResponse()` | Currently only handles opcode 0x22. If you add new opcodes that produce async responses, this needs extension. For your use case, it may not need changes since continuous read still uses 0x22. |
| `setProtocolParameters()` | Currently declared with no parameters and no implementation. Needs a real implementation that accepts Gen2 protocol parameter codes and values. |
| `getProtocolParameters()` | Has a skeleton that sends 2 bytes but doesn't parse the response meaningfully. |

### 7.3 New Constants/Enums Needed

```cpp
// Gen2 Protocol Parameter Keys (for opcode 0x9B / 0x6B)
// These values are from the Mercury API (serial_reader_imp.h)
// IMPORTANT: You will need to verify these against the actual Mercury API SDK
#define TMR_SR_GEN2_PARAM_SESSION     0x00  // Verify against SDK
#define TMR_SR_GEN2_PARAM_TARGET      0x01  // Verify against SDK
#define TMR_SR_GEN2_PARAM_Q           0x02  // Verify against SDK

// Session values
#define GEN2_SESSION_S0  0x00
#define GEN2_SESSION_S1  0x01
#define GEN2_SESSION_S2  0x02
#define GEN2_SESSION_S3  0x03

// Target values
#define GEN2_TARGET_A    0x00
#define GEN2_TARGET_B    0x01
#define GEN2_TARGET_AB   0x02
#define GEN2_TARGET_BA   0x03
```

> ⚠️ **Important caveat**: The exact parameter key bytes for `0x9B` are NOT determinable from this library alone. The comments reference `serial_reader_imp.h` and `serial_reader_l3.c` from the Mercury API SDK. You will need to consult those files to get the correct parameter key values. The values I've listed above are representative but may not match ThingMagic's actual encoding.

### 7.4 Protocol Parameter Command Structure (Inferred)

Based on the patterns visible in the library (particularly how `setReaderConfiguration()` uses a key-value format for opcode `0x9A`), the `0x9B` (SET_PROTOCOL_PARAM) command likely follows a similar structure:

```
Probable structure for SET_PROTOCOL_PARAM (0x9B):
TX: FF [LEN] 9B [protocol] [param_key_hi] [param_key_lo] [value(s)] [CRC_hi] [CRC_lo]

Where:
  [protocol]     = 0x00 0x05 (Gen2, as 16-bit value matching setTagProtocol format)
  [param_key]    = 16-bit parameter identifier
  [value(s)]     = Parameter-specific value(s)
```

And for GET_PROTOCOL_PARAM (0x6B):
```
TX: FF [LEN] 6B [protocol] [param_key_hi] [param_key_lo] [CRC_hi] [CRC_lo]

Response will contain the current value in the data field.
```

**You should verify this by**:
1. Downloading the ThingMagic Mercury API C SDK.
2. Looking at `serial_reader_l3.c`, specifically `TMR_SR_cmdSetProtocolParam()` and `TMR_SR_cmdGetProtocolParam()`.
3. Looking at `serial_reader_imp.h` for the `TMR_SR_GEN2_Configuration` enum which lists all parameter keys.
4. Using the Universal Reader Assistant with "Transport Logs" enabled — change session/target/Q in the GUI and observe what bytes are sent.

### 7.5 Alternative Approach: Modifying the `startReading()` Blob

If reverse-engineering the `0x9B` command proves difficult, there's a second approach: figure out which bytes in the `startReading()` config blob correspond to session, target, and Q, and modify them directly. The blob is:

```
0x00, 0x00, 0x01, 0x22, 0x00, 0x00, 0x05, 0x07, 0x22, 0x10, 0x00, 0x1B, 0x03, 0xE8, 0x01, 0xFF
```

By using Transport Logs from the Universal Reader Assistant and varying only one parameter at a time, you can identify which bytes change. For example:
- Start a continuous read with Session S0, record the blob.
- Change to S1, record the blob.
- Diff the two — the byte(s) that changed encode the session.

This is the same reverse-engineering technique the library's author used to derive the blob in the first place.

### 7.6 Recommended Implementation Strategy for Your Exhibit

Given that your use case is well-defined (detect exactly 3 tags on button press), here's the highest-reliability approach:

1. **Use continuous reading mode** with properly configured session/target/Q.
2. **Maintain a tag set in your ESP32 code**: a dictionary mapping EPC → last-seen timestamp.
3. **On each `check()` + `RESPONSE_IS_TAGFOUND`**: extract the EPC, update the timestamp.
4. **On button press**: check which tags have been seen in the last N milliseconds (e.g., 500ms).
5. **Prune stale tags**: any tag not seen in the last N ms is considered absent.

This approach decouples the RF reading (continuous, handled by the module) from the application logic (button press triggers a snapshot of which tags are present). It's robust against occasional missed reads and doesn't require precise inventory round synchronization.

The key extension you need is configuring Session S1 so tags get properly suppressed and all 3 are reliably detected within your collection window.

---

## Appendix: Quick Reference — Where to Find Mercury API Documentation

To implement the extensions described in Section 7, you'll need:

| Resource | Where to Find | What to Look For |
|---|---|---|
| Mercury API C SDK | ThingMagic/Jadak website or GitHub forks | `serial_reader_l3.c`, `serial_reader_imp.h`, `tmr_gen2.h` |
| Universal Reader Assistant | ThingMagic downloads page | "Transport Logs" feature — shows raw UART bytes for any GUI action |
| EPC Gen2 Standard (ISO 18000-63) | GS1 / ISO | Definitive reference for sessions, targets, Q, and air interface |
| SparkFun Library Issues/PRs | github.com/sparkfun/Simultaneous_RFID_Tag_Reader | Community may have already implemented some extensions |
