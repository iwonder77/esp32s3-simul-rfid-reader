# ESP32-S3-ETH + M7E Hecto — UHF RFID Multi-Tag Detection

## Overview

This repository documents my work getting reliable **simultaneous multi-tag detection** out of [SparkFun's Simultaneous RFID Reader (M7E Hecto)](https://www.sparkfun.com/sparkfun-simultaneous-rfid-reader-m7e-hecto.html) paired with Waveshare's [ESP32-S3-ETH module](https://www.waveshare.com/wiki/ESP32-S3-ETH).

The end goal is to build a solid foundational prototype to iterate on for the beaker interactive at the Salon exhibit in Kidopolis at Thanksginving Point's Museum of Natural Curiosity. This interactive will detect a combination of physical molecule pucks that have been placed inside a beaker. Each puck contains a UHF RFID tag with a unique EPC (in the future I'll probably write some simpler data to each tag to differentiate pucks). When a visitor presses a button, the system scans, identifies all present pucks, and triggers a response (sends UDP commands to a BrightSign via a direct Ethernet connection) based on the specific combination detected. The catch: it needs to reliably detect **all 3 pucks simultaneously**, not just whichever one happens to have the strongest signal.

## Repository Structure

### `old-lib/`

Contains sketches built on SparkFun's stock [Simultaneous RFID Tag Reader library](https://github.com/sparkfun/SparkFun_Simultaneous_RFID_Tag_Reader_Library/tree/master). These sketches, along with a deeper review of the library's source code, revealed just how insufficient it is at leveraging the M7E Hecto's actual capabilities.

To be fair, SparkFun does note that their library is a "stripped-down implementation" of the [ThingMagic Mercury API](https://www.jadaktech.com/product/thingmagic-mercury-api/). But the parts they stripped out happen to include **the Gen2 session and target parameters that make simultaneous reading actually work**. Without those, and based on the results of the sketches, I strongly believe their hardcoded config blob made the reader default to Session S0, where every tag resets its inventoried flag almost instantly, making the strongest tag dominate every inventory round. The result: you get the same tag reported over and over while the others are drowned out or rarely appear. For a product marketed as a _simultaneous_ RFID reader, the library not exposing simultaneous-reading configuration out of the box was... a bit disappointing. :(

One of the more critical findings: opcode `0x9B` (`SET_PROTOCOL_PARAM`) is _defined_ in the header file and even has a method _declared_ — but it's never actually implemented. This is the opcode needed to configure Gen2 sessions, targets, and Q values, which are the mechanisms that allow the reader to systematically work through a tag population instead of letting one tag monopolize the airwaves (see the README.md in the `gen2/` directory for a deeper dive into these concepts).

### `gen2/`

This is where things get much more interesting. Many users in Sparkfun's forums posted similar issues relating to the library's lack of simultaneous reading capabilities, and one user stood out that responded to nearly every one of these posts, **paulvha**. In one of these replies I stumbled across his [ThingMagic repository](https://github.com/paulvha/ThingMagic), which contains an extended fork of SparkFun's library that implements proper Gen2 parameter control. Specifically, on first glance, it appeared to add working implementations of:

- **`setGen2Session()`** — Configure which Gen2 session (S0–S3) the reader uses, controlling how long tags stay quiet after being inventoried
- **`setGen2Target()`** — Set whether the reader queries tags in state A, B, or alternates between both (AB/BA)
- **`setGen2Q()`** — Configure the anti-collision Q algorithm (static or dynamic) and initial slot count
- **`setGen2RFmode()`** — Select pre-defined RF modulation profiles on the M7E (BLF, Miller encoding, Tari)

With Session S1 (tags suppress for 0.5–5 seconds after being read, functionally turning "quiet"), Target AB (ping-pong between flag states), and Dynamic Q (auto-adjusting slot count), all 3 pucks are reliably detected within the first ~100–200ms of a scan window. The sketch in this directory demonstrates this in action (video coming soon).

Major thanks to [paulvha](https://github.com/paulvha) for building and maintaining this extension. His library, thorough documentation, and active presence on the SparkFun forums saved what would have otherwise been a deep dive into reverse-engineering ThingMagic's serial protocol from scratch.
