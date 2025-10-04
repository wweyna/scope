# STM32 USB (UART‑over‑USB) Scope — README

This project is a minimal “USB oscilloscope” built on an STM32 microcontroller. It samples one analog channel with ADC + DMA, performs simple trigger detection, and streams frames to a Windows host application over a UART (typically exposed to the PC as a USB virtual COM port via an on‑board ST‑Link or external USB–UART bridge).

---

## High‑level pipeline

```
ADC1 → DMA (circular, left/right halves)
          │
          ├─(IRQ: half/complete)→ Record buffer (4 frames + 1‑frame margins)
          │                                    │
          └────────────────────────────────────┘
                               ▼
                       Trigger selection
                               ▼
                Output back‑buffer (with 5‑byte header)
                               ▼
                     3×10‑bit → 2×16‑bit packer
                               ▼
                      LPUART1 DMA → Windows app
```

---

## Time bases

Three fixed time bases are provided. Each defines the **samples per frame** and the ADC sampling time. The firmware restarts acquisition when the time base changes.

| Code | Frame time | Samples per frame | ADC sampling time |
| ---: | ---------- | ----------------- | ----------------- |
|  `1` | 10 ms      | 1154              | 247.5 cycles      |
|  `2` | 1 ms       | 811               | 24.5 cycles       |
|  `3` | 250 ms     | 500               | 2.5 cycles        |

> Notes: Samples are 12‑bit in ADC memory (`uint16_t`). Transport reduces them to 10 bits per sample.

---

## Buffers (what they are and how big they get)

All sizes below are expressed in **samples** unless stated otherwise. One sample = `uint16_t` (2 bytes).

### 1) ADC DMA ring buffer

* **Name:** `scope_adc_dma_buffer`
* **Layout:** circular buffer split into **left** and **right** halves.
* **Half size:** `SCOPE_ADC_MAX_FRAME_SIZE * SCOPE_ADC_FRAMES_IN_HALF_BUFFER`.
* **Whole size:** 2 × half size.
* **Purpose:** continuous intake of ADC samples. Each half‑complete/complete DMA interrupt indicates two frames are ready to be copied onward.

### 2) Record buffer (producer: ADC IRQ, consumer: TX path)

* **Name:** `scope_record_buffer`
* **Capacity:** `SCOPE_RECORD_BUFFER_FRAME_COUNT + 2×SCOPE_MARGIN_FRAME_COUNT` frames.

  * Default layout is **1 margin + 4 active frames + 1 margin**.
* **Why margins?** They give room to shift the start position based on trigger offset without falling out of bounds.
* **Fill policy:** Every **ADC half‑buffer** (2 frames) is appended; after two such writes, the record becomes **Completed**.
* **States:** `Idle` → `InProgress` → `Completed` (then unlocked back to `Idle` after the TX path copies one output frame).

### 3) Output frame double‑buffer

* **Names:** `scope_out_frame_buffer_1`, `scope_out_frame_buffer_2` with pointers `scope_out_frame_front_buffer`/`_back_buffer`.
* **Contents:** `5‑byte header + one frame of raw 16‑bit samples`.
* **Header (5 bytes):**

  1. `0xFF`
  2. `0xFE`
     3–4. VREF (LSB, MSB) measured at startup
  3. time‑base code (`1`, `2`, `3`)
* **Swap rule:** never swap while a UART DMA transfer is in progress; swapping is performed just before encoding/transmit so the freshly prepared frame becomes the **front**.

### 4) Encoded transmit buffer

* **Name:** `scope_out_encoded_frame_buffer`
* **Contents:** `5‑byte header + packed payload`.
* **Why separate?** The encoder writes a packed form (see below) and needs its own contiguous buffer for `HAL_UART_Transmit_DMA`.

---

## Triggering and frame selection

* **Edge:** rising or falling.
* **Modes:**

  * **Auto:** scan the recorded region (excluding margins) for a crossing of the mid‑level between the local min and max. A small multi‑sample confirmation window reduces spurious picks. The start of the frame is then placed at `trigger_position` (default: 50%) *before* the detected crossing.
  * **None:** use the first frame in the record (immediately after the left margin).

After selecting the start address, exactly **one frame** is copied from the record buffer into the **output back‑buffer** (after its header).

---

## Encoding (3×10‑bit → 2×16‑bit)

To cut bandwidth, each group of three ADC samples is reduced to 10 bits and packed into two 16‑bit words:

* Take `s0`, `s1`, `s2` (12‑bit). Discard two LSBs → `s*_10`.
* Emit two words:

  * `w0 = s0_10 | ((s2_10 & 0x001F) << 10)`
  * `w1 = s1_10 | ((s2_10 & 0x03E0) << 5)`
* **Bit 15 is always 0** to avoid colliding with the header marker pattern.
* If the frame size isn’t a multiple of three, the **tail** (1–2 samples) is stored as standalone 16‑bit words containing the 10‑bit values.

> **Host decoding sketch:** `s0 = w0 & 0x03FF; s1 = w1 & 0x03FF; s2 = ((w0>>10)&0x1F) | (((w1>>10)&0x1F)<<5);` (optionally `<< 2` to put them back on a 12‑bit scale).

---

## Data‑flow, step‑by‑step

1. **Run**: `Scope_Run()` calibrates ADC (reads VREF), configures the selected time base, sets buffer pointers/sizes, clears buffers, writes headers, and starts **ADC1 in DMA circular mode**.
2. **Acquire**: Each DMA **half‑complete/complete** IRQ calls a common handler that appends **two frames** from the just‑finished half into the **record buffer** and advances a counter. After two appends (four frames total), the record is **Completed** and left untouched until consumed.
3. **Select**: `Scope_TransmitFrame()` waits for `Completed` (processing incoming commands while waiting), then applies the current edge/trigger settings and chooses the frame start (triggered or first frame).
4. **Copy**: One frame is copied into the **output back‑buffer** (after the header). The record buffer is unlocked (`Idle`) so acquisition can start building the next record concurrently.
5. **Encode + TX**: It waits for any previous UART DMA to finish (with a timeout), **swaps** the output buffers, **encodes** front‑buffer samples into the **encoded buffer**, and starts `HAL_UART_Transmit_DMA()` on **LPUART1**. A TX‑complete interrupt marks the transfer `Completed`.

---

## Host protocol (control plane)

Single‑byte commands from the Windows app:

* `G` — Run
* `S` — Stop
* `R` — Rising edge
* `F` — Falling edge
* `A` — Auto trigger
* `N` — No trigger
* `1` — 10 ms time base
* `2` — 1 ms time base
* `3` — 250 ms time base

---

## Build & integration notes

* **Word size:** All ADC/record math is in **samples (uint16_t)**; output sizes are in **bytes**. Keep unit conversions explicit.
* **Concurrency:** ADC and UART use DMA with lightweight ISR handlers. Shared state between ISR and foreground (e.g., recording/tx statuses) should be treated carefully.

---

## Glossary of key constants (by intent)

* `SCOPE_ADC_MAX_FRAME_SIZE` — maximum samples per frame (limits buffer allocations)
* `SCOPE_ADC_FRAMES_IN_HALF_BUFFER` — how many frames fit into one DMA half (default: 2)
* `SCOPE_MARGIN_FRAME_COUNT` — how many safety frames kept on each side of the record (default: 1)
* `SCOPE_RECORD_BUFFER_FRAME_COUNT` — how many *active* frames per record (default: 4)
* `SCOPE_OUT_FRAME_HEADER_SIZE` — header length (5 bytes)

---

