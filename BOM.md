# RAFT – Bill of Materials (BOM)

Everything needed to build **one** RAFT arm. How it goes together: [`Assembly manual/readme.md`](Assembly%20manual/readme.md).

> [!NOTE]
> If any product links are broken, unavailable, or no longer point to the correct item, please [open an issue](https://github.com/Source-Robotics/RAFT-teleoperation-arm/issues/new), send us a message on Discord, or contact us via email. You can also use the reference images to find the same or an equivalent part from another supplier. Where the pictures come from: [`BOM reference images/README.md`](BOM%20reference%20images/README.md).

---

## Servos

| Type | Quantity | Description | Link | Reference Image |
|------|----------|-------------|------|-----------------|
| FEETECH HLS3606M, 6 V, 6 kg·cm | 7 | J1 to J6 and the trigger, servo IDs 0 to 6. Each comes with its horn and screws. 20.6 g each, 25-tooth output, 15 cm lead with a 5264 3-pin plug (1 GND, 2 Vcc, 3 signal). Datasheet: [`datasheets/HL_3606_C001_datasheet.pdf`](datasheets/HL_3606_C001_datasheet.pdf) | [Link](https://www.aliexpress.com/item/1005010298067392.html) | [Link](BOM%20reference%20images/HLS3606M_servo.jpg) |

---

## Screws

| Diameter / Type | Length | Quantity | Description | Link |
|-----------------|--------|----------|-------------|------|
| M2 | 4 or 5 mm | 36 | Printed parts to the servo horns (4 per horn, 7 horns = 28) and the ESP32-PoE to the Base's three posts (3): 31 used, plus 5 spare | |
| M3 | 8 mm | 15 | The lids (2 each on the Base, Shoulder and Elbow lids) and the 4 joint stops: a printed part stops against the screw at the joint's limit. 10 used, plus 5 spare | |
| Servo screws | | come with the servos | Servo bodies into the printed parts, and each horn onto its servo | |

---

## Electronics

| Type | Quantity | Description | Link | Reference Image |
|------|----------|-------------|------|-----------------|
| ESP32-POE | 1 | Motor controller. The WROOM-32 version (the firmware's Ethernet setup assumes it). Not the -ISO: never connect USB while it runs on PoE | [Link](https://www.olimex.com/Products/IoT/ESP32/ESP32-POE/open-source-hardware) | [Link](BOM%20reference%20images/ESP32-POE.jpg) |
| TTLinker mini | 1 | Feetech FE-TTLinker-Mini, the half-duplex bus adapter for the servos (74HC126, 5 V logic). Datasheet: [`datasheets/TTLINKER.pdf`](datasheets/TTLINKER.pdf) | [Link](https://www.aliexpress.com/item/1005011897724178.html) | [Link](BOM%20reference%20images/TTLinker_mini.jpg) |
| Servo supply 5–7 V | 1 | Any DC adapter of 1.5 A or more, 6 V is best (5 to 7 V works); 2 to 3 A leaves headroom for fast moves and force feedback. Floating output (not earthed). A bench supply with a current limit works too |  | [Link](BOM%20reference%20images/bench_power_supply.jpg) |
| Supply leads, switch | 1 | 0.5 mm² (20 AWG) or thicker; the switch in the + lead is the hard stop |  | [Link](BOM%20reference%20images/toggle_switch.jpg) |
| PoE switch or injector (802.3af) | 1 | Powers the ESP32 over the network cable; or USB at the bench (then a network port without PoE) |  | [Link](BOM%20reference%20images/PoE_injector.jpg) |

---

## J2 counterbalance (optional)

| Type | Quantity | Description | Link | Reference Image |
|------|----------|-------------|------|-----------------|
| Extension spring | 1 | 0.8 mm stainless wire, 6 mm outside, 35 mm long: pick that variant (sold in packs of 5). The 40 mm one is the softer option. The listing gives no rate: about 0.79 N/mm, measure it before fitting ([`firmware/docs/spring_mount.md`](firmware/docs/spring_mount.md)) | [Link](https://www.aliexpress.com/item/1005004052186320.html?cn=%7B%22c%22%3A%22NgewdvyPA4UybATE1782475394%22%2C%22e%22%3A%221020%22%7D&aff_fcid=0a03354edfdb4792a4dc2c3c1de7bb6d-1793893871132-05974-6mia6uv&aff_fsk=6mia6uv&aff_platform=aaf&sk=6mia6uv&aff_trace_key=0a03354edfdb4792a4dc2c3c1de7bb6d-1793893871132-05974-6mia6uv&terminal_id=6f6a07bafed64133a81ef5a35dd2a6fa&afSmartRedirect=y) | [Link](BOM%20reference%20images/extension_spring.jpg) |
| Dyneema cable 0.5 mm | 0.3 m | Runs from the spring over the eyelet to the anchor |  | [Link](BOM%20reference%20images/braided_line.jpg) |
| Eyelet | 1 | A small screw eye pin (4–7 mm, sold for jewelry making) screwed into the eyelet standoff; a ceramic thread-guide ring or a 2–3 mm slice of 2 × 4 mm PTFE tube works too. Keep the loop smooth: a burr frays the line | [Link](https://www.aliexpress.com/item/1005007217310561.html) | [Link](BOM%20reference%20images/eyelet.png) |

---

> **Note:** Equivalent parts from other suppliers may be used as long as specifications match.
