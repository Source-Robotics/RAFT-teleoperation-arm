# Assembling RAFT

From printed parts to a wired arm, in order. The parts list is in [`BOM.md`](../BOM.md). After the last step, the
[bring-up guide](../firmware/docs/bringup.md) takes over: flashing, calibration at the stops, first torque.

![The parts of RAFT](images/00_arm.png)

## Before you start

- **Print the 11 parts** with the settings in the [print table](../3D%20print%20files/RAFT%20Teleop%20Arm%20Print%20Table%20-%20Sheet1.pdf),
  and remove all support material. The firmware's gravity model expects the masses in that table.
- **Number the servos first.** All seven servos share one bus, so each needs its own ID, 0 to 6. It is easiest to
  set them before they go into the arm: follow [bring-up step 3](../firmware/docs/bringup.md#3-check-the-servo-ids),
  one servo at a time, then put a label with its joint on each servo.

  | Joint | J1 base | J2 shoulder | J3 elbow | J4 | J5 | J6 | J7 trigger |
  |---|---|---|---|---|---|---|---|
  | Servo ID | 0 | 1 | 2 | 3 | 4 | 5 | 6 |

- **Tools.** Screwdrivers for the M2 and M3 screws and for the screws that come with the servos.

## How it goes together

Every joint is built the same way:
- **Servo:** it sits in one printed part, fixed with the 4 screws that come with it.
- **Horn:** it goes onto the servo's output with its own screw (it comes with the servo).
- **Next part:** it fixes to the horn with 4 M2 screws (4 or 5 mm).

| Joint | Servo ID | Servo sits in | Its horn carries |
|---|---|---|---|
| J1 | 0 | Base | Shoulder |
| J2 | 1 | Shoulder | Upper arm (lower end) |
| J3 | 2 | Elbow | Upper arm (upper end): the elbow turns about it |
| J4 | 3 | Elbow | Lower arm |
| J5 | 4 | Lower arm | Wrist |
| J6 | 5 | Wrist | Gripper handle |
| J7 | 6 | Gripper handle | Gripper trigger |

**Stop screws.** Each joint has a mechanical stop: an M3 × 8 screw that the neighbouring part runs into at the
joint's limit. The firmware knows the angle of every stop (`firmware/src/main/arm_design.h`), and the calibration
captures each joint's zero against it. So:
- Screw every stop screw fully home before the calibration, and leave it that way.
- A stop screw that sits differently moves that joint's zero. If you replace or loosen one, capture that joint's zero
  again.

**Any horn angle works:** the calibration at the stops finds each joint's zero after assembly. Assemble in or near
the **standby pose**, as in the picture above: upper arm vertical, forearm level and pointing forward, wrist
straight, trigger parallel to the handle's face.

**In the pictures:**

| Colour | Means |
|---|---|
| Green | The printed part you fit in that step |
| Dark | Servos |
| Light blue | Horns |
| Purple | Screws |
| Grey | Parts already fitted |

A see-through part has a servo going into it. A part drawn away from the arm goes where its arrow points.

## Steps

### 1. J1 servo into the Base

Fit the J1 servo (ID 0) into the Base with its 4 screws, then its horn onto the output with the horn screw. Lead the
servo's cable out toward the electronics.

![Step 1](images/01_base.png)

### 2. Shoulder

**2a.** Fix the Shoulder to J1's horn with 4 × M2.

![Step 2a](images/02a_shoulder.png)

**2b.** Fit the J2 servo (ID 1) into the Shoulder, and its horn onto it. Screw in the M3 × 8 stop screw. Close the
Shoulder with its lid and 2 × M3 × 8.

![Step 2b](images/02b_j2_servo.png)

### 3. Upper arm onto J2's horn

Fix the lower end of the Upper arm to J2's horn with 4 × M2, standing vertical.

![Step 3](images/03_upper_arm.png)

### 4. Elbow

**4a.** Fit the J3 servo (ID 2) and the J4 servo (ID 3) into the Elbow, each with its horn. Screw in the M3 × 8 stop
screw. Close the Elbow with its lid and 2 × M3 × 8.

![Step 4a](images/04a_elbow_servos.png)

**4b.** Fix J3's horn to the upper end of the Upper arm with 4 × M2.

![Step 4b](images/04b_elbow_on.png)

### 5. Lower arm

**5a.** Fix the Lower arm to J4's horn with 4 × M2.

![Step 5a](images/05a_lower_arm.png)

**5b.** Fit the J5 servo (ID 4) into the Lower arm, and its horn onto it. Screw in the M3 × 8 stop screw.

![Step 5b](images/05b_j5_servo.png)

### 6. Wrist

**6a.** Fix the Wrist to J5's horn with 4 × M2.

![Step 6a](images/06a_wrist.png)

**6b.** Fit the J6 servo (ID 5) into the Wrist, and its horn onto it. Screw in the M3 × 8 stop screw.

![Step 6b](images/06b_j6_servo.png)

### 7. Gripper handle

**7a.** Fix the Gripper handle to J6's horn with 4 × M2.

![Step 7a](images/07a_handle.png)

**7b.** Fit the trigger servo (ID 6) into the handle, and its horn onto it.

![Step 7b](images/07b_j7_servo.png)

### 8. Gripper trigger onto its horn

Fix the Gripper trigger to the trigger servo's horn with 4 × M2.

![Step 8](images/08_trigger.png)

### 9. ESP32-PoE onto its three posts

Lay the ESP32-PoE on the three posts of the Base plate, with its Ethernet jack at the plate's edge: its three mounting
holes sit over the posts. Fix it to the posts with 3 × M2 (4 or 5 mm).

![Step 9](images/09_esp32.png)

### 10. Base lid

When the cables are in place (see Wiring), close the Base with its lid and 2 × M3 × 8.

![Step 10](images/10_base_lid.png)

**Check every joint.** Turn each joint gently by hand from stop to stop. It must reach both stops without rubbing or
catching a cable.

## Wiring

The ESP32-PoE sits on its three posts on the base plate (step 9). The full wiring, with
the checks before the first power-up, is in [bring-up step 2](../firmware/docs/bringup.md#2-wiring). In short:

```
  6.0 V supply (+) --[switch]--> TTLinker VIN          6.0 V supply (-) --> TTLinker GND --> ESP32-PoE GND
  ESP32-PoE GPIO32 ------------> TTLinker TX1
  TTLinker RX0 ----------------> ESP32-PoE GPIO33
  TTLinker servo port ---> J1 ---> J2 ---> J3 ---> J4 ---> J5 ---> J6 ---> J7 (trigger)
```

- **Supply off.** Plug and unplug servo cables only with the servo supply off.
- **Servo plug.** 5264 3-pin: pin 1 GND, pin 2 Vcc, pin 3 signal.
- **Slack at every joint.** Leave enough cable at each joint that it does not pull when the joint turns from stop to
  stop.
- **Connectors fully home.** Every connector must click fully home. A loose plug or a poor crimp on the signal wire
  does not stop the arm at once. It shows as missed reads on the web UI's Health page: if one servo keeps missing
  reads, check its plug and its lead first.

## After assembly

1. **Check the IDs on the bus.** With all seven connected, the scan in
   [bring-up step 3](../firmware/docs/bringup.md#3-check-the-servo-ids) must show IDs 0 to 6, each answering 5 of 5.
2. **Flash the firmware and bring it up.** Follow [bring-up step 4](../firmware/docs/bringup.md#4-flash-and-watch-the-boot) onward.
3. **Zero at the stops.** Capture each joint at its mechanical stop once
   ([bring-up step 8](../firmware/docs/bringup.md#8-zero-at-the-stops)). Redo a joint only when its servo, its horn or
   its stop screw was taken off.
4. **Spring (optional).** The J2 counterbalance: [`firmware/docs/spring_mount.md`](../firmware/docs/spring_mount.md).

The pictures are rendered from the arm's 3D model (the simulators' model in `firmware/tools/sim/model`) by
[`tools/render_assembly.py`](../tools/render_assembly.py). Run it again after a CAD change. The ESP32-PoE is not part
of the arm's model: the tool adds it from Olimex's board STEP (`--esp32 ESP32-PoE_Rev_L.step`, needs
`pip install cascadio`).
