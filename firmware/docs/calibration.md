# RAFT bench calibration

What the firmware needs to know about the real arm before it may apply torque, and how to measure it on the
bench. Everything runs on the ESP32. You type commands on the USB serial console (shorthand shown) or send the
JSON form to TCP port 5000 (`docs/teleop_protocol.md`).

The whole procedure was rehearsed in MuJoCo against servos with wrong zeros, flipped signs, torque constants
off by 10 %, and a spring 10 % stiffer than configured (`tools/sim/calib_sim.cpp`). It recovered every value:

| Quantity | Truth | Recovered |
|---|---|---|
| Encoder zeros, 7 joints | arbitrary, incl. near the tick wrap | exact |
| Signs, 7 joints | +1 -1 +1 -1 +1 +1 -1 | exact |
| J3 torque constant | 0.4854 Nm/A | 0.4857 Nm/A |
| J2 torque constant | 0.3972 Nm/A | 0.3974 Nm/A |
| Friction J2 / J3 | 0.020 Nm | 0.021 / 0.020 Nm |
| Spring strength | 1.100 x configured | 1.098 x |
| Worst J2/J3 gravity-compensation error | 29 mNm before | 1.5 mNm after |

The torque constants stay within 0.5 % from zero friction up to 0.08 Nm, half the J3 load. The simulated
friction and armature are estimates until the bench measures them.

## Before you start

- **Servos.** All 7 answer: the firmware does not boot past the servo check otherwise.
- **Model.** The boot log says `Model self-test: PASS`.
- **Spring.** It can stay mounted. Steps 1 and 2 need no torque. The J2 sweep in step 6 runs with the upper arm near vertical, where the spring gives almost nothing, and the firmware subtracts that little from its model.
- **Second pair of hands.** Steps 6 and 7 apply torque. Keep a hand near the arm.

`status` shows the progress at any time: `cal_flags` per joint (1 zero, 2 sign, 4 torque constant), the torque
constants `kt`, the measured `cur_reg_per_unit`, and the last calibration message.

## What is stored where

| What | Where | Same on every arm? |
|---|---|---|
| Joint directions (`kJointDir`: J1 and the trigger reversed), stop angles (`kStopDeg`), DH model | firmware, `src/main/arm_design.h` and `raft_model.h` | yes: change them in the code |
| The servo's current direction (`kCurrentSign` = -1: a positive goal current turns the HLS3606 toward lower encoder counts, measured on the first arm) | firmware, `src/main/control.h` | yes: a property of the servo model |
| The servo's current register unit: 3.25 mA (not the vendor example's 6.5: with 6.5 the arm needed twice the model's current, while the datasheet Kt 0.441 Nm/A and the CAD masses fit 3.25), and the cap of 400 units = the 1.3 A stall current | default in `src/main/control.cpp`, stored per arm as `ma_per_unit` | yes |
| Encoder zeros, torque constants and friction, current unit, ranges, spring, gravity scale, teleoperation settings, PAR6 address | the ESP32's flash: NVS namespace `raft`, key `cfg`, one 308-byte blob with a CRC, written by `save` | no: measured per arm |

The blob survives power-offs and firmware updates (a normal upload does not touch NVS; only a full flash erase
does). It still has a direction field per joint for its layout, but the firmware always puts `kJointDir` there,
whatever was stored.

## 1. Zero at the mechanical stops (recommended)

Every joint has a mechanical stop at a known angle, `kStopDeg` in `src/main/arm_design.h` (CAD convention, 0 in
the standby pose; the trigger's stop is 39.63 deg past the pose where it is parallel to the handle's face). One
joint at a time, limp:

```
mode limp
calib_stop 1
```

Hold J1 still against its stop for 0.3 s: the firmware averages its reading there (and refuses if it moved more
than 10 ticks), and with the joint's fixed direction the stop angle gives the zero. Repeat for joints 2 to 7. The
web UI's Calibrate page guides through the same steps. `{"cmd": "calib_stop", "joint": 3, "deg": 92.5}` uses a
measured stop angle instead of the table.

Each reply line says `stop: J1 zero 2970 ticks (stop at -136.28 deg)`. Afterwards the standby pose must read about
0 on every joint, and the 3D view must copy the real arm. If a joint turns the other way in the view, its servo is
mounted differently from the design (`kJointDir`).

The test `tools/sim/test_app.cpp` runs it against seven servos with known zeros and recovers them all.

## 1b. Zero pose (without stops)

Put the arm in the CAD zero pose: the viewer's standby pose with every slider at 0, upper arm vertical. Compare
the arm with the viewer joint by joint, including the wrist and the trigger. A printed jig makes this
repeatable. Hold it still.

```
mode limp
calib_zero
```

The firmware averages 0.5 s and refuses if any joint moved more than 10 ticks (0.9 deg).

## 2. Joint directions

Nothing to do: `kJointDir` in `src/main/arm_design.h`, the same on every RAFT. `calib_dir` and `set_dir` answer
that the directions are fixed.

## 3. Spring

Tell the firmware which spring is mounted. Rates of the federnshop and Lee Spring springs are below. For any other
spring, measure its rate (`docs/spring_mount.md`) and give it in N/m.

| Spring | Command |
|---|---|
| Z-075NI | `{"cmd": "spring", "type": "cable", "k": 565, "a_mm": 13, "b_mm": 20}` |
| Z-078AI | `{"cmd": "spring", "type": "cable", "k": 718, "a_mm": 13, "b_mm": 20}` |
| Z-078I | `{"cmd": "spring", "type": "cable", "k": 824, "a_mm": 13, "b_mm": 20}` |
| Z-077AI | `{"cmd": "spring", "type": "cable", "k": 1027, "a_mm": 13, "b_mm": 20}` |
| LEM080C 05 M | `{"cmd": "spring", "type": "cable", "k": 1028, "a_mm": 13, "b_mm": 20}` |
| LEM080C 06 M | `{"cmd": "spring", "type": "cable", "k": 821, "a_mm": 13, "b_mm": 20}` |
| LEM080C 06 S | `{"cmd": "spring", "type": "cable", "k": 685, "a_mm": 13, "b_mm": 20}` |
| LEM080C 07 M | `{"cmd": "spring", "type": "cable", "k": 683, "a_mm": 13, "b_mm": 20}` |
| LEM080C 08 M | `{"cmd": "spring", "type": "cable", "k": 585, "a_mm": 13, "b_mm": 20}` |
| none | `spring none` |

## 4. Save

```
save
```

The zeros are enough for torque, so save now in case a later step goes wrong.

## 5. First torque

Hold the arm in the zero pose and switch on MOVE, which holds the current pose:

```
mode move
```

Let go slowly. The arm should stay where it is. In the simulation it moved 0.2 deg. If a joint pushes away or
falls hard, type `mode limp` and check that joint's sign.

## 6. Torque constants, J3 then J2

Stay in the zero pose. MOVE keeps holding the other joints while one is measured.

```
calib_torque 3
calib_torque 2
```

Each sweep moves the joint 4 deg up, 8 deg down and back over about 6 s. It measures the current it needs while
moving up and down: their mean is the load and their difference is friction. The reply looks like
`torque: joint 3 Kt 0.486 Nm/A (assumed 0.441), friction 0.020 Nm at 0.160 Nm load`.

A joint needs at least 0.04 Nm of gravity load. J1 never carries load, and in the zero pose the model gives J4
0.024 Nm, J5 0.032 Nm and J6 0.014 Nm, so the firmware refuses them. Their nominal 0.441 Nm/A is fine: a 10 %
error there is about 3 mNm, well below the gearbox friction.

The measured value is torque per register unit, so an error in the 6.5 mA per unit assumption ends up in Kt and
cancels out. The sweep also measures the present-current register per commanded unit (`cur_reg_per_unit`).

## 7. Spring strength

Tilt the upper arm by 45 deg with the forearm level, then measure:

```
{"cmd": "move", "q_deg": [0, 45, 45, 0, 0, 0, 0], "speed_deg": 20}
calib_spring
```

The reply says how much the spring really gives and multiplies `spring.scale` by that factor, for example
`spring: gives -0.163 Nm where the model says -0.148 Nm -> multiply spring.scale by 1.098`.

## 8. Save and check

```
save
mode gravity
```

Let go at a few poses: upper arm forward, back, forearm folded. The arm should float without drifting. If it
sinks or rises everywhere by the same amount, fine-tune with `gravity_scale` (0.95 to 1.05).

## 9. Joint ranges

```
calib_range
```

Limp or weightless. Move every joint gently to both of its ends (do not force a hard stop), then:

```
calib_range_stop
save
```

The reply lists the measured ranges. Joints moved through less than 10 deg keep their defaults (J2 -105..130 and
J3 -75..150 deg from the CAD, +-180 deg for the others). The ranges are stored 2 deg inside the ends; they clamp
move targets and place the haptic barriers (`docs/teleop_protocol.md`, `barriers_always`). It ends by itself after
3 minutes.

Torque constants and the current unit (`set_kt`, `set_ma_per_unit`, `load`, `defaults`) change
only while the arm is limp: a sign flipped while gravity compensation runs would push the arm the wrong way.

## When something refuses

| Message | Meaning and fix |
|---|---|
| `not calibrated: capture every joint at its stop (calib_stop), or calib_zero` | Torque needs a zero on every joint first (step 1). |
| `zero: joint N moved by M ticks while sampling` | Hold the arm still, or use the jig. |
| `direction: the joint was not turned within 15 s` | Start again and turn sooner. |
| `torque: joint N carries only X Nm here` | Pose that joint's link nearer horizontal, or skip it. |
| `spring: tilt the upper arm at least 40 deg` | Move J2 further from vertical. |
| `sweep: the joint did not follow ... stopped and let go` | The arm went limp. Usually a servo mounted the other way round than `kJointDir` says (the 3D view turns that joint against the real one). A blocked joint gives the same message. |
| `torque: implausible result ...` | The measured constant is outside 0.4x to 2.5x the assumed one. Check the sign and `set_ma_per_unit`. |
| `fault: ... (clear_fault first)` | A servo stopped answering or overheated. Fix the cause, then `clear_fault`. |
| `set mode limp first: this changes how torque is computed` | `set_kt`, `set_ma_per_unit`, `load`, `defaults` need LIMP. |
| `range: no joint was moved through 10 deg` | Move the joints further during `calib_range`. |
| `a calibration is running (calib_abort first)` | Settings and `save` wait until the running procedure ends. |
