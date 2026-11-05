# J2 spring mount: what to add to the CAD (layout v2)

Zero-length cable balancer for J2. Interactive check: `docs/raft_viewer.html`, built by `python tools/build_viewer.py` (Spring view, "Sweep J2" /
"Sweep J3"). It opens with the AliExpress springs below selected. The mount is the same for the AliExpress, federnshop
and Lee Spring springs; the Springs Online set needs the mount in brackets.

## Where everything goes

Coordinates in mm. **Assembly** = the SolidWorks assembly's default coordinate system at the standby pose
(Y up, Z forward, X along the J2 axis). **CS2** = "Coordinate System2" (on the J2 axis, the shoulder
bracket's frame). **CS3** = "Coordinate System3" (on the J3 axis, the upper arm's frame).

| Point | Assembly X Y Z | Part frame | Notes |
|---|---|---|---|
| J2 axis | 14.3 0 13 (line along X) | CS2 origin | reference |
| Upper-arm bar outer face | X = -13.9 (plane) | CS2 z = -28.2 | everything below sits outside this face |
| Cable plane | X = -18.7 (plane) | CS2 z = -33, CS3 z = +33 | 4.8 mm outside the bar face (Springs Online: -35) |
| Anchor A, top of the post | -18.7 13.0 13.0 | CS2 (0, -13, -33) | 13 mm above the J2 axis (SO: 16) |
| Crossing plate | Y = -13.5, X -7.1 .. -18.7, Z 13 | CS2 y = 13.5, z -21.4 .. -33 | 3 mm thick, ~8 mm wide |
| Eyelet B | -18.7 20.0 13.0 | CS3 (-80, 0, 33) | 20 mm up the arm from J2 (SO: 27) |
| Spring pin F | -18.7 101.0 13.0 | CS3 (1, 0, 33) | 1 mm past the J3 axis (SO: 103 -> CS3 x = 3) |

## Changes per part

1. **Shoulder bracket (link1 parts): plate + post.** The upper arm sweeps everything around the J2 axis
   in its own plane over J2 = -105..+130 deg, except straight below the axis. So the plate leaves the
   bracket's outer face (CS2 z = -21.4) *below* the axis, in the 6 mm gap between the bar's rounded end
   (10.5 mm below the axis) and the base top (16.8 mm below): a 3 mm plate centred 13.5 mm below the axis.
   At the cable plane it turns up into a 4 mm post that ends 13 mm above the axis. The cable ties to its top
   (a 1.5 mm hole or a small screw head). The post rotates with J1 over the base: keep the plate's
   underside clear of the base top (1.8 mm in the model).
2. **Upper arm bar (link2): eyelet standoff.** On the outer face, 20 mm from the J2 axis, a standoff
   reaching to the cable plane carrying a short, smooth ring, hole axis along the arm. The cable runs along
   the arm through it and on to the anchor. It leaves the ring at up to 40.5 deg to either side (largest at
   J2 = +-50 deg, where the angle is asin(13/20); 36 deg on the Springs Online mount). A short ring with both
   edges rounded is enough, no pulley. Good parts: a ceramic snap-in thread-guide eyelet (alumina, about 2 mm
   bore, pressed into a hole in the standoff), a small screw eye pin (round wire loop, the BOM's) screwed into the
   standoff, or a 2-3 mm slice of 2 x 4 mm PTFE Bowden tube pressed into a 4 mm hole with both ends countersunk. Not a long tube, which would bend the cable over its sharp end, and
   not a bare printed hole or a raw metal edge, which wear or fray the line. With a friction coefficient
   around 0.1 the ring changes the spring torque by about +-3.5 % depending on the direction of motion, at
   most about 6 mNm, well below the servo friction.
3. **Upper arm bar (link2): spring pin.** A 3 mm pin on the outer face at 101 mm from the J2 axis, standing
   out to the cable plane plus 2.5 mm, for the spring's fixed eyelet. The spring lies along the bar's outer
   face with its moving end toward J2. At the zero pose the federnshop springs are 38-52 mm long
   pin-to-pin; they stretch to at most 75 mm at 135 deg tilt (room: 77 mm).
4. **Cable:** about 0.5 mm braided line (Dyneema, or fishing line). Set its length so that with the upper
   arm vertical the spring is stretched by its pre-stretch (federnshop: Z-075NI 1.5 mm, Z-078AI 2.5 mm,
   Z-078I 3.1 mm, Z-077AI 3.9 mm). Then the spring never goes slack and its torque is exactly
   k * 13 * 20 * sin(J2).

## Springs Online springs on this mount

With the cable plane 4.8 mm outside the bar face, a spring wider than about 7 mm rubs the bar. Only three
Springs Online springs pass every check on this mount (rate, never slack, length at 135 deg, rated stretch
and force): SOTRI0057 (stainless, 0.88 N/mm, 101 %), SOTRI0069 (stainless, 0.98 N/mm, 112 %) and SOTR0057
(carbon, 1.06 N/mm, 121 %), pre-stretch 1.6 / 1.2 / 1.7 mm. The softer Springs Online springs are 9-12 mm
wide: they need the Springs Online mount in brackets above (a = 16, b = 27, cable plane 6.8 mm out).

## Lee Spring springs on this mount (sold as single pieces)

Lee Spring Germany (leespring.de) sells from one spring: EUR 15 service fee on orders below EUR 50 net, shipping
extra outside Germany (prices checked in November 2026). The LEM080C series is 0.79 mm wire, 6.3 mm outside, initial
tension 3.25 N; it fits this mount as it is.

| Spring | Material | L0 | Rate | Balance | Pre-stretch | Within Lee's rating | Price |
|---|---|---:|---:|---:|---:|---|---:|
| LEM080C 05 M | music wire | 30 mm | 1.028 N/mm | 118 % | 3.8 mm | up to J2 +-90 deg | EUR 3.19 |
| LEM080C 06 M | music wire | 35 mm | 0.821 N/mm | 94 % | 3.0 mm | up to J2 +-128 deg | EUR 3.19 |
| LEM080C 06 S | stainless | 35 mm | 0.685 N/mm | 79 % | 3.0 mm | up to J2 +-128 deg | EUR 3.57 |
| LEM080C 07 M | music wire | 40 mm | 0.683 N/mm | 78 % | 2.3 mm | whole range | EUR 3.37 |
| LEM080C 08 M | music wire | 45 mm | 0.585 N/mm | 67 % | 1.5 mm | whole range | EUR 3.37 |

Lee's maximum load (24.5 N, stainless 20.4 N) is lower than federnshop's 34.5 N for the same size. The upper arm
only goes past horizontal (90 deg) when reaching down, so 05 M is still fine for testing.

## The AliExpress springs (ordered November 2026)

0.8 mm stainless wire, from [this listing](https://www.aliexpress.com/item/1005004052186320.html) (packs of 5, 6 to 10 mm
outside). The listing gives no rate, so the rates below are estimates for stainless from wire, diameter and length; the
ratings are those of a comparable stainless spring (Lee Spring). Measure them before fitting (below).

| Spring | Packs | Rate (estimate) | Balance | Pre-stretch at vertical | At the end of J2's range |
|---|---:|---:|---:|---:|---|
| 0.8 x 6 x 35 mm, the main one | 2 | 0.79 N/mm | 91 % | about 3.2 mm | over that rating beyond J2 +-100 deg |
| 0.8 x 6 x 40 mm, softer | 1 | 0.66 N/mm | 76 % | about 2.5 mm | within rating |
| 0.8 x 7 x 30 mm, backup | 1 | 0.64 N/mm | 74 % | about 3.1 mm | over that rating beyond J2 +-116 deg |

- **Pre-stretch** assumes an initial tension of about 3 N (2.5 N for the 7 mm). With the measured values it is
  7 - initial tension / rate, in mm: set the cable so the spring is stretched by that much with the upper arm vertical.
- **The 7 mm spring is close to the bar.** The spring hangs on the cable plane, 4.8 mm outside the bar face, so a 7 mm
  spring leaves about 1.3 mm to the bar (6 mm: 1.8 mm). The viewer does not check this gap. If you redesign the mount,
  making the post, the eyelet standoff and the spring pin 1 mm longer (cable plane at CS2 z = -34) gives either spring
  1 mm more.
- **In the firmware,** start with `k` = 790, 660 or 640 N/m, then the measured rate, then `calib_spring` on the arm.

## Any other spring (eBay, AliExpress, a hardware store)

A spring works on this mount when:

1. It is at most 7 mm wide (outside diameter). 0.8 mm wire and 6 mm outside is the size to search for.
2. Its rate is 0.55 to 1.05 N/mm. 0.87 N/mm cancels the J2-only load exactly.
3. Its initial tension divided by its rate is under about 5.5 mm, so it is never slack near vertical.
4. Free length + 30 mm - (initial tension / rate) is at most 77 mm (the room at the end of J2's range).
5. It takes rate x 30 mm (about 26 N at 0.87 N/mm) without staying longer afterwards.

Rough guide for 0.8 mm spring steel, 5.8 to 6.3 mm outside, free length measured inside the hooks (from the
federnshop and Lee Spring catalogues): 30-34 mm is about 1.0 N/mm, 35-40 mm about 0.8, 40-45 mm about 0.7,
45-50 mm about 0.6. Stainless is about 15 % softer. 0.7 mm wire (estimate for 0.7 x 6 mm): 30 mm is about 0.5-0.56
N/mm (56-64 %), 35 mm about 0.4-0.45, 40-45 mm about 0.3-0.37; it carries only about 16-18 N, so no 0.7 mm
spring can balance more than about 65 % over the whole J2 range. Listings often give the length
over the hooks, and the hooks vary, so measure every spring before fitting it:

- Hang a 0.5 l water bottle (4.9 N) on it and measure the length L1, then a 1.5 l bottle (14.7 N) and measure
  L2. Rate = 9.8 / (L2 - L1) N/mm.
- Initial tension = 4.9 - rate x (L1 - free length). If the 0.5 l bottle does not open the coils at all,
  the initial tension is too high for this mount.
- Enter the rate in the firmware (`spring` command, `k` in N/m) and let `calib_spring` correct it on the arm.

## Clearances in the model (whole J2 -105..130 and J3 -75..150 range, wrist varied)

| Pair | Clearance |
|---|---|
| crossing plate / upper arm's rounded end (J2 near -95 deg) | 1.5 mm |
| eyelet / post top (arm vertical) | 2.0 mm |
| post / bar face | 2.8 mm |
| cable / upper arm | 4.3 mm |
| everything else | 8 mm or more |

These are vertex-based estimates from the STL meshes. Print a test piece and check the plate gap first.

## Joint ranges (estimated from the CAD, also in RAFT.urdf / RAFT.xml)

J2 -105..+130 deg, J3 -75..+150 deg (at J2 = 0). Below J3 = -80 deg the wrist runs into the shoulder
bracket and the base. With the upper arm lying back (J2 near -82 deg) the forearm folds further: the rest pose is
J2 -82, J3 -80 deg.

## Firmware

`spring.type = ZeroLengthCable`, `spring.k` = rate in N/m (AliExpress estimates 790, 660 or 640 until measured; federnshop 565, 718, 824 or 1027; Lee Spring 1028, 821, 685, 683 or 585), `spring.a = 0.013`,
`spring.b = 0.020` (Springs Online: 0.016 / 0.027), then trim `spring.scale` on the bench.
