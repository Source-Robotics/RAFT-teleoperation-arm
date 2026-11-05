# Safety Warning & Disclaimer
## Definitions

For the purposes of this Safety Warning & Disclaimer, “PROJECT” refers to all materials, files, designs, models, simulations, URDF/MJCF files, control software, firmware, documentation, and any modifications or derivatives provided as part of this robotic arm project.

# Safety Warning

THIS PROJECT INVOLVES ROBOTIC SYSTEMS WITH MOVING MECHANICAL PARTS, ACTUATORS, MOTORS, ELECTRICAL POWER SYSTEMS, AND SOFTWARE-CONTROLLED MOTION. IMPROPER DESIGN, ASSEMBLY, CONFIGURATION, OR OPERATION CAN RESULT IN UNEXPECTED MOVEMENT, PINCH POINTS, CRUSH HAZARDS, COLLISIONS, ELECTRICAL SHOCK, PROPERTY DAMAGE, SERIOUS INJURY, OR DEATH.

Robotic arms can generate significant forces and move at high speeds. Even small systems may cause injury if safety precautions are not followed. Simulation results may not accurately represent real-world behavior, and transferring control software from simulation to physical hardware introduces additional risks.

ONLY QUALIFIED INDIVIDUALS WITH APPROPRIATE TRAINING AND EXPERIENCE IN ROBOTICS, MECHANICAL SYSTEMS, AND ELECTRICAL SYSTEMS SHOULD BUILD, MODIFY, OR OPERATE ANY PHYSICAL IMPLEMENTATION OF THIS PROJECT. ALWAYS FOLLOW APPLICABLE SAFETY STANDARDS, LOCAL REGULATIONS, AND BEST PRACTICES FOR ROBOTIC SYSTEMS. USE APPROPRIATE SAFETY MEASURES INCLUDING, BUT NOT LIMITED TO, EMERGENCY STOPS, PHYSICAL GUARDS, SOFTWARE LIMITS, AND PERSONAL PROTECTIVE EQUIPMENT.

IF YOU DO NOT FULLY UNDERSTAND THE RISKS ASSOCIATED WITH ROBOTIC SYSTEMS, DO NOT USE, BUILD, OR DEPLOY THIS PROJECT ON PHYSICAL HARDWARE.

# RAFT hazards

RAFT is a small arm, but it is driven by servos under software control and it commands a larger robot. The
[bring-up guide](firmware/docs/bringup.md#0-safety-rules) has the rules for each step; these apply always.

- **The servo supply switch is the only hard stop.** Keep it within reach whenever the servos are powered. RAFT has
  no physical E-stop button. LIMP in the web UI, Esc on the page and `mode limp` on the console switch torque off in
  software only.
- **The arm moves by itself.** Align, Hold, the moves to a pose and replays drive the joints without a hand on the
  arm. Keep fingers out of the trigger hinge and the elbow while the arm is under torque.
- **The arm drops when torque goes off.** LIMP, every fault and a reset cut torque at once. Support the arm when
  torque goes on or off, and clamp the base to the table.
- **Teleoperation moves PAR6.** PAR6 is a larger and stronger robot that follows the hand. Run it in a clear area,
  with its own E-stop within reach of you or a second person. A software or network fault on either side can make
  it move in a way you did not intend.
- **Electrical.** The servos run from a 5 to 7 V supply of 1.5 A or more; use leads rated for it and a
  supply with a current limit if you can. The ESP32-PoE without -ISO is not isolated: never connect USB while it is
  powered over Ethernet (PoE), and with PoE the servo supply's output must be floating. Plug and unplug servos
  only with the supply off.
- **No login.** The web UI and TCP port 5000 have no password: anyone on the same network can move the arm and,
  through it, PAR6. Use it on a network you trust.

# Disclaimer

THE PROJECT IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.

BY USING THIS PROJECT, YOU ACKNOWLEDGE AND ACCEPT ALL RISKS ASSOCIATED WITH ITS USE, INCLUDING RISKS ARISING FROM SOFTWARE ERRORS, SIMULATION INACCURACIES, CONTROL FAILURES, HARDWARE MALFUNCTIONS, OR IMPROPER CONFIGURATION.

IN NO EVENT SHALL THE AUTHORS, CONTRIBUTORS, OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGE, INJURY, DEATH, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION WITH THE PROJECT OR THE USE OR OTHER DEALINGS IN THE PROJECT.

ANY ASSEMBLY, MODIFICATION, DEPLOYMENT, OR OPERATION OF THIS PROJECT — WHETHER IN SIMULATION OR ON PHYSICAL HARDWARE — IS PERFORMED ENTIRELY AT YOUR OWN RISK.