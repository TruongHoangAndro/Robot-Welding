# TST-002 G2 (C1 scene sanity) — PASS

Record: `g2_log.json` (commit, input hashes, scene hash, backends, head model).

- colliding invalid: 10 / 10
- free valid: 10 / 10
- contact-zone invalid: 5 / 5
- expected object in a contact pair: 13 / 13 (reported, not a criterion)
- failing: none; too few poses: none

| Pose | Kind | θ deg | Label | Expect | Result | Caught by | Contact pairs (F4) | 5 mm clear | Note |
|---|---|---|---|---|---|---|---|---|---|
| F01 | free | 0 | S-E+W- | valid | valid PASS | - | - | yes | SRDF group state home of laser_arm (beam down, high above the cell) |
| F02 | free | 0 | S-E+W- | valid | valid PASS | - | - | yes | J2 contact pose phi 30 |
| F03 | free | 0 | S-E+W- | valid | valid PASS | - | - | yes | J2 contact pose phi 60 (tool X turned 270 deg about the beam by the strict rule) |
| F04 | free | 0 | S-E+W- | valid | valid PASS | - | - | yes | J2 contact pose phi 0 |
| F05 | free | 0 | S-E+W- | valid | valid PASS | - | - | yes | J2 approach pose phi 30, tcp 40 mm out along seam Z |
| F06 | free | 0 | S-E+W- | valid | valid PASS | - | - | yes | J2 contact pose phi 45, start psi 180 (conduit toward the robot): not free (runs 1-2: head-arm self-collision shoulder_link<->conduit; no strict-rule solution at psi 180), so no free pose with the conduit on the arm side is in the set (tool X turned 90 deg about the beam by the strict rule) |
| F07 | free | 0 | S-E+W- | valid | valid PASS | - | - | yes | J2 contact pose phi 15 |
| F09 | free | 20 | S-E+W- | valid | valid PASS | - | - | yes | J2 contact pose phi 30, positioner at 20 deg (part, holders and robot target follow the angle) |
| F08 | free | 0 | S-E+W- | valid | valid PASS | - | - | yes | tcp 150 mm above the J2 centre, beam down |
| F10 | free | 0 | S+E-W+ | valid | valid PASS | - | - | yes | beam down in free space in front of the jig (y -601); twin of C09 at positioner 0 deg |
| C01 | colliding | 0 | S-E+W- | invalid | invalid PASS | validity | part<->nozzle_body; part<->nozzle_tip | no | J2 contact pose phi 30 pushed 3 mm into the tube (nozzle_tip 2.7 mm into the part) |
| C02 | colliding | 0 | S-E+W- | invalid | invalid PASS | validity | htpk20s5m150_a_c15_1<->nozzle_body; htpk20s5m150_a_c15_1<->nozzle_tip; jig_1<->nozzle_body; jig_1<->nozzle_tip … | no | tcp 10 mm inside the jig (highest inside point of the jig mesh on a 10 mm grid, minus 10 mm), beam down |
| C03 | colliding | 0 | S-E+W- | invalid | invalid PASS | validity | htpk20s5m150_a_c15_1<->nozzle_body; jig_1<->nozzle_body; jig_1<->nozzle_tip; part1_1<->nozzle_body | no | tcp inside the rotating holder Part1 (on the positioner axis, x -156.7), beam down |
| C04 | colliding | 0 | S+E+W- | invalid | invalid PASS | validity | part2_1<->nozzle_body; part2_1<->nozzle_tip | no | J2 +X side (preview frame 2) approach 40 mm out: nozzle inside the rotating holder plate Part2 |
| C05 | colliding | 0 | S+E-W+ | invalid | invalid PASS | validity | part2_1<->nozzle_body; part2_1<->nozzle_tip; part3_1<->nozzle_body; part<->nozzle_body | no | tcp inside the rotating holder Part3 (clamps the elbow end), beam down |
| C06 | colliding | 0 | S-E+W+ | invalid | invalid PASS | validity | c_2500001_1<->nozzle_body | no | tcp 15 mm below the Pismo plate top (lower frame hull), beam down |
| C07 | colliding | 0 | S-E+W- | invalid | invalid PASS | validity | c_60cm30x_3d_1<->nozzle_body; htpk20s5m150_a_c15_1<->head_body; jig_1<->head_body; part1_1<->head_body … | no | tcp inside the positioner motor, beam down |
| C08 | colliding | 0 | S+E-W+ | invalid | invalid PASS | validity | htpk20s5m150_a_c15_4<->nozzle_body; jig_1<->nozzle_body; part2_1<->head_body; part3_1<->head_body … | no | tcp inside the rotating pulley :4 on the part axis (x 93), beam down |
| C09 | colliding | 90 | S+E-W+ | invalid | invalid PASS | validity | part3_1<->nozzle_body; part3_1<->nozzle_tip; part<->nozzle_body; part<->nozzle_tip | no | same robot target as F10 with the positioner at 90 deg: the elbow end and holder Part3 have turned onto the nozzle |
| C10 | colliding | 0 | S+E+W+ | invalid | invalid PASS | validity | c_2500001_1<->upper_arm_link; c_2500001_1<->wrist_3_link; part5_1<->upper_arm_link; upper_arm_link<->base_link_inertia | no | arm pointing down (shoulder lift +90 deg): upper arm through the pedestal / Pismo plate (arm links, not the head) |
| Z1 | contact_zone | 0 | S-E+W- | invalid | invalid PASS | validity, contact:contact_off_seam | part<->nozzle_tip | no | tip 0.5 mm into a non-seam face: top of the solid tube F28, x -75 (J2 contact check), head across Y so only the nozzle is near the part |
| Z2 | contact_zone | 0 | S-E+W- | invalid | invalid PASS | validity, contact:other_face_within_standoff | part3_1<->nozzle_tip | no | rim into a corner wall: tcp on the elbow top (J2 n2 face F35, highest mesh vertex at x 33.5..34.5, beam = -local normal) next to the wall of holder Part3; nozzle_tip end disc 1.06 mm inside the holder (bounded by the 1.8 mm lip of the holder hull above the elbow top); the rim also passes the elbow end edge (F36) (tool X turned 90 deg about the beam by the strict rule) |
| Z3 | contact_zone | 0 | S+E-W+ | invalid | invalid PASS | validity | c_2500001_1<->nozzle_tip | no | flat tip tilted 8 deg on a flat face: Pismo plate top (z -89.0, bisection of the hull), beam tilted about world X; no flat part face is reachable (end faces are inside holders) |
| Z4 | contact_zone | 0 | S-E+W- | invalid | invalid PASS | validity | part<->nozzle_tip | no | flat tip tilted 8 deg on the J2 seam (phi 30) about the seam tangent: the rim dips into the straight tube direction (seam faces, contact check passes; F4 must catch it) |
| Z5 | contact_zone | 0 | S+E-W+ | invalid | invalid PASS | contact:other_face_within_standoff | - | yes | F9 rule (b) on part faces: J1 contact pose at phi 80 (circle through the JE2 preview frames; first of phi 160, 150, 170, 70, 80, 190, 50, 60, 180 with a strict-rule solution); nipple thread flanks ['F17', 'F19'] lie 0.290 mm from the nozzle disc (< s_c 0.3 mm) -> other_face_within_standoff; head otherwise clear, so F4 is expected valid and only F9 catches it |

## Notes

- F9 rule (a) (contact point off the seam faces) is exercised by Z1. Rule (b) (other face within s_c) is exercised on part faces by Z5 (J1 thread flanks F17 / F19 at 0.290 mm vs s_c 0.3 mm: margin only about 0.01 mm and dependent on the 0.5 mm chord of the thread faces; J1 points with about 0.15 mm margin (phi 150-190) have no clean head / arm solution) and by Z2 together with F4 (elbow end edge F36). Robust coverage of rule (b) is by the unit tests test_core_contact.py::test_rim_into_corner_wall_fails, ::test_wall_gap_against_standoff, ::test_wall_meeting_only_one_seam_face_is_not_excused_by_the_seam_edge_rule; rule (a): ::test_tcp_inside_a_non_seam_face_fails_off_seam, ::test_tcp_above_the_seam_within_and_beyond_tolerance.
- The golden part has no reachable flat face and no concave corner wall large enough for the rim: Z3 uses the Pismo plate top, Z2 the holder Part3 lip (about 1.8 mm above the elbow top), Z5 the thread grooves next to J1.
