# WP-S S.5b — J2 seam bins that no positioner angle can open (head ↔ rotating set)

Record `s5b_mode_b_rotating_set_check.json`; commit `c4ad92f4b6caaf5a60416cdce021c4127700f1e2`, dirty: false; from the screening record `s5b_mode_b_screening.json` (commit `74f49e66a92e06c1dc4d7b752f476c6038b8df5f`). Head links nozzle_tip, nozzle_body, head_body, head_mount, conduit; rotating set (parent frame positioner_flange, Assumption I3 / OI-27) htpk20s5m150_a_c15_1, htpk20s5m150_a_c15_4, part, part1_1, part2_1, part3_1; bins 1 mm.

Method: Which seam bins no positioner angle can open (Mode B). With work / travel angle fixed, the head pose relative to the part and to every object that turns with it depends only on (s, ψ), not on θ or the IK branch. A contact between a head link and the rotating set is therefore the same at every θ. ``rows`` = (s_mm, ψ_deg, kinematic reason, F4 contacts) of the candidates of all screened θ. A bin is ``blocked`` when every ψ of ``psi_all`` was kinematically valid somewhere in the bin at some θ and every such candidate had a head ↔ rotating-set contact (F4, 3 mm padding). ψ never kinematically valid in the bin are listed (``psi_unseen``): for them the screening has no evidence and the bin is not marked blocked.

**Ψ-FREE:** blocked at every θ: s 3–39.0 mm (36.0 mm = 43.0 % of the seam). Pairs: part<->nozzle_tip ×499104, part2_1<->head_body ×342936, part<->nozzle_body ×337176, part2_1<->nozzle_body ×214560, part3_1<->head_body ×194760, part<->head_body ×135232.

**Ψ-WIRE:** blocked at every θ: s 3–39.0 mm (36.0 mm = 43.0 % of the seam). Pairs: part<->nozzle_tip ×86856, part2_1<->head_body ×61362, part<->nozzle_body ×58786, part2_1<->nozzle_body ×37576, part3_1<->head_body ×7764.

