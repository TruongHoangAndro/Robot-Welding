# awm_seam (M2, A21 / A22 / A24)

Seam definition on the edge catalogue (no OCC needed): pick edges by id or by a 3D point (a UI
click), validate the chain (connected; concave root for fillets), attributes (joint type, weld
side, leg, sequence, process hint, material, thickness), optional partial seam (A22 start / end
picks -> `range_m`), export `seams.json` (D04) with fingerprints, re-match after a STEP revision.

```bash
ros2 run awm_seam seam define --job <job> --seam-id S01 --pick 0,0.005,0.010 --joint-type fillet \
    --leg-mm 4 --sequence 1 --material S235JR --thickness-mm 10 --process-hint MIG
ros2 run awm_seam seam define ... --start-pick x,y,z --end-pick x,y,z [--reversed]
ros2 run awm_seam seam from-picks --job <job> --picks picks.json [--replace]
ros2 run awm_seam seam list|remove|rematch ...
```
Weld direction: from the free end of the first picked edge; `--reversed` flips it.
Automatic detection (A23, Phase 2) will produce the same `SeamRequest` objects.
