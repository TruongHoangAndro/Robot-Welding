# awm_process (M3, A31-A33)

Process per seam (drawing hint, else configured default), WPS lookup in `<repo>/cell/wps/*.json`
(process, material, joint type, thickness and leg ranges; APPROVED > DRAFT > SAMPLE), parameters
copied into `process.json` (D06) with the WPS hashes. SAMPLE WPS are refused unless
`--allow-sample-wps` (simulation / development).

```bash
ros2 run awm_process assign --job <job> [--default-process MIG] [--allow-sample-wps]
```
Exit 2 when a seam has no WPS (listed with the reason in `process.json`).
The SAMPLE WPS files live in `awm_bringup/repo_template/cell/wps/` (not qualified WPS).
