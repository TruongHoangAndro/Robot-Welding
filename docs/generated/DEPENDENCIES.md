# Installed dependencies

Every install made by the agent. apt installs are mirrored in `docker/Dockerfile`.

| Date | Where | Package | Version | Why |
|---|---|---|---|---|
| 2026-10-19 | awm_dev (apt, root) | python3-jsonschema | 4.10.3-2ubuntu1 | JSON schema validation in awm_core |
| 2026-10-19 | conda env awm_occ (mamba, conda-forge) | openpyxl | 3.1.5 | Export of DES-003 xlsx to markdown |
