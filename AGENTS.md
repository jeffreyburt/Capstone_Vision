# Working on this shared robot repository

Read `docs/HANDOFF.md` first, then the document for the subsystem being changed. Current mixed
Robot 1 hardware is documented in `docs/FOUR_WHEEL_DRIVE.md`; older dated runbooks may be superseded.

- Inspect Git status and live service/process definitions before editing or deploying. Preserve
  other workers' changes and do not commit unrelated work. Fetch/rebase before pushing shared main.
- Maintain `docs/HANDOFF.md` and subsystem documentation alongside changes, including validation,
  deployment state, calibration assumptions, unresolved hardware checks, and rollback instructions.
- Preserve measured calibration in `calib/` and its raw evidence. Do not invent speed ranges,
  remove calibration guards, or claim floor/direction validation from software tests.
- Keep the Mac's Internet Wi-Fi unchanged; reach robot Pis through the existing main-Pi SSH route.
- One process owns chassis output on each robot. Disable dashboard driving and stop the sole
  laptop controller before drive deployment. Stop a robot's drive service before direct GPIO
  diagnostic tools. Coordinate hardware tests across agents; do not run them concurrently.
- Retain physical Hiwonder M1=0 on Robot 1. Its logical M1 is the calibrated TB6612 replacement.
- Run meaningful tests for motion/control changes. Verify zero commands and status before any
  physical test. Physical floor/wheel status in the active user session takes precedence.
- Commit and push authorized changes; keep credentials/private keys out of repository files.
- When a physical test went wrong, read its control black box first (`python tools/blackbox.py summary`,
  then `events` / `window`; see `docs/BLACKBOX.md`) instead of reasoning from chat descriptions alone.
