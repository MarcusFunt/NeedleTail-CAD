# Checklist: Collect Feature Branch Work onto Main

Main baseline: PR #3 merge `41f0fb6b98ac547c43445f7e0afadcb51be1c1d8`.
Source commits: `347b2bb` geometry candidate and `dc799ca` exploratory SU2 workflow.

## PR 1 — Revised geometry candidate

- [x] Create a fresh branch from current main; keep the stale source branch untouched as a reference.
- [x] Record baseline behavior and verify the current merged-main geometry remains the default.
- [x] Port the airfoil data/catalog, candidate geometry, candidate comparison tools, and design notes from `347b2bb`.
- [x] Keep the candidate in its own named parameter configuration with the source branch defaults; do not retune or promote it.
- [x] Adapt CAD, AeroSandbox, motor/pod, and propeller-envelope consumers to select baseline or candidate explicitly.
- [x] Preserve PR #3 optimizer defaults and named mass scenarios; record geometry identity in outputs.
- [x] Add/adjust geometry and adapter acceptance checks for coordinate alignment, section thickness, caps, envelopes, and propeller clearances.
- [x] Update README and design notes to distinguish baseline from candidate and document model limits.
- [x] Run the locked default and AeroSandbox verification; PR #5 was reviewed/CI-checked and merged to main.

## PR 2 — Exploratory CFD workflow

- [x] Branch from updated main after PR 1 merges; port `dc799ca`'s CFD workflow as a separate optional feature.
- [x] Keep Gmsh/SU2 requirements out of default test discovery and add CFD unit/config tests to CI without CFD extras.
- [x] Verify solver/source/geometry/config hashes, safe output behavior, and failure/resource-stop reporting.
- [x] Add bounded five-zone mesh/boundary checks, force normalization, torque/power sign, and run-completeness checks.
- [x] Preserve the source branch's operating point, mesh/time-step sequence, thread, runtime, mesh, disk, and RAM defaults for later use.
- [ ] Run preflight and a short solver compatibility case; do not run the full three-case, 54-solver-hour campaign. Deferred: Gmsh is absent and only 1.44 GiB RAM was available, too close to the 0.8 GiB stop floor for mesh generation.
- [ ] Open and merge the focused PR after review/CI.

## Mainline completion

- [ ] Audit all 33 source-branch changed paths against merged main; document any intentionally omitted item.
- [ ] Confirm baseline remains default, revised geometry remains an explicit candidate, and CFD remains optional.
- [ ] Confirm both integration PRs are merged to main.
- [ ] Stop at a collected, verified mainline state; leave topology/hardware choices and detailed CFD for the user's next decision.
