# Desultory Signals AT18 handcam render

The current batch uses `batch-chart.yml` and chart-independent contact rules.
`batch_queue.py` reserves one running workflow for each of two queues:

- `index2`: `EntrancetotheChaos-IN-index2`, then `ExoplanetaryMirage-IN-index2`.
  Both select the IN chart and allow only the left and right index fingers.
- `original`: the original seven chart variants, using their configured IN/AT
  difficulties and the normal finger set.

Each workflow plans the complete song, bakes eight parts in parallel with shared
initialization history, checks all seven joins, renders eight parts in parallel,
and delivers a complete 1920×1080 60 FPS video. Variant names distinguish the
index-only videos from earlier unrestricted deliveries. Assignment and final
validation both enforce the allowed finger set. `validation.json` records the
fingers actually used, geometry checks, timing adjustments and provenance.

The following notes describe the earlier Desultory-specific delivery:

Temporary isolated rendering task for a 150.75-second, 2026-note chart.
The source snapshot is recorded in `source-manifest.json`.

Baseline planning and baking are followed by manual review and task-specific fingering edits.
Final delivery target: 1920×1080, 60 FPS, 9045 frames, continuous music and hitsounds.
Large source media, animation and video intermediates remain on GitHub.

This snapshot supports Phigros 4.0.1's `blockAreaList` in both planning and rendering.
See [BLOCK_AREAS.md](BLOCK_AREAS.md) for the APK evidence, coordinate/easing rules,
subtract parity, material reconstruction, and limits of the sampled checks.
All 160 areas and 2026 note identities are retained. The reviewed plan passes
touch lifecycle, all-note judgement-strip and 1 ms block checks. Its hashes and
check results are pinned in [reviewed-plan-validation.json](reviewed-plan-validation.json).

Eight inspected baked parts are rendered without rebaking. The final workflow
verifies each video, joins the video streams, encodes a continuous audio mix,
checks all 9045 frames with FFmpeg and generates an inspection sheet including
both sides of the seven joins. Delivery reports retain source/plan provenance,
sampled touch checks, actual pose residuals and the final video SHA-256.

This is a synthetic handcam animation with remaining pad-fit errors and hand
intersections. A valid planned touch does not guarantee valid rendered skin
contact. Native Phigros scoring is not validated.
Source code retains its original WTFPL license. Media belongs to its respective authors.

# Load-only variants

`EntrancetotheChaos-IN-index2` and `ExoplanetaryMirage-IN-index2` use only the
two index fingers and the fatigue, repeated-press and hand-load objective
throughout initial planning and final assignment. Both use eight parallel
bake workers and eight render workers at 1080p60.

For these variants, movement distance and pose preferences have zero scoring
weight, and finger/contact/wrist speed and acceleration limits are disabled.
Contact occupancy, note coverage, blocked areas and simultaneous finger order
remain validated. Reports record `fingering_objective: load` and
`speed_limits: false`; disabling speed checks does not establish physical
playability or native game AP.

Both index variants allow bounded judgment-time adjustments and continuous
Tap/Drag/Flick sweeps. Per-note offsets and a 60 Hz phase-sampling check are
reported against the documented PhiZone Player reference model. Original
chart timing and Hold tails are preserved; native Phigros AP is not established.
