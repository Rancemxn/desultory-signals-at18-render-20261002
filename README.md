# Desultory Signals AT18 handcam render

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
