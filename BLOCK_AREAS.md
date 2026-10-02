# Phigros 4.0.1 block areas

This isolated Desultory Signals export reads all 160 `blockAreaList` entries.
The planner and renderer share the geometry in `block_area.py`; timestamps and
source note IDs are preserved. These changes are confined to this export snapshot.

## Native evidence

The user supplied Phigros version 4.0.1, Android versionCode 157. Read-only
inspection used its ARM64 IL2CPP library and plain version-31 metadata. Relevant
addresses, relative to image base zero:

| Native method | Address | Observed rule |
| --- | --- | --- |
| PreviewBlockControl.UpdateBlockAnimations | `0x1CDB864` | Scale, rotate, then add movement relative to the original center |
| UpdateScale / UpdateRotation | `0x1CDBBF0` / `0x1CDBF6C` | Completed steps use the previous event's anchor; anchors do not interpolate |
| InterpolateMoveEvent | `0x1CDC5A4` | Percentage coordinates, separate X/Y easing |
| IsActive | `0x1CDCAB8` | `enableTime <= t < disableTime` |
| GetEase.Instantiation / GetEaseWithProgress | `0x1CC95E4` / `0x1CC92FC` | 101-point float tables, linearly interpolated |
| JudgeControl.TryGetBlockingBlock | `0x1D735D8` | Normal union XOR subtract coverage parity, evaluated twice |
| TryGetBlockTouchHalfSize | `0x1D73D08` | Shrink normal rectangles, expand subtract rectangles, then intersect with the original mask |
| BlockRender.Start | `0x1D6DBE4` | Mask size 1/8, effect size 1/4, scene-color capture 1/6 |

The actual easing function differs from the similarly named TweenInfo enum:
0 is linear; 1–3 quadratic in/out/in-out; 4–6 cubic; 7–9 quartic; 10–12 quintic;
13 is constant zero; 14 is constant one. The current event selects the easing.

Coordinates and anchors are percentages of the screen with the native origin at
bottom left. The implementation converts the final rotated rectangle to the
renderer/planner's top-left origin. Scale changes alter both the dimensions and
the center around their anchor; rotation applies after that center calculation.

The level12 serialized JudgeControl component sets touch inset to **3% of screen
height**, with a cap of **0.25 in rectangle-local coordinates**. This overrides
the constructor's 5% default. Let N be the union of normal rectangles and S the
odd-coverage mask of subtract rectangles. The touch mask is
`(N XOR S) intersect (N_shrunk XOR S_expanded)`.

## Planning and validation

Algo5 removes the forbidden mask before choosing a landing point. It checks
interpolated paths, including Tap dwell and the early portion of Flicks, at 1 ms
spacing and immediately around every block event. This is a hard constraint;
`allow_degraded` does not relax it. Charts without block areas keep the previous
behavior, including lightweight synthetic chart objects used by the tests.

`block_motion.py` adjusts the authored export paths along legal judgement strips,
retains contact times and note IDs, and adds clearance for the interval between
the last MOVE and UP. Moving Holds use denser samples. `refine_plan.py` then checks
all 2026 notes and Hold coverage, performs the block audit again, and saves the
results in `manual-review.json`.

The reviewed-v6 export has 1632 contact segments. Its 1 ms block audit and all-note
judge-strip audit pass. These are geometric, sampled checks, not a proof over
continuous time or an execution inside the native Phigros judgement engine.

## Rendering

The SkSL material paths reproduce the idle active, ready and disabled effects:
moving noise displacement, pixelated edges, red fill, sparks, sampled scene-color
distortion, glow and readiness shine. The two noise textures were extracted from
the supplied APK. The renderer retains the native reduced mask/effect sizes.

An isolated offscreen OpenGL context runs only the block passes. Windows uses WGL;
Linux uses EGL and can run on Mesa's software renderer without a physical GPU.
The rest of the existing chart renderer and encoder continues to use raster
frames. If context creation is unavailable, the log reports a slower raster
fallback. The optional API switch `use_gpu=False` supports CPU comparisons.

This is a reconstruction, not a pixel-identical Unity capture: platform color
spaces, rasterization, disabled fade details and texture sampling can differ.
Blocked-finger hover/shine feedback is not synthesized for the validated autoplay
input. The original APK is unchanged and is not included in the public snapshot.

## Export limitations

All eight parts of bake run `37019323139` completed and their inspection sheets
were viewed. Block effects are present, including the shrinking/rotating allowed
windows. The 3D solver still has visible fit/intersection limitations: maximum
recorded contact error is 85.581 mm, and some samples penetrate the screen or
other fingers. A valid planned touch path is not evidence that every rendered
skin contact is valid. The delivery reports retain these diagnostics, and native
AP validation remains false.
