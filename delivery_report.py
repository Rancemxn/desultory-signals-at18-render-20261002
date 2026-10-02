"""Attach the exact reviewed plan's sampled checks to a media delivery report."""
import json
from pathlib import Path


def attach_plan_review(report, review_path):
    review = json.loads(Path(review_path).read_text(encoding='utf-8'))
    for bake in report['bakes']:
        for key in ('plan_sha256', 'psap_sha256'):
            if bake[key] != review[key]:
                raise ValueError(f'Delivery bake does not match reviewed plan: {key}')
    report['planned_touch_validation'] = review
    report['native_ap_validated'] = False
    return report


def delivery_notes(report):
    quality = report['baked_pose_diagnostics']
    review = report['planned_touch_validation']
    blocks = review['block_area_geometry']
    return (
        '# Desultory Signals AT 18\n\n'
        'Full synthetic handcam video: **1920 × 1080, 60 FPS, 9045 frames, '
        '150.750 seconds**, with continuous music and hitsounds. '
        'Full FFmpeg decode and video/audio stream checks passed.\n\n'
        'The original chart and all **2026 note identities** are retained. '
        'Fingering was refined after inspecting baked poses, including recurring '
        'Flick/Drag strokes, centre handoffs, stable left/right chord assignments, '
        'held-note exchanges and legal spacing along judgement strips.\n\n'
        f"All **{blocks['areas']} `blockAreaList` entries** from Phigros 4.0.1 are "
        'used by both planning and rendering. Movement, scaling, rotation, native '
        'easing, activation times and subtract-coverage parity are supported. '
        'The material reconstruction includes red noise, edges, glow, readiness '
        'and disabled effects; it is not a pixel-identical Unity capture.\n\n'
        f"The reviewed plan has **{blocks['violations']} sampled block violations** "
        f"at {blocks['sample_step_ms']} ms spacing, with event boundaries also checked. "
        'All-note judgement-strip and touch-lifecycle checks passed. These are '
        'sampled geometric checks, not a continuous-time proof or a native game run.\n\n'
        '**3D pose quality remains imperfect:** '
        f"maximum pad error {quality['max_error_mm']:.3f} mm; "
        f"{quality['contact_errors_over_1mm']} contact samples exceed 1 mm; "
        f"{quality['collision_samples']} unresolved collision samples; "
        f"minimum screen clearance {quality['min_screen_clearance_mm']:.3f} mm. "
        'Visible fit errors and intersections remain. Passing the planned-touch '
        'checks does not establish valid rendered skin contact. '
        '**Native Phigros AP validation has not been performed.**\n\n'
        'The seven video joins are included in `final-review.jpg`; measured '
        'bone-tail steps are in `validation.json`. These measurements include '
        'normal motion and do not certify pose continuity. Full per-part pose '
        'diagnostics are preserved in `pose-diagnostics.json`.\n\n'
        f"SHA-256: `{report['sha256']}`\n"
    )
