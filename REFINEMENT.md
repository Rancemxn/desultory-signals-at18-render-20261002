# Desultory Signals contact and pose refinement

This revision addresses the reported block mismatch near 79 seconds, excessive
held-contact motion, finger crossing, idle-finger twitching, and abrupt wrist
movement. Work stays in the isolated export snapshot. No images are opened for
inspection during this revision; validation uses source, geometry, joint data,
and media metadata/decoding.

## Contact design

- The requested Hold and Drag bands are the central 50% and 30% of the full
  judgement width, respectively. A contact stays stationary when the intersection
  of its legal regions allows it.
- The active block boundary is aligned to authored geometry. The planner leaves
  clearance for the low-resolution mask, its edge, and the contact point.
- A Drag can travel between its judgement windows. It is no longer forced to
  jump between the nearest note's two disconnected strips at their time midpoint.
- The 84–89 s pair uses an authored two-hand relay: one hand receives the held
  contact while the other prepares the later central corridor. Note coverage
  remains continuous across different fingers.
- The dense 111 s figure combines compatible Drags and carries two Flicks into
  their following Drags. Completed standalone Flicks release after 16 ms;
  stationary Taps use at most 30 ms dwell, retaining their original DOWN time.
- Fingering search includes ten fingers, penalizes frequent use and rapid
  transfers, and rejects inverted same-hand finger order through a high cost.
  A separate audit checks the selected result.

`finalize_refinement.py` binds the result back to PSAP, verifies lifecycles,
checks individual note coverage, and samples contact geometry at 1 ms plus exact
block event boundaries and the end of each final MOVE dwell. These checks are
not native Phigros AP validation or a continuous-collision proof.

## Pose changes

Fitting now uses a fixed iteration budget instead of a CPU-time cutoff. The
non-thumb MCP spread limits are narrower. Collision correction has a per-step
wrist budget and a 25 mm accumulated wrist-offset bound; the final wrist is
also limited after collision correction. Refined plans specify 0.55 m/s wrist
speed and 1.2 m/s nominal finger travel speed.

An airborne FK finger previously jumped toward a distant inactive target when
avoidance enabled IK. IK now starts at the current bone endpoint. The reverse
transition preserves the evaluated joint pose before applying curl. Pose
snapshots preserve bone location/scale exactly and restore the contact-state
history used by avoidance decay.

`test_pose_transitions.py` checks those control transitions, 1000 exact pose
restorations, and the wrist correction bound. The six-held-finger fixture checks
the saved mesh, not just target objects. `inspect_pose_numeric.py` measures actual
saved joint changes, finger order, wrist motion and projected skin contacts.

The new workflow first bakes focused numerical probes. The intended full export
uses one continuous bake shared by all render segments, avoiding independent
IK histories at segment boundaries. Actual full-song pose quality must be
reported from the completed bake; geometric planning success alone is insufficient.
