# Phased TeleOp hardware I/O implementation

Spec: docs/superpowers/specs/2026-10-17-phased-hardware-io-design.md

Goal: read hardware once at the beginning of each TeleOp frame, compute from cached inputs, commit final changed outputs once, and report timings without hardware getters.

Architecture: explicit phased APIs on the existing subsystem controllers retain the legacy immediate API for autonomous. A single physical Pedro localizer feeds a snapshot localizer; a buffered drivetrain submits to the same drive output owner as manual drive. TeleOp owns MANUAL hub cache clearing at frame start.

Global constraints: no robot deployment or execution; preserve tuning, control mappings, autonomous APIs, async voltage polling, always-on timing logs, effective Pedro calibration and scheduler fix. Work in the existing detached checkout; checkpoint 5122b0d is saved. No implementation approval gates remain.

## Task 1: Buffered subsystem I/O
Write failing tests for cached feedback, no hardware calls during compute/getters, final-only writes, unchanged output suppression and forced stop. Run tests (expected failure), add phased APIs and shared controllers, rerun tests (expected pass).
Interfaces: beginFrame reads feedback/voltage; control APIs compute; commitOutputs writes final changes; forceStop bypasses cache.

## Task 2: Pedro and frame coordinator
Write failing tests for snapshot-only Pedro updates, shared manual/hold output ownership, once-per-frame read/compute/write order and failure cleanup. Implement snapshot localizer, buffered drivetrain and coordinator. Verify tests pass.
Interfaces: snapshot state feeds Pedro; buffered drivetrain feeds DriveSubsystem; coordinator runs read/compute/write and stops on exceptions.

## Task 3: TeleOp integration and verification
Connect preUpdate reads, command compute, onUpdate output commit before cached telemetry. Remove end-of-frame bulk cache clearing; copy gamepad controls. Update tests for cached inputs and add lifecycle coverage. Run focused regressions, full local unit suite, local assembly and diff check. Expected: focused tests/build pass; only documented pre-existing full-suite failures may remain.

Review focus: real NextFTC hook ordering, cache invalidation, hold/manual sign conventions and threshold handoff, one Pinpoint sample including hive-save, exception/shutdown safety, legacy autonomous behavior, truthful timing boundaries. Final fresh-context review after implementation.
