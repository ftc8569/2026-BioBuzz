---
name: ftc-autonomous-debugging
description: Diagnose FTC Java/Kotlin autonomous INIT, START, LOOP, and STOP failures; review Pedro path integration and add lifecycle regressions using the project's installed SDK and libraries.
---

# FTC autonomous debugging

Read the active repository's AGENTS.md and dependency declarations. Honor its
robot-access restriction. Debugging permission does not imply permission to
install, launch, or run robot code.

Ask for the selected OpMode, failure phase, exception text, and stack when they
are missing. Continue independent source review and local reproduction. Do not
invent a root cause when the device error is unavailable.

Use installed library source/jars and primary documentation. Pedro 2 examples
and Pedro 3 examples have different APIs. Inspect actual lifecycle behavior,
including what `stop()` writes and whether `update()` polls hardware.

Trace initialization order, missing hardware-map devices, partial construction,
finite sensor data, controller units, bulk-cache freshness, and cleanup. Keep
control loops nonblocking. READY must precede movement; failed startup must not
run a partial routine. A handled exception must leave an explicit failed state,
visible phase/error, and a logged stack rather than resuming silently.

Stop independent outputs even when one write fails. Preserve the first cause and
attach cleanup errors. Do not require a working localizer or cached controller
state to zero motors. Retain acquired motor handles during partial initialization.

Preserve routes, shooting settings, telemetry, and unrelated tuning. A starting
pose/first-heading mismatch can request a turn, but changing a pose declaration
does not establish the physical robot's true heading. For direction complaints,
compare the selected path export and the user's physical placement.

Use deterministic clocks for timers. Add at least one relevant lifecycle test
using real installed follower/controller code and hardware doubles, not only a
mock follower. Inject the confirmed failure and verify shutdown and reporting.
Run the repository's local unit/build checks and separate their evidence from
physical validation. Sync Git using normal pull/merge/push without force.

If present, read `docs/ftc-autonomous-guidelines.md` for this project's hardware
names, library versions, and regression entry points.

Primary starting points:
- https://github.com/FIRST-Tech-Challenge/FtcRobotController
- https://ftc-docs.firstinspires.org/en/latest/control_system_troubleshooting/index.html
- https://pedropathing.com/docs/pathing/guide/setting-up-auto
- https://pedropathing.com/docs/pathing/reference/interpolation
- https://github.com/Pedro-Pathing/PedroPathing
