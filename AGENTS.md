# Robot access preference

The user has prohibited installing, deploying, starting, or running code on the
robot. Do not issue robot installation, Sloth deployment, app launch, or OpMode
execution commands. Limit development work to local edits, local builds and
tests, and Git operations. Do not infer robot deployment authorization from
requests to fix or implement code. This prohibition remains in effect unless
the user explicitly revokes it.

# FTC development rules

- Use the FTC autonomous debugging skill in `.agents/skills/ftc-autonomous-debugging/`
  when investigating autonomous lifecycle failures. Research notes and guidelines
  are in `docs/ftc-autonomous-guidelines.md`.
- Preserve current routes, shooter settings, driver features, and telemetry unless
  the user changes them. Shared shooter preset is 0.72; Basic autos feed for two
  seconds after the speed threshold. The current controller is LQR, not PIDF.
- Match the installed Pedro/FTC API versions before using online examples.
- An auto must reach READY before START can move or shoot. Failures must remain
  visible with phase and exception, and preserve the stack trace in RobotLog.
- Shutdown must attempt every motor and blocker even after a device error; it
  must not depend on another odometry read. Keep partial initialization stoppable.
- Exercise the real installed follower/controller with fake hardware in lifecycle
  tests; mock-only command tests do not demonstrate runtime integration.
- Keep manual bulk-cache invalidation once per active hardware loop and restore
  cache mode on every exit. Do not add competing hardware-control threads.
- Report local test evidence separately from robot validation. Do not claim a
  crash is reproduced or fixed on hardware without evidence.
