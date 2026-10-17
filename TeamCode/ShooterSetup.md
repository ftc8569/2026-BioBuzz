# BioBuzz shooter setup

- **Hold right bumper:** LQR velocity shooting.
- **Hold Y:** voltage-normalized manual power only while held (takes priority over right bumper).
- **Release both:** shooter and automatic feeding stop.
- **Left bumper:** reverse at -0.8.
- **D-pad up/down:** change the preset by 0.005. Default is 0.72 / 3240 RPM.

Both shooter motors use the custom **BioBuzz shooter goBILDA 1:1** profile:
28 encoder counts per output revolution. The generic SDK goBILDA profile is
99.5:1 and must not be selected for these motors. The application uses 4500 RPM
as its preset reference; this is separate from the motor's rated 6000 RPM.

Panels > **Configurables > ShooterLqrTuning** exposes `kV`, `kA`,
`rpmTolerance`, and `voltageTolerance`. Both motors get the same LQR voltage
calculated from RPM 1. Initial model parameters are estimates; see
[the LQR tuning guide](../docs/control/shooter-lqr.md). Edits apply on the next
velocity update. Invalid parameters stop output and block feeding.

In LQR mode, feeding latches on once **flywheel 1 RPM** reaches **±1%** of target
(**3207.6–3272.4 RPM** at the default target). Intake and the blocker stay enabled
through later RPM drops. Releasing shooting, reversing, changing modes or the
preset, or invalid model parameters or RPM feedback resets the latch. B and forward trigger cannot
bypass the initial speed check. A stops intake while held. Reverse remains available for jams.
Manual mode retains its three-second automatic feed timer and B override.

See [SHOOTER_TUNING.md](SHOOTER_TUNING.md) for controller details, Panels defaults,
and local checks. Robot deployment and execution by agents are prohibited.
