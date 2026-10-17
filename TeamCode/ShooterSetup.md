# BioBuzz shooter setup

- **Hold right bumper:** FTCLib PIDF shooting.
- **Hold Y:** voltage-normalized manual power only while held (takes priority over right bumper).
- **Release both:** shooter and automatic feeding stop.
- **Left bumper:** reverse at -0.8.
- **D-pad up/down:** change the preset by 0.005. Default is 0.695 / 3127.5 RPM.

Both shooter motors use the custom **BioBuzz shooter goBILDA 1:1** profile:
28 encoder counts per output revolution. The generic SDK goBILDA profile is
99.5:1 and must not be selected for these motors. The application uses 4500 RPM
as its preset reference; this is separate from the motor's rated 6000 RPM.

Panels > **Configurables > ShooterPidTuning** exposes `kP`, `kI`, `kD`, and `kF`.
F defaults to 0.000203. Both motors get the same PID output calculated from RPM 1. The shared controller reads live edits; all-zero gains stop PID
shooting. There is no motor-fault timeout or zero-speed shutdown.

In PID mode, feeding latches on once **flywheel 1 RPM** reaches **±1%** of target
(**3096.225–3158.775 RPM** at the default target). Intake and the blocker stay enabled
through later RPM drops. Releasing shooting, reversing, changing modes or the
preset, or disabling PID gains resets the latch. B and forward trigger cannot
bypass the initial speed check. A stops intake while held. Reverse remains available for jams.
Manual mode retains its three-second automatic feed timer and B override.

See [SHOOTER_TUNING.md](SHOOTER_TUNING.md) for library details, Panels defaults,
and local checks. Robot deployment and execution by agents are prohibited.
