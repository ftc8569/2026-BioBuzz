package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Backup Blue Left", group = "Backup")
public class BackupAutoBlueLeft extends BioBuzzBackupAuto {
    public BackupAutoBlueLeft() {
        super(true, 112.1, 7.8, -90, 0, 35.5);
    }
}
