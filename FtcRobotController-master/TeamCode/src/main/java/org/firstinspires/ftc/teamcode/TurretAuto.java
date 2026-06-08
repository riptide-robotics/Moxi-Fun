package org.firstinspires.ftc.teamcode;

public class TurretAuto implements LinearOpMode {

    DcMotor upperMotor;
    DcMotor lowerMotor;

    @Override
    public void runOpMode() {
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            return;
        }
    }
}