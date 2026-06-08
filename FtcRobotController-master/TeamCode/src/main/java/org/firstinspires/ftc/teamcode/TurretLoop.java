package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Turret Loop")
public class TurretLoop extends LinearOpMode {

    public DcMotor motor;
    public Servo rotator;

    @Override
    public void runOpMode() {

        motor = hardwareMap.dcMotor.get("turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotator = hardwareMap.servo.get("rotator");
        rotator.setDirection(Servo.Direction.FORWARD);
        rotator.setPosition(0);

        double distance = 0;
        double angle = 1;
        double height = 1;

        double overlycomplexcoefficient = 60 / (2 * Math.PI * 1.4/*wheel radius*/);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double vel = overlycomplexcoefficient * Math.sqrt((9.8 * Math.pow(distance, 2)) / (2 * Math.pow(Math.cos(angle), 2) * ((distance * Math.tan(angle)) - height)));

            //    motor.setPower(gamepad1.left_stick_y/1.5);
            //    rotator.setPosition((gamepad1.left_stick_x - 0.5) * 2);

            pidtunedmotor1(wantedrpm);
        }
    }


    public double currPos;
    public double prevPos;
    public double time;
    public double cooldown;
    public List<Double> records = new ArrayList<>();
    public double queueSize = 5;
    public double wantedrpm = 2500;

    PIDController rpmcontroller = new PIDController(0.00025, 0.0075, 0.00000155);

    public double kf = 0.0002;

    public void pidtunedmotor1(double rpm) {
        prevPos = currPos;
        currPos = motor.getCurrentPosition();

        double dTheta = (currPos - prevPos) / 28;   //rotations?

        double dt = System.nanoTime() / 1e9 - time;
        time = System.nanoTime() / 1e9;

        double currRPM = dTheta / (dt / 60);

        //stop calculations if rpm cant be actually calculated
        if (currRPM <= 0 && cooldown < 0.25) {
            cooldown += dt;
            return;
        }
        if (currRPM > 0) cooldown = 0;

        records.add(currRPM);
        while (records.size() > queueSize) records.remove(0);

        double undividedAverage = 0;

        for (int i = 0; i < records.size(); i++)
            undividedAverage += records.get(i);

        double average;

        if (records.size() == queueSize) average = undividedAverage / queueSize;
        else average = currRPM;

        double wantedWheelPowerAverage = rpmcontroller.calculate(average, rpm) + (wantedrpm * kf);
        if (wantedWheelPowerAverage == 0) wantedWheelPowerAverage = wantedrpm;

        motor.setPower(wantedrpm == 0 ? 0 : wantedWheelPowerAverage);
    }
}
