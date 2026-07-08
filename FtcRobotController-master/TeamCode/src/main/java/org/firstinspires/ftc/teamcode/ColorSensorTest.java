//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.robot.Robot;
//
//
//@Config
//@TeleOp(name = "Color Sensor Test")
//public class ColorSensorTest extends LinearOpMode {
//    String[] order = new String[3];
//    public static float gain = 67;
//    int[] pgratio = new int[2];
//
//    NormalizedColorSensor colorSensor;
//
//
//    public void runOpMode() {
//        char sendColor;
//        char prevString = 'a';
//        //
//
//        //colorSensor = hardwareMap.colorSensor;
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            colorSensor.setGain(gain);
//            sendColor = 'n';
//            if (sendColor != prevString) {
//                prevString = sendColor;
//                telemetry.addData("Color Detected", sendColor);
//            }
//            telemetry.addData("Current color ", sendColor);
//            telemetry.update();
//        }
//    }
//}
