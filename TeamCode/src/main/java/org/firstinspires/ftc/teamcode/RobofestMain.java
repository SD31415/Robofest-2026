package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp
@Config
public class RobofestMain extends LinearOpMode {
    /** @noinspection FieldCanBeLocal*/
    private Follower follower;
    private Servo elbow;
    private Servo wrist;
    private Servo gripLeft;
    private Servo gripRight;
    private AlphaDisplay display;

    // Grip

    public static double GRIP_OPEN = 0;
    public static  double GRIP_CLOSE = 1;

    // Elbow

    public static  double ELBOW_TRAVEL = 0.6;
    public static  double ELBOW_BALL = 0.55;
    public static double ELBOW_BEAM = 0.5;
    public static  double ELBOW_PICKUP = 0.015;
    
    // Wrist

    public static  double WRIST_TRAVEL = 0.35;
    public static  double WRIST_BALL = 0.5;
    public static double WRIST_BEAM = 0.5;
    public static  double WRIST_PICKUP = 0.01;

    private final Timer stateTime = new Timer();
    private int state = -1;
    private int end = 1;
    private int oldState = -1;
    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.6);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        display = hardwareMap.get(AlphaDisplay.class, "display");
        elbow = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        gripLeft = hardwareMap.get(Servo.class, "gripLeft");
        gripRight = hardwareMap.get(Servo.class, "gripRight");

        TouchSensor button = hardwareMap.get(TouchSensor.class, "button");
        boolean oldPressed = false;
        /// FINE TUNE ALL THE POSES
        // Start Poses

        Pose startPoseNorth = new Pose(toInches(62), toInches(6.5), Math.toRadians(90));
        // Pose startPoseEast = new Pose(5.5, 14, Math.toRadians(0));
        // Pose startPoseWest = new Pose(5.5, 14, Math.toRadians(180));
        // Pose startPoseSouth = new Pose(5.5, 14.5, Math.toRadians(-90));

        // Beam 2 Poses

        Pose beam2Pose = new Pose(toInches(36), toInches(20), Math.toRadians(90));
        Pose halfwayToBeam2 = new Pose(toInches(36), toInches(6.5), Math.toRadians(90));

        // Ball Poses

        Pose ballPickupPose = new Pose(toInches(72), toInches(40), Math.toRadians(180));

        // Bridge location poses

        Pose bridgeCrossingPose = new Pose(toInches(144), toInches(30),Math.toRadians(0));
        Pose bridgeNorthPose = new Pose(5.5, 14, Math.toRadians(0));
        Pose bridgeSouthPose = new Pose(5.5, 14, Math.toRadians(0));
        Pose bridgeEastPose = new Pose(5.5, 14, Math.toRadians(0));

        // ==================================
        // CHANGE THIS STUFF!
        // ==================================
        Pose startPose = startPoseNorth;
        Pose bridgeLocation = bridgeNorthPose;





        follower.setStartingPose(startPose);


        PathChain beam2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, halfwayToBeam2))
                .setLinearHeadingInterpolation(startPose.getHeading(), halfwayToBeam2.getHeading())
                .addPath(new BezierLine(halfwayToBeam2, beam2Pose))
                .setLinearHeadingInterpolation(halfwayToBeam2.getHeading(), beam2Pose.getHeading())
                .build();
        PathChain ballPickup = follower.pathBuilder()
                .addPath(new BezierLine(bridgeLocation, ballPickupPose))
                .setLinearHeadingInterpolation(bridgeLocation.getHeading(), ballPickupPose.getHeading())
                .build();
        PathChain bridgeBuilder = follower.pathBuilder()
                .addPath(new BezierLine(beam2Pose, halfwayToBeam2))
                .setLinearHeadingInterpolation(beam2Pose.getHeading(), halfwayToBeam2.getHeading())
                .addPath(new BezierLine(halfwayToBeam2, bridgeCrossingPose))
                .setLinearHeadingInterpolation(halfwayToBeam2.getHeading(), bridgeCrossingPose.getHeading())
                .addPath(new BezierLine(bridgeCrossingPose, bridgeLocation))
                .setLinearHeadingInterpolation(bridgeCrossingPose.getHeading(), bridgeLocation.getHeading())
                .build();


        changeState(0);

        while(!isStopRequested()) {
            follower.update();

            boolean pressed = button.isPressed();
            if (pressed && !oldPressed && stateTime.getElapsedTimeSeconds() > 0.2) {
                if (state == 0) {
                //  display.writeNumber(3);
                    display.writeCharacter('2', 0, false);
                    display.writeCharacter('1', 1, false);
                    display.writeCharacter('6', 2, false);
                    display.writeCharacter('1', 3, false);
                    display.updateDisplay(); // don't forget to call updateDisplay() or maybe do it automatically
                    changeState(10);
                } else {
                    changeState(0);
                }

            }


            boolean enter = state != oldState;
            oldState = state;

            switch (state) {
                case 0:
                    if (enter) {
                        armPickup();
                        openLeft();
                        openRight();
                        display.writeCharacter(' ', 0, false);
                        display.writeCharacter(' ', 1, false);
                        display.writeCharacter('G', 2, false);
                        display.writeCharacter('O', 3, false);
                        display.updateDisplay();
                    }
                    if (gamepad1.left_bumper) {
                        closeLeft();
                    } else if (gamepad1.right_bumper) {
                        closeRight();
                    } else if (gamepad1.left_trigger > 0.5) {
                        openLeft();
                    } else if (gamepad1.right_trigger > 0.5) {
                        openRight();
                    } else if (gamepad1.dpad_up) {
                        armTravel();
                    } else if (gamepad1.dpad_down) {
                        armPickup();
                    } else if (gamepad1.dpad_left) {
                        armBall();
                    } else if (gamepad1.dpad_right) {
                        armBeam();
                    }
                    follower.breakFollowing();
                    follower.setPose(startPose);
                    break;

                case 10:
                    if (enter) {
                        closeRight();
                    } else if (stateTime.getElapsedTime() >= 1500) {
                        changeState(20);
                    }
                    break;
                case 20:
                   if (enter) {
                       armTravel();
                   } else if (stateTime.getElapsedTime() >= 1500) {
                        changeState(30);
                    }
                    break;
                case 30:
                    if (enter) {
                        follower.followPath(beam2);
                    } else if (!follower.isBusy()) {
                        changeState(40);
                    }
                    break;
                case 40:
                    if (enter) {
                        armPickup();
                    } else if (stateTime.getElapsedTime() >= 1500) {
                        changeState(50);
                    }
                    break;
                case 50:
                    if (enter) {
                        closeLeft();
                    } else if (stateTime.getElapsedTime() >= 1500) {
                        changeState(60);
                    }
                    break;
                case 60:
                    if (enter) {
                        armTravel();
                    } else if (stateTime.getElapsedTime() <= 1500) {
                        changeState(70);
                    }
                case 70:
                    if (enter) {
                    follower.followPath(bridgeBuilder);
                    } else if (!follower.isBusy()) {
                        changeState(80);
                    }
                    break;
                case 80:
                    if (enter) {
                        follower.followPath(ballPickup);
                    } else if (!follower.isBusy()) {
                        changeState(90);
                    }
                    break;
                case 90:
                    if (enter) {
                    // Add drop off and pickup  for ball
                    // Add endgame task
                    }
                    break;
            }
            oldPressed = pressed;
            telemetry.addData("state", state);
            telemetry.addData("stateTime", stateTime.getElapsedTimeSeconds());
            telemetry.addData("busy", follower.isBusy());
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("buttonIsPressed", button.isPressed());
            //telemetry.addData("lift", lift.getPosition());
            //telemetry.addData("claw", claw.getPosition());
            telemetry.update();
        }
    }

    private void closeLeft() {
        gripLeft.setPosition(GRIP_CLOSE);
    }
    private void openLeft() {
        gripLeft.setPosition(GRIP_OPEN);
    }
    private void closeRight() {
        gripRight.setPosition(GRIP_CLOSE);
    }
    private void openRight() {
        gripRight.setPosition(GRIP_OPEN);
    }
    private void armPickup() {
        elbow.setPosition(ELBOW_PICKUP);
        wrist.setPosition(WRIST_PICKUP);
    }
    private void armBall() {
        elbow.setPosition(ELBOW_BALL);
        wrist.setPosition(WRIST_BALL);
    }
    private void armTravel() {
        elbow.setPosition(ELBOW_TRAVEL);
        wrist.setPosition(WRIST_TRAVEL);
    }
    private void armBeam() {
        elbow.setPosition(ELBOW_BEAM);
        wrist.setPosition(WRIST_BEAM);
    }
    private void ballArmPickup() {

    }
    private void ballArmTravel() {

    }
    private void ballArmDropoff() {

    }
    private static double toInches(double centimeters) {
        return(centimeters/2.54);
    }
//    private void liftUp() {
//        lift.setPosition(LIFT_UP);
//    }
//
//    private void liftStart() {
//        lift.setPosition(LIFT_START);
//    }
//
//    private void liftDown() {
//        lift.setPosition(LIFT_DOWN);
//    }
//    private void liftDrop() {
//        lift.setPosition(LIFT_DROP);
//    }
//
//    private void liftMedal() {
//        lift.setPosition(LIFT_MEDAL);
//    }
//
//    private void closeClaw() {
//        claw.setPosition(CLAW_CLOSED);
//    }

    private void changeState(int newState) {
        stateTime.resetTimer();
        display.writeNumber(newState);
        display.updateDisplay();
        oldState = state;
        state = newState;
    }
}
