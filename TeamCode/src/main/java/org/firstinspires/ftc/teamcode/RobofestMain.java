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
    /// @noinspection FieldCanBeLocal
    /** @noinspection VariablesCanBeRedundant*/
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
    public static  double ELBOW_BALL = 0.65;
    public static double ELBOW_BEAM = 0.45;
    public static  double ELBOW_PICKUP = 0.015;
    
    // Wrist

    public static  double WRIST_TRAVEL = 0.35;
    public static  double WRIST_BALL = 0.72;
    public static double WRIST_BEAM = 0.5;
    public static  double WRIST_PICKUP = 0.01;

    public static int ANSWER = 4;
    private final Timer stateTime = new Timer();
    private int doneCounter = 0;
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
        // Origin is 11 in for the beam from west edge and 27.75 in for start line
        // Start Poses

        // Edge to closest side of line

        Pose startPoseNorth = new Pose(24,4.6 , Math.toRadians(90));
        Pose startPoseEast = new Pose(31.3, 4, Math.toRadians(0));
        Pose startPoseWest = new Pose(25, 4.3, Math.toRadians(180));
        Pose startPoseSouth = new Pose(23.8, 11.7, Math.toRadians(-90));

        // Beam 2 Poses

        // Edge to closet side of beam

        Pose beam2Pose = new Pose(13.2, 12.7, Math.toRadians(90));
        Pose halfwayToBeam2 = new Pose(beam2Pose.getX(), 9, Math.toRadians(90));

        // Ball Poses

        Pose ballPickupPose = new Pose(27.6, 11.5, Math.toRadians(90));
        Pose ballDropoffNorthPose = new Pose (57.5, 15.8, Math.toRadians(90));
        Pose ballDropoffEastPose = new Pose (58, 17, Math.toRadians(0));
        Pose ballDropoffSouthPose = new Pose (57, 12.5, Math.toRadians(-90));

        // Footing one poses

        Pose footingPoseNorth = new  Pose(58, 20, Math.toRadians(90));
        Pose footingPoseSouth = new  Pose(56.8, 8.2, Math.toRadians(-90));
        Pose footingPoseEast = new  Pose(62.5 + 0.5, 16.5, Math.toRadians(0));
        Pose preFootingNorthPose = new Pose(footingPoseNorth.getX(), footingPoseNorth.getY() - 5, Math.toRadians(90));
        Pose preFootingSouthPose = new Pose(footingPoseSouth.getX(), footingPoseEast.getY() + 2, Math.toRadians(-90));
        Pose preFootingEastPose = new Pose(footingPoseEast.getX() - 4, footingPoseEast.getY() + 1, Math.toRadians(0));

        // Bridge location poses

        Pose bridgeCrossingPose = new Pose(toInches(140), toInches(30),Math.toRadians(90));
        Pose bridgeNorthPose = new Pose(footingPoseNorth.getX(), 16.1, Math.toRadians(90));
        Pose bridgeSouthPose = new Pose(footingPoseSouth.getX(), 11.5, Math.toRadians(-90));
        Pose bridgeEastPose = new Pose(59,footingPoseEast.getY(), Math.toRadians(0));

        // Ending Poses

        Pose northEdgePose = new Pose(58, 21, Math.toRadians(90));
        Pose southEdgePose = new Pose(40, 8, Math.toRadians(-90));
        Pose eastEdgePose = new Pose(62.5, 16.5, Math.toRadians(0));
        Pose westEdgePose = new Pose(5, 8, Math.toRadians(180));
//        Pose crossingPose = new Pose(58, 21, Math.toRadians(90));

        // ==================================
        // CHANGE THE STUFF BELOW
        // ==================================
        //noinspection UnnecessaryLocalVariable
        Pose startPose = startPoseNorth;
        //noinspection UnnecessaryLocalVariable
        Pose bridgeLocation = bridgeEastPose;
        //noinspection UnnecessaryLocalVariable
        Pose footingPusher = footingPoseEast;
        //noinspection UnnecessaryLocalVariable
        Pose preFootingPusher = preFootingEastPose;
        //noinspection UnnecessaryLocalVariable
        Pose ballDropoffPose = ballDropoffEastPose;
        //noinspection UnnecessaryLocalVariable
//        Pose endingPose = northEdgePose;
//        setAnswer(9);
        // ==================================
        // CHANGE THE STUFF ABOVE
        // ==================================
        follower.setStartingPose(startPose);

        PathChain beam2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, halfwayToBeam2))
                .setConstantHeadingInterpolation(halfwayToBeam2.getHeading())
                .addPath(new BezierLine(halfwayToBeam2, beam2Pose))
                .setConstantHeadingInterpolation(beam2Pose.getHeading())
                .build();
        PathChain bridgeTravel = follower.pathBuilder()
                .addPath(new BezierLine(beam2Pose, halfwayToBeam2))
                .setConstantHeadingInterpolation(halfwayToBeam2.getHeading())
                .addPath(new BezierLine(halfwayToBeam2, preFootingPusher))
                .setConstantHeadingInterpolation(preFootingPusher.getHeading())
                .build();
        PathChain bridgePusher =  follower.pathBuilder()
                .addPath(new BezierLine(preFootingPusher, footingPusher))
                .setConstantHeadingInterpolation(footingPusher.getHeading())
                .addPath(new BezierLine(footingPusher, bridgeLocation))
                .setConstantHeadingInterpolation(bridgeLocation.getHeading())
                .build();
        PathChain ballPickup = follower.pathBuilder()
                .addPath(new BezierLine(bridgeLocation, bridgeCrossingPose))
                .setLinearHeadingInterpolation(bridgeLocation.getHeading(), bridgeCrossingPose.getHeading())
                .addPath(new BezierLine(bridgeCrossingPose, ballPickupPose))
                .setConstantHeadingInterpolation(ballPickupPose.getHeading())
                .build();
        PathChain ballDrop = follower.pathBuilder()
                .addPath(new BezierLine(ballPickupPose, bridgeCrossingPose))
                .setConstantHeadingInterpolation(ballDropoffPose .getHeading())
                .addPath(new BezierLine(bridgeCrossingPose, ballDropoffPose))
                .setConstantHeadingInterpolation(ballDropoffPose.getHeading())
                .build();
        PathChain endgameTask = follower.pathBuilder()
                .addPath(new BezierLine(ballDropoffPose, startPoseEast))
                .setLinearHeadingInterpolation(ballDropoffPose.getHeading(), startPoseEast.getHeading())
                .build();

        changeState(0);

        while(!isStopRequested()) {
            follower.update();
            if (follower.isBusy()) {
                doneCounter = 0;
            } else {
                doneCounter++;
            }
            boolean followerDone = doneCounter >= 3;
            boolean pressed = button.isPressed();
            if (pressed && !oldPressed && stateTime.getElapsedTimeSeconds() > 0.2) {
                if (state == 0) {
//                    display.writeNumber(3);
//                    display.writeCharacter('2', 0, false);
//                    display.writeCharacter('1', 1, false);
//                    display.writeCharacter('6', 2, false);
//                    display.writeCharacter('1', 3, false);
//                    display.updateDisplay(); // don't forget to call updateDisplay() or maybe do it automatically
                    changeState(10);
                } else {
                    changeState(0);
                }

            }


            boolean enter = state != oldState;
            oldState = state;

//            if((System.currentTimeMillis()%1000) > 500) {
//                display.writeNumber(12);
//                display.updateDisplay();
//            } else {
//                display.writeNumber(3456);
//                display.updateDisplay();
//            }

            switch (state) {
                case 0:
                    if (enter) {
                        armPickup();
                        openLeft();
                        openRight();
                        display.writeCharacter('G', 0, false);
                        display.writeCharacter('O', 1, false);
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
                    } else if (stateTime.getElapsedTime() >= 1250) {
                        changeState(20);
                    }
                    break;
                case 20:
                   if (enter) {
                       armTravel();
                   } else if (stateTime.getElapsedTime() >= 1000) {
                        changeState(30);
                    }
                    break;
                case 30:
                    if (enter) {
                        follower.followPath(beam2);
                    } else if (followerDone) {
                        changeState(40);
                    }
                    break;
                case 40:
                    if (enter) {
                        armPickup();
                    } else if (stateTime.getElapsedTime() >= 1000) {
                        changeState(50);
                    }
                    break;
                case 50:
                    if (enter) {
                        closeLeft();
                    } else if (stateTime.getElapsedTime() >= 1250) {
                        changeState(60);
                    }
                    break;
                case 60:
                    if (enter) {
                        armTravel();
                    } else if (stateTime.getElapsedTime() <= 1000) {
                        changeState(70);
                    }
                    break;
                case 70:
                    if (enter) {
                        follower.followPath(bridgeTravel, 0.9, true);
                    } else if (followerDone) {
                        changeState(75);
                    }
                    break;
                case 75:
                    if (enter) {
                        follower.followPath(bridgePusher, 0.3, true);
                    } else if (followerDone) {
                        changeState(80);
                    }
                    break;
                case 80:
                    if (enter) {
                    armBeam();
                    } else if (stateTime.getElapsedTime() >= 2000) {
                        changeState(90);
                    }
                    break;
                case 90:
                    if (enter) {
                        openRight();
                        openLeft();
                    } else if (stateTime.getElapsedTime() >= 1250) {
                        changeState(100);
                    }
                    break;
                case 100:
                    if (enter) {
                        armTravel();
                    } else if (stateTime.getElapsedTime() >= 1000) {
                        changeState(110);
                    }
                    break;
                case 110:
                    if (enter) {
                        follower.followPath(ballPickup, 0.6, true);
                    } else if (followerDone) {
                        changeState(120);
                    }
                    break;
                case 120:
                    if (enter) {
                        armPickup();
                    } else if (stateTime.getElapsedTime() >= 1000) {
                        changeState(140);
                    }
                    break;
                case 140:
                    if (enter) {
                        closeLeft();
                        closeRight();
                    } else if (stateTime.getElapsedTime() >= 1250) {
                        changeState(150);
                    }
                    break;
                case 150:
                    if (enter) {
                        armTravel();
                    } else if (stateTime.getElapsedTime() >= 1000) {
                        changeState(160);
                    }
                    break;
                case 160:
                    if (enter) {
                        follower.followPath(ballDrop);
                    } else if (followerDone) {
                        changeState(170);
                    }
                    break;
                case 170:
                    if (enter) {
                        armBall();
                    } else if (stateTime.getElapsedTime() >= 1000) {
                        changeState(180);
                    }
                    break;
                case 180:
                    if (enter) {
                        openRight();
                        openLeft();
                    } else if (stateTime.getElapsedTime() >= 1250) {
                        changeState(190);
                    }
                    break;
                case 190:
                    if (enter) {
                        follower.followPath(endgameTask);
                    } else if (followerDone) {
                        follower.breakFollowing();
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
    public static void setAnswer(int newValue) {
        ANSWER = newValue;
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
        if (ANSWER == 0) {
            display.writeNumber(newState);
        } else {
            display.writeNumber(ANSWER);
        }

        display.updateDisplay();
        oldState = state;
        state = newState;
    }
}
