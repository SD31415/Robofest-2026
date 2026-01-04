package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;

public class DefaultOpMode {
    private static String DEFAULT_OP_MODE = "RobofestMain";

    private static DefaultOpMode instance;
    private FtcEventLoop eventLoop;
    
    @OnCreate
    public static void start(Context context) {
        if (instance == null) {
            instance = new DefaultOpMode();
        }
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        if (instance != null) {
            instance.internalAttachEventLoop(eventLoop);
        }
    }

    private class RunDefaultOpMode implements Runnable {
        @Override
        public void run() {
            RegisteredOpModes.getInstance().waitOpModesRegistered();

            OpModeManagerImpl opModeManager = eventLoop.getOpModeManager();
            opModeManager.initOpMode(DEFAULT_OP_MODE);
            opModeManager.startActiveOpMode();
        }
    }

    private void internalAttachEventLoop(FtcEventLoop eventLoop) {
        this.eventLoop = eventLoop;

        Thread t = new Thread(new RunDefaultOpMode());
        t.start();
    }
}