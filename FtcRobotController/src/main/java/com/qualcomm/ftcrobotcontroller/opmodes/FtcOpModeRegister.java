package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

public class FtcOpModeRegister implements OpModeRegister {

  public void register(OpModeManager manager) {
    manager.register("TeleOp", TeleOp.class);
    manager.register("AutoDrive", AutoDrive.class);
    manager.register("NullOp", NullOp.class);

  }
}
