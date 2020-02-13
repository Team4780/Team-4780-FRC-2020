/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;

/**
 * This class defines the Pixy camera functionality.
 */
public class PixyCam {
  private final Link link = new SPILink();
  public final Pixy2 pixy = Pixy2.createInstance(link);
  private Block biggestBlock = null;

  public PixyCam() {
    pixy.init();
    biggestBlock = getBiggestBlock();
  }

  public int getXCoordinate() {
    return biggestBlock.getX();
  }

  public int getYCoordinate() {
    return biggestBlock.getY();
  }

  public int getAngle() {
    return biggestBlock.getAngle();
  }

  public int getWidth() {
    return biggestBlock.getWidth();
  }

  public int getHeight() {
    return biggestBlock.getHeight();
  }

  /**
   * This finds the largest target in the current frame.
   * 
   * @return The biggest target in the current frame.
   */
  public Block getBiggestBlock() {
    final int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);

    if (blockCount <= 0) {
      return null;
    }

    final ArrayList<Block> blocks = pixy.getCCC().getBlocks();
    Block largestBlock = null;
    for (Block block : blocks) {
      if (largestBlock == null) {
        largestBlock = block;
      } else if (block.getWidth() > largestBlock.getWidth()) {
        largestBlock = block;
        double xcoord = blocks.get( 0 ).getX(); // x position of the largest target
        double ycoord = blocks.get( 0 ).getY(); // y position of the largest target
        String data = blocks.get( 0 ).toString(); // string containing target info
        SmartDashboard.putBoolean( "present" , true ); // show there is a target present
          SmartDashboard.putNumber( "Xccord" ,xcoord);
        SmartDashboard.putNumber( "Ycoord" , ycoord);
          SmartDashboard.putString( "Data" , data );
        SmartDashboard.putBoolean( "present" , false );
          SmartDashboard.putNumber( "size" , blocks.size()); //push to dashboard how many targets are detected
      }
    }
    return largestBlock;
  }
}