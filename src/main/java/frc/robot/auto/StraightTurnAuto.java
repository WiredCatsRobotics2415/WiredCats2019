package frc.robot.auto;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;

public class StraightTurnAuto {
    /**
     * method to get a list of commands to follow the path by going straight and turning
     * 
     * @param path array of point to follow [x,y,straightSpeed,afterTurnSpeed,time]
     */
    public static Command[] getCommands(double[][] path, double startX, double startY, double startAngle) {
        if(path.length == 0) {
            return new Command[0];
        }
        Command[] commands = new Command[path.length*2];
        commands[0] = new DriveStraightCommand(pythag(startX,path[0][0],startY,path[0][1]), path[0][2], (long)path[0][4], false);
        commands[1] = new TurnToAngleCommand(getAngle(startX,path[0][0],startY,path[0][1]), (long)path[0][4], path[0][3], false);
        for(int i = 1; i < path.length; i++) {
            commands[i*2] = new DriveStraightCommand(pythag(startX,path[i][0],startY,path[i][1]), path[i][2], (long)path[i][4], false);
            commands[i*2+1] = new TurnToAngleCommand(getAngle(startX,path[i][0],startY,path[i][1]), (long)path[i][4], path[i][3], false);
        }
        return commands;
    }

    /**
     * method to get a list of commands to follow the path by going straight and turning with turn pid
     * 
     * @param path array of point to follow [x,y,straightSpeed,afterTurnSpeed,time]
     */
    public static Command[] getCommandsPID(double[][] path, double startX, double startY, double startAngle) {
        if(path.length == 0) {
            return new Command[0];
        }
        Command[] commands = new Command[path.length*2];
        commands[0] = new DriveStraightCommand(pythag(startX,path[0][0],startY,path[0][1]), path[0][2], (long)path[0][4], true);
        commands[1] = new TurnToAngleCommand(getAngle(startX,path[0][0],startY,path[0][1]), (long)path[0][4], path[0][3], true);
        for(int i = 1; i < path.length; i++) {
            commands[i*2] = new DriveStraightCommand(pythag(startX,path[i][0],startY,path[i][1]), path[i][2], (long)path[i][4], true);
            commands[i*2+1] = new TurnToAngleCommand(getAngle(startX,path[i][0],startY,path[i][1]), (long)path[i][4], path[i][3], true);
        }
        return commands;
    }

    private static double pythag(double x1, double x2, double y1, double y2) {
        return Math.sqrt(Math.pow(x1-x2,2)+Math.pow(y1-y2,2));
    }

    private static double getAngle(double x1, double x2, double y1, double y2) {
        double angle = Math.atan2(y2-y1, x2-x1);
        angle = Math.toDegrees(angle);
        if(x2-x1 < 0 && y2-y1 > 0) { //II Quadrant
            angle = 90 - angle;
        }
        if(x2-x1 > 0 && y2-y1 < 0) { //IV Quadrant
            angle = 270 - angle;
        }
        return angle;
    }
}