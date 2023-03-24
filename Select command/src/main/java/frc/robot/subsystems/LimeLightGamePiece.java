// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class LimeLightGamePiece {
    public enum GamePiece {
        CONE(),
        CUBE();
    };

    private double x;
    private double y;
    private double area;
    private GamePiece piece;

    public LimeLightGamePiece(double x, double y, double area, GamePiece piece) {
        this.x = x;
        this.y = y;
        this.area = area;
        this.piece = piece;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getArea() {
        return area;
    }

    public GamePiece getPiece() {
        return piece;
    }

}
