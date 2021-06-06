package org.ma.ftc.lib.geometry;

public class Vector2D {
    private double x;
    private double y;

    public Vector2D() {
        this.x = 0f;
        this.y = 0f;
    }

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getMagnitude() {
        return Math.sqrt( (Math.pow(this.x, 2) + Math.pow(this.y, 2)) );
    }

    public double getAngle() {
        return Math.atan2(this.y, this.x);
    }

    public void setPolar(double mag, double angle) {
        this.x = mag * Math.cos(angle);
        this.y = mag * Math.sin(angle);
    }

    public void rotate(double angle) {
        this.x = Math.cos(angle) * this.x - Math.sin(angle) * this.y;
        this.y = Math.sin(angle) * this.x + Math.cos(angle) * this.y;
    }
}
