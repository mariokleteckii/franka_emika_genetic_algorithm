package com.fer.NextMove.messages;


public class Data {

    private int id;
    private String valid = "false";
    private String fraction = "";

    public Data() {
    }

    public Data(int id, String valid, String fraction) {
        this.id = id;
        this.valid = valid;
        this.fraction = fraction;
    }

    public int getId() {
        return id;
    }

    public String getValid() {
        return valid;
    }

    public String getFraction() {
        return fraction;
    }
}
