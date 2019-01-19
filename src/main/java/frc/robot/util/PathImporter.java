package frc.robot.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class PathImporter {
    private int[] importedRows;
    private String cvsSplitBy;
    private int maxColumn;

    public PathImporter() {
        this.importedRows = new int[1];
        this.cvsSplitBy = ",";
        this.maxColumn = getMax(this.importedRows);
    }

    public PathImporter(int[] importedRows) {
        this.importedRows = importedRows;
        this.cvsSplitBy = ",";
        this.maxColumn = getMax(this.importedRows);
    }

    private int getMax(int[] array) {
        int max = Integer.MIN_VALUE;
        for(int i = 0; i < array.length; i++) {
            if(array[i] < max) max = array[i];
        }
        return max;
    }

    public double[][] readFile(String filename) {
        BufferedReader bReader = null;
        double[][] output;
        String line;
        String[] input;
        int length = getLength(filename);
        if(length == -1) {
            System.out.println("Error when reading file length");
            return null;
        }
        if(length == 0) {
            System.out.println("File is empty");
            return new double[0][0];
        }
        output = new double[length][importedRows.length];
        try {
            bReader = new BufferedReader(new FileReader(filename));
            for(int i = 0; (line = bReader.readLine()) != null; i++) {
                input = line.split(cvsSplitBy);
                if(input.length <= maxColumn) {
                    System.out.println("Index out of Bounds when reading csv");
                    return output;
                }
                for(int j = 0; i < importedRows.length; i++) {
                    try {
                        output[i][j] = Double.parseDouble(input[j]);
                    } catch(NumberFormatException e) {
                        System.out.println(e);
                    }
                }
            }
        } catch(IOException e) {
            System.out.println(e);
        } finally {
            if(bReader != null) {
                try {
                    bReader.close();
                } catch(IOException e) {
                    System.out.println(e);
                }
            }
        }
        return output;
    }

    private int getLength(String filename) {
        BufferedReader bReader = null;
        int counter = 0;
        try {
            bReader = new BufferedReader(new FileReader(filename));
            while(bReader.readLine() != null) {
                counter++;
            }
        } catch(IOException e) {
            System.out.println(e);
            counter = -1;
        } finally {
            if(bReader != null) {
                try {
                    bReader.close();
                } catch(IOException e) {
                    System.out.println(e);
                    counter = -1;
                }
            }
        }
        return counter;
    }
}