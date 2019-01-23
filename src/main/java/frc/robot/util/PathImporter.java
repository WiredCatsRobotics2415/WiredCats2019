package frc.robot.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

/**
 * Class to import csv containing path data
 * 
 * @author Matthew Propp
 */
public class PathImporter {
    /**
     * stores all of the columns that will be saved in output and the order to save them
     */
    private int[] importedColumns;
    /**
     * the string that seperates values in the csv
     */
    private String csvSplitBy;
    /**
     * the largest column that will be accessed
     * <p>
     * used to prevent index out of bounnds exceptions
     * </p>
     */
    private int maxColumn;
    /**
     * the number of the rows at the top to ignore when reading the csv
     * <p>
     * used to skip over headers
     * </p>
     */
    private int ignoreTopRows;

    /**
     * Sets which columns to read
     * @param importedColumns which columns to read
     */
    public PathImporter(int[] importedColumns) {
        this.importedColumns = importedColumns;
        this.csvSplitBy = ",";
        this.maxColumn = getMax(this.importedColumns);
        this.ignoreTopRows = 0;
    }

    /**
     * Sets which columns to read and how many rows to ignore at the top of the csv
     * @param importedColumns which columns to read
     * @param ignoreTopRows how many rows to ignore at the top of the csv
     */
    public PathImporter(int[] importedColumns, int ignoreTopRows) {
        this(importedColumns);
        this.ignoreTopRows = ignoreTopRows;
    }

    /**
     * simple method to get the max value out of an array of ints
     * @return the max value
     */
    private int getMax(int[] array) {
        int max = Integer.MIN_VALUE;
        for(int i = 0; i < array.length; i++) {
            if(array[i] < max) max = array[i];
        }
        return max;
    }

    /**
     * Reads a csv file and creates a 2d array of doubles to hold the data
     * @param filename the file to be read
     * @return 2d array of the values in the csv [row][column]
     */
    public double[][] readFile(String filename) {
        BufferedReader bReader = null;
        double[][] output;
        String line;
        String[] input;
        int length = getLength(filename);
        if(length == -1) { //an error accured when finding the file legnth
            System.out.println("Error when reading file length");
            return null;
        }
        if(length == 0) { //file was empty
            System.out.println("File is empty");
            return new double[0][0];
        }
        output = new double[length][importedColumns.length];
        try {
            bReader = new BufferedReader(new FileReader(filename));
            for(int i = 0; (line = bReader.readLine()) != null; i++) {
                if(i < ignoreTopRows) continue; //ignores the top rows
                input = line.split(csvSplitBy);
                if(input.length <= maxColumn) { //checks if there are the right amount of columns in the row
                    System.out.println("Index out of Bounds when reading csv");
                    return output;
                }
                for(int j = 0; j < importedColumns.length; j++) { //for every input for that row
                    try {
                        //sets the output array to the matching value in the csv based on the columns in the importColumns array
                        output[i][j] = Double.parseDouble(input[importedColumns[j]]);
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

    /**
     * Finds the number of rows in a given csv
     * @param filename the filename of the csv
     * @return the length of the csv, -1 if an exception accurs
     */
    private int getLength(String filename) {
        BufferedReader bReader = null;
        int counter = 0;
        try {
            bReader = new BufferedReader(new FileReader(filename));
            while(bReader.readLine() != null) {
                counter++; //count each line
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