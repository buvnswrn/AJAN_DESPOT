package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util;

import java.util.Vector;

public class Floor {
    int num_rows_, num_cols_;
    protected long nativeptr;
//    int p_p_floor__;
//    static int INVALID;
//    Vector<Coord> cells_;
//    native Vector<Double> ComputeDistances(int source);
public Floor() {
        nativeptr = Init();
    }

    public Floor(int num_rows_, int num_cols_){
        nativeptr = Init(num_rows_, num_cols_);
    }

    public void AddCell(int x, int y)
    {
        AddCell (x, y,nativeptr);
    }

    public Coord GetCell(int i ) {
        return GetCell(i, nativeptr);
    }

    public int GetIndex(Coord coord) {
        return GetIndex(coord, nativeptr);
    }

    public boolean Inside(Coord coord ) {
        return Inside(coord, nativeptr);
    }
    public boolean Inside(int x, int y ) {
        return Inside( x, y, nativeptr);
    }

    public void ComputeDistances() {
        ComputeDistances(nativeptr);
    }

    public double Distance(int c1, int c2 ) {
        return Distance( c1, c2, nativeptr);
    };

    public Vector<Integer> ComputeShortestPath(int start, int end ) {
        return ComputeShortestPath( start, end, nativeptr);
    }

    public int num_rows() {
        return num_rows(nativeptr);
    }
    public int num_cols() {
        return num_cols(nativeptr);
    }
    public int NumCells() {
        return NumCells(nativeptr);
    }
    public void destroyNativeClass() {
        destroyNativeClass(nativeptr);
    }

    //region Native Methods
    native long Init();
    native long Init(int x, int y);
    native void AddCell(int x, int y, long reference);

    native Coord GetCell(int i, long reference);

    native int GetIndex(Coord coord, long reference);

    native boolean Inside(Coord coord, long reference);
    native boolean Inside(int x, int y, long reference);

    native void ComputeDistances(long reference);

    native double Distance(int c1, int c2, long reference);

    native Vector<Integer> ComputeShortestPath(int start, int end, long reference);

    native int num_rows(long reference);
    native int num_cols(long reference);
    native int NumCells(long reference);
    native void destroyNativeClass(long reference);

    @Override
    protected void finalize() throws Throwable {
        destroyNativeClass(nativeptr);
        nativeptr = 0L;
    }

    //endregion



}
