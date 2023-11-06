#pragma once
// #include <iostream>
#include <vector>

class MovingAverage
{
private:
    std::vector<float> placeHolderMovAvg_vector;
    int stepSize = 0;
    int indexCounter = 0;
    bool startCalculate = false;

    /* 
    NOTE: Jika ingin menggunakan objek yang sama pada data stream yang berbeda maka INVOKE resetForNewStream() terlebih dahulu.
          Hal tersebut dilakukan karena untuk melakukan moving average, variabel berikut harus direset

            std::vector<float> placeHolderMovAvg_vector; // direset menjadi vector dengan tiap item bernilai 0
            int indexCounter = 0;
            bool startCalculate = false;
    */

public:
    MovingAverage(int movAvgStep);
    void resetForNewStream();
    float calculateAvg(std::vector<float>& placeHolder);
    float movingAverage(float data);

};