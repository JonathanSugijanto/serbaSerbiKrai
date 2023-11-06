#include "MovingAverage.h"

MovingAverage::MovingAverage(int movAvgStep)
{
    this->stepSize = movAvgStep;
    placeHolderMovAvg_vector.reserve(movAvgStep);
}

void MovingAverage::resetForNewStream()
{

    this->indexCounter = 0;
    this->startCalculate = false;
    for (int i = 0; i < stepSize; i++)
    {
        placeHolderMovAvg_vector[i] = 0.0f;
    } 
}

float MovingAverage::calculateAvg(std::vector<float>& placeHolder)
{
    float sum = 0.0f;
    for (int i = 0; i < stepSize; i++)
    {
        sum += placeHolder[i];
    }
    return sum/(float)stepSize;
}

float MovingAverage::movingAverage(float data)
{
    float firstSum = 0.0f;

    if (!startCalculate)
    {
        placeHolderMovAvg_vector[indexCounter] = data;
        indexCounter++;

        if (indexCounter == stepSize)
        {
            // for (float a:placeHolderMovAvg_vector)
            // {
            //     std::cout << a << ' ';
            // }
            // std::cout << '\n';

            // std::cout << calculateAvg(placeHolderMovAvg_vector) << std::endl;

            startCalculate = true;
            indexCounter = 0;
            return calculateAvg(placeHolderMovAvg_vector);
        }

        for (int i = 0; i < stepSize; i++)
        {
            firstSum += placeHolderMovAvg_vector[i];
        }

        return firstSum/(float)indexCounter;
    }
    else
    {
        placeHolderMovAvg_vector[indexCounter] = data;
        indexCounter++;

        // for (float a:placeHolderMovAvg_vector)
        // {
        //     std::cout << a << ' ';
        // }
        // std::cout << '\n';

        // std::cout << calculateAvg(placeHolderMovAvg_vector) << std::endl;

        if (indexCounter == stepSize)
        {
            startCalculate = true;
            indexCounter = 0;
        }
        return calculateAvg(placeHolderMovAvg_vector);
    }
}