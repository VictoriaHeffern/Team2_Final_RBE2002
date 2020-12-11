#include "Median_filter.h"

void MedianFilter::Init(void)
{
    // no initialization required, but good practice
}

void MedianFilter::Sort(int index_a, int index_b)
{
    if(sortedArray[index_a] < sortedArray[index_b]){
        float temp = sortedArray[index_a];
        sortedArray[index_a] = sortedArray[index_b];
        sortedArray[index_b] = temp;
    }
}

float MedianFilter::Filter(float measurement)
{
    static int oldestIndex = 0;
    array[oldestIndex] = measurement;
    oldestIndex = (oldestIndex+1)%5;
    for(int i = 0; i < 5; i++){
        sortedArray[i] = array[i];
    }
    Sort(0,1);
    Sort(3,4);
    Sort(0,2);
    Sort(1,2);
    Sort(0,3);
    Sort(2,3);
    Sort(1,4);
    Sort(1,2);
    return sortedArray[2];
}
void MedianFilter::Clear(){
    for(int i = 0; i < 5; i++){
        array[i] = 0;
    }
}