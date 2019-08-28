#ifndef CUDAMEM_H
#define CUDAMEM_H

//VERSION: 0.0.1

#include <iostream>

class cudaMem
{
public:
    cudaMem(){
        calcMem();
    }
    void calcMem();
    size_t getMemFree();
    size_t getMemUsed();
    size_t getMemTotal();

private:
    size_t memFree;
    size_t memUsed;
    size_t memTotal;
};

#endif // CUDAMEM_H