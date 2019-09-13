#include "buffer.h"


void bfNew(Buffer *bf, real*d, int m, int n)
{
    bf->d = d;
    bf->m = m;
    bf->n = n;
    bf->i = -1;
    bf->full = 0;
    bf->num = 0;
}

void bfPut(Buffer *bf, real* d)
{
    int i;
    int m = bf->m;              // row number
    int n = bf->n;              // column number

    bf->i += 1;                 // move to the next position
    if(bf->full == 0)           // buffer is not full
    {
        bf->num = bf->i + 1;
        if(bf->i == n-1)        // buffer become full
        {
            bf->full = 1;
        }
    }
    if(bf->i==n)                //buffer is full, overwrite the oldest data
    {
        bf->i = 0;
    }
    // put data into buffer
    for(i=0;i<m;i++)
    {
        bf->d[i*n+bf->i] = d[i];
    }
}

int bfGet(Buffer *bf,real *d, int idx)
{
    int i;
    int m,n;
    m = bf->m;
    n = bf->n;
    if(idx>=n)  // idx exceeds the max column number
    {
        for(i=0;i<m;i++)
            d[i] = 0.0;
        return 0;
    }
    if(bf->full)// buffer is full
    {
        idx = bf->i - idx;
        if(idx<0)
            idx += n;
        for(i=0;i<m;i++)
            d[i] = bf->d[i*n+idx];
    }
    else//buffer is not full
    {
        idx = bf->i - idx;
        if(idx<0)// idx exceeds the storage range
        {
            for(i=0;i<m;i++)
                d[i] = 0.0;
            return 0;
        }
        else// idx within the storage range
        {
            for(i=0;i<m;i++)
                d[i] = bf->d[i*n+idx];
        }
    }
    return 1;
}

void bfClear(Buffer *bf)
{
    bf->i = -1;
    bf->full = 0;
    bf->num = 0;
}
