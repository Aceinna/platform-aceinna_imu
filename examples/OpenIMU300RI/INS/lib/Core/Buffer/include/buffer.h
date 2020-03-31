/*****************************************************************************
 * buffer operations
 * @file    buffer.h
 * @brief   ring buffer
 * @author  Dong Xiaoguang
 * @version 1.0
 * @date    20180813
 *****************************************************************************/


#ifndef BUFFER_H_INCLUDED
#define BUFFER_H_INCLUDED

#include "GlobalConstants.h"


//======================data struct definitions======================
typedef struct _buffer
{
    real *d;    // data storage, each column represents a set of data
    int m;      //row number
    int n;      //column number
    int i;      //index for data being put into the buffer, -1 means buffer is empty
    int full;   //1 means buffer is full, 0 not
    int num;    //data number in the buffer
} Buffer;

//-------------------------------------------------------------------
// new a buffer
// input:   bf--pointer to the buffer
//            d--pointer to the memory for data storage
//          m--row number(length of each set of data)
//          n--column number(number of sets of data)
// output:
// return:
void bfNew(Buffer *bf, real*d, int m, int n);

//-------------------------------------------------------------------
// put data into the buffer
// input:   bf--buffer pointer
//          d--pointer to the data being put into the buffer
// output:
// return:
void bfPut(Buffer *bf, real* d);

//-------------------------------------------------------------------
// read data from the buffer
// input:   bf--buffer pointer
//          idx--data index, idx=0 means the latest data, idx=1 means data before the latest...
// output:
// return: 1 menas OK, 0 menas idx out of bound
int bfGet(Buffer *bf,real *d, int idx);

//-------------------------------------------------------------------
// clear the buffer
// input:   bf--buffer pointer
// output:
// return:
void bfClear(Buffer *bf);


#endif // BUFFER_H_INCLUDED
