/*
*  essential library that deals with matrix indexing
*  federico.corradi@inilabs.com
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct ImageCoordinate {
   int x;
   int y;
   int index;
   unsigned char* image_data;   
   int sizeX;
   int sizeY;
}ImageCoordinate;

void calculateIndex(ImageCoordinate *coordinates, int columns,int x,int  y);
void calculateCoordinates(ImageCoordinate *coordinates, int index,int columns, int rows);
void ImageCoordinateInit(ImageCoordinate *ts, int sizex, int sizey, int channels);
void normalizeImage(ImageCoordinate *ar);

void ImageCoordinateInit(ImageCoordinate *ts, int sizeX, int sizeY, int channel)
{
    /* Set ts options to be ready */
    ts->x = NULL;
    ts->y = NULL;
    ts->index = NULL;
    ts->sizeX = sizeX;
    ts->sizeY = sizeY;
    ts->image_data = (unsigned char*)malloc(sizeX*sizeY*channel);
}


void calculateIndex(ImageCoordinate *ar, int columns,int x,int  y){
    ar->index = y * columns + x;
    return;
}

void calculateCoordinates(ImageCoordinate *ar, int index,int columns, int rows){
    int i =0;
    //for each row
    for(i=0; i<rows; i++){
        //check if the index parameter is in the row
        if(index < (columns * i) + columns && index >= columns * i){
            //return x, y
	    ar->x = index - columns * i;
	    ar->y = i;
        }
    }
    return;
}

void normalizeImage(ImageCoordinate *ar){
    int sum,i,j = 0;
    double tmp = 0.0;
    //printf("size X %d size Y %d\n", ar->sizeX, ar->sizeY);
    for(i=0; i<ar->sizeX; i++){
    	for(j=0; j<ar->sizeY; j++){
    		calculateIndex(ar, ar->sizeY, i, j); 
		sum = sum + ar->image_data[ar->index]; 	
        }
    }
    for(i=0; i<ar->sizeX; i++){
	for(j=0; j<ar->sizeY; j++){
		calculateIndex(ar, ar->sizeY, i, j);
		tmp = (double)(ar->image_data[ar->index]);
	 	ar->image_data[ar->index] = (int) (tmp * (255));
	        //printf("data[%d] : %u x:%d y:%d\n" , ar->index, (unsigned char)ar->image_data[ar->index], i, j );
	}
    }
    //printf("sum %d\n", sum);
}

