#include "linerow.h"


LineRow::LineRow(int size, int density, float maxDifference) : size(size), density(density), maxDifference(maxDifference)
{
  RowArray = new line_t[size];
}

/* Set a new Line out of the given PointCloud piece
 * @param cloudPiece: the piece of the PointCloud which a line should be fit on.
 * @param position: The starting position of the new line. has to be a one time int, else it would override a other line.
 *
 */
void LineRow::setNewLine(pcl::PointCloud<pcl::PointXYZ> cloudPiece, int position){
  int x = position;
  int value;
  float q;
  float m;
  float r;

  if(cloudPiece.width < density){
    value = -1;
    q = 0;
    m = 0;
    r = 0;
  }
  else{
    float xMean = 0;
    float yMean = 0;

    for(int i =0; i< cloudPiece.width;i++){
      xMean += cloudPiece[i].x;
      yMean += cloudPiece[i].y;
    }

    xMean = xMean/cloudPiece.width;
    yMean = yMean/cloudPiece.width;

    float numerator = 0;
    float denumerator = 0;

    for(int i = 0; i < cloudPiece.width; i++){
      numerator += (cloudPiece[i].x-xMean)*(cloudPiece[i].y - yMean);
      denumerator += cloudPiece[i].x * cloudPiece[i].x;
    }

    m = numerator/(denumerator * cloudPiece.width * xMean * yMean);
    q = yMean - m*xMean;

    r = 0;
    int counter = 0;
    for(int i = 0; i < cloudPiece.width; i++){
      float onePointError = m * cloudPiece[i].x + q - cloudPiece[i].y;

      if(abs((int)onePointError) > maxDifference){  //Test if the Error is greater then the maximal acepted Difference
        counter ++;
      }
      r += onePointError;
    }

    if(counter > 2){ //If there are too much height difference in more then one point -> probebly a staff or something
      value = -1;
    }
  }

  //Save calculated values int the Array
  RowArray[iterator].x = x;
  RowArray[iterator].q = q;
  RowArray[iterator].m = m;
  RowArray[iterator].r = r;
  RowArray[iterator].value = value;

  iterator ++;
}

line_p LineRow::getRowArray(){
  return RowArray;
}

int LineRow::getSize(){
  return size;
}

//int LineRow::getLineValueByPosition(int position){
//  for(int i = 0 ; i < size; i++){

//  }
//  else{
//    return -1;
//  }
//}

//int LineRow::getLineValueByIndex(int index){
//  if(index < size){
//    return RowArray[index].value;
//  }
//  else{
//    return -2;
//  }
//}

//float LineRow::getLineHight(int x){
//  if(x < size){
//    return CloumnArray[x].q;
//  }
//  else{
//    return -1;
//  }
//}
