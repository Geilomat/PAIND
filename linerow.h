#ifndef LINEROW_H
#define LINEROW_H

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>



typedef struct line{
  int x;
  int q;
  int m;
  int r;
  int value;
}line_t;

typedef line_t* line_p;


class LineRow
{
public:
  LineRow(int size, int density, float maxDifference);
  void setNewLine(pcl::PointCloud<pcl::PointXYZ> cloudPiece, int position);
  line_p getRowArray();
};

#endif // LINEROW_H
