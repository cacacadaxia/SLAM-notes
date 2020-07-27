#### opencv相关内容

[TOC]

##### 1. cv::Mat

CV_8UC1 是指一个8位无符号整型单通道矩阵,
CV_32FC2是指一个32位浮点型双通道矩阵

```python
//表示矩阵
cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, Scalar::all(0));  /* 摄像机内参数矩阵 */
cameraMatrix.at<float>(0,0) = fx;/*就是和vector一样的，对吧？*/

/*另外一种赋值的方法*/
Mat K = ( Mat_<double> ( 3,3 ) << 2080.553808095492, 0, 702.9939963321261,0, 2060.785293956283, 531.9695272311889,0, 0, 1);
```



<span style='color:red'>**InputArray**</span>代表啥意思？

```python
CV_EXPORTS_W Mat findEssentialMat( InputArray points1, InputArray points2,
                                 InputArray cameraMatrix, int method = RANSAC,
                                 double prob = 0.999, double threshold = 1.0,
                                 OutputArray mask = noArray() );
```



class KeyPoint

{

Point2f  pt;    //特征点的坐标

float  size;     //特征点邻域直径

float  angle; //特征点的方向0-360，负值表示不使用,有了这个方向,能够让特征点拥有更高的辨识度,否则仅仅坐标和直径有时会误判特征点

float  response;//响应程度,代表该点的强壮程度,也就是该点角点程度,用于后期使用和排序

int  octave; //特征点所在的图像金字塔的组

int  class_id; //用于聚类的id

}


