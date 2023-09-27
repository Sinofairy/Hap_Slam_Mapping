#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <opencv/cv.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <queue>
#include <cmath>
#include <vector>
#include <map>
#include "feature.h"
#include <algorithm>
#include "include/InvoAlgSlamTypes.h"
#include "include/InvoAlgPsdTypes.h"
#include "include/InvoAlgMosTypes.h"
using namespace pcl;
 

 

// ##############################################################################
//   1. get IPM picture and Odomtry data; confirm that the time is synchronism;
//   2. get muilti obeject seg result;
//   3. transform feature map to pointcloud map; transform veichle coords to world coords;
//   4. Preprocess Pointcloud(remove far or near points) ;
//   5. compute robot's pose with method of ndt or icp;
//   6. add segement result to PointCloud , brocast Pose message;
// ###############################################################################

//  3. transform feature map to pointcloud map; transform veichle coords to world coords
//     receive extracted feature map, then add to  point cloud;

#define PointSet 1
typedef pcl::PointXYZRGB PointType;
std::vector<int> PsdPool;
std::vector<int> MosPool;
std::map<int,int> PsdMap;
std::map<int,int> MosMap;

/*定义最大车位点集个数和检测输出车位的个数*/
#define MAX_POINTSETS_NUM    1024
// #define MAX_PARKSLOTS_NUM     14
#define ContourNum 20
/*定义保存车位点集的新结构体，与原有车位输出位置一致*/
typedef struct
{
	int32_t                 iPointSetsNum;
	tInvoAlgImagePoint      point_set[MAX_POINTSETS_NUM];
}tInvoPsdCropInfo;
typedef struct
{
	uint32_t number;
	tInvoPsdCropInfo PsdInfo[INVO_PSD_PARKSLOT_NUM];
}tInvoPsdListCropInfo;
/*定义存储变量*/
tInvoPsdListCropInfo g_stPsdCropInfo;
/*车位点集裁剪接口函数声明*/
static void CropParkSlotPointSets(tInvoPsdObjList* pstPsdObj);

typedef struct _ContourPoints
{
    uint32_t num;
    tInvoAlgPixelPoint pts[INVO_MOS_CONTOUR_NUM];
} ContourPoints;

#if PointSet
typedef struct _ContourPointSet
{
    uint32_t num;
    tInvoAlgPixelPoint pts[INVO_MOS_POINTSET_NUM];
} ContourPointSet;
#endif

typedef struct _Contour
{
    uint32_t type;
    uint32_t area;
    float score;
    tInvoAlgPixelPoint orientationPt;
    tInvoAlgPixelPoint box[INVO_MOS_BOXPOINT_NUM];
    ContourPoints contourPoints;
#if PointSet
    ContourPointSet contourPointSet;
#endif
} Contour;

typedef struct _ContourList
{
    uint32_t num;
    Contour contours[ContourNum];
} ContourList;

void getPointSet(tInvoMosObjList * mosObjects, ContourList *contours);

void TransVihcle2World(OdomData* OdoData,
                      tInvoMosObjList* MosData,
                      tInvoPsdObjList* stPsdData,
                      pcl::PointCloud<PointType> &cameraCloud)
{

    int OdoTime = OdoData->time;
    int MosTime = MosData->timer;
    int PsdTime = stPsdData->timer;
    //std::cout << "OdoTime: " << OdoTime << std::endl;
    //std::cout << "MosTime: " << MosTime << std::endl;

    PointType map_point;
    int time_thres = 50;
    int cameraRealiableDis = 15000;
    // pcl::PointCloud<PointType>::Ptr map_points;

    //  3.1 confirm that the time is synchronism;
    // double seg_time = it->time;
    if (abs(OdoTime - MosTime) > time_thres || abs(OdoTime - PsdTime) > time_thres)
    {
        std::cout << "time is not sysn........." << std::endl;
        //return;
    }
    // 3.2 get SEGMENTation class according the type number ;
    //     transform feature map to pointcloud map;
    //     trans from image coord to vihcle coord
    //     trans from vihcle coord to world coord
    cameraCloud.clear();

    ContourList* Contours = new(ContourList);

    getPointSet(MosData, Contours);

    int ObjNum = Contours->num;

    if(MosData->ivalid)
    {
        for (int i = 0; i < ObjNum; i++)
        {
            Contour obj = Contours->contours[i];
            int PointNum = obj.contourPointSet.num;
            INVO_MOS_OBJECT_TYPE cls_type = MosData->objects[i].type;
            int key = MosData->objects[i].track_id;
            if(MosMap.count(key) == 0)
            {
                MosMap[key] = 1;
            }
            // if(std::find(MosPool.begin(), MosPool.end(), key) != MosPool.end())
            // {
            //     continue;
            // }
            else if (MosMap[key] > 30)
            {
                continue;
            }
            else{
                MosMap[key] += 1;
            }
            
            if(MosMap[key] % 3 == 1)
            {
                //MosPool.push_back(key);
                for (int j = 0; j < PointNum; j++)
                {
                    // image coords to vehicle coords
                    int x = obj.contourPointSet.pts[j].x * 4 - 640;
                    int y = 640 - obj.contourPointSet.pts[j].y * 4;
                    int z = 0;

                    Eigen::Vector4d point_curr(x, y, z, 1);
                    //std::cout << "point_curr: " << point_curr.transpose() << std::endl;
                    Eigen::Vector4d point_w = OdoData->pose * point_curr;
                    //std::cout << "OdoData->pose: " << OdoData->pose.transpose() << std::endl;
                    //std::cout << "point_w: " << point_w.transpose() << std::endl;
                    map_point.x = point_w[0];
                    map_point.y = point_w[1];
                    map_point.z = point_w[2];

                    double dis = sqrt(map_point.x * map_point.x + map_point.y * map_point.y);
                    if (dis > cameraRealiableDis)
                        //continue;

                    map_point.r = id_label_map[cls_type].red;
                    map_point.g = id_label_map[cls_type].green;
                    map_point.b = id_label_map[cls_type].blue;

                    cameraCloud.push_back(map_point);
                }
            }
        }

    }
    

    CropParkSlotPointSets(stPsdData);
    int PsdNum = g_stPsdCropInfo.number;
    if(stPsdData->ivalid)
    {
        for (int i = 0; i < PsdNum; i++)
        {
            tInvoPsdCropInfo ps = g_stPsdCropInfo.PsdInfo[i];
            int PointNum = ps.iPointSetsNum;

            int ps_type = stPsdData->parkslots[i].type;
            int Pkey = stPsdData->parkslots[i].index;
            // if(std::find(PsdPool.begin(), PsdPool.end(), Pkey) != PsdPool.end())
            // {
            //     continue;
            // }
            // MosPool.push_back(Pkey);
            if(PsdMap.count(Pkey) == 0)
            {
                PsdMap[Pkey] = 1;
            }
            else if (PsdMap[Pkey] > 30)
            {
                continue;
            }
            else
            {
                PsdMap[Pkey] += 1;
            }

            if(PsdMap[Pkey] % 3 == 1)
            {
                for (int j = 0; j < PointNum; j++)
                {
                    // image coords to vehicle coords
                    int x = ps.point_set[j].x * 4 - 640;
                    int y = 640 - ps.point_set[j].y * 4;
                    int z = 0;

                    Eigen::Vector4d point_curr(x, y, z, 1);
                    //std::cout << "point_curr: " << point_curr.transpose() << std::endl;
                    Eigen::Vector4d point_w = OdoData->pose * point_curr;
                    //std::cout << "OdoData->pose: " << OdoData->pose.transpose() << std::endl;
                    //std::cout << "point_w: " << point_w.transpose() << std::endl;
                    map_point.x = point_w[0];
                    map_point.y = point_w[1];
                    map_point.z = point_w[2];

                    double dis = sqrt(map_point.x * map_point.x + map_point.y * map_point.y);
                    if (dis > cameraRealiableDis)
                        //continue;

                    map_point.r = ps_label_map[ps_type].red;
                    map_point.g = ps_label_map[ps_type].green;
                    map_point.b = ps_label_map[ps_type].blue;

                    cameraCloud.push_back(map_point);
                }
            }
        }

    }
    
}

//4. Preprocess Pointcloud(remove far or near points) ;
int imageRowIncrease = 1;
int imageColIncrease = 1;
// double cameraRealiableDis=15.0;
int skyColor = 178;
float pointCloudLeafSize = 0.1; //0.1;
double rotateDeg1, rotateDeg2, rotateDeg3, rotateDeg4, rotateDeg5;

// delete close point
//template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointType> &cloud_in,
                            pcl::PointCloud<PointType> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

// delete far point
//template <typename PointT>
void removeFarPointCloud(const pcl::PointCloud<PointType> &cloud_in,
                         pcl::PointCloud<PointType> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z > thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

//   5. compute robot's pose with method of ndt or icp;
//   match current feature point cloud to  global map;
//   compute robot's pose with method of ndt or icp;
//   create new keyFrame, add pointCloud to global map
//   feature extration
//   feature registration
//   compute robot's pose
//   create new keyFrame, and add pointCloud to global map
//   typedef pcl::PointXYZRGB PointType;

pcl::PointCloud<PointType>::Ptr currentFeatureCloud(new pcl::PointCloud<PointType>());
//pcl::PointCloud<PointType>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr globalFeatureCloud(new pcl::PointCloud<PointType>());

bool systemInitial = false;

double odomKeyFramDisThresh = 1.0;

double lastPoseX = -20;
double lastPoseY = -20;
double lastPoseZ = -20;
double lastPoseRoll = 0;
double lastPosePitch = 0;
double lastPoseYaw = 0;

float currentX = 0;
float currentY = 0;
float currentZ = 0;
float currentRoll = 0;
float currentPitch = 0;
float currentYaw = 0;

int frameCount = 0;
int invalidColorThresh = 60;

double icpMaxCorrespondenceDistance = 50;//20
int icpMaximumIterations = 100;
double icpTransformationEpsilon = 1e-10;
double icpEuclideanFitnessEpsilon = 0.001;
double icpFitnessScoreThresh = 0.8; //0.3

double ndtTransformationEpsilon = 1e-10;
double ndtResolution = 0.1;
double ndtFitnessScoreThresh = 0.3;

std::string mapSaveLocation = "./data/icpLaserCloud.pcd";
//float pointCloudLeafSize = 0.1;
bool useICP = true;
bool useNDT = false;
bool mapSave = true;

pcl::PointCloud<pcl::PointXYZRGB> cameraCloudIn;

void GetNewRT(pcl::PointCloud<PointType>::Ptr currentFeatureCloud, 
              pcl::PointCloud<PointType>::Ptr globalFeatureCloud,
              Eigen::Affine3f& transWorldCurrent){
// only number of point of current frame is sufficient ,compute pose
    if (currentFeatureCloud->points.size() < 10)
        return;

    if (!systemInitial)
    {

        //  global map initialize
        *globalFeatureCloud = *globalFeatureCloud + *currentFeatureCloud;

        lastPoseX = 0;
        lastPoseY = 0;
        lastPoseZ = 0;
        systemInitial = true;
        return;
    }

    // compute robot's pose with method of ndt
    if (useNDT)
    {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(ndtTransformationEpsilon);
        ndt.setResolution(ndtResolution);
        ndt.setInputSource(currentFeatureCloud);
        ndt.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
        ndt.align(*transCurrentCloudInWorld, transWorldCurrent.matrix());
        if (ndt.hasConverged() == false || ndt.getFitnessScore() > ndtFitnessScoreThresh)
        {
            std::cout << "ndt locolization failed;    the score is   " << ndt.getFitnessScore() << std::endl;
            return;
        }
        else
        {
            transWorldCurrent = ndt.getFinalTransformation();
            std::cout << "ndt locolization succed;    the score is   " << ndt.getFitnessScore() << std::endl;
        }
            
    }

    //   compute robot's pose with method of icp
    if (useICP)
    {
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance);
        icp.setMaximumIterations(icpMaximumIterations);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);

        icp.setInputSource(currentFeatureCloud);
        icp.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
        icp.align(*transCurrentCloudInWorld, transWorldCurrent.matrix());

        if (icp.hasConverged() == false || icp.getFitnessScore() > icpFitnessScoreThresh)
        {
            std::cout << "ICP locolization failed    the score is   " << icp.getFitnessScore() << std::endl;
            return;
        }
        else
            transWorldCurrent = icp.getFinalTransformation();
    }
}


//点集去重
void uniqueForPointSet(tInvoAlgPixelPoint *pts, uint32_t *ptNum)
{
    uint32_t i, j;
    int32_t lastX = 0, lastY = 0, currentX = 0, currentY = 0;
    uint32_t indexSet[1024] = { 0 };
    uint32_t indexSetNum = 0;
    bool delFlag = FALSE;
    uint32_t ptNumNew = 0;
    for (i = 0; i < *ptNum; i++)
    {
        currentX = pts[i].x, currentY = pts[i].y;
        if (currentX == lastX && currentY == lastY)
        {
            indexSet[indexSetNum] = i;
            indexSetNum++;
        }
        lastX = currentX, lastY = currentY;
    }
    for (i = 0; i < *ptNum; i++)
    {
        for (j = 0; j < indexSetNum; j++)
        {
            if (i == indexSet[j])
            {
                delFlag = TRUE;
                break;
            }
        }
        pts[ptNumNew] = pts[i];
        if (!delFlag)
        {
            ptNumNew++;
        }
        delFlag = FALSE;
    }
    *ptNum = ptNumNew;
}

//点集排序（按x升序）
void sortByX(tInvoAlgPixelPoint *pts, uint32_t ptNum)
{
    uint32_t i, j;
    tInvoAlgPixelPoint tempPt;
    for (i = 0; i < ptNum - 1; i++)
    {
        for (j = 0; j < ptNum - i - 1; j++)
        {
            if (pts[j].x > pts[j + 1].x)
            {
                tempPt.x = pts[j].x;
                tempPt.y = pts[j].y;
                pts[j].x = pts[j + 1].x;
                pts[j].y = pts[j + 1].y;
                pts[j + 1] = tempPt;
            }
        }
    }
}

//点集排序（按y升序）
void sortByY(tInvoAlgPixelPoint *pts, uint32_t ptNum)
{
    uint32_t i, j;
    tInvoAlgPixelPoint tempPt;
    for (i = 0; i < ptNum - 1; i++)
    {
        for (j = 0; j < ptNum - i - 1; j++)
        {
            if (pts[j].y > pts[j + 1].y)
            {
                tempPt.x = pts[j].x;
                tempPt.y = pts[j].y;
                pts[j].x = pts[j + 1].x;
                pts[j].y = pts[j + 1].y;
                pts[j + 1] = tempPt;
            }
        }
    }
}

static void getObjPointSetByContour(tInvoMosObj * object, Contour *contour)
{
    int i;
    contour->contourPoints.num = object->image_info.counter_num;
    for (i = 0; i < object->image_info.counter_num; i++)
    {
        contour->contourPoints.pts[i].x = object->image_info.counter[i].x;
        contour->contourPoints.pts[i].y = object->image_info.counter[i].y;
    }
    
    uint32_t lastX = 0, lastY = 0, currentX = 0, currentY = 0;
    uint32_t startY = 0;
    sortByY(contour->contourPoints.pts, contour->contourPoints.num);
    sortByX(contour->contourPoints.pts, contour->contourPoints.num);
    uniqueForPointSet(contour->contourPoints.pts, &contour->contourPoints.num);
    contour->contourPointSet.num = 0;
    for (i = 0; i < contour->contourPoints.num; i++)
    {
        currentX = contour->contourPoints.pts[i].x, currentY = contour->contourPoints.pts[i].y;
        if (i)
        {
            if (lastX != currentX)
            {
                while (startY <= lastY)
                {
                    startY++;
                    contour->contourPointSet.pts[contour->contourPointSet.num].x = lastX;
                    contour->contourPointSet.pts[contour->contourPointSet.num].y = startY;
                    contour->contourPointSet.num++;
                }
                startY = currentY;
            }
        }
        else
        {
            startY = currentY;
        }
        lastX = currentX, lastY = currentY;
    }
}

void getPointSet(tInvoMosObjList * mosObjects, ContourList *contours)
{
    int i;
    contours->num = mosObjects->number;
    for (i = 0; i < mosObjects->number; i++)
    {
        getObjPointSetByContour(&(mosObjects->objects[i]), &(contours->contours[i]));
    }
}

/*车位点集裁剪接口函数定义*/
static void CropParkSlotPointSets(tInvoPsdObjList* pstPsdObj)
{
	float iLeftUpCenterX, iLeftUpCenterY, iRightUpCenterX, iRightUpCenterY;
	float iLeftDownCenterX, iLeftDownCenterY, iRightDownCenterX, iRightDownCenterY;
	float iMinX, iMaxX, iMinY, iMaxY, iTmpX, iTmpY;
	int32_t k, i, j, m;
	tInvoPsdImageInfo* pstImageInfo;

	/*Init*/
	memset(&g_stPsdCropInfo, 0, sizeof(tInvoPsdListCropInfo));

	g_stPsdCropInfo.number = pstPsdObj->number;


	for (k = 0; k < pstPsdObj->number; k++)
	{
		pstImageInfo = &pstPsdObj->parkslots[k].image_info;
		iLeftUpCenterX = (pstImageInfo->point_set[0].x + pstImageInfo->point_set[4].x) / 2;
		iLeftUpCenterY = (pstImageInfo->point_set[0].y + pstImageInfo->point_set[4].y) / 2;
		iRightUpCenterX = (pstImageInfo->point_set[1].x + pstImageInfo->point_set[5].x) / 2;
		iRightUpCenterY = (pstImageInfo->point_set[1].y + pstImageInfo->point_set[5].y) / 2;
		iLeftDownCenterX = (pstImageInfo->point_set[3].x + pstImageInfo->point_set[7].x) / 2;
		iLeftDownCenterY = (pstImageInfo->point_set[3].y + pstImageInfo->point_set[7].y) / 2;
		iRightDownCenterX = (pstImageInfo->point_set[2].x + pstImageInfo->point_set[6].x) / 2;
		iRightDownCenterY = (pstImageInfo->point_set[2].y + pstImageInfo->point_set[6].y) / 2;

		/*leftup to rightup*/
		iMinX = max(iLeftUpCenterX, (float)0.0);
		iMaxX = min(iRightUpCenterX, (float)320.0);
		
		for (i = (int32_t)iMinX + 1; i < (int32_t)iMaxX; i += 2/*i++*/)
		{
			iTmpY = iLeftUpCenterY + (iRightUpCenterY - iLeftUpCenterY) * (i - iLeftUpCenterX) / (iRightUpCenterX - iLeftUpCenterX);
			if (g_stPsdCropInfo.PsdInfo[k].iPointSetsNum < MAX_POINTSETS_NUM)
			{
				g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].x = i;
				g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].y = iTmpY;
				g_stPsdCropInfo.PsdInfo[k].iPointSetsNum++;
				for (m = 0; m < 3; m++)
				{
					g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].x = i;
					g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].y = iTmpY + m - 1;
					g_stPsdCropInfo.PsdInfo[k].iPointSetsNum++;
				}	
			}
		}
		/*leftdown to rightdown*/
		iMinX = max(iLeftDownCenterX, (float)0.0);
		iMaxX = min(iRightDownCenterX, (float)320.0);
		
		for (i = (int32_t)iMinX + 1; i < (int32_t)iMaxX; i += 2/*i++*/)
		{
			iTmpY = iLeftDownCenterY + (iRightDownCenterY - iLeftDownCenterY) * (i - iLeftDownCenterX) / (iRightDownCenterX - iLeftDownCenterX);
			if (g_stPsdCropInfo.PsdInfo[k].iPointSetsNum < MAX_POINTSETS_NUM)
			{
				g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].x = i;
				g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].y = iTmpY;
				g_stPsdCropInfo.PsdInfo[k].iPointSetsNum++;
				for (m = 0; m < 3; m++)
				{
					g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].x = i;
					g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].y = iTmpY + m - 1;
					g_stPsdCropInfo.PsdInfo[k].iPointSetsNum++;
				}
			}
		}
		/*leftup to leftdown and rightup to rightdown*/
		iMinY = max(iLeftUpCenterY, (float)0.0);
		iMaxY = min(iLeftDownCenterY, (float)320.0);

		for (j = (int32_t)iMinY + 1; j < (int32_t)iMaxY; j += 2/*i++*/)
		{
			iTmpX = iLeftUpCenterX + (iLeftUpCenterX - iLeftDownCenterX) * (j - iLeftUpCenterY) / (iLeftUpCenterY - iLeftDownCenterY);
			if (g_stPsdCropInfo.PsdInfo[k].iPointSetsNum < MAX_POINTSETS_NUM && (iTmpX > 0.0 && iTmpX < 320.0))
			{
				g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].x = iTmpX;
				g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].y = j;
				g_stPsdCropInfo.PsdInfo[k].iPointSetsNum++;
				for (m = 0; m < 3; m++)
				{
					g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].x = iTmpX + m - 1;
					g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].y = j;
					g_stPsdCropInfo.PsdInfo[k].iPointSetsNum++;
				}
			}
		}
		/*rightup to rightdown*/
		iMinY = max(iRightUpCenterY, (float)0.0);
		iMaxY = min(iRightDownCenterY, (float)320.0);

		for (j = (int32_t)iMinY + 1; j < (int32_t)iMaxY; j += 2/*i++*/)
		{
			iTmpX = iRightUpCenterX + (iRightUpCenterX - iRightDownCenterX) * (j - iRightUpCenterY) / (iRightUpCenterY - iRightDownCenterY);
			if (g_stPsdCropInfo.PsdInfo[k].iPointSetsNum < MAX_POINTSETS_NUM && (iTmpX > 0.0 && iTmpX < 320.0))
			{
				g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].x = iTmpX;
				g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].y = j;
				g_stPsdCropInfo.PsdInfo[k].iPointSetsNum++;
				for (m = 0; m < 3; m++)
				{
					g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].x = iTmpX + m - 1;
					g_stPsdCropInfo.PsdInfo[k].point_set[g_stPsdCropInfo.PsdInfo[k].iPointSetsNum].y = j;
					g_stPsdCropInfo.PsdInfo[k].iPointSetsNum++;
				}
			}
		}
	}
}

void saveMap_callback(bool savemap)
{
    if(!savemap){
        return;
    }
    std::cout << "map save start" << std::endl;
    pcl::PointCloud<PointType> globalMap = *globalFeatureCloud;
    pcl::io::savePCDFileASCII(mapSaveLocation, *globalFeatureCloud);
    std::cout << "map save over" << std::endl;
}