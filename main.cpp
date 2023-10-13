#include "ParseData.hpp"
#include <yaml-cpp/yaml.h>
#include "mapping.hpp"
#include <time.h> 

//##########################################################################################
//  1. get IPM picture and Odomtry data; confirm that the time is synchronism;
//  2. get muilti obeject seg result;
//  3. transform feature map to pointcloud map; transform veichle coords to world coords;
//  4. Preprocess Pointcloud(remove far or near points) ;
//  5. compute robot's pose with method of ndt or icp; 
//  6. add segement result to PointCloud , brocast Pose message;
//###########################################################################################

int main()
{
     // get parameter from config file 
    std::string config_path = "./config/configFile.yaml";
    std::cout << config_path << std::endl;
    YAML::Node config = YAML::LoadFile(config_path);
    //YAML::Node config = YAML::LoadFile("../config/config.yaml");
    //std::cout << "api: " << config["api"].as<std::string>() << std::endl;
    //std::cout << "v: " << config["v"].as<int>() << std::endl;
    std::string mapSaveLocation = config["mapSaveLocation"].as<std::string>();
    double odomKeyFramDisThresh = config["odomKeyFramDisThresh"].as<double>(); 
    int invalidColorThresh = config["InvalidColorThresh"].as<int>();
    bool useICP = config["useICP"].as<bool>();
    double icpMaxCorrespondenceDistance = config["IcpMaxCorrespondenceDistance"].as<double>();
    int icpMaximumIterations = config["IcpMaximumIterations"].as<int>();
    double icpTransformationEpsilon = config["IcpTransformationEpsilon"].as<double>();
    double icpEuclideanFitnessEpsilon = config["IcpEuclideanFitnessEpsilon"].as<double>();
    double icpFitnessScoreThresh = config["IcpFitnessScoreThresh"].as<double>();
    float pointCloudLeafSize = config["pointCloudLeafSize"].as<float>();
    bool useNDT = config["useNDT"].as<bool>();
    double ndtTransformationEpsilon = config["ndtTransformationEpsilon"].as<double>();
    double ndtResolution = config["ndtResolution"].as<double>();
    double ndtFitnessScoreThresh = config["ndtFitnessScoreThresh"].as<double>();
    bool mapSave = config["mapSave"].as<bool>();
    double farPointThresh = config["farPointThresh"].as<double>();
    double closePointThresh = config["closePointThresh"].as<double>();

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));

    
	FILE* inOdomFile = fopen("./data/odom/NOR_20230309_060320_M_00001/imagePose.txt", "rb");
	OdomData tOdomData;
	// MosData stMosData;	
	// PsdData stPsdData;
	tInvoPsdObjList tPsdData;
	tInvoMosObjList tMosData;
	/*calc odom frame num*/
	int32_t iOdomFrame = -1;

	CalcOdomFrameAndTimeStamp(inOdomFile, &iOdomFrame);
	fseek(inOdomFile, 0, SEEK_SET);

	vector<string> MosFile;
	MosFile = getFileList(std::string("data/mos/NOR_20230309_060320_M_00001/postprocess_mos/"));
	sort(MosFile.begin(), MosFile.end());
	int32_t iMosFrame = 0;
	iMosFrame = MosFile.size();
	vector<string> PsdFile;
	PsdFile = getFileList(std::string("data/psd/NOR_20230309_060320_M_00001/json/"));
	sort(PsdFile.begin(), PsdFile.end());
	int32_t iPsdFrame = 0;
	iPsdFrame = PsdFile.size();

	int32_t iMinFrame = min(max(iMosFrame, iPsdFrame), iOdomFrame);

	double dMinTime = 0.0, dMaxTime = 0.0;
	time_t start, end, start_0, end_0; 
	start_0 = clock();

	for (int32_t i = 0; i < iMinFrame; i++)
	{
		start=clock();
		memset(&tOdomData, 0, sizeof(OdomData));
		memset(&tMosData, 0, sizeof(tInvoMosObjList));
		memset(&tPsdData, 0, sizeof(tInvoPsdObjList));
		if(g_iOdomFrame >= iOdomFrame || g_iMosFrame >= iMosFrame || g_iPsdFrame >= iPsdFrame)
		{
			break;
		}
        //##############################################################################
	    //1. get IPM picture,muilti obeject seg result and psd data;
        //##############################################################################
		ReadImuGnssData(inOdomFile, &tOdomData);
		//cout << "begain: " << endl;
		//cout << "Odom time:" << stOdomData.time << endl;
		ReadMosData(MosFile, &tMosData);
		//cout << "Mos time:" << stMosData.time << endl;
		ReadPsdData(PsdFile, &tPsdData);
		//cout << "Psd time:" << stPsdData.time << endl;
		//cout << endl;

        //##############################################################################
        //2. confirm that the time is synchronism;
        //##############################################################################
		dMinTime = min(double(min(tMosData.timer, tPsdData.timer)), tOdomData.time);
		dMaxTime = max(double(min(tMosData.timer, tPsdData.timer)), tOdomData.time);
		/*check is same frame*/
		if (tOdomData.time == dMinTime && dMaxTime != dMinTime)
		{
			while (tOdomData.time < dMaxTime)
			{
				g_iOdomFrame++;
				memset(&tOdomData, 0, sizeof(OdomData));
				if (g_iOdomFrame >= iOdomFrame)
				{
					break;
				}
				ReadImuGnssData(inOdomFile, &tOdomData);
				/*cout << "Odom time:" << stOdomData.time << endl;*/
				if (tOdomData.time == dMaxTime)
				{
					break;
				}
			}
		}
		if (tMosData.timer == dMinTime && tMosData.timer < tOdomData.time)
		{
			while (tMosData.timer < tOdomData.time)
			{
				g_iMosFrame++;
				memset(&tMosData, 0, sizeof(tInvoMosObjList));
				if (g_iMosFrame >= iMosFrame)
				{
					break;
				}
				ReadMosData(MosFile, &tMosData);
				/*cout << "Mos time:" << stMosData.time << endl;*/
				if (tMosData.timer == tOdomData.time/*dMaxTime*/)
				{
					break;
				}
			}
		}

		if (tPsdData.timer == dMinTime && tPsdData.timer < tOdomData.time)
		{
			while (tPsdData.timer < tOdomData.time)
			{
				g_iPsdFrame++;
				memset(&tPsdData, 0, sizeof(tInvoPsdObjList));
				if (g_iPsdFrame >= iPsdFrame)
				{
					break;
				}
				ReadPsdData(PsdFile, &tPsdData);
				/*cout << "Psd time:" << stPsdData.time << endl;*/
				if (tPsdData.timer == tOdomData.time/*dMaxTime*/)
				{
					break;
				}
			}
		}

		/*three odom, mos and psd same timestamp*/
		if (tOdomData.time == tMosData.timer && tOdomData.time == tPsdData.timer)
		{
			tMosData.ivalid = 1;
			tPsdData.ivalid = 1;
		}
		/*odom and mos same timestamp*/
		else if (tOdomData.time == tMosData.timer)
		{
			tMosData.ivalid = 1;
			tPsdData.ivalid = 0;
		}
		/*odom and psd same timestamp*/
		else if (tOdomData.time == tPsdData.timer)
		{
			tMosData.ivalid = 0;
			tPsdData.ivalid = 1;
		}
		/*invalid, no output*/
		else
		{
			tMosData.ivalid = 0;
			tPsdData.ivalid = 0;
		}

		cout << "begain: " << endl;
		cout << "Odom time:" << tOdomData.time << endl;
		cout << "Mos time:" << tMosData.timer << endl;
		cout << "Psd time:" << tPsdData.timer << endl;
		cout << endl;
        //##############################################################################
        //  3. transform feature map to pointcloud map; transform veichle coords to world coords
        //     receive extracted feature map, then add to  point cloud;
        //     MosObject obj = stMosData.stMosObj[i];
        //##############################################################################
        TransVihcle2World(&tOdomData, &tMosData, &tPsdData, *currentFeatureCloud);
        //std::cout << "local map  save start" << std::endl;
        //pcl::PointCloud<PointType> globalMap = *globalFeatureCloud;
        //pcl::io::savePCDFileASCII(mapSaveLocation, *currentFeatureCloud);
        //std::cout << "local map save over" << std::endl;
        //创建viewer对象
        // pcl::visualization::CloudViewer viewer("Cloud Viewer");
        // viewer.showCloud(currentFeatureCloud);
        // viewer.runOnVisualizationThreadOnce(viewerOneOff);
        // system("pause");
		
        //##############################################################################
        //  4. Preprocess Pointcloud(remove far or near points) ;
        //##############################################################################
        pcl::PointCloud<PointType> cloud_out;
        //removeClosedPointCloud(*currentFeatureCloud, cloud_out, closePointThresh);
        //removeFarPointCloud(cloud_out, *currentFeatureCloud, farPointThresh);

        //##############################################################################
        //  5. compute robot's pose with method of ndt or icp;
        //##############################################################################
        Eigen::Affine3f transWorldCurrent;
        GetNewRT(currentFeatureCloud, globalFeatureCloud, transWorldCurrent);
        pcl::getTranslationAndEulerAngles(transWorldCurrent, currentX, currentY, currentZ, currentRoll, currentPitch, currentYaw);
        std::cout << "now robot is in x " << transWorldCurrent(0, 3) << " y " << transWorldCurrent(1, 3) << "  z  " << transWorldCurrent(2, 3) << std::endl;
        
        //##############################################################################
        //  6. add segement result to PointCloud
        //  create new keyFrame, and add pointCloud to global map
        //##############################################################################
        double dis = (currentX - lastPoseX) * (currentX - lastPoseX) + (currentY - lastPoseY) * (currentY - lastPoseY) + (currentZ - lastPoseZ) * (currentZ - lastPoseZ);
        frameCount++;
        std::cout << "dis: " << dis << std::endl;
        if (dis > odomKeyFramDisThresh || frameCount >= 1)
        {
            *globalFeatureCloud = *globalFeatureCloud + *currentFeatureCloud;
            pcl::PointCloud<PointType> globalMapDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(globalFeatureCloud);
            downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
            downSizeFilter.filter(globalMapDS);
            *globalFeatureCloud = globalMapDS;

			//pcl::visualization::CloudViewer viewer("simple cloud viewer");
			//viewer.showCloud(globalFeatureCloud);
			// while (!viewer.wasStopped())
			// {
			// 	// todo::
			// }

			// system("pause");
			//定义点云数据
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			// 定义读取对象
			pcl::PCDReader reader;
			//if(!globalMap->empty ())
			// saveMap_callback(mapSave);
 
			// // 读取点云文件
			// reader.read<pcl::PointXYZ> ("./data/icpLaserCloud.pcd", *cloud);

			// viewer1->removeAllPointClouds();  // 移除当前所有点云
			// viewer1->addPointCloud(cloud, "test");  
			// viewer1->updatePointCloud(cloud, "test"); 
			// viewer1->spinOnce(0.0000000000001);
			//viewer1->showCloud(globalFeatureCloud);

            frameCount = 0;
            lastPoseX = currentX;
            lastPoseY = currentY;
            lastPoseZ = currentZ;
        }

		g_iSameFrame++;
		g_iOdomFrame++;
		g_iMosFrame++;
		g_iPsdFrame++;
		end =clock();
		std::cout << "time per frame is: " << (end - start)/1000 << " ms" << std::endl; 
		
	}

	end_0 = clock();

	std::cout << "time per frame is: " << (end_0 - start_0)/(1000 * iMinFrame ) << " ms" << std::endl; 
	
    saveMap_callback(mapSave);	
	return 0;
}
