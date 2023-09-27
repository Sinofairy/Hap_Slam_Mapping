#include "feature.h"
#include <document.h>
#include <writer.h>
#include <stringbuffer.h>
#include <algorithm>
#include <stdio.h>
#include <io.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <vector>
#include "include/InvoAlgSlamTypes.h"
#include "include/InvoAlgPsdTypes.h"
#include "include/InvoAlgMosTypes.h"

using namespace std;
#define MAX_FRAME  10000

static int32_t g_iSameFrame = 0;
static int32_t g_iOdomFrame = 0;
static int32_t g_iMosFrame = 0;
static int32_t g_iPsdFrame = 0;

static void ReadImuGnssData(FILE* fp, OdomData* pstOdomData);
static void ReadPsdData(vector<string> Psdfile, tInvoPsdObjList* pstPsdData);
//void getAllFiles(string path, vector<string>& files, string fileType);
static void ReadMosData(vector<string> Mosfile, tInvoMosObjList* pstMosData);
static void CalcOdomFrameAndTimeStamp(FILE* fp, int32_t* iFrameNum);
std::vector<std::string> getFileList(std::string dir_name);

std::vector<std::string> getFileList(std::string dir_name){
    std::vector<std::string> result;
    const char *dir_name_c = dir_name.c_str();
    if(NULL == dir_name_c){
        std::cout << "dir_name is null !" << std::endl;
    }
    struct stat s;
    lstat(dir_name_c, &s);
    if(!S_ISDIR(s.st_mode)){
        std::cout << "dir_name is not a valid directory !" << std::endl;
        return result;
    }
    struct dirent *filename;
    DIR *dir;
    dir = opendir(dir_name_c);
    if(NULL == dir){
        std::cout << "Can not open dir " << dir_name_c << " !" << std::endl;
        return result;
    }
    while ((filename = readdir(dir)) != NULL)
    {
        std::string filePath = dir_name;
        if(strcmp(filename->d_name, ".") == 0 ||
           strcmp(filename->d_name, "..") == 0)
           continue;
        filePath += filename->d_name;
        result.push_back(filePath);
    }
    return result;
}

static void ReadImuGnssData(FILE* fp, OdomData* pstOdomData)
{
	char line[1000], * p;
	static int32_t iCount = 0;
	int32_t i;
	char cTmp[1000] = { 0 }, c;
	int32_t iNum = 0;
	double data[15];

	if (fp != NULL)
	{
		while (fgets(line, 1000, fp) != 0)
		{
			iCount++;
			/*if (iCount <= g_iSameFrame)
			{
				continue;
			}*/

			if (iCount < 1/*2*/)
			{
				continue;
			}
			else if (iCount == 1/*2*/ && g_iOdomFrame == 0)
			{
				p = line;

				for (i = 0; i < 8; i++)
				{
					iNum = 0;
					while (*p != ' ' && *p != '\n')
					{
						c = *p;
						p++;
						sprintf(cTmp + iNum, "%c", c);
						iNum++;
					}
					data[i] = atof(cTmp);
					p++;
				}
				pstOdomData->time = data[0];
				/*orignal pose*/
				Eigen::Quaterniond q0(data[7], data[4], data[5], data[6]);
				pstOdomData->pose.block<3, 3>(0, 0) = q0.normalized().toRotationMatrix();
				Eigen::Vector3d t0(data[1], data[2], data[3]);
				pstOdomData->pose.block<3, 1>(0, 3) = t0;

				Eigen::Vector4d t04(0, 0, 0, 1);
				pstOdomData->pose.block<1, 4>(3, 0) = t04;
				
				/*cout << pstOdomData->pose << endl;*/

				/*delta pose*/
				Eigen::Vector3d t1(0, 0, 0);
				pstOdomData->delta_pose.block<3, 1>(0, 3) = t1;
				Eigen::Quaterniond q1(1.000000, 0.000000, 0.000000, 0.000000);
				pstOdomData->delta_pose.block<3, 3>(0, 0) = q1.normalized().toRotationMatrix();

				Eigen::Vector4d t14(0, 0, 0, 1);
				pstOdomData->delta_pose.block<1, 4>(3, 0) = t14;
				/*cout << pstOdomData->delta_pose<<endl;*/
				break;
			}
			else
			{
				p = line;

				for (i = 0; i < 15; i++)
				{
					iNum = 0;
					while (*p != ' ' && *p != '\n')
					{
						c = *p;
						p++;
						sprintf(cTmp + iNum, "%c", c);
						iNum++;
					}
					data[i] = atof(cTmp);
					p++;
				}
				pstOdomData->time = data[0];
				/*orignal pose*/
				Eigen::Quaterniond q0(data[7], data[4], data[5], data[6]);
				pstOdomData->pose.block<3, 3>(0, 0) = q0.normalized().toRotationMatrix();
				Eigen::Vector3d t0(data[1], data[2], data[3]);
				pstOdomData->pose.block<3, 1>(0, 3) = t0;

				Eigen::Vector4d t04(0, 0, 0, 1);
				pstOdomData->pose.block<1, 4>(3, 0) = t04;
				/*cout << pstOdomData->pose << endl;*/

				/*delta pose*/
				Eigen::Vector3d t1(data[8], data[9], data[10]);
				pstOdomData->delta_pose.block<3, 1>(0, 3) = t1;
				Eigen::Quaterniond q1(data[14], data[11], data[12], data[13]);
				pstOdomData->delta_pose.block<3, 3>(0, 0) = q1.normalized().toRotationMatrix();

				Eigen::Vector4d t14(0, 0, 0, 1);
				pstOdomData->delta_pose.block<1, 4>(3, 0) = t14;
				/*cout << pstOdomData->delta_pose << endl << endl;*/
				break;
			}
		}
	}
}

static void ReadPsdData(vector<string> Psdfile, tInvoPsdObjList* pstPsdData)
{
	int32_t i, j, k, m;
	int32_t iLength;
	FILE* psdFile = NULL;
	char fileName[200];
	const char* str;

	if (g_iPsdFrame < Psdfile.size())
	{
		std::ifstream file(Psdfile[g_iPsdFrame]);
		std::string strJson((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
		rapidjson::Document doc;
		doc.Parse(strJson.c_str());

		if (doc.HasParseError())
		{
			printf("Parse json file is failed, please check json file is true!!!\n");
		}

		rapidjson::Value& jsonObj = doc;

		/*if (jsonObj.HasMember("imgName"))
		{
			std::string imgName = jsonObj["imgName"].GetString();
			iLength = imgName.length();
			str = imgName.c_str();
			memcpy(pstPsdData->PsdName, str, iLength * sizeof(char));
		}*/

		if (jsonObj.HasMember("timeStamp"))
		{
			double time = jsonObj["timeStamp"].GetDouble();
			pstPsdData->timer = time;
		}

		if (jsonObj.HasMember("number"))
		{
			int8_t objNum = jsonObj["number"].GetInt();
			pstPsdData->number = objNum;
			if (objNum > 0)
			{
				if (jsonObj.HasMember("PsdObjList"))
				{
					const rapidjson::Value& array = jsonObj["PsdObjList"];

					int32_t len = array.Size();

					for (i = 0; i < len; i++)
					{
						const rapidjson::Value& object = array[i];

						if (object.IsObject())
						{
							if (object.HasMember("index"))
							{
								int32_t iID = object["index"].GetInt();
								pstPsdData->parkslots[i].index = iID;
							}
							if (object.HasMember("side"))
							{
								int8_t direction = object["side"].GetInt();
								pstPsdData->parkslots[i].side = (INVO_PSD_SIDE)direction;
							}

							if (object.HasMember("type"))
							{
								int8_t type = object["type"].GetInt();
								/*cout << "type: " << type << endl;*/
								pstPsdData->parkslots[i].type = (INVO_PSD_TYPE)type;
							}

							if (object.HasMember("box_point"))
							{
								const rapidjson::Value& box = object["box_point"];
								if (box.HasMember("image_info_coords"))
								{
									const rapidjson::Value& arraypixel = box["image_info_coords"];
									for (j = 0; j < 4; j++)
									{
										if (arraypixel[j].IsArray())
										{
											pstPsdData->parkslots[i].image_info.box_point[j].x = arraypixel[j][0].GetFloat();
											pstPsdData->parkslots[i].image_info.box_point[j].y = arraypixel[j][1].GetFloat();
										}
									}
								}
								if (box.HasMember("local_info_coords"))
								{
									const rapidjson::Value& arrayworld = box["local_info_coords"];
									for (j = 0; j < 4; j++)
									{
										if (arrayworld[j].IsArray())
										{
											pstPsdData->parkslots[i].local_info.box_point[j].x = arrayworld[j][0].GetFloat();
											pstPsdData->parkslots[i].local_info.box_point[j].y = arrayworld[j][1].GetFloat();
											pstPsdData->parkslots[i].local_info.box_point[j].z = (float)0.0;
										}
									}
								}
							}



							if (object.HasMember("point_set"))
							{
								const rapidjson::Value& pointSet = object["point_set"];
								if (pointSet.HasMember("iPointSetsNum"))
								{
									int32_t ptNum = pointSet["iPointSetsNum"].GetInt();
									pstPsdData->parkslots[i].image_info.iPointSetsNum = ptNum;

									if (pointSet.HasMember("point_set_coords"))
									{
										const rapidjson::Value& arraypts = pointSet["point_set_coords"];
										for (j = 0; j < ptNum; j++)
										{
											if (arraypts[j].IsArray())
											{
												pstPsdData->parkslots[i].image_info.point_set[j].x = arraypts[j][0].GetFloat();
												pstPsdData->parkslots[i].image_info.point_set[j].y = arraypts[j][1].GetFloat();
											}
										}
									}

								}
							}
						}
					}
				}

			}
		}
	}
}


static void ReadMosData(vector<string> Mosfile, tInvoMosObjList* pstMosData)
{
	int32_t i, j, k, m;
	int32_t iLength;
	FILE* mosFile = NULL;
	char fileName[200];
	const char* str;

	std::ifstream file(Mosfile[g_iMosFrame/*g_iSameFrame*/]);
	std::string strJson((std::istreambuf_iterator<char>(file)),
		std::istreambuf_iterator<char>());
	rapidjson::Document doc;
	doc.Parse(strJson.c_str());

	if (doc.HasParseError())
	{
		printf("Parse json file is failed, please check json file is true!!!\n");
	}

	rapidjson::Value& jsonObj = doc;

	// if (jsonObj.HasMember("imgName"))
	// {
	// 	std::string imgName = jsonObj["imgName"].GetString();
	// 	iLength = imgName.length();
	// 	str = imgName.c_str();
	// 	memcpy(pstMosData->MosName, str, iLength * sizeof(char));
	// }

	if (jsonObj.HasMember("timeStamp"))
	{
		i_time time = jsonObj["timeStamp"].GetInt();
		pstMosData->timer = time;
	}

	if (jsonObj.HasMember("objNum"))
	{
		uint32_t objNum = jsonObj["objNum"].GetInt();
		pstMosData->number = objNum;
		if (objNum > 0)
		{
			if (jsonObj.HasMember("objList"))
			{
				const rapidjson::Value& array = jsonObj["objList"];
				int32_t len = array.Size();

				for (i = 0; i < len; i++)
				{
					/*cout << "len: " << len << endl;*/
					const rapidjson::Value& object = array[i];

					if (object.IsObject())
					{
						if (object.HasMember("trackId"))
						{
							int32_t iID = object["trackId"].GetInt();
							pstMosData->objects[i].track_id = iID;
						}

						if (object.HasMember("type"))
						{
							int32_t type1 = object["type"].GetInt();
							/*cout << "type: " << type1 << endl;*/
							pstMosData->objects[i].type = (INVO_MOS_OBJECT_TYPE)type1;
						}

						if (object.HasMember("box"))
						{
							const rapidjson::Value& box = object["box"];
							if (box.HasMember("boxPtNum"))
							{
								int32_t boxPtNum = box["boxPtNum"].GetInt();

								if (box.HasMember("boxPts"))
								{
									const rapidjson::Value& arraybox = box["boxPts"];
									for (j = 0; j < boxPtNum; j++)
									{
										if (arraybox[j].IsArray())
										{
											pstMosData->objects[i].image_info.box_point[j].x = arraybox[j][0].GetFloat();
											pstMosData->objects[i].image_info.box_point[j].y = arraybox[j][1].GetFloat();
										}
									}
								}
							}
						}

						 if (object.HasMember("contour"))
						 {
						 	const rapidjson::Value& contour = object["contour"];
						 	if (contour.HasMember("contourPtNum"))
						 	{
						 		int32_t contourPtNum = contour["contourPtNum"].GetInt();
						 		pstMosData->objects[i].image_info.counter_num = contourPtNum;

						 		if (contour.HasMember("contourPts"))
						 		{
						 			const rapidjson::Value& arraycontour = contour["contourPts"];
						 			for (j = 0; j < contourPtNum; j++)
						 			{
						 				if (arraycontour[j].IsArray())
						 				{
											pstMosData->objects[i].image_info.counter[j].x = arraycontour[j][0].GetFloat();
											pstMosData->objects[i].image_info.counter[j].y = arraycontour[j][1].GetFloat();
						 				}
						 			}
						 		}
						 	}
						 }
					}
				}
			}
		}
	}
}


static void CalcOdomFrameAndTimeStamp(FILE* fp, int32_t* iFrameNum)
{
	char line[1000];
	*iFrameNum = -1;

	if (fp != NULL)
	{
		while (fgets(line, 1000, fp) != 0)
		{
			(*iFrameNum)++;
		}
	}
}
