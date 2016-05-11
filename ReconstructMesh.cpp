#include <string>

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#include <boost/program_options.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/random_simplify_point_set.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/IO/read_xyz_points.h>
#include <boost/property_map/property_map.hpp>
#include <CGAL/IO/write_xyz_points.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>

//#include "Camera.h"

using namespace MVS;
using namespace std;

// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("ReconstructMesh")


bool get_filelist_from_dir(string path, vector<string>& files)
{
	long   hFile = 0;
	struct _finddata_t fileinfo;
	files.clear();
	if ((hFile = _findfirst(path.c_str(), &fileinfo)) != -1)
	{
		do
		{
			if (!(fileinfo.attrib &  _A_SUBDIR))
				files.push_back(fileinfo.name);
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
		return true;
	}
	else
		return false;
}


bool find_in_map(cv::Mat &lineMap, const double &x, const double &y)
{
	if (int(x)<0 || int(x)>lineMap.cols - 1 || int(y)<0 || int(y)>lineMap.rows - 1)
	{
		return false;
	}
	return (lineMap.at<uchar>(int(y), int(x)) == 255 ? true : false);
}

bool construct_line_map(Scene &scene, string undistortImagePath, vector<cv::Mat> &lineMapVec, int lineWidth)
{
	const int maxDis(lineWidth);
	string imageSearchPath = undistortImagePath + "\\*.tif";
	vector<string> imageNameVec;
	get_filelist_from_dir(imageSearchPath, imageNameVec);
	for (int imageIndex = 0; imageIndex < imageNameVec.size(); imageIndex++)
	{
		cout << "processing image " << imageIndex << endl;
		using namespace cv;
		vector<Point> pointOnLine;

		Mat image = cv::imread(undistortImagePath + "\\" + imageNameVec.at(imageIndex), IMREAD_COLOR);
		if (!image.data)
		{
			cout << "can not load image " << undistortImagePath + "\\" + imageNameVec.at(imageIndex) << endl;
			waitKey(3000);
			return false;
		}

		Mat imageCanny;
		cv::cvtColor(image, imageCanny, CV_BGR2GRAY);
		Canny(imageCanny, imageCanny, 170, 230, 3, true); // Apply canny edge

		vector <Mat> bgr_chanels;
		split(image, bgr_chanels);



		for (int chanelIndex = 0; chanelIndex < 3; chanelIndex++)
		{
			Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
			vector<cv::Vec4f> lines_std;

			cout << "processing channle " << chanelIndex << endl;
			// Detect the lines
			ls->detect(bgr_chanels.at(chanelIndex), lines_std);

			cout << "finshed" << endl;
			// Show found lines
			//Mat drawnLines(image.size(), CV_8UC1, Scalar(0));;
			for (int i = 0; i < lines_std.size(); i++)
			{
				double lineLenth2 = (lines_std.at(i)[0] - lines_std.at(i)[2])*(lines_std.at(i)[0] - lines_std.at(i)[2]) + (lines_std.at(i)[1] - lines_std.at(i)[3])*(lines_std.at(i)[1] - lines_std.at(i)[3]);
				if (lineLenth2<400)
				{
					continue;
				}
				line(imageCanny, Point(lines_std.at(i)[0], lines_std.at(i)[1]), Point(lines_std.at(i)[2], lines_std.at(i)[3]), 255, maxDis);
			}

		}

		lineMapVec.push_back(imageCanny);
		imwrite(string("lsdimage\\") + imageNameVec.at(imageIndex), imageCanny);
	}
	return true;

}

bool pointcloud_simplicate(Scene &scene, vector<Matrix3x4d> &pMatVec,const char*imagePath, double removed_percentage, int lineWidth)
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
	typedef Kernel::Point_3 Point;
	typedef Kernel::Vector_3 Vector;
	typedef SEACAVE::cList<Point, const Point&, 2, 8192> PointArr;

	std::vector<Point> points;

	std::cout << scene.pointcloud.points.size() << " input points" << std::endl;
	std::vector<std::size_t> indices(scene.pointcloud.points.size());
	for (std::size_t i = 0; i < scene.pointcloud.points.size(); ++i) {
		points.push_back(Point(scene.pointcloud.points[i].x, scene.pointcloud.points[i].y, scene.pointcloud.points[i].z));
		indices[i] = i;
	}
	// simplification by clustering using erase-remove idiom
	//const double removed_percentage = 90.0; // percentage of points to remove

	std::vector<std::size_t>::iterator end;
	end = CGAL::random_simplify_point_set(indices.begin(), indices.end(), &(scene.pointcloud.points[0]), removed_percentage);


	std::size_t k = end - indices.begin();

	std::cerr << "Keep " << k << " of " << indices.size() << "indices" << std::endl;
	std::vector<int> indexVec;
	std::set<int> indexSet;
	std::ofstream indexWriter("indexFile.txt");
	{
		for (std::size_t i = 0; i < k; ++i) {
			indexVec.push_back(indices[i]);
			indexSet.insert(indices[i]);
		}
	}




	//vector<ANNkd_tree*> kdTreeVec;
	//construct_ann_tree(scene, "F:\\GitHub\\openMVS_sample\\undistorted_images_samll\\out", kdTreeVec);
	vector<cv::Mat> lineMapVec;
	construct_line_map(scene, imagePath, lineMapVec, lineWidth);
	for (int pointIndex = 0; pointIndex < scene.pointcloud.points.size(); pointIndex++)
	{
		if (pointIndex % 10000 == 0)
		{
			cout << pointIndex << endl;
		}
		TPoint3<float> pointMVS = scene.pointcloud.points[pointIndex];
		Vec4d point3d;
		point3d[0] = pointMVS.x;
		point3d[1] = pointMVS.y;
		point3d[2] = pointMVS.z;
		point3d[3] = 1;

		bool pointNearLine(false);
		for (int viewIndex = 0; viewIndex < scene.pointcloud.pointViews[pointIndex].size(); viewIndex++)
		{
			int imageIndex = scene.pointcloud.pointViews[pointIndex][viewIndex];
			Vec3d point2d = pMatVec.at(imageIndex) * point3d;
			double x = point2d[0] / point2d[2];
			double y = point2d[1] / point2d[2];
			if (find_in_map(lineMapVec.at(imageIndex), point2d[0] / point2d[2], point2d[1] / point2d[2]))
			{
				pointNearLine = true;
			}

		}
		if (pointNearLine)
		{
			indexVec.push_back(pointIndex);
			indexSet.insert(pointIndex);
		}

	}

	std::sort(indexVec.begin(), indexVec.end());
	for (int i = 0; i < indexVec.size(); i++)
	{
		indexWriter << indexVec.at(i) << std::endl;
	}
	indexWriter.close();

	const int pointSize = scene.pointcloud.GetSize();
	for (int i = pointSize - 1; i > -1; i--)
	{
		if (indexSet.find(i) == indexSet.end())
		{
			scene.pointcloud.RemovePoint(i);
		}
	}

	scene.pointcloud.Save("simplicated.ply");
	scene.Save("simplicated.mvs");
	return true;
}

int main(int argc, char** argv)
{

	typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
	typedef Kernel::Point_3 Point;
	typedef Kernel::Vector_3 Vector;
	typedef SEACAVE::cList<Point, const Point&, 2, 8192> PointArr;

	Scene sceneOri;
	//sceneOri.Load("F:\\GitHub\\openMVS_sample\\undistorted_images\\sence2.mvs");
	sceneOri.Load(argv[1]);


	vector<Matrix3x4d> pMatVec;
	for (int i = 0; i < sceneOri.images.size(); i++)
	{
		Camera camera = sceneOri.images[i].camera;
		camera.ComposeP();
		cout << camera.P << endl;
		pMatVec.push_back(camera.P);
	}


	Scene scene;
	scene.Load(argv[2]);
	std::cout << scene.platforms.begin()->cameras.size();

	pointcloud_simplicate(scene, pMatVec,argv[3], atof(argv[4]),atoi(argv[5]));

	//sence.pointcloud.pointViews[10][0];
	//Camera camera = scene.images[0].camera;
	//string imageName = scene.images[0].name;
	//string platForm = scene.platforms[0].name;

	//std::cout << "K mat: " << std::endl << camera.K << std::endl;
	//std::cout << "C mat: " << std::endl << camera.C << std::endl;
	//std::cout << "R mat: " << std::endl << camera.R << std::endl;

	//camera.ComposeP();
	//std::cout << camera.P << std::endl;

	//std::cout << sence.pointcloud.points.size() << " points after the simplification" << std::endl;

	//Camera camera = sence.platforms[0].GetCamera(0, 0);
	//camera.ComposeP();
	//PMatrix pMat = camera.P;
	//std::cout << camera.K;
	//std::cout << camera.R;
	//std::cout << camera.C;
	//std::cout << pMat;
	//sence.Save("sence.mvs", ARCHIVE_TEXT);
	return 0;
}
