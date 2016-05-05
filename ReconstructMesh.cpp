/*
* ReconstructMesh.cpp
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#include <boost/program_options.hpp>

//#include "Camera.h"

using namespace MVS;
using namespace std;
typedef long double Longouble;
typedef TMatrix<double, 3, 4> Matrix3x4ld;

#define APPNAME _T("ProjectMatrix2PhotoScanXML")

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

bool readInPmatFile(vector<string> &pMatNameVec, vector<Matrix3x4d> &pMatVec)
{
	for (int i = 0; i < pMatNameVec.size(); i++)
	{
		ifstream pMatReader(pMatNameVec.at(i));
		if (!pMatReader)
		{
			cout << "open projection matrix " << pMatNameVec.at(i) << " failed!" << endl;
			getchar();
		}
		Matrix3x4d pMat;
		string tempString;
		pMatReader >> tempString;
		pMatReader >> pMat(0, 0) >> pMat(0, 1) >> pMat(0, 2) >> pMat(0, 3)
					>> pMat(1, 0) >> pMat(1, 1) >> pMat(1, 2) >> pMat(1, 3)
					>> pMat(2, 0) >> pMat(2, 1) >> pMat(2, 2) >> pMat(2, 3);
		pMatVec.push_back(pMat);
	}

	return true;
}

bool writeXML(vector<Camera> cameraVec, string XMLName, vector<string>imageNameVec)
{
	ofstream xmlWriter(XMLName, std::ios_base::binary | std::ios_base::out);
	xmlWriter << setiosflags(ios::scientific)<< setprecision(16);
	xmlWriter << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl
		<< "<document version=\"0.9.1\">" << endl
		<< "  <chunk>" << endl
		<< "    <sensors>" << endl;
	double zerof(0.0);
	double onef(1.0);

	for (int i = 0; i < cameraVec.size(); i++)
	{
		xmlWriter << "      <sensor id=\"" << i << "\" label=\"unknown\" type=\"frame\">" << endl
			<< "        <resolution width=\"8176\" height=\"6132\"/>" << endl
			<< "        <property name=\"fixed\" value=\"false\"/>" << endl
			<< "        <calibration type=\"frame\" class=\"adjusted\">" << endl
			<< "          <resolution width=\"8176\" height=\"6132\"/>" << endl
			<< "          <fx>" << cameraVec.at(i).K(0, 0) << "</fx>" << endl
			<< "          <fy>" << cameraVec.at(i).K(1, 1) << "</fy>" << endl
			<< "          <cx>" << cameraVec.at(i).K(0, 2) << "</cx>" << endl
			<< "          <cy>" << cameraVec.at(i).K(1, 2) << "</cy>" << endl
			<< "          <k1>" << zerof << "</k1>" << endl
			<< "          <k2>" << zerof << "</k2>" << endl
			<< "          <k3>" << zerof << "</k3>" << endl
			<< "        </calibration>" << endl
			<< "      </sensor>"<<endl;
	}
	xmlWriter << "    </sensors>" << endl 
		<< "    <cameras>" << endl;
	for (int i = 0; i < cameraVec.size(); i++)
	{
		xmlWriter << "      <camera id=\"" << i << "\" label=\"" << imageNameVec.at(i) << "\" sensor_id=\"" << i << "\" enabled=\"true\"> " << endl
					<< "        <resolution width=\"8176\" height=\"6132\"/> " << endl
					<< "        <transform>" << cameraVec.at(i).R(0, 0) << " " << cameraVec.at(i).R(1, 0) << " " << cameraVec.at(i).R(2, 0) << " " << cameraVec.at(i).C.x -533500.0 << " "
					<< cameraVec.at(i).R(0, 1) << " " << cameraVec.at(i).R(1, 1) << " " << cameraVec.at(i).R(2, 1) << " " << cameraVec.at(i).C.y -337960.0<< " "
					<< cameraVec.at(i).R(0, 2) << " " << cameraVec.at(i).R(1, 2) << " " << cameraVec.at(i).R(2, 2) << " " << cameraVec.at(i).C.z << " "
					<< zerof << " " << zerof << " " << zerof << " " << onef << "</transform>" << endl
					<< "      </camera>" << endl;
	}

	xmlWriter << "    </cameras>"<<endl
		<<"  </chunk>"<<endl
		<<"</document>";
	xmlWriter.close();
	return true;
}

int main(int argc, char** argv)
{
	//string pMatPath = "D:\\Projects\\mapping_angle_area\\data\\wdob_lib_pmatrix";
	string pMatPath = "data";
	vector<Matrix3x4d> pMatVec;
	string search_path = pMatPath + "\\*.txt";
	vector<string> pMatNameVec;
	if (!get_filelist_from_dir(search_path, pMatNameVec))
	{
		cout << "open Projection Matrix failed!" << endl;
		return false;
	}
	vector<string> imageNameVec;
	for (int i = 0; i < pMatNameVec.size(); i++)
	{		
		string imageName = pMatNameVec.at(i).substr(0, pMatNameVec.at(i).find_last_of('.')) + ".jpg";
		imageNameVec.push_back(imageName);
	}

	for (int i = 0; i < pMatNameVec.size(); i++)
	{
		pMatNameVec.at(i) = pMatPath +"\\"+ pMatNameVec.at(i);
	}

	readInPmatFile(pMatNameVec, pMatVec);
	vector<Camera> cameraVec;
	for (int i = 0; i < pMatNameVec.size(); i++)
	{
		Camera camera(pMatVec.at(i));
		camera.DecomposeP();
		cameraVec.push_back(camera);
	}


	writeXML(cameraVec, "cam.xml", imageNameVec);
	//string imageName = scene.images[0].name;
	//string platForm = scene.platforms[0].name;

	//std::cout << "K mat: " << std::endl << camera.K << std::endl;
	//std::cout << "C mat: " << std::endl << camera.C << std::endl;
	//std::cout << "R mat: " << std::endl << camera.R << std::endl;

	//Camera camera2(camera.K, camera.R, camera.C);
	//camera2.ComposeP();
	//cout << "P Mat: " << endl << camera2.P << endl;
	//getchar();
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