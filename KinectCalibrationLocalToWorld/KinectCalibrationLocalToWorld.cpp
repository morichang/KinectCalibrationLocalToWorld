// KinectCalibrationLocalToWorld.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"

//算出されたRとtから変換パラメータの行列を作ってやる
inline cv::Mat make_3D_array(cv::Mat R, cv::Mat T)
{
	if (R.cols == 3 && T.cols == 1)
	{
		cv::Mat param3D_array = (cv::Mat_<double>(4, 4) <<
			R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2),
			0, 0, 0, 1);
		return param3D_array;
	}
	else{
		std::cout << "Error" << std::endl;
	}
}

//点の書き込まれたtxtファイルから要素を取り出す
void split(std::vector<std::string> &v, const std::string &input_string, const std::string &delimiter){
	std::string::size_type index = input_string.find_first_of(delimiter);

	if (index != std::string::npos) {
		v.push_back(input_string.substr(0, index));
		split(v, input_string.substr(index + 1), delimiter);
	}
	else {
		v.push_back(input_string);
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	std::vector<cv::Point2d> uvPoints;
	std::vector<cv::Point3d> localPoints;
	std::vector<cv::Point3d> worldPoints;
	std::string str;
	std::string dind_id("DIND112");  //DIND104 to DIND112

	std::ifstream ifs_uv("../../../../CalibrationParameter/" + dind_id + "-20170109/" + dind_id + "-PC_uvPoints.txt");
	if (ifs_uv.fail()){
		std::cerr << "失敗" << std::endl;
		return -1;
	}
	while (getline(ifs_uv, str)){
		std::vector<std::string> v_uv;
		split(v_uv, str, ",");
		cv::Point2d p(std::stod(v_uv[0]), std::stod(v_uv[1]));

		uvPoints.push_back(p);
	}

	std::ifstream ifs_local("../../../../CalibrationParameter/" + dind_id + "-20170109/" + dind_id + "-PC_localPoints.txt");
	if (ifs_local.fail()){
		std::cerr << "失敗" << std::endl;
		return -1;
	}
	while (getline(ifs_local, str)){
		std::vector<std::string> v_local;
		split(v_local, str, ",");
		cv::Point3d p(std::stod(v_local[0]), std::stod(v_local[1]), std::stod(v_local[2]));

		localPoints.push_back(p);
	}

	std::ifstream ifs_world("../../../../CalibrationParameter/" + dind_id + "-20170109/" + dind_id + "-PC_worldPoints.txt");
	if (ifs_world.fail()){
		std::cerr << "失敗" << std::endl;
		return -1;
	}
	while (getline(ifs_world, str)){
		std::vector<std::string> v_world;
		split(v_world, str, ",");
		cv::Point3d p(std::stod(v_world[0]), std::stod(v_world[1]), std::stod(v_world[2]));

		worldPoints.push_back(p);
	}

	//WorldPoints確認
	/*int i = 0;
	while (i < worldPoints.size()){
		std::cout << worldPoints[i] << std::endl;
		i++;
	}*/

	//カメラ行列の定義
	double fx = 1081.37, fy = 1081.37, cx = 959.5, cy = 539.5;
	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
		fx, 0, cx,
		0, fy, cy,
		0, 0, 1);
	cv::Mat R, rot, t;

	//回転行列を出す
	try {
		cv::solvePnP(worldPoints, uvPoints, cameraMatrix, cv::Mat(), rot, t);
		cv::Rodrigues(rot, R);  //なんかcv::Rodrigues(R, R);だとOut of rangeが出た
	}
	catch (std::exception ex) {
		std::cout << ex.what() << std::endl;
	}

	//cv::FileStorage cvfs("../../DIND108-20160530/DIND108-PC_extrinsic.xml", cv::FileStorage::READ);  //DIND毎にここのパラメータxmlを変える
	//if (!cvfs.isOpened()){
	//	std::cout << "File can not be opened." << std::endl;
	//	return -1;
	//}
	//cv::FileNode node(cvfs.fs, NULL);
	//cv::Mat R, t;

	////cv::FileNode fn = node[std::string("Rot")];
	//cv::read(node[std::string("Rot")], R);
	////if (fn.empty()) std::cout << "Rot パースエラー" << std::endl;
	////else cv::read(fn, R);
	////cvfs["Rot"] >> R;
	////std::cout << R << std::endl;

	////cv::FileNode fn2 = node[std::string("trans")];
	//cv::read(node[std::string("trans")], t);
	////if (fn2.empty()) std::cout << "trans パースエラー" << std::endl;
	////else cv::read(fn2, t);
	////cvfs["trans"] >> t;
	////std::cout << t << std::endl;

	auto ER = make_3D_array(R, t);
	//std::cout << ER << std::endl;

	std::cout << "書き込みm@s!" << std::endl;

	cv::FileStorage wfs("../../../../CalibrationParameter/" + dind_id + "-20170109/" + dind_id + "-PC_extrinsic_calc.xml", cv::FileStorage::WRITE);
	wfs << "param3D_array" << ER;
	wfs.release();

	std::cout << "Finish!" << std::endl;
	Sleep(INFINITE);

	return 0;
}