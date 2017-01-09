#pragma warning(disable:4819)
#pragma warning(disable:4244)
#pragma warning(disable:4267)

#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <limits>
#include <list>
#include "octree.h"

// 用于判断投影是否在visual hull内部
struct Projection
{
	Eigen::Matrix<float, 3, 4> m_projMat;
	cv::Mat m_image;
	const uint m_threshold = 125;

	bool outOfRange(int x, int max)const
	{
		return x < 0 || x >= max;
	}

	bool checkRange(double x, double y, double z)const
	{
		Eigen::Vector3f vec3 = m_projMat * Eigen::Vector4f(x, y, z, 1);
		int indX = vec3[1] / vec3[2];
		int indY = vec3[0] / vec3[2];

		if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
			return false;
		return m_image.at<uchar>((uint)(vec3[1] / vec3[2]), (uint)(vec3[0] / vec3[2])) > m_threshold;
	}
};

// 用于index和实际坐标之间的转换
struct CoordinateInfo
{
	int m_resolution;
	double m_min;
	double m_max;

	double index2coor(int index)
	{
		return m_min + index * (m_max - m_min) / m_resolution;
	}

	CoordinateInfo(int resolution = 10, double min = 0.0, double max = 100.0)
		: m_resolution(resolution)
		, m_min(min)
		, m_max(max)
	{
	}
};

class Model
{
public:
	typedef Octree Voxel;

	Model(int res = 100);
	~Model();

	void saveModel(const char* pFileName);
	void saveModelWithNormal(const char* pFileName);
	void loadMatrix(const char* pFileName);
	void loadImage(const char* pDir, const char* pPrefix, const char* pSuffix);
	void SetNodeColor(Onode* node_ptr);
	void getModel();
	void getSurface();
	Eigen::Vector3f getNormal(float coorX, float coorY, float coorZ);

private:

	CoordinateInfo m_corrX;
	CoordinateInfo m_corrY;
	CoordinateInfo m_corrZ;

	int m_neiborSize;

	std::vector<Projection> m_projectionList;

	Voxel *m_voxel;
	std::list<Onode*> m_surface;

	double dot_pitch_x;
	double dot_pitch_y;
	double dot_pitch_z;
};

Model::Model(int res) : m_corrX(res, -5, 5)
, m_corrY(res, -10, 10)
, m_corrZ(res, 15, 30)
{
	if (res > 100)
		m_neiborSize = res / 100;
	else
		m_neiborSize = 1;
	int generation = 2;
	while (pow(2, generation) < res) {	//找到适合当前分辨率的八叉树代数，最低为2代（共9个点）
		generation++;
	}
	m_voxel = new Voxel(generation-1);	//首先生成（代数-1）代的八叉树骨干用于初步搜索
//	m_voxel->Print();

	dot_pitch_x = (m_voxel->x_max - m_voxel->x_min)*pow(2, -generation);
	dot_pitch_y = (m_voxel->y_max - m_voxel->y_min)*pow(2, -generation);
	dot_pitch_z = (m_voxel->z_max - m_voxel->z_min)*pow(2, -generation);
}

Model::~Model()
{
	delete m_voxel;
}

void Model::saveModel(const char* pFileName)
{
	std::ofstream fout(pFileName);

	for (auto it = m_surface.cbegin(); it != m_surface.cend(); it++) {
		fout << (*it)->x << ' ' << (*it)->y << ' ' << (*it)->z << std::endl;
	}

	//for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
	//	for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
	//		for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
	//			if (m_surface[indexX][indexY][indexZ])
	//			{
	//				double coorX = m_corrX.index2coor(indexX);
	//				double coorY = m_corrY.index2coor(indexY);
	//				double coorZ = m_corrZ.index2coor(indexZ);
	//				fout << coorX << ' ' << coorY << ' ' << coorZ << std::endl;
	//			}
}

void Model::saveModelWithNormal(const char* pFileName)
{
	std::ofstream fout(pFileName);

	for (auto it = m_surface.cbegin(); it != m_surface.cend(); it++) {
		fout << (*it)->x << ' ' << (*it)->y << ' ' << (*it)->z << ' ';
		Eigen::Vector3f nor = getNormal((*it)->x, (*it)->y, (*it)->z);
		fout << nor(0) << ' ' << nor(1) << ' ' << nor(2) << std::endl;

	}

	//double midX = m_corrX.index2coor(m_corrX.m_resolution / 2);
	//double midY = m_corrY.index2coor(m_corrY.m_resolution / 2);
	//double midZ = m_corrZ.index2coor(m_corrZ.m_resolution / 2);

	//for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
	//	for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
	//		for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
	//			if (m_surface[indexX][indexY][indexZ])
	//			{
	//				double coorX = m_corrX.index2coor(indexX);
	//				double coorY = m_corrY.index2coor(indexY);
	//				double coorZ = m_corrZ.index2coor(indexZ);
	//				fout << coorX << ' ' << coorY << ' ' << coorZ << ' ';

	//				Eigen::Vector3f nor = getNormal(indexX, indexY, indexZ);
	//				fout << nor(0) << ' ' << nor(1) << ' ' << nor(2) << std::endl;
	//			}
}

void Model::loadMatrix(const char* pFileName)
{
	std::ifstream fin(pFileName);

	int num;
	Eigen::Matrix<float, 3, 3> matInt;
	Eigen::Matrix<float, 3, 4> matExt;
	Projection projection;
	while (fin >> num)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				fin >> matInt(i, j);

		double temp;
		fin >> temp;
		fin >> temp;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				fin >> matExt(i, j);

		projection.m_projMat = matInt * matExt;
		m_projectionList.push_back(projection);
	}
}

void Model::loadImage(const char* pDir, const char* pPrefix, const char* pSuffix)
{
	int fileCount = m_projectionList.size();
	std::string fileName(pDir);
	fileName += '/';
	fileName += pPrefix;
	for (int i = 0; i < fileCount; i++)
	{
		std::cout << fileName + std::to_string(i) + pSuffix << std::endl;
		m_projectionList[i].m_image = cv::imread(fileName + std::to_string(i) + pSuffix, CV_8UC1);
	}
}

void Model::SetNodeColor(Onode* node_ptr) {
	double coorX = node_ptr->x;
	double coorY = node_ptr->y;
	double coorZ = node_ptr->z;

	int projection_count = m_projectionList.size();

	for (int i = 0; i < projection_count; i++) {
		if (!m_projectionList[i].checkRange(coorX, coorY, coorZ)) {	//如果该点落在任一投影外部
			node_ptr->status = BLACK;	//设置为黑色状态
			return;
		}
	}
	node_ptr->status = WHITE;	//否则落在投影内部，设置为白色
}

void Model::getModel(){

	struct CheckAndUpdate {
		
		Model* model;

		CheckAndUpdate(Model* m):model(m){};

		void operator()(Onode* node_ptr) {
			switch (node_ptr->status){
				case(BODY_END):{	//若是树枝干末端的点，根据是否是内部三维点设置颜色
					model->SetNodeColor(node_ptr);
					break;
				}
				case(BODY): {	//若不是末端的点
					switch (model->m_voxel->UpdateStatus(node_ptr)){	//根据孩子结点状况更新其状态
						case(MIXED): {	//如果孩子含且只含黑白两色
							for (int i = 0; i < 8; i++) {	//为所有孩子扩展叶子节点
								model->m_voxel->Expand(node_ptr->childs[i], LEAVES);
								for (int j = 0; j < 8; j++) {
									model->m_surface.push_back(node_ptr->childs[i]->childs[j]);	//并将扩展的结点放入队列稍后处理
								}
							}
							break;
						}
						case(BLACK):case(WHITE): {	//如果孩子为纯色
							for (int i = 0; i < 8; i++) {	//删除所有孩子，以其本身颜色代表各孩子
								node_ptr->childs[i]->status = UNUSED;
							}
						}
						default:break;	//如果孩子中有支撑性的结点，什么都不做
					}
				}
			}
		}
	}updater(this);

	m_voxel->TravPost(m_voxel->GetRoot(), updater);

//	m_voxel->Print();

	std::cout << "large scale done, num of surface point: "<<m_surface.size() << std::endl;
	
	for(auto it=m_surface.begin();it!=m_surface.end();it++){
		SetNodeColor(*it);
	}
//	m_voxel->Print();

}

void Model::getSurface()
{
	// 邻域：上、下、左、右、前、后。
	int dx[6] = { -1, 0, 0, 0, 0, 1 };
	int dy[6] = { 0, 1, -1, 0, 0, 0 };
	int dz[6] = { 0, 0, 0, 1, -1, 0 };

	// lambda表达式，用于判断某个点是否在Voxel的范围内
	auto outOfRange = [&](double xx, double yy, double zz){
		return xx <= m_voxel->x_min || m_voxel->x_max <= xx
			|| yy <= m_voxel->y_min || m_voxel->y_max <= yy
			|| zz < -m_voxel->z_min || m_voxel->z_max <= zz;
	};

	for (auto it = m_surface.begin(); it != m_surface.end();) {
		if ((*it)->status == WHITE) {
			for (int i = 0; i < 6; i++) {
				double corr_x = (*it)->x + dx[i] * dot_pitch_x;
				double corr_y = (*it)->y + dy[i] * dot_pitch_y;
				double corr_z = (*it)->z + dz[i] * dot_pitch_z;
				Onode* node_ptr = m_voxel->FindPoint(corr_x, corr_y, corr_z, m_voxel->GetRoot());	//找到表面点的六个相邻点位置
				if ((node_ptr == NULL) || (node_ptr->status == BLACK)) {	//若其中有不在三维点云内的
					(*it)->status = SURFACE;	//标记为表面点
					break;
				}
			}
		}
		if ((*it)->status != SURFACE) {	//该点本身不在点云内或包括它和六个相邻点在内七个点都位于点云内
			it = m_surface.erase(it);	//删除该点
		}
		else {
			it++;
		}
	}
	std::cout << m_surface.size() << std::endl;
}

Eigen::Vector3f Model::getNormal(float coor_x, float coor_y, float coor_z)
{
	auto outOfRange = [&](float coorX, float coorY, float coorZ) {
		return coorX <= m_voxel->x_min || m_voxel->x_max <= coorX
			|| coorY <= m_voxel->y_min || m_voxel->y_max <= coorY
			|| coorZ < -m_voxel->z_min || m_voxel->z_max <= coorZ;
	};

	std::vector<Eigen::Vector3f> neiborList;
	std::vector<Eigen::Vector3f> innerList;

	for (int dX = -m_neiborSize; dX <= m_neiborSize; dX++)
		for (int dY = -m_neiborSize; dY <= m_neiborSize; dY++)
			for (int dZ = -m_neiborSize; dZ <= m_neiborSize; dZ++)
			{
				if (!dX && !dY && !dZ)
					continue;
				float neiborX = coor_x + dX * dot_pitch_x;
				float neiborY = coor_y + dY * dot_pitch_y;
				float neiborZ = coor_z + dZ * dot_pitch_z;
				if (!outOfRange(neiborX, neiborY, neiborZ))
				{

					Onode* node_ptr = m_voxel->FindPoint(neiborX, neiborY, neiborZ, m_voxel->GetRoot());
					if (node_ptr == NULL) {
						continue;
					}
					switch (node_ptr->status)
					{
					case(SURFACE): {
						neiborList.push_back(Eigen::Vector3f(neiborX, neiborY, neiborZ));
					}
					case(WHITE): {
						innerList.push_back(Eigen::Vector3f(neiborX, neiborY, neiborZ));
					}
					default:
						break;
					}
				}
			}

	Eigen::Vector3f point(coor_x, coor_y, coor_z);

	Eigen::MatrixXf matA(3, neiborList.size());
	for (int i = 0; i < neiborList.size(); i++)
		matA.col(i) = neiborList[i] - point;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(matA * matA.transpose());
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
	int indexEigen = 0;
	if (abs(eigenValues[1]) < abs(eigenValues[indexEigen]))
		indexEigen = 1;
	if (abs(eigenValues[2]) < abs(eigenValues[indexEigen]))
		indexEigen = 2;
	Eigen::Vector3f normalVector = eigenSolver.eigenvectors().col(indexEigen);
	
	Eigen::Vector3f innerCenter = Eigen::Vector3f::Zero();
	for (auto const& vec : innerList)
		innerCenter += vec;
	innerCenter /= innerList.size();

	if (normalVector.dot(point - innerCenter) < 0)
		normalVector *= -1;
	return normalVector;
}

int main(int argc, char** argv)
{
	clock_t t = clock();

	// 设置xyz方向的Voxel分辨率
	Model model(220);

	// 读取相机的内外参数
	model.loadMatrix("../../calibParamsI.txt");

	// 读取投影图片
	model.loadImage("../../wd_segmented", "WD2_", "_00020_segmented.png");

	// 得到Voxel模型
	model.getModel();
	std::cout << "get model done\n";

	// 获得Voxel模型的表面
	model.getSurface();
	std::cout << "get surface done\n";

	//// 将模型导出为xyz格式
	model.saveModel("../../WithoutNormal.xyz");
	std::cout << "save without normal done\n";

	model.saveModelWithNormal("../../WithNormal.xyz");
	std::cout << "save with normal done\n";

	system("PoissonRecon.x64 --in ../../WithNormal.xyz --out ../../mesh.ply");
	std::cout << "save mesh.ply done\n";

	t = clock() - t;
	std::cout << "time: " << (float(t) / CLOCKS_PER_SEC) << "seconds\n";

	return (0);
}