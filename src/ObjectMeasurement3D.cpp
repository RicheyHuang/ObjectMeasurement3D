#define _CRT_SECURE_NO_WARNINGS
#include <pcl\point_cloud.h>
#include <pcl\io\ply_io.h>
#include <pcl\surface\convex_hull.h>
#include <pcl\surface\concave_hull.h>
#include <pcl\search\kdtree.h>
#include <pcl\features\moment_of_inertia_estimation.h>
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\common\geometry.h>
#include <vector>
#include <algorithm>

typedef std::pair<double, pcl::PointCloud<pcl::PointXYZ>> Length_Line;
typedef std::vector<std::pair<double, pcl::PointCloud<pcl::PointXYZ>>> Length_Lines;

class InteractiveConvexHullEgdeMeasure
{
public:
	InteractiveConvexHullEgdeMeasure():
		_obj(new pcl::PointCloud<pcl::PointXYZ>),
		_tree(new pcl::search::KdTree<pcl::PointXYZ>),
		_convex_points(new pcl::PointCloud<pcl::PointXYZ>),
		_line_index(0),
		_cloud_viewer(new pcl::visualization::PCLVisualizer)
	{};
	static bool comparePair(std::pair<double, pcl::PointCloud<pcl::PointXYZ>> pair1, std::pair<double, pcl::PointCloud<pcl::PointXYZ>> pair2)
	{
		return pair1.first > pair2.first;
	}
	void setInputCloudPath(std::string data_path)
	{
		_data_path = data_path;
	}
	void KeyboardEventOccured(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
	{
		if (event.getKeySym() == "Up" && event.keyDown() && event.isCtrlPressed())
		{
			// 因为pt1->pt2和pt2->pt1是一样的，所以会有两条在vector中顺序紧挨且长度相等的线
			_line_index += 2;

			if (_line_index < _length_lines.size())
			{
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _length_lines[_line_index].second.at(0);
				pcl::PointXYZ pt2 = _length_lines[_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 255, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
			}
			else
			{
				_line_index -= 2;
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _length_lines[_line_index].second.at(0);
				pcl::PointXYZ pt2 = _length_lines[_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 255, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
				std::cout << "No more line to measure!!" << std::endl;
			}
			
		}
		if (event.getKeySym() == "Down" && event.keyDown() && event.isCtrlPressed())
		{
			_line_index -= 2;

			if (_line_index >= 0)
			{
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _length_lines[_line_index].second.at(0);
				pcl::PointXYZ pt2 = _length_lines[_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 255, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
			}
			else
			{
				_line_index += 2;
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _length_lines[_line_index].second.at(0);
				pcl::PointXYZ pt2 = _length_lines[_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 255, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
				std::cout << "No more line to measure!!" << std::endl;
			}
			
		}
		// show all
		if (event.getKeySym() == "a"  && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			_cloud_viewer->addPolylineFromPolygonMesh(_mesh);
			_line_index = -2;
			//_cloud_viewer->addPolygonMesh(mesh);
			
		}
		// reset the index
		if (event.getKeySym() == "r" && event.keyDown() && event.isShiftPressed() && event.isCtrlPressed())
		{
			_line_index = -2;
			_cloud_viewer->removeAllShapes();
			std::cout << "Reset!!" << std::endl;
		}
	}
	int extract()
	{
		pcl::io::loadPLYFile(_data_path, *_obj);
		_convex_hull.setInputCloud(_obj);
		_convex_hull.setDimension(3);
		_convex_hull.setSearchMethod(_tree);
		_convex_hull.reconstruct(_mesh);
		_convex_hull.reconstruct(*_convex_points, _polygons);

		int viewport = 0;
		double coor_sys_scale = 0.1;
		_cloud_viewer->setBackgroundColor(0, 0, 0, viewport);
		_cloud_viewer->addCoordinateSystem(coor_sys_scale, viewport);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(_obj, 0, 128, 255);
		_cloud_viewer->addPointCloud(_obj, cloud_color, "cloud", viewport);

		_cloud_viewer->registerKeyboardCallback(&InteractiveConvexHullEgdeMeasure::KeyboardEventOccured, *this);

		std::stringstream line_length_txt_ss;

		std::cout << "convex points: " << _convex_points->size() << std::endl;
		std::cout << "convex facets: " << _polygons.size() << std::endl << std::endl;

		std::cout << "Measure the next edge (shorter) : Press Ctrl + Up" << std::endl;
		std::cout << "Measure the last edge (longer) : Press Ctrl + Down" << std::endl;
		std::cout << "Show all the edges: Press Ctrl + A" << std::endl;
		std::cout << "Reset: Press Shift + Ctrl + R" << std::endl;

		for (int i = 0; i < _polygons.size(); i++)
		{
			pcl::PointXYZ pt1 = _convex_points->at(_polygons[i].vertices[0]);
			pcl::PointXYZ pt2 = _convex_points->at(_polygons[i].vertices[1]);
			pcl::PointXYZ pt3 = _convex_points->at(_polygons[i].vertices[2]);

			double line1_length = pcl::geometry::distance(pt1, pt2);
			Length_Line pair1;
			pair1.first = line1_length;
			pair1.second.push_back(pt1);
			pair1.second.push_back(pt2);
			_length_lines.push_back(pair1);

			double line2_length = pcl::geometry::distance(pt2, pt3);
			Length_Line pair2;
			pair2.first = line2_length;
			pair2.second.push_back(pt2);
			pair2.second.push_back(pt3);
			_length_lines.push_back(pair2);

			double line3_length = pcl::geometry::distance(pt3, pt1);
			Length_Line pair3;
			pair3.first = line3_length;
			pair3.second.push_back(pt3);
			pair3.second.push_back(pt1);
			_length_lines.push_back(pair3);
		}

		std::sort(_length_lines.begin(), _length_lines.end(), &InteractiveConvexHullEgdeMeasure::comparePair);

		pcl::PointXYZ pt1 = _length_lines[0].second.at(0);
		pcl::PointXYZ pt2 = _length_lines[0].second.at(1);
		_cloud_viewer->addLine(pt1, pt2, 255, 255, 0);

		double line_length = pcl::geometry::distance(pt1, pt2);
		line_length_txt_ss << line_length << " m" << std::endl;
		std::string line_length_txt = line_length_txt_ss.str();
		line_length_txt_ss.str("");

		double txt_scale_coeff = 0.04;
		pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

		_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);

		while (!_cloud_viewer->wasStopped())
		{
			_cloud_viewer->spinOnce(100);
		}
		return 0;
	}
private:
	std::string _data_path;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _obj;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr _tree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _convex_points;
	std::vector<pcl::Vertices> _polygons;
	pcl::ConvexHull<pcl::PointXYZ> _convex_hull;
	pcl::PolygonMesh _mesh;
	pcl::visualization::PCLVisualizer::Ptr _cloud_viewer;
	Length_Lines _length_lines;
	int _line_index;
};
class InteractiveConcaveHullEgdeMeasure
{
public:
	InteractiveConcaveHullEgdeMeasure() :
		_obj(new pcl::PointCloud<pcl::PointXYZ>),
		_tree(new pcl::search::KdTree<pcl::PointXYZ>),
		_concave_points(new pcl::PointCloud<pcl::PointXYZ>),
		_line_index(0),
		_alpha(1e3),
		_cloud_viewer(new pcl::visualization::PCLVisualizer)
	{};
	static bool comparePair(std::pair<double, pcl::PointCloud<pcl::PointXYZ>> pair1, std::pair<double, pcl::PointCloud<pcl::PointXYZ>> pair2)
	{
		return pair1.first > pair2.first;
	}
	void setInputCloudPath(std::string data_path)
	{
		_data_path = data_path;
	}
	void setAlpha(double alpha)
	{
		_alpha = alpha;
	}
	void KeyboardEventOccured(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
	{
		if (event.getKeySym() == "Up" && event.keyDown() && event.isCtrlPressed())
		{
			// 因为pt1->pt2和pt2->pt1是一样的，所以会有两条在vector中顺序紧挨且长度相等的线
			_line_index += 2;

			if (_line_index < _length_lines.size())
			{
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _length_lines[_line_index].second.at(0);
				pcl::PointXYZ pt2 = _length_lines[_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 0, 255);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
			}
			else
			{
				_line_index -= 2;
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _length_lines[_line_index].second.at(0);
				pcl::PointXYZ pt2 = _length_lines[_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 0, 255);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
				std::cout << "No more line to measure!!" << std::endl;
			}

		}
		if (event.getKeySym() == "Down" && event.keyDown() && event.isCtrlPressed())
		{
			_line_index -= 2;

			if (_line_index >= 0)
			{
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _length_lines[_line_index].second.at(0);
				pcl::PointXYZ pt2 = _length_lines[_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 0, 255);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
			}
			else
			{
				_line_index += 2;
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _length_lines[_line_index].second.at(0);
				pcl::PointXYZ pt2 = _length_lines[_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 0, 255);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
				std::cout << "No more line to measure!!" << std::endl;
			}

		}
		// show all
		if (event.getKeySym() == "a"  && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			_cloud_viewer->addPolylineFromPolygonMesh(_mesh);
			_line_index = -2;
			//_cloud_viewer->addPolygonMesh(mesh);

		}
		// reset the index
		if (event.getKeySym() == "r" && event.keyDown() && event.isShiftPressed() && event.isCtrlPressed())
		{
			_line_index = -2;
			_cloud_viewer->removeAllShapes();
			std::cout << "Reset!!" << std::endl;
		}
	}
	int extract()
	{
		pcl::io::loadPLYFile(_data_path, *_obj);
		_concave_hull.setInputCloud(_obj);
		_concave_hull.setDimension(3);
		_concave_hull.setSearchMethod(_tree);
		// 该值越小，凹壳越精细，面片数更多，该值衡量的是顶点到其所在面片中心的最大距离
		_concave_hull.setAlpha(_alpha);
		_concave_hull.reconstruct(_mesh);
		_concave_hull.reconstruct(*_concave_points, _polygons);

		int viewport = 0;
		double coor_sys_scale = 0.1;
		_cloud_viewer->setBackgroundColor(0, 0, 0, viewport);
		_cloud_viewer->addCoordinateSystem(coor_sys_scale, viewport);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(_obj, 0, 128, 255);
		_cloud_viewer->addPointCloud(_obj, cloud_color, "cloud", viewport);

		_cloud_viewer->registerKeyboardCallback(&InteractiveConcaveHullEgdeMeasure::KeyboardEventOccured, *this);

		std::stringstream line_length_txt_ss;

		std::cout << "concave points: " << _concave_points->size() << std::endl;
		std::cout << "concave facets: " << _polygons.size() << std::endl << std::endl;

		std::cout << "Measure the next edge (shorter) : Press Ctrl + Up" << std::endl;
		std::cout << "Measure the last edge (longer) : Press Ctrl + Down" << std::endl;
		std::cout << "Show all the edges: Press Ctrl + A" << std::endl;
		std::cout << "Reset: Press Shift + Ctrl + R" << std::endl;

		for (int i = 0; i < _polygons.size(); i++)
		{
			pcl::PointXYZ pt1 = _concave_points->at(_polygons[i].vertices[0]);
			pcl::PointXYZ pt2 = _concave_points->at(_polygons[i].vertices[1]);
			pcl::PointXYZ pt3 = _concave_points->at(_polygons[i].vertices[2]);

			double line1_length = pcl::geometry::distance(pt1, pt2);
			Length_Line pair1;
			pair1.first = line1_length;
			pair1.second.push_back(pt1);
			pair1.second.push_back(pt2);
			_length_lines.push_back(pair1);

			double line2_length = pcl::geometry::distance(pt2, pt3);
			Length_Line pair2;
			pair2.first = line2_length;
			pair2.second.push_back(pt2);
			pair2.second.push_back(pt3);
			_length_lines.push_back(pair2);

			double line3_length = pcl::geometry::distance(pt3, pt1);
			Length_Line pair3;
			pair3.first = line3_length;
			pair3.second.push_back(pt3);
			pair3.second.push_back(pt1);
			_length_lines.push_back(pair3);
		}

		std::sort(_length_lines.begin(), _length_lines.end(), &InteractiveConcaveHullEgdeMeasure::comparePair);

		pcl::PointXYZ pt1 = _length_lines[0].second.at(0);
		pcl::PointXYZ pt2 = _length_lines[0].second.at(1);
		_cloud_viewer->addLine(pt1, pt2, 255, 0, 255);

		double line_length = pcl::geometry::distance(pt1, pt2);
		line_length_txt_ss << line_length << " m" << std::endl;
		std::string line_length_txt = line_length_txt_ss.str();
		line_length_txt_ss.str("");

		double txt_scale_coeff = 0.04;
		pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

		_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);

		while (!_cloud_viewer->wasStopped())
		{
			_cloud_viewer->spinOnce(100);
		}
		return 0;
	}
private:
	std::string _data_path;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _obj;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr _tree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _concave_points;
	std::vector<pcl::Vertices> _polygons;
	pcl::ConcaveHull<pcl::PointXYZ> _concave_hull;
	pcl::PolygonMesh _mesh;
	pcl::visualization::PCLVisualizer::Ptr _cloud_viewer;
	Length_Lines _length_lines;
	int _line_index;
	double _alpha;
};
class BoundingBoxEdgeMeasure
{
public:
	BoundingBoxEdgeMeasure():
	_obj(new pcl::PointCloud<pcl::PointXYZ>),
	_txt_scale_coeff(0.04),
	_cloud_viewer(new pcl::visualization::PCLVisualizer){}
	void setInputCloudPath(std::string data_path)
	{
		_data_path = data_path;
	}
	void KeyboardEventOccured(const pcl::visualization::KeyboardEvent &event, void* viewer_void) 
	{
		// A AABB
		if (event.getKeySym() == "a" && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			_length_AABB_txt.str("");
			_width_AABB_txt.str("");
			_height_AABB_txt.str("");

			pcl::PointXYZ pt1(_max_point_AABB.x, _max_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt2(_min_point_AABB.x, _max_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt3(_min_point_AABB.x, _min_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt4(_max_point_AABB.x, _min_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt5(_max_point_AABB.x, _max_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt6(_min_point_AABB.x, _max_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt7(_min_point_AABB.x, _min_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt8(_max_point_AABB.x, _min_point_AABB.y, _max_point_AABB.z);

			// 3->7 length  3->4 width 3->2 height
			_cloud_viewer->addLine(pt1, pt2, 255, 0, 0, "line1");
			_cloud_viewer->addLine(pt2, pt3, 255, 0, 0, "line2");
			_cloud_viewer->addLine(pt3, pt4, 255, 0, 0, "line3");
			_cloud_viewer->addLine(pt4, pt1, 255, 0, 0, "line4");
			_cloud_viewer->addLine(pt5, pt6, 255, 0, 0, "line5");
			_cloud_viewer->addLine(pt6, pt7, 255, 0, 0, "line6");
			_cloud_viewer->addLine(pt7, pt8, 255, 0, 0, "line7");
			_cloud_viewer->addLine(pt8, pt5, 255, 0, 0, "line8");
			_cloud_viewer->addLine(pt1, pt5, 255, 0, 0, "line9");
			_cloud_viewer->addLine(pt2, pt6, 255, 0, 0, "line10");
			_cloud_viewer->addLine(pt3, pt7, 255, 0, 0, "line11");
			_cloud_viewer->addLine(pt4, pt8, 255, 0, 0, "line12");

			_length_AABB = pcl::geometry::distance(pt3, pt7);
			_width_AABB = pcl::geometry::distance(pt3, pt4);
			_height_AABB = pcl::geometry::distance(pt3, pt2);

			_length_AABB_txt << _length_AABB << " m" << std::endl;
			_width_AABB_txt << _width_AABB << " m" << std::endl;
			_height_AABB_txt << _height_AABB << " m" << std::endl;

			_length_txt_pos_AABB = pcl::PointXYZ((pt3.x + pt7.x) / 2.0, (pt3.y + pt7.y) / 2.0, (pt3.z + pt7.z) / 2.0);
			_width_txt_pos_AABB = pcl::PointXYZ((pt3.x + pt4.x) / 2.0, (pt3.y + pt4.y) / 2.0, (pt3.z + pt4.z) / 2.0);
			_height_txt_pos_AABB = pcl::PointXYZ((pt3.x + pt2.x) / 2.0, (pt3.y + pt2.y) / 2.0, (pt3.z + pt2.z) / 2.0);

			_cloud_viewer->addText3D(_length_AABB_txt.str(), _length_txt_pos_AABB, _txt_scale_coeff*_length_AABB, 255, 255, 255, "length_AABB", 0);
			_cloud_viewer->addText3D(_width_AABB_txt.str(), _width_txt_pos_AABB, _txt_scale_coeff*_width_AABB, 255, 255, 255, "width_AABB", 0);
			_cloud_viewer->addText3D(_height_AABB_txt.str(), _height_txt_pos_AABB, _txt_scale_coeff*_height_AABB, 255, 255, 255, "height_AABB", 0);
		}
		// O OBB
		if (event.getKeySym() == "o" && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			_length_OBB_txt.str("");
			_width_OBB_txt.str("");
			_height_OBB_txt.str("");

			Eigen::Vector3f p1(_max_point_OBB.x, _max_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p2(_min_point_OBB.x, _max_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p3(_min_point_OBB.x, _min_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p4(_max_point_OBB.x, _min_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p5(_max_point_OBB.x, _max_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p6(_min_point_OBB.x, _max_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p7(_min_point_OBB.x, _min_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p8(_max_point_OBB.x, _min_point_OBB.y, _max_point_OBB.z);

			Eigen::Vector3f position_OBB(_position_OBB.x, _position_OBB.y, _position_OBB.z);

			p1 = _rotation_matrix_OBB * p1 + position_OBB;
			p2 = _rotation_matrix_OBB * p2 + position_OBB;
			p3 = _rotation_matrix_OBB * p3 + position_OBB;
			p4 = _rotation_matrix_OBB * p4 + position_OBB;
			p5 = _rotation_matrix_OBB * p5 + position_OBB;
			p6 = _rotation_matrix_OBB * p6 + position_OBB;
			p7 = _rotation_matrix_OBB * p7 + position_OBB;
			p8 = _rotation_matrix_OBB * p8 + position_OBB;

			pcl::PointXYZ pt1(p1(0), p1(1), p1(2));
			pcl::PointXYZ pt2(p2(0), p2(1), p2(2));
			pcl::PointXYZ pt3(p3(0), p3(1), p3(2));
			pcl::PointXYZ pt4(p4(0), p4(1), p4(2));
			pcl::PointXYZ pt5(p5(0), p5(1), p5(2));
			pcl::PointXYZ pt6(p6(0), p6(1), p6(2));
			pcl::PointXYZ pt7(p7(0), p7(1), p7(2));
			pcl::PointXYZ pt8(p8(0), p8(1), p8(2));

			// 3->7 length  3->4 width 3->2 height
			_cloud_viewer->addLine(pt1, pt2, 255, 255, 0, "line1");
			_cloud_viewer->addLine(pt2, pt3, 255, 255, 0, "line2");
			_cloud_viewer->addLine(pt3, pt4, 255, 255, 0, "line3");
			_cloud_viewer->addLine(pt4, pt1, 255, 255, 0, "line4");
			_cloud_viewer->addLine(pt5, pt6, 255, 255, 0, "line5");
			_cloud_viewer->addLine(pt6, pt7, 255, 255, 0, "line6");
			_cloud_viewer->addLine(pt7, pt8, 255, 255, 0, "line7");
			_cloud_viewer->addLine(pt8, pt5, 255, 255, 0, "line8");
			_cloud_viewer->addLine(pt1, pt5, 255, 255, 0, "line9");
			_cloud_viewer->addLine(pt2, pt6, 255, 255, 0, "line10");
			_cloud_viewer->addLine(pt3, pt7, 255, 255, 0, "line11");
			_cloud_viewer->addLine(pt4, pt8, 255, 255, 0, "line12");

			_length_OBB = pcl::geometry::distance(pt3, pt7);
			_width_OBB = pcl::geometry::distance(pt3, pt4);
			_height_OBB = pcl::geometry::distance(pt3, pt2);

			_length_OBB_txt << _length_OBB << " m" << std::endl;
			_width_OBB_txt << _width_OBB << " m" << std::endl;
			_height_OBB_txt << _height_OBB << " m" << std::endl;

			_length_txt_pos_OBB = pcl::PointXYZ((pt3.x + pt7.x) / 2.0, (pt3.y + pt7.y) / 2.0, (pt3.z + pt7.z) / 2.0);
			_width_txt_pos_OBB = pcl::PointXYZ((pt3.x + pt4.x) / 2.0, (pt3.y + pt4.y) / 2.0, (pt3.z + pt4.z) / 2.0);
			_height_txt_pos_OBB = pcl::PointXYZ((pt3.x + pt2.x) / 2.0, (pt3.y + pt2.y) / 2.0, (pt3.z + pt2.z) / 2.0);

			_cloud_viewer->addText3D(_length_OBB_txt.str(), _length_txt_pos_OBB, _txt_scale_coeff*_length_OBB, 255, 255, 255, "length_OBB", 0);
			_cloud_viewer->addText3D(_width_OBB_txt.str(), _width_txt_pos_OBB, _txt_scale_coeff*_width_OBB, 255, 255, 255, "width_OBB", 0);
			_cloud_viewer->addText3D(_height_OBB_txt.str(), _height_txt_pos_OBB, _txt_scale_coeff*_height_OBB, 255, 255, 255, "height_OBB", 0);
		}
		// B Both
		if (event.getKeySym() == "b" && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			pcl::PointXYZ pt1_AABB(_max_point_AABB.x, _max_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt2_AABB(_min_point_AABB.x, _max_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt3_AABB(_min_point_AABB.x, _min_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt4_AABB(_max_point_AABB.x, _min_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt5_AABB(_max_point_AABB.x, _max_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt6_AABB(_min_point_AABB.x, _max_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt7_AABB(_min_point_AABB.x, _min_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt8_AABB(_max_point_AABB.x, _min_point_AABB.y, _max_point_AABB.z);

			// 3->7 length  3->4 width 3->2 height
			_cloud_viewer->addLine(pt1_AABB, pt2_AABB, 255, 0, 0, "line1_AABB");
			_cloud_viewer->addLine(pt2_AABB, pt3_AABB, 255, 0, 0, "line2_AABB");
			_cloud_viewer->addLine(pt3_AABB, pt4_AABB, 255, 0, 0, "line3_AABB");
			_cloud_viewer->addLine(pt4_AABB, pt1_AABB, 255, 0, 0, "line4_AABB");
			_cloud_viewer->addLine(pt5_AABB, pt6_AABB, 255, 0, 0, "line5_AABB");
			_cloud_viewer->addLine(pt6_AABB, pt7_AABB, 255, 0, 0, "line6_AABB");
			_cloud_viewer->addLine(pt7_AABB, pt8_AABB, 255, 0, 0, "line7_AABB");
			_cloud_viewer->addLine(pt8_AABB, pt5_AABB, 255, 0, 0, "line8_AABB");
			_cloud_viewer->addLine(pt1_AABB, pt5_AABB, 255, 0, 0, "line9_AABB");
			_cloud_viewer->addLine(pt2_AABB, pt6_AABB, 255, 0, 0, "line10_AABB");
			_cloud_viewer->addLine(pt3_AABB, pt7_AABB, 255, 0, 0, "line11_AABB");
			_cloud_viewer->addLine(pt4_AABB, pt8_AABB, 255, 0, 0, "line12_AABB");



			Eigen::Vector3f p1_OBB(_max_point_OBB.x, _max_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p2_OBB(_min_point_OBB.x, _max_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p3_OBB(_min_point_OBB.x, _min_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p4_OBB(_max_point_OBB.x, _min_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p5_OBB(_max_point_OBB.x, _max_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p6_OBB(_min_point_OBB.x, _max_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p7_OBB(_min_point_OBB.x, _min_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p8_OBB(_max_point_OBB.x, _min_point_OBB.y, _max_point_OBB.z);

			Eigen::Vector3f position_OBB(_position_OBB.x, _position_OBB.y, _position_OBB.z);

			p1_OBB = _rotation_matrix_OBB * p1_OBB + position_OBB;
			p2_OBB = _rotation_matrix_OBB * p2_OBB + position_OBB;
			p3_OBB = _rotation_matrix_OBB * p3_OBB + position_OBB;
			p4_OBB = _rotation_matrix_OBB * p4_OBB + position_OBB;
			p5_OBB = _rotation_matrix_OBB * p5_OBB + position_OBB;
			p6_OBB = _rotation_matrix_OBB * p6_OBB + position_OBB;
			p7_OBB = _rotation_matrix_OBB * p7_OBB + position_OBB;
			p8_OBB = _rotation_matrix_OBB * p8_OBB + position_OBB;

			pcl::PointXYZ pt1_OBB(p1_OBB(0), p1_OBB(1), p1_OBB(2));
			pcl::PointXYZ pt2_OBB(p2_OBB(0), p2_OBB(1), p2_OBB(2));
			pcl::PointXYZ pt3_OBB(p3_OBB(0), p3_OBB(1), p3_OBB(2));
			pcl::PointXYZ pt4_OBB(p4_OBB(0), p4_OBB(1), p4_OBB(2));
			pcl::PointXYZ pt5_OBB(p5_OBB(0), p5_OBB(1), p5_OBB(2));
			pcl::PointXYZ pt6_OBB(p6_OBB(0), p6_OBB(1), p6_OBB(2));
			pcl::PointXYZ pt7_OBB(p7_OBB(0), p7_OBB(1), p7_OBB(2));
			pcl::PointXYZ pt8_OBB(p8_OBB(0), p8_OBB(1), p8_OBB(2));

			// 3->7 length  3->4 width 3->2 height
			_cloud_viewer->addLine(pt1_OBB, pt2_OBB, 255, 255, 0, "line1_OBB");
			_cloud_viewer->addLine(pt2_OBB, pt3_OBB, 255, 255, 0, "line2_OBB");
			_cloud_viewer->addLine(pt3_OBB, pt4_OBB, 255, 255, 0, "line3_OBB");
			_cloud_viewer->addLine(pt4_OBB, pt1_OBB, 255, 255, 0, "line4_OBB");
			_cloud_viewer->addLine(pt5_OBB, pt6_OBB, 255, 255, 0, "line5_OBB");
			_cloud_viewer->addLine(pt6_OBB, pt7_OBB, 255, 255, 0, "line6_OBB");
			_cloud_viewer->addLine(pt7_OBB, pt8_OBB, 255, 255, 0, "line7_OBB");
			_cloud_viewer->addLine(pt8_OBB, pt5_OBB, 255, 255, 0, "line8_OBB");
			_cloud_viewer->addLine(pt1_OBB, pt5_OBB, 255, 255, 0, "line9_OBB");
			_cloud_viewer->addLine(pt2_OBB, pt6_OBB, 255, 255, 0, "line10_OBB");
			_cloud_viewer->addLine(pt3_OBB, pt7_OBB, 255, 255, 0, "line11_OBB");
			_cloud_viewer->addLine(pt4_OBB, pt8_OBB, 255, 255, 0, "line12_OBB");

		}

		// E Empty
		if (event.getKeySym() == "e" && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			std::cout << "Empty the measurements!!" << std::endl;
		}
	}
	int extract()
	{
		pcl::io::loadPLYFile(_data_path, *_obj);

		std::cout << "Measure the edges by AABB: Press Ctrl + A" << std::endl;
		std::cout << "Measure the edges by OBB: Press Ctrl + O" << std::endl;
		std::cout << "Show both of AABB and OBB: Press Ctrl + B" << std::endl;
		std::cout << "Empty the results: Press Ctrl + E" << std::endl;

		_estimator.setInputCloud(_obj);
		_estimator.compute();

		std::cout << "Bounding boxes are calculated!!" << std::endl;

		_estimator.getAABB(_min_point_AABB, _max_point_AABB);
		_estimator.getOBB(_min_point_OBB, _max_point_OBB, _position_OBB, _rotation_matrix_OBB);

		double coor_sys_scale = 0.1;
		_cloud_viewer->setBackgroundColor(0, 0, 0);
		_cloud_viewer->addCoordinateSystem(coor_sys_scale);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(_obj, 0, 128, 255);
		_cloud_viewer->addPointCloud(_obj, cloud_color);
		_cloud_viewer->registerKeyboardCallback(&BoundingBoxEdgeMeasure::KeyboardEventOccured, *this);


		while (!_cloud_viewer->wasStopped())
		{
			_cloud_viewer->spinOnce(100);
		}

		return 0;
	}
private:
	std::string _data_path;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _obj;
	pcl::visualization::PCLVisualizer::Ptr _cloud_viewer;
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> _estimator;
	pcl::PointXYZ _min_point_AABB;
	pcl::PointXYZ _max_point_AABB;
	pcl::PointXYZ _min_point_OBB;
	pcl::PointXYZ _max_point_OBB;
	pcl::PointXYZ _position_OBB;
	Eigen::Matrix3f _rotation_matrix_OBB;
	double _length_AABB;
	double _length_OBB;
	double _width_AABB;
	double _width_OBB;
	double _height_AABB;
	double _height_OBB;
	std::stringstream _length_AABB_txt;
	std::stringstream _length_OBB_txt;
	std::stringstream _width_AABB_txt;
	std::stringstream _width_OBB_txt;
	std::stringstream _height_AABB_txt;
	std::stringstream _height_OBB_txt;
	double _txt_scale_coeff;
	pcl::PointXYZ _length_txt_pos_AABB;
	pcl::PointXYZ _width_txt_pos_AABB;
	pcl::PointXYZ _height_txt_pos_AABB;
	pcl::PointXYZ _length_txt_pos_OBB;
	pcl::PointXYZ _width_txt_pos_OBB;
	pcl::PointXYZ _height_txt_pos_OBB;
};

class ObjectMeasure3D
{
public:
	ObjectMeasure3D() :
		_obj(new pcl::PointCloud<pcl::PointXYZ>),
		_txt_scale_coeff(0.04),
		_tree(new pcl::search::KdTree<pcl::PointXYZ>),
		_convex_points(new pcl::PointCloud<pcl::PointXYZ>),
		_concave_points(new pcl::PointCloud<pcl::PointXYZ>),
		_convex_line_index(-2),
		_concave_line_index(-2),
		_alpha(1e3),
		_cloud_viewer(new pcl::visualization::PCLVisualizer) {}
	void setInputCloudPath(std::string data_path)
	{
		_data_path = data_path;
	}
	static bool comparePair(std::pair<double, pcl::PointCloud<pcl::PointXYZ>> pair1, std::pair<double, pcl::PointCloud<pcl::PointXYZ>> pair2)
	{
		return pair1.first > pair2.first;
	}
	void setAlpha(double alpha)
	{
		_alpha = alpha;
	}
	void KeyboardEventOccured(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
	{
		// A AABB  red
		if (event.getKeySym() == "a" && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			_length_AABB_txt.str("");
			_width_AABB_txt.str("");
			_height_AABB_txt.str("");

			pcl::PointXYZ pt1(_max_point_AABB.x, _max_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt2(_min_point_AABB.x, _max_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt3(_min_point_AABB.x, _min_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt4(_max_point_AABB.x, _min_point_AABB.y, _min_point_AABB.z);
			pcl::PointXYZ pt5(_max_point_AABB.x, _max_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt6(_min_point_AABB.x, _max_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt7(_min_point_AABB.x, _min_point_AABB.y, _max_point_AABB.z);
			pcl::PointXYZ pt8(_max_point_AABB.x, _min_point_AABB.y, _max_point_AABB.z);

			// 3->7 length  3->4 width 3->2 height
			_cloud_viewer->addLine(pt1, pt2, 255, 0, 0, "line1");
			_cloud_viewer->addLine(pt2, pt3, 255, 0, 0, "line2");
			_cloud_viewer->addLine(pt3, pt4, 255, 0, 0, "line3");
			_cloud_viewer->addLine(pt4, pt1, 255, 0, 0, "line4");
			_cloud_viewer->addLine(pt5, pt6, 255, 0, 0, "line5");
			_cloud_viewer->addLine(pt6, pt7, 255, 0, 0, "line6");
			_cloud_viewer->addLine(pt7, pt8, 255, 0, 0, "line7");
			_cloud_viewer->addLine(pt8, pt5, 255, 0, 0, "line8");
			_cloud_viewer->addLine(pt1, pt5, 255, 0, 0, "line9");
			_cloud_viewer->addLine(pt2, pt6, 255, 0, 0, "line10");
			_cloud_viewer->addLine(pt3, pt7, 255, 0, 0, "line11");
			_cloud_viewer->addLine(pt4, pt8, 255, 0, 0, "line12");

			_length_AABB = pcl::geometry::distance(pt3, pt7);
			_width_AABB = pcl::geometry::distance(pt3, pt4);
			_height_AABB = pcl::geometry::distance(pt3, pt2);

			_length_AABB_txt << _length_AABB << " m" << std::endl;
			_width_AABB_txt << _width_AABB << " m" << std::endl;
			_height_AABB_txt << _height_AABB << " m" << std::endl;

			_length_txt_pos_AABB = pcl::PointXYZ((pt3.x + pt7.x) / 2.0, (pt3.y + pt7.y) / 2.0, (pt3.z + pt7.z) / 2.0);
			_width_txt_pos_AABB = pcl::PointXYZ((pt3.x + pt4.x) / 2.0, (pt3.y + pt4.y) / 2.0, (pt3.z + pt4.z) / 2.0);
			_height_txt_pos_AABB = pcl::PointXYZ((pt3.x + pt2.x) / 2.0, (pt3.y + pt2.y) / 2.0, (pt3.z + pt2.z) / 2.0);

			_cloud_viewer->addText3D(_length_AABB_txt.str(), _length_txt_pos_AABB, _txt_scale_coeff*_length_AABB, 255, 255, 255, "length_AABB", 0);
			_cloud_viewer->addText3D(_width_AABB_txt.str(), _width_txt_pos_AABB, _txt_scale_coeff*_width_AABB, 255, 255, 255, "width_AABB", 0);
			_cloud_viewer->addText3D(_height_AABB_txt.str(), _height_txt_pos_AABB, _txt_scale_coeff*_height_AABB, 255, 255, 255, "height_AABB", 0);
		}
		// O OBB  yellow
		if (event.getKeySym() == "o" && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			_length_OBB_txt.str("");
			_width_OBB_txt.str("");
			_height_OBB_txt.str("");

			Eigen::Vector3f p1(_max_point_OBB.x, _max_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p2(_min_point_OBB.x, _max_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p3(_min_point_OBB.x, _min_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p4(_max_point_OBB.x, _min_point_OBB.y, _min_point_OBB.z);
			Eigen::Vector3f p5(_max_point_OBB.x, _max_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p6(_min_point_OBB.x, _max_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p7(_min_point_OBB.x, _min_point_OBB.y, _max_point_OBB.z);
			Eigen::Vector3f p8(_max_point_OBB.x, _min_point_OBB.y, _max_point_OBB.z);

			Eigen::Vector3f position_OBB(_position_OBB.x, _position_OBB.y, _position_OBB.z);

			p1 = _rotation_matrix_OBB * p1 + position_OBB;
			p2 = _rotation_matrix_OBB * p2 + position_OBB;
			p3 = _rotation_matrix_OBB * p3 + position_OBB;
			p4 = _rotation_matrix_OBB * p4 + position_OBB;
			p5 = _rotation_matrix_OBB * p5 + position_OBB;
			p6 = _rotation_matrix_OBB * p6 + position_OBB;
			p7 = _rotation_matrix_OBB * p7 + position_OBB;
			p8 = _rotation_matrix_OBB * p8 + position_OBB;

			pcl::PointXYZ pt1(p1(0), p1(1), p1(2));
			pcl::PointXYZ pt2(p2(0), p2(1), p2(2));
			pcl::PointXYZ pt3(p3(0), p3(1), p3(2));
			pcl::PointXYZ pt4(p4(0), p4(1), p4(2));
			pcl::PointXYZ pt5(p5(0), p5(1), p5(2));
			pcl::PointXYZ pt6(p6(0), p6(1), p6(2));
			pcl::PointXYZ pt7(p7(0), p7(1), p7(2));
			pcl::PointXYZ pt8(p8(0), p8(1), p8(2));

			// 3->7 length  3->4 width 3->2 height
			_cloud_viewer->addLine(pt1, pt2, 255, 255, 0, "line1");
			_cloud_viewer->addLine(pt2, pt3, 255, 255, 0, "line2");
			_cloud_viewer->addLine(pt3, pt4, 255, 255, 0, "line3");
			_cloud_viewer->addLine(pt4, pt1, 255, 255, 0, "line4");
			_cloud_viewer->addLine(pt5, pt6, 255, 255, 0, "line5");
			_cloud_viewer->addLine(pt6, pt7, 255, 255, 0, "line6");
			_cloud_viewer->addLine(pt7, pt8, 255, 255, 0, "line7");
			_cloud_viewer->addLine(pt8, pt5, 255, 255, 0, "line8");
			_cloud_viewer->addLine(pt1, pt5, 255, 255, 0, "line9");
			_cloud_viewer->addLine(pt2, pt6, 255, 255, 0, "line10");
			_cloud_viewer->addLine(pt3, pt7, 255, 255, 0, "line11");
			_cloud_viewer->addLine(pt4, pt8, 255, 255, 0, "line12");

			_length_OBB = pcl::geometry::distance(pt3, pt7);
			_width_OBB = pcl::geometry::distance(pt3, pt4);
			_height_OBB = pcl::geometry::distance(pt3, pt2);

			_length_OBB_txt << _length_OBB << " m" << std::endl;
			_width_OBB_txt << _width_OBB << " m" << std::endl;
			_height_OBB_txt << _height_OBB << " m" << std::endl;

			_length_txt_pos_OBB = pcl::PointXYZ((pt3.x + pt7.x) / 2.0, (pt3.y + pt7.y) / 2.0, (pt3.z + pt7.z) / 2.0);
			_width_txt_pos_OBB = pcl::PointXYZ((pt3.x + pt4.x) / 2.0, (pt3.y + pt4.y) / 2.0, (pt3.z + pt4.z) / 2.0);
			_height_txt_pos_OBB = pcl::PointXYZ((pt3.x + pt2.x) / 2.0, (pt3.y + pt2.y) / 2.0, (pt3.z + pt2.z) / 2.0);

			_cloud_viewer->addText3D(_length_OBB_txt.str(), _length_txt_pos_OBB, _txt_scale_coeff*_length_OBB, 255, 255, 255, "length_OBB", 0);
			_cloud_viewer->addText3D(_width_OBB_txt.str(), _width_txt_pos_OBB, _txt_scale_coeff*_width_OBB, 255, 255, 255, "width_OBB", 0);
			_cloud_viewer->addText3D(_height_OBB_txt.str(), _height_txt_pos_OBB, _txt_scale_coeff*_height_OBB, 255, 255, 255, "height_OBB", 0);
		}
		// E Empty
		if (event.getKeySym() == "e" && event.keyDown() && event.isCtrlPressed())
		{
			_convex_line_index = -2;
			_concave_line_index = -2;
			_cloud_viewer->removeAllShapes();
			std::cout << "Empty the measurements!!" << std::endl;
		}

		// convex hull  orange
		if (event.getKeySym() == "Up" && event.keyDown() && event.isAltPressed())
		{
			// 因为pt1->pt2和pt2->pt1是一样的，所以会有两条在vector中顺序紧挨且长度相等的线
			_convex_line_index += 2;

			if (_convex_line_index < _convex_length_lines.size())
			{
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _convex_length_lines[_convex_line_index].second.at(0);
				pcl::PointXYZ pt2 = _convex_length_lines[_convex_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 60, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
			}
			else
			{
				_convex_line_index = _convex_length_lines.size() - 2;
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _convex_length_lines[_convex_line_index].second.at(0);
				pcl::PointXYZ pt2 = _convex_length_lines[_convex_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 60, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
				std::cout << "This is the longest convex line!" << std::endl;
			}

		}
		if (event.getKeySym() == "Down" && event.keyDown() && event.isAltPressed())
		{
			_convex_line_index -= 2;

			if (_convex_line_index >= 0)
			{
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _convex_length_lines[_convex_line_index].second.at(0);
				pcl::PointXYZ pt2 = _convex_length_lines[_convex_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 60, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
			}
			else
			{
				_convex_line_index = 0;
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _convex_length_lines[_convex_line_index].second.at(0);
				pcl::PointXYZ pt2 = _convex_length_lines[_convex_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 255, 60, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
				std::cout << "This is the shortest convex line!" << std::endl;
			}

		}
		// show all convex 
		if (event.getKeySym() == "v"  && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			_cloud_viewer->addPolylineFromPolygonMesh(_convex_mesh);
			_convex_line_index = -2;
			//_cloud_viewer->addPolygonMesh(mesh);

		}
		
		// concave hull  green
		if (event.getKeySym() == "Up" && event.keyDown() && event.isCtrlPressed())
		{
			// 因为pt1->pt2和pt2->pt1是一样的，所以会有两条在vector中顺序紧挨且长度相等的线
			_concave_line_index += 2;

			if (_concave_line_index < _concave_length_lines.size())
			{
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _concave_length_lines[_concave_line_index].second.at(0);
				pcl::PointXYZ pt2 = _concave_length_lines[_concave_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 0, 180, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
			}
			else
			{
				_concave_line_index = _concave_length_lines.size() - 2;
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _concave_length_lines[_concave_line_index].second.at(0);
				pcl::PointXYZ pt2 = _concave_length_lines[_concave_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 0, 180, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
				std::cout << "This is the longest concave line!" << std::endl;
			}

		}
		if (event.getKeySym() == "Down" && event.keyDown() && event.isCtrlPressed())
		{
			_concave_line_index -= 2;

			if (_concave_line_index >= 0)
			{
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _concave_length_lines[_concave_line_index].second.at(0);
				pcl::PointXYZ pt2 = _concave_length_lines[_concave_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 0, 180, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
			}
			else
			{
				_concave_line_index = 0;
				_cloud_viewer->removeAllShapes();

				std::stringstream line_length_txt_ss;

				pcl::PointXYZ pt1 = _concave_length_lines[_concave_line_index].second.at(0);
				pcl::PointXYZ pt2 = _concave_length_lines[_concave_line_index].second.at(1);

				_cloud_viewer->addLine(pt1, pt2, 0, 180, 0);

				double line_length = pcl::geometry::distance(pt1, pt2);
				line_length_txt_ss << line_length << " m" << std::endl;
				std::string line_length_txt = line_length_txt_ss.str();
				line_length_txt_ss.str("");

				double txt_scale_coeff = 0.04;
				pcl::PointXYZ line_txt_pos = pcl::PointXYZ((pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0, (pt1.z + pt2.z) / 2.0);

				_cloud_viewer->addText3D(line_length_txt, line_txt_pos, txt_scale_coeff*line_length, 255, 255, 255);
				std::cout << "This is the shortest concave line!" << std::endl;
			}

		}
		// show all concave
		if (event.getKeySym() == "c"  && event.keyDown() && event.isCtrlPressed())
		{
			_cloud_viewer->removeAllShapes();
			_cloud_viewer->addPolylineFromPolygonMesh(_concave_mesh);
			_concave_line_index = -2;
			//_cloud_viewer->addPolygonMesh(mesh);

		}

	}
	int extract()
	{
		pcl::io::loadPLYFile(_data_path, *_obj);

		std::cout << "Measure the edges by AABB: Press Ctrl + A" << std::endl;
		std::cout << "Measure the edges by OBB: Press Ctrl + O" << std::endl;
		std::cout << "Measure the next convex edge (shorter) : Press Alt + Up" << std::endl;
		std::cout << "Measure the last convex edge (longer) : Press Alt + Down" << std::endl;
		std::cout << "Show all the convex edges: Press Ctrl + V" << std::endl;
		std::cout << "Measure the next concave edge (shorter) : Press Ctrl + Up" << std::endl;
		std::cout << "Measure the last concave edge (longer) : Press Ctrl + Down" << std::endl;
		std::cout << "Show all the concave edges: Press Ctrl + C" << std::endl;
		std::cout << "Empty the results: Press Ctrl + E" << std::endl;

		_estimator.setInputCloud(_obj);
		_estimator.compute();

		std::cout << "Bounding boxes are calculated!!" << std::endl;

		_estimator.getAABB(_min_point_AABB, _max_point_AABB);
		_estimator.getOBB(_min_point_OBB, _max_point_OBB, _position_OBB, _rotation_matrix_OBB);

		_convex_hull.setInputCloud(_obj);
		_convex_hull.setDimension(3);
		_convex_hull.setSearchMethod(_tree);
		_convex_hull.reconstruct(_convex_mesh);
		_convex_hull.reconstruct(*_convex_points, _convex_polygons);

		std::cout << "convex points: " << _convex_points->size() << std::endl;
		std::cout << "convex facets: " << _convex_polygons.size() << std::endl << std::endl;

		for (int i = 0; i < _convex_polygons.size(); i++)
		{
			pcl::PointXYZ pt1 = _convex_points->at(_convex_polygons[i].vertices[0]);
			pcl::PointXYZ pt2 = _convex_points->at(_convex_polygons[i].vertices[1]);
			pcl::PointXYZ pt3 = _convex_points->at(_convex_polygons[i].vertices[2]);

			double line1_length = pcl::geometry::distance(pt1, pt2);
			Length_Line pair1;
			pair1.first = line1_length;
			pair1.second.push_back(pt1);
			pair1.second.push_back(pt2);
			_convex_length_lines.push_back(pair1);

			double line2_length = pcl::geometry::distance(pt2, pt3);
			Length_Line pair2;
			pair2.first = line2_length;
			pair2.second.push_back(pt2);
			pair2.second.push_back(pt3);
			_convex_length_lines.push_back(pair2);

			double line3_length = pcl::geometry::distance(pt3, pt1);
			Length_Line pair3;
			pair3.first = line3_length;
			pair3.second.push_back(pt3);
			pair3.second.push_back(pt1);
			_convex_length_lines.push_back(pair3);
		}

		std::sort(_convex_length_lines.begin(), _convex_length_lines.end(), &ObjectMeasure3D::comparePair);

		_concave_hull.setInputCloud(_obj);
		_concave_hull.setDimension(3);
		_concave_hull.setSearchMethod(_tree);
		// 该值越小，凹壳越精细，面片数更多，该值衡量的是顶点到其所在面片中心的最大距离
		_concave_hull.setAlpha(_alpha);
		_concave_hull.reconstruct(_concave_mesh);
		_concave_hull.reconstruct(*_concave_points, _concave_polygons);

		std::cout << "concave points: " << _concave_points->size() << std::endl;
		std::cout << "concave facets: " << _concave_polygons.size() << std::endl << std::endl;

		for (int i = 0; i < _concave_polygons.size(); i++)
		{
			pcl::PointXYZ pt1 = _concave_points->at(_concave_polygons[i].vertices[0]);
			pcl::PointXYZ pt2 = _concave_points->at(_concave_polygons[i].vertices[1]);
			pcl::PointXYZ pt3 = _concave_points->at(_concave_polygons[i].vertices[2]);

			double line1_length = pcl::geometry::distance(pt1, pt2);
			Length_Line pair1;
			pair1.first = line1_length;
			pair1.second.push_back(pt1);
			pair1.second.push_back(pt2);
			_concave_length_lines.push_back(pair1);

			double line2_length = pcl::geometry::distance(pt2, pt3);
			Length_Line pair2;
			pair2.first = line2_length;
			pair2.second.push_back(pt2);
			pair2.second.push_back(pt3);
			_concave_length_lines.push_back(pair2);

			double line3_length = pcl::geometry::distance(pt3, pt1);
			Length_Line pair3;
			pair3.first = line3_length;
			pair3.second.push_back(pt3);
			pair3.second.push_back(pt1);
			_concave_length_lines.push_back(pair3);
		}

		std::sort(_concave_length_lines.begin(), _concave_length_lines.end(), &ObjectMeasure3D::comparePair);

		double coor_sys_scale = 0.1;
		_cloud_viewer->setBackgroundColor(0, 0, 0);
		_cloud_viewer->addCoordinateSystem(coor_sys_scale);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(_obj, 0, 128, 255);
		_cloud_viewer->addPointCloud(_obj, cloud_color);
		_cloud_viewer->registerKeyboardCallback(&ObjectMeasure3D::KeyboardEventOccured, *this);


		while (!_cloud_viewer->wasStopped())
		{
			_cloud_viewer->spinOnce(100);
		}

		return 0;
	}
private:
	std::string _data_path;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _obj;
	pcl::visualization::PCLVisualizer::Ptr _cloud_viewer;
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> _estimator;
	pcl::PointXYZ _min_point_AABB;
	pcl::PointXYZ _max_point_AABB;
	pcl::PointXYZ _min_point_OBB;
	pcl::PointXYZ _max_point_OBB;
	pcl::PointXYZ _position_OBB;
	Eigen::Matrix3f _rotation_matrix_OBB;
	double _length_AABB;
	double _length_OBB;
	double _width_AABB;
	double _width_OBB;
	double _height_AABB;
	double _height_OBB;
	std::stringstream _length_AABB_txt;
	std::stringstream _length_OBB_txt;
	std::stringstream _width_AABB_txt;
	std::stringstream _width_OBB_txt;
	std::stringstream _height_AABB_txt;
	std::stringstream _height_OBB_txt;
	double _txt_scale_coeff;
	pcl::PointXYZ _length_txt_pos_AABB;
	pcl::PointXYZ _width_txt_pos_AABB;
	pcl::PointXYZ _height_txt_pos_AABB;
	pcl::PointXYZ _length_txt_pos_OBB;
	pcl::PointXYZ _width_txt_pos_OBB;
	pcl::PointXYZ _height_txt_pos_OBB;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr _tree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _convex_points;
	std::vector<pcl::Vertices> _convex_polygons;
	pcl::ConvexHull<pcl::PointXYZ> _convex_hull;
	pcl::PolygonMesh _convex_mesh;
	Length_Lines _convex_length_lines;
	int _convex_line_index;

	pcl::PointCloud<pcl::PointXYZ>::Ptr _concave_points;
	std::vector<pcl::Vertices> _concave_polygons;
	pcl::ConcaveHull<pcl::PointXYZ> _concave_hull;
	pcl::PolygonMesh _concave_mesh;
	Length_Lines _concave_length_lines;
	int _concave_line_index;
	double _alpha;
};

int main()
{
	//InteractiveConcaveHullEgdeMeasure hull;
	//hull.setInputCloudPath("..//data//hand.ply");
	//hull.extract();

	//BoundingBoxEdgeMeasure bb;
	//bb.setInputCloudPath("..//data//hand.ply");
	//bb.extract();

	ObjectMeasure3D measure;
	measure.setInputCloudPath("..//data//hand.ply");
	measure.extract();

	return 0;
}