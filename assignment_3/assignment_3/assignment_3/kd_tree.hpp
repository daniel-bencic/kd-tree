#pragma once
#include <vector>
#include <algorithm>
#include <stack>
#include <iterator>
#include <memory>

#include <geometric_objects/geometric_objects.hpp>

template<int DIM>
struct Box
{
	Point<DIM> lo, hi;

	Box() : lo(Point<DIM>{}), hi(Point<DIM>{}) { }

	Box(Point<DIM> lo, Point<DIM> hi) : lo(lo), hi(hi) {	}
};

template<int DIM>
double distance(const Box<DIM>& box, const Point<DIM>& point)
{
	double d = 0;
	for (int i = 0; i < DIM; i++)
	{
		if (point.x[i] < box.lo.x[i]) d += (box.lo.x[i] - point.x[i]) * (box.lo.x[i] - point.x[i]);
		if (point.x[i] > box.hi.x[i]) d += (point.x[i] - box.hi.x[i]) * (point.x[i] - box.hi.x[i]);
	}
	return std::sqrt(d);
}

template<int DIM>
struct Box_node : Box<DIM>
{
	Point<DIM> point;
	std::shared_ptr<Box_node<DIM>> parent, left_child, right_child;

	Box_node<DIM>(Point<DIM> lo, Point<DIM> hi) : Box<DIM>(lo, hi), parent(nullptr), left_child(nullptr), right_child(nullptr) { }

	Box_node<DIM>(Point<DIM> lo, Point<DIM> hi, std::shared_ptr<Box_node<DIM>> parent,
		std::shared_ptr<Box_node<DIM>> left_child, std::shared_ptr<Box_node<DIM>> right_child)
			: Box<DIM>(lo, hi), point(point), parent(parent), left_child(left_child), right_child(right_child) { }
};

template<int DIM>
struct Subdivision_task
{
	int dim, ptlo, pthi;
	std::shared_ptr<Box_node<DIM>> box_node;

	Subdivision_task(std::shared_ptr<Box_node<DIM>> box_node, int dim, int ptlo, int pthi) 
		: dim(dim), ptlo(ptlo), pthi(pthi), box_node(box_node) { }
};

struct Coord_comp
{
	const int points_count;
	const std::vector<double> coords;
	const int curr_dim;

	Coord_comp(const int points_count, const std::vector<double>& coords, int curr_dim) : points_count(points_count), coords(coords), curr_dim(curr_dim) { }

	bool operator()(const int& a, const int& b) const
	{
		return coords[curr_dim * points_count + a] < coords[curr_dim * points_count + b];
	}
};

inline void partition_point_indexes(std::vector<int>& point_indexes, std::vector<double>& coords,
	int start_index, int partition_index, int end_index, int points_count, int curr_dim)
{
	auto iter = point_indexes.begin();
	std::nth_element(iter + start_index, iter + partition_index, iter + end_index + 1, Coord_comp{ points_count, coords, curr_dim });
}

template<int DIM>
struct KD_tree
{
	std::shared_ptr<Box_node<DIM>> root;

	KD_tree<DIM>(const Box<DIM>& bounding_box, const std::vector<Point<DIM>>& points)
	{
		const int points_count = points.size();

		if (points.empty()) return;

		std::vector<int> point_indexes;
		for (int i = 0; i < points_count; i++) point_indexes.push_back(i);

		// Coordinate list in form: <p1.x0, p2.x0, p3.x0, p1.x1, p2.x1, p3.x1, ...>
		std::vector<double> coords;
		coords.reserve(DIM * points_count);
		for (int i = 0; i < DIM; i++)
		{
			for (int j = 0; j < points_count; j++) coords.push_back(points[j].x[i]);
		}

		Point<DIM> lo = bounding_box.lo;
		Point<DIM> hi = bounding_box.hi;

		this->root = std::shared_ptr<Box_node<DIM>>{ new Box_node<DIM>{ lo, hi } };
		std::stack<Subdivision_task<DIM>> subdivision_stack;
		Subdivision_task<DIM> first_task{ this->root, 1, 0, points_count - 1 };
		subdivision_stack.push(first_task);


		while (!subdivision_stack.empty())
		{
			Subdivision_task<DIM> task = subdivision_stack.top();	// Get next subdivision task.
			subdivision_stack.pop();
			int ptlo = task.ptlo;	// Index for minimum point (on point_indexes), contained by this box.
			int pthi = task.pthi;	// Index for maximum point (on point_indexes), contained by this box.
			int points_count_subdivision = pthi - ptlo + 1;
			int half_index = (points_count_subdivision - 1) / 2;
			partition_point_indexes(point_indexes, coords, ptlo, ptlo + half_index, pthi, points_count, task.dim);
			task.box_node->point = points[point_indexes[ptlo + half_index]];
			Point<DIM> lo = task.box_node->lo;
			Point<DIM> hi = task.box_node->hi;
			lo.x[task.dim] = hi.x[task.dim] = coords[task.dim * points_count + point_indexes[ptlo + half_index]];
			task.box_node->left_child = std::shared_ptr<Box_node<DIM>>{ new Box_node<DIM>{ task.box_node->lo, hi, task.box_node, nullptr, nullptr } };
			task.box_node->right_child = std::shared_ptr<Box_node<DIM>>{ new Box_node<DIM>{ lo, task.box_node->hi, task.box_node, nullptr, nullptr } };
			if (half_index > 0)
			{
				subdivision_stack.push(Subdivision_task<DIM>{ task.box_node->left_child, (task.dim + 1) % DIM, ptlo, ptlo + half_index - 1 });
			}
			if (points_count_subdivision - half_index > 1)
			{
				subdivision_stack.push(Subdivision_task<DIM>{ task.box_node->right_child, (task.dim + 1) % DIM, ptlo + half_index + 1, pthi });
			}
		}
	}
};
