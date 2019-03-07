#pragma once
#include <SDL.h>

#include "kd_tree.hpp"

struct RGB_color
{
	int r, g, b;
};

enum class Point_conversion
{
	SDL_TO_CARTESIAN = 1,
	CARTESIAN_TO_SDL = 2
};

class Window
{
public:
	bool initialized;

	Window(int width, int height, int margin);
	void handle_events();
	void render() const;
	~Window();

private:
	static constexpr double scale = 1.0;
	int width, height, margin;
	bool range_query_started, range_query_available;
	Point<2> range_query_lo, range_query_hi;
	std::vector<Point<2>> points;
	std::vector<Point<2>> range_query_res;
	std::unique_ptr<KD_tree<2>> kd_tree;
	Box<2> bounding_box;
	SDL_Renderer* renderer;
	SDL_Window* window;

	void on_mouse_release(SDL_MouseButtonEvent mouse_button_event);
	void draw_coordinate_system() const;
	void draw_points(const std::vector<Point<2>>& points, RGB_color color) const;
	void draw_kd_tree() const;
	void draw_box(const std::shared_ptr<Box_node<2>>& box_node) const;
	void calc_kd_tree();
	void calc_bounding_box();
	void draw_bounding_box() const;
	void calc_range_query_result(const std::shared_ptr<Box_node<2>>& box_node, Box<2> range, int dim);
	double calc_x_scale(std::vector<Point<2>>& points);
	double calc_y_scale(std::vector<Point<2>>& points);
	void convert_point(Point_conversion conversion, Point<2>& point) const;
	SDL_Point point_to_sdl_point(Point<2> point) const;
};
