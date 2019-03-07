#include <SDL.h>

#include "window.hpp"
#include <iostream>

Window::Window(int width, int height, int margin) 
	: width(width), height(height), margin(margin), kd_tree(nullptr), renderer(nullptr), window(nullptr)
{
	if (SDL_Init(SDL_INIT_VIDEO) != 0)
	{
		SDL_LogError(SDL_LOG_CATEGORY_ERROR, "Unable to initialize SDL: %s", SDL_GetError());
		return;
	}

	this->window = SDL_CreateWindow("Convex Hull", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_ALLOW_HIGHDPI);
	if (window == nullptr)
	{
		SDL_LogError(SDL_LOG_CATEGORY_ERROR, "Could not create window: %s", SDL_GetError());
		return;
	}

	this->renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (renderer == nullptr)
	{
		SDL_LogError(SDL_LOG_CATEGORY_ERROR, "Could not create Renderer: %s", SDL_GetError());
		return;
	}

	this->calc_bounding_box();
	this->calc_kd_tree();
	this->initialized = true;
}

void Window::handle_events()
{
	SDL_Event event;
	while (SDL_PollEvent(&event))
	{
		switch (event.type)
		{
		case SDL_QUIT:
			this->initialized = false;
			break;
		case SDL_MOUSEBUTTONUP:
			this->on_mouse_release(event.button);
			break;
		
		}
	}
}

void Window::on_mouse_release(SDL_MouseButtonEvent mouse_button_event)
{
	if (mouse_button_event.button == SDL_BUTTON_RIGHT)
	{
		Point<2> point{ static_cast<double>(mouse_button_event.x), static_cast<double>(mouse_button_event.y) };
		this->convert_point(Point_conversion::SDL_TO_CARTESIAN, point);
		this->points.emplace_back(point);
		this->calc_kd_tree();
		this->range_query_res.clear();
		this->range_query_started = false;
		this->range_query_available = false;
	}
	else if (mouse_button_event.button == SDL_BUTTON_LEFT)
	{
		if (this->range_query_started)
		{
			Point<2> point{ static_cast<double>(mouse_button_event.x), static_cast<double>(mouse_button_event.y) };
			this->convert_point(Point_conversion::SDL_TO_CARTESIAN, point);
			this->range_query_hi = point;
			if (this->range_query_lo.x[0] > this->range_query_hi.x[0])
			{
				double temp = this->range_query_lo.x[0];
				this->range_query_lo.x[0] = this->range_query_hi.x[0];
				this->range_query_hi.x[0] = temp;
			}
			if (this->range_query_lo.x[1] > this->range_query_hi.x[1])
			{
				double temp = this->range_query_lo.x[1];
				this->range_query_lo.x[1] = this->range_query_hi.x[1];
				this->range_query_hi.x[1] = temp;
			}
			this->calc_range_query_result(this->kd_tree->root, Box<2>{ this->range_query_lo, this->range_query_hi }, 1);
			this->range_query_available = true;
			this->range_query_started = false;
		}
		else
		{
			Point<2> point{ static_cast<double>(mouse_button_event.x), static_cast<double>(mouse_button_event.y) };
			this->convert_point(Point_conversion::SDL_TO_CARTESIAN, point);
			this->range_query_lo = point;
			this->range_query_res.clear();
			this->range_query_started = true;
			this->range_query_available = false;
		}
	}
}

void Window::render() const
{
	SDL_SetRenderDrawColor(this->renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
	SDL_RenderClear(this->renderer);

	this->draw_coordinate_system();
	this->draw_points(this->points, RGB_color{ 0, 102, 0 });
	this->draw_kd_tree();
	this->draw_bounding_box();
	this->draw_points(this->range_query_res, RGB_color{ 179, 0, 0 });
	SDL_RenderPresent(this->renderer);
}

void Window::draw_coordinate_system() const
{
	SDL_SetRenderDrawColor(this->renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
	SDL_RenderDrawLine(this->renderer, 0, (this->height / 2) + 1, this->width, (this->height / 2) + 1);
	SDL_RenderDrawLine(this->renderer, (this->width / 2) + 1, 0, (this->width / 2) + 1, this->height);
}

void Window::draw_points(const std::vector<Point<2>>& points, RGB_color color) const
{
	SDL_SetRenderDrawColor(this->renderer, color.r, color.g, color.b, SDL_ALPHA_OPAQUE);
	for (auto point : points)
	{
		this->convert_point(Point_conversion::CARTESIAN_TO_SDL, point);
		SDL_Point p = this->point_to_sdl_point(point);
		SDL_Rect rect;
		rect.x = p.x - 1;
		rect.y = p.y - 1;
		rect.h = 3;
		rect.w = 3;
		SDL_RenderDrawRect(this->renderer, &rect);
	}
}

void Window::draw_kd_tree() const
{
	if (this->kd_tree == nullptr) return;
	this->draw_box(this->kd_tree->root);
}

void Window::draw_box(const std::shared_ptr<Box_node<2>>& box_node) const
{
	if (box_node == nullptr) return;
	Point<2> lo = box_node->lo;
	Point<2> hi = box_node->hi;
	this->convert_point(Point_conversion::CARTESIAN_TO_SDL, lo);
	this->convert_point(Point_conversion::CARTESIAN_TO_SDL, hi);
	SDL_SetRenderDrawColor(this->renderer, 0, 0, 255, SDL_ALPHA_OPAQUE);
	SDL_RenderDrawLine(this->renderer, lo.x[0], lo.x[1], hi.x[0], lo.x[1]);
	SDL_RenderDrawLine(this->renderer, hi.x[0], lo.x[1], hi.x[0], hi.x[1]);
	SDL_RenderDrawLine(this->renderer, hi.x[0], hi.x[1], lo.x[0], hi.x[1]);
	SDL_RenderDrawLine(this->renderer, lo.x[0], hi.x[1], lo.x[0], lo.x[1]);
	if (box_node->left_child != nullptr) this->draw_box(box_node->left_child);
	if (box_node->right_child != nullptr) this->draw_box(box_node->right_child);
}

void Window::calc_kd_tree()
{
	this->kd_tree = std::unique_ptr<KD_tree<2>>{ new KD_tree<2>{this->bounding_box, this->points} };
}

void Window::calc_bounding_box()
{
	Point<2> lo{ 0.0, static_cast<double>(this->height - 1) };
	Point<2> hi{ static_cast<double>(this->width - 1), 0.0 };
	this->convert_point(Point_conversion::SDL_TO_CARTESIAN, lo);
	this->convert_point(Point_conversion::SDL_TO_CARTESIAN, hi);
	Box<2> bounding_box{ lo, hi };
	this->bounding_box = bounding_box;
}

void Window::draw_bounding_box() const
{
	if (!this->range_query_available) return;
	Point<2> lo = this->range_query_lo;
	Point<2> hi = this->range_query_hi;
	this->convert_point(Point_conversion::CARTESIAN_TO_SDL, lo);
	this->convert_point(Point_conversion::CARTESIAN_TO_SDL, hi);
	SDL_SetRenderDrawColor(this->renderer, 179, 0, 0, SDL_ALPHA_OPAQUE);
	SDL_RenderDrawLine(this->renderer, lo.x[0], lo.x[1], hi.x[0], lo.x[1]);
	SDL_RenderDrawLine(this->renderer, hi.x[0], lo.x[1], hi.x[0], hi.x[1]);
	SDL_RenderDrawLine(this->renderer, hi.x[0], hi.x[1], lo.x[0], hi.x[1]);
	SDL_RenderDrawLine(this->renderer, lo.x[0], hi.x[1], lo.x[0], lo.x[1]);
}

void Window::calc_range_query_result(const std::shared_ptr<Box_node<2>>& box_node, Box<2> range, int dim)
{
	if (box_node == nullptr) return;
	if (!distance<2>(range, box_node->point)) this->range_query_res.push_back(box_node->point);
	if (range.lo.x[dim] < box_node->point.x[dim]) this->calc_range_query_result(box_node->left_child, range, (dim + 1) % 2);
	if (range.hi.x[dim] > box_node->point.x[dim]) this->calc_range_query_result(box_node->right_child, range, (dim + 1) % 2);
}

void Window::convert_point(Point_conversion conversion, Point<2>& point) const
{
	switch (conversion)
	{
	case Point_conversion::SDL_TO_CARTESIAN:
		point.x[0] = (point.x[0] - (this->width / 2 + 1)) / Window::scale;
		point.x[1] = (point.x[1] - (this->height / 2 + 1)) / -Window::scale;
		break;
	case Point_conversion::CARTESIAN_TO_SDL:
		point.x[0] = (this->width / 2 + 1) + point.x[0] * Window::scale;
		point.x[1] = (this->height / 2 + 1) - point.x[1] * Window::scale;
		break;
	}
}

SDL_Point Window::point_to_sdl_point(Point<2> point) const
{
	SDL_Point sdl_point;
	sdl_point.x = point.x[0];
	sdl_point.y = point.x[1];
	return sdl_point;
}

Window::~Window()
{
	SDL_DestroyRenderer(this->renderer);
	SDL_DestroyWindow(this->window);
	SDL_Quit();
}
