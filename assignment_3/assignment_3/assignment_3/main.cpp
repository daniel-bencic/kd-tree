#include "window.hpp"

int main(int argc, char *args[])
{
	Window window{ 801, 801, 50 };
	while (window.initialized)
	{
		window.handle_events();
		window.render();
	}

	return 0;
}
