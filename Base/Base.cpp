#include <iostream>

int main()
{
	std::cout << 0x00001 << std::endl;
	std::cout << (0x00002) << std::endl;
	std::cout << (0x00001 << 1) << std::endl;
	std::cout << ((0x00001 << 1) & 0x00001) << std::endl;
	std::cout << ((0x00001 << 1) & 0x00002) << std::endl;
	std::cout << ((0x00001 << 1) & 0x00003) << std::endl;
	return 0;
}
