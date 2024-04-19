#include <iostream>
#include <fstream>
#include <string>
#include "dynamic_window_algorithm.h"
int main()
{
    std::shared_ptr<DynWinAlgorithm> dwa = std::make_shared<DynWinAlgorithm>();
    dwa->Excute();
    return 1;
}
