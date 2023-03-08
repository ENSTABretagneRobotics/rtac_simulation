#include <iostream>
using namespace std;

#include <rtac_simulation/Waveform.h>
using namespace rtac::simulation;

int main()
{
    auto waveform = Waveform_Sine::Create(3.0, 1.0, 2);
    std::cout << *waveform << std::endl;

    return 0;
}

