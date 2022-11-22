#include <iostream>
using namespace std;

#ifndef RTAC_SIMULATION_TESTS_CONFIG_FILE
#define RTAC_SIMULATION_TESTS_CONFIG_FILE "None"
#endif //RTAC_SIMULATION_TESTS_CONFIG_FILE


int main()
{
    cout << RTAC_SIMULATION_TESTS_CONFIG_FILE << endl;
    return 0;
}
