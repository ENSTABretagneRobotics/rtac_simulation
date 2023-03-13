#include <iostream>
using namespace std;

#include <rtac_simulation/factories/utilities.h>
using namespace rtac::simulation;

int main()
{
    auto finder = FileFinder::Get();
    cout << *finder << endl;

    auto paths = finder->find(".*.yaml");
    for(const auto& p : paths) {
        cout << "- " << p << endl;
    }

    return 0;
}


