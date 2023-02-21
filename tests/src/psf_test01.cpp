#include <iostream>
using namespace std;

#include <rtac_simulation/factories/PSFGenerator.h>
using namespace rtac::simulation;


void print(std::ostream& os, const PSFGenerator_Real::Ptr& generator)
{
    if(!generator) {
        os << "Null generator !" << std::endl;
        return;
    }
    os << "Real generator (size " << generator->size() << ")\n-";
    for(unsigned int i = 0; i < generator->size(); i++) {
        os << ' ' << (*generator)[i];
    }
    os << std::endl << std::endl;
}

void print(std::ostream& os, const PSFGenerator_Complex::Ptr& generator)
{
    if(!generator) {
        os << "Null generator !" << std::endl;
        return;
    }
    os << "Complex generator (size " << generator->size() << ")\n-";
    for(unsigned int i = 0; i < generator->size(); i++) {
        os << ' ' << (*generator)[i];
    }
    os << std::endl << std::endl;
}

int main()
{
    auto squarePsf = RangePSF_Square::Create(10.0);
    print(cout, squarePsf);

    auto sinPsf = RangePSF_Sine::Create(1500.0, 1.2e6, 0.03);
    print(cout, sinPsf);

    auto csinPsf = RangePSF_ComplexSine::Create(1500.0, 1.2e6, 0.03);
    print(cout, csinPsf);

    auto bsincPsf = BearingPSF_Sinc::Create(130.0f, 0.6f);
    print(cout, bsincPsf);

    auto bgaussPsf = BearingPSF_Gauss::Create(10*0.6f, 0.6f);
    print(cout, bgaussPsf);

    return 0;
}
