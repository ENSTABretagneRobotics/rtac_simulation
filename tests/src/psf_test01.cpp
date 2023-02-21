#include <iostream>
using namespace std;

#include <rtac_base/cuda/vector_utils.h>
using namespace rtac::cuda;

#include <rtac_display/Display.h>
#include <rtac_display/renderers/ImageRenderer.h>
using namespace rtac::display;

#include <rtac_simulation/factories/PSFGenerator.h>
#include <rtac_simulation/PointSpreadFunction.h>
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
    auto squarePsf = RangePSF_Square::Create(0.03);
    print(cout, squarePsf);

    auto sinPsf = RangePSF_Sine::Create(0.03, 1500.0, 1.2e6);
    print(cout, sinPsf);

    auto csinPsf = RangePSF_ComplexSine::Create(0.03, 1500.0, 1.2e6);
    print(cout, csinPsf);

    auto bsincPsf = BearingPSF_Sinc::Create(130.0f, 0.6f);
    print(cout, bsincPsf);

    auto bgaussPsf = BearingPSF_Gauss::Create(10*0.6f, 0.6f);
    print(cout, bgaussPsf);

    auto psf0 = PSF2D_Real::Create(bsincPsf, sinPsf);
    cout << *psf0 << endl;
    auto psf1 = PSF2D_Complex::Create(bsincPsf, csinPsf);
    cout << *psf1 << endl;

    Display display0;
    auto renderer0 = display0.create_renderer<ImageRenderer>(View::Create());
    renderer0->enable_colormap();
    renderer0->texture()->set_image({psf0->width(), psf0->height()},
        GLVector<float>(rescale(psf0->real_cast()->render().container(), 0.0f, 1.0f)));

    Display display1;
    auto renderer1 = display1.create_renderer<ImageRenderer>(View::Create());
    renderer1->enable_colormap();
    auto tmp = abs(psf1->complex_cast()->render().container());
    renderer1->texture()->set_image({psf1->width(), psf1->height()},
        GLVector<float>(rescale(tmp, 0.0f, 1.0f)));

    while(!display0.should_close() && !display1.should_close()) {
        display0.draw();
        display1.draw();
    }

    return 0;
}
