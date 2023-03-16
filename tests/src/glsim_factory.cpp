#include <iostream>
using namespace std;

#include <rtac_simulation/EmitterGL.h>
#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/EmitterFactory.h>
#include <rtac_simulation/SimulationGL.h>
using namespace rtac::simulation;

int main()
{
    auto emitterConf = FileFinder::Get()->find_one("oculus_M1200d_1_emitter_gl.yaml");
    auto emitter = EmitterFactory::MakeEmitterGL(YAML::LoadFile(emitterConf));
    
    cout << *emitter << endl;
    cout << *std::dynamic_pointer_cast<EmitterGL>(emitter) << endl;

    auto sim = SimulationGL::Create("oculus_M1200d_1_emitter_gl.yaml",
                                    "oculus_M1200d_1_receiver.yaml");
    getchar();

    return 0;
}
