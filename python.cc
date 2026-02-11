#include <vamp_python_init.hh>

#include <vector>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

namespace vb = vamp::binding;

NB_MODULE(_core_ext, pymodule)
{
    vb::init_settings(pymodule);
    vb::init_environment(pymodule);

        vb::init_sphere(pymodule);
    vb::init_ur5(pymodule);
    vb::init_panda(pymodule);
    vb::init_fetch(pymodule);
    vb::init_baxter(pymodule);
    vb::init_bimanualpanda(pymodule);


    pymodule.def(
        "robots",
        []() -> std::vector<std::string>
        {
            return
            {
                "sphere","ur5","panda","fetch","baxter","bimanualpanda",
            };
        });
}
