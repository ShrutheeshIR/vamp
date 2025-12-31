#include <vamp_python_init.hh>

#include <vamp/bindings/robot_helper.hh>
#include <vamp/robots/baxter.hh>

void vamp::binding::init_baxter(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Baxter>(pymodule);
}
