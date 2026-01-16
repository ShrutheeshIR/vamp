#include <vamp_python_init.hh>

#include <vamp/bindings/robot_helper.hh>
#include <vamp/robots/panda.hh>

void vamp::binding::init_panda(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Panda>(pymodule);
}
