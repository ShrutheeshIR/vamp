#include <vamp_python_init.hh>

#include <vamp/bindings/robot_helper.hh>
#include <vamp/robots/ur5.hh>

void vamp::binding::init_ur5(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::UR5>(pymodule);
}
