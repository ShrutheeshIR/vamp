#include <vamp_python_init.hh>

#include <vamp/bindings/robot_helper.hh>
#include <vamp/robots/bimanualpanda.hh>

void vamp::binding::init_bimanualpanda(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::BimanualPanda>(pymodule);
}
