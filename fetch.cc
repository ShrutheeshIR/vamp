#include <vamp_python_init.hh>

#include <vamp/bindings/robot_helper.hh>
#include <vamp/robots/fetch.hh>

void vamp::binding::init_fetch(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Fetch>(pymodule);
}
