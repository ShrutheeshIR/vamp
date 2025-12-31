#include <vamp_python_init.hh>

#include <vamp/bindings/robot_helper.hh>
#include <vamp/robots/sphere.hh>

void vamp::binding::init_sphere(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Sphere>(pymodule);
}
