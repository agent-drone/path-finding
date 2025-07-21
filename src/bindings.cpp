/*
DATE
  21.07.25

DESCRIPTION
  This file serves as the common "binding" point
  for all our custom classes which utilize the
  `Octomap` and `OMPL` libraries. They are all
  earmarked under a "pathfinder" module.
*/

#include "../include/common.h"
#include "../include/woctomap.h"
#include "../include/wompl.h"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <vector>
#include <array>

namespace py = pybind11;

PYBIND11_MODULE(pathfinder, m) {
  py::class_<Woctomap, std::shared_ptr<Woctomap>>(m, "Octomap")
      .def(py::init<float>(), py::arg("size") = 0.1f)
      .def(py::init<const char *>(), py::arg("file"))
      .def("read_from_text", &Woctomap::read_from_text)
      .def("read_from_bt", &Woctomap::read_from_bt)
      .def("add_point", &Woctomap::add_point)
      .def("save_to_bt", &Woctomap::save_to_bt)
      .def("free", &Woctomap::free);

  py::class_<Wompl, std::shared_ptr<Wompl>>(m, "OMPL")
      .def(py::init<std::shared_ptr<Woctomap>>())
      .def("set_start", &Wompl::set_start)
      .def("set_goal", &Wompl::set_goal)
      .def("criterion", &Wompl::criterion)
      .def("solve", &Wompl::solve)
      .def("get_computed_waypoints", &Wompl::get_computed_waypoints);
}
