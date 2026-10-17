# demo: the viewer's built-in demo simulation (ARCH-06) - a few native aircraft flown by a small autopilot on an
# internal simulation thread. Demo composition only: it composes the native flight module directly, as the demo always
# has; the live/replay viewer draws its snapshots and owns no flight model. Headless: it needs no graphics.
add_library(fsim_demo_publisher STATIC
    DemoSimulation.h
    DemoSimulation.cpp
    DemoAutopilot.h)
add_library(fsim::demo ALIAS fsim_demo_publisher)
fsim_include_view(fsim_demo_publisher)
target_link_libraries(fsim_demo_publisher PUBLIC fsim::contracts PRIVATE fsim::sim fsim::io fsim::core fsim::platform)
fsim_set_warnings(fsim_demo_publisher)
