Import("env")
import os

lib_dir = os.path.join(env["PROJECT_DIR"], "lib", "micro_ros_arduino", "src", "esp32")
env.Append(
    LIBPATH=[lib_dir],
    LIBS=["microros"],
)
