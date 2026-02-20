Import("env")

env.Append(CMAKE_EXTRA_ARGS=[
    "-DBUILD_TESTING=OFF"
])
