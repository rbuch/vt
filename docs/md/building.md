\page vt-build How to Build
\brief Building DARMA/vt with cmake

\vt can be built with `cmake` or built inside a `docker` container. Most of the
external dependencies come bundled with \vt for ease of compiling.

\section how-to-build Building

To build \vt, one must obtain the following dependencies:

\subsection required-deps Required
  - detector,   (*vt* ecosystem)
  - checkpoint, (*vt* ecosystem)
  - MPI         (mpich/openmpi/mvapich/IBM Spectrum MPI/Cray MPICH/etc.)

\subsection optional-deps Optional (if threading enabled)

  - OpenMP       _or_
  - Default to `std::thread`

\subsection automatic-build-deps Automatically build dependencies

Assuming MPI is installed and accessible via CC/CXX, the only other dependencies
that are required are checkpoint and detector. The easiest way to get these
built are to clone them inside `vt/lib`:

```bash
$ git clone git@github.com:DARMA-tasking/vt
$ cd vt/lib
$ git clone git@github.com:DARMA-tasking/checkpoint
$ git clone git@github.com:DARMA-tasking/detector
```

With these in `vt/lib`, cmake will automatically build them and stitch them into
*vt*'s linking process.

\subsection use-cmake-directly-vars Using cmake directly

One may use `cmake` as normal on *vt*, with checkpoint and detector cloned in
`vt/lib` to compile them all together as explained above. The following are some
custom configuration build options that can be provided to `cmake` to change the
build configuration:

| CMake Variable                  | Default Value   | Description |
| ------------------              | --------------- | ----------- |
| `vt_lb_enabled`                 | 0               | Compile with support for runtime load balancing |
| `vt_trace_enabled`              | 0               | Compile with support for runtime tracing (Projections-format) |
| `vt_test_trace_runtime_enabled` | 0               | Force tracing on at runtime for VT tests |
| `vt_doxygen_enabled`            | 0               | Enable doxygen generation |
| `vt_mimalloc_enabled`           | 0               | Enable `mimalloc`, alternative allocator for debugging memory usage/frees/corruption |
| `vt_asan_enabled`               | 0               | Enable building with address sanitizer |
| `vt_pool_enabled`               | 1               | Use memory pool in *vt* for message allocation |
| `vt_zoltan_enabled`             | 0               | Build with Zoltan enabled for `ZoltanLB` support |
| `vt_mpi_guards`                 | 0               | Guards against mis-use of MPI calls in code using *vt* |
| `vt_fcontext_enabled`           | 0               | Enable user-level threads through boost fcontext |
| `vt_priorities_enabled`         | 1               | Enable prioritization of work (adds bits in envelope) |
| `vt_priority_bits_per_level`    | 3               | Number of bits per level of priority in envelope |
| `CODE_COVERAGE`                 | 0               | Enable code coverage for VT examples/tests |
| `VT_BUILD_TESTS`                | 1               | Build all VT tests |
| `VT_BUILD_EXAMPLES`             | 1               | Build all VT examples |


\subsection using-the-build-script Using the Build Script

Instead of running `cmake`, one may invoke the `vt/ci/build_cpp.sh` script which
will run `cmake` for *vt* with environment variables for most configuration
parameters.

\subsubsection building-environment-variables Build Script Environment Variables

| Variable                    | Default Value   | Description |
| ------------------          | --------------- | ----------- |
| `CMAKE_BUILD_TYPE`          | Release         | The `cmake` build type |
| `VT_LB_ENABLED`             | ON              | Compile with support for runtime load balancing |
| `VT_TRACE_ENABLED `         | OFF             | Compile with support for runtime tracing (Projections-format) |
| `VT_TRACE_RUNTIME_ENABLED ` | OFF             | Force tracing on at runtime (used in CI for automatically testing tracing on all tests/examples) |
| `VT_DOXYGEN_ENABLED `       | OFF             | Enable doxygen generation |
| `VT_MIMALLOC_ENABLED `      | OFF             | Enable `mimalloc`, alternative allocator for debugging memory usage/frees/corruption |
| `VT_ASAN_ENABLED `          | OFF             | Enable building with address sanitizer |
| `VT_POOL_ENABLED `          | ON              | Use memory pool in *vt* for message allocation |
| `VT_ZOLTAN_ENABLED `        | OFF             | Build with Zoltan enabled for `ZoltanLB` support |
| `ZOLTAN_DIR `               | (empty)         | Directory pointing to Zoltan installation |
| `VT_MPI_GUARD_ENABLED `     | OFF             | Guards against mis-use of MPI calls in code using *vt* |

With these set, invoke the script with two arguments: the path to the *vt* root
directory and the build path. Here's an example assuming that *vt* is cloned
into `/usr/src/vt` with trace enabled in debug mode.

**Usage for building:**

```bash
$ vt/ci/build_cpp.sh <full-path-to-vt-source> <full-path-to-build-dir>
```

**Example:**

```bash
$ cd /usr/src
$ git clone git@github.com:DARMA-tasking/vt
$ VT_TRACE_ENABLED=1 CMAKE_BUILD_TYPE=Debug /usr/src/vt/ci/build_cpp.sh /usr/src/vt /usr/build/vt
```

\subsection docker-build Building with `docker` containerization

The easiest way to build *vt* is by using `docker` with the available containers
that contain the proper compilers, MPI, and all other dependencies. First,
install `docker` on the system. On some systems, `docker-compose` might also
need to be installed.

The `docker` builds are configured through `docker-compose` to use a shared,
cached filesystem mount with the host for `ccache` to enable fast re-builds.

For `docker-compose`, the following variables can be set to configure the
build. One may configure the architecture, compiler type and version, Linux
distro (ubuntu or alpine), and distro version.

The default set of the docker configuration options is located in `vt/.env`,
which `docker-compose` will read.

```
# Variables:
#   ARCH={amd64, arm64v8, ...}
#   COMPILER_TYPE={gnu, clang}
#   COMPILER={gcc-5, gcc-6, gcc-7, gcc-8, gcc-9, gcc-10,
#             clang-3.9, clang-4.0, clang-5.0, clang-6.0, clang-7, clang-8,
#             clang-9, clang-10}
#   REPO=lifflander1/vt
#   UBUNTU={18.04, 20.04}
#   ULIMIT_CORE=0
#
# DARMA/vt Configuration Variables:
#   VT_LB=1              # Enable load balancing
#   VT_TRACE=0           # Enable tracing
#   VT_MIMALLOC=0        # Enable mimalloc memory allocator
#   VT_DOCS=0            # Enable doxygen build
#   VT_TRACE_RT=0        # Enable tracing at runtime (for testing)
#   VT_ASAN=0            # Enable address sanitizer
#   BUILD_TYPE=release   # CMake build type
```

With these set, run the following for a non-interactive build with ubuntu:

```bash
$ cd vt
$ docker-compose run -e BUILD_TYPE=debug -e VT_TRACE=1 ubuntu-cpp
```

Or, alternatively, run a non-interactive build with alpine:

```bash
$ cd vt
$ docker-compose run -e BUILD_TYPE=debug -e VT_TRACE=1 alpine-cpp
```

For an interactive build with ubuntu, where one can build, debug, and run
`valgrind`, etc:

```bash
$ cd vt
$ docker-compose run -e BUILD_TYPE=debug -e VT_TRACE=1 ubuntu-cpp-interactive
# /vt/ci/build_cpp.sh /vt /build
# /vt/ci/test_cpp.sh /vt /build
```

The same call applies to alpine distro builds if you swap
`ubuntu-cpp-interactive` for `alpine-cpp-interactive`.

For more detailed information on configuring the docker build, read the
documentation in `vt/docker-compose.yml`.

\section test-vt Testing

After *vt* is built succesfully, one may invoke the tests several ways. One may
run `make test` or `ninja test` (depending on the generator used) or `ctest`, to
run all the tests. Alternatively, the tests can be run automatically from the CI
script:

```bash
$ vt/ci/test_cpp.sh <full-path-to-vt-source> <full-path-to-build-dir>
```