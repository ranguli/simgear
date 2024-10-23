# SimGear

SimGear is a set of open-source libraries that provide the base for the FlightGear Flight Simulator.

## Building

For more a more in-depth explanation about building SimGear, please consult the [FlightGear Wiki](https://wiki.flightgear.org), the FlightGear forums, or the FlightGear developer mailing list.

### Dependencies
SimGear utilizes a number of open-source libraries - these must be present on your system in order to build SimGear. A non-exhaustive list of
dependencies include:

* OpenSceneGraph (OSG)
* zlib
* Open Audio Library (OpenAL), or optionally AeonWave

### Compiling
First, ensure that you have `git` installed, and obtain a copy of SimGear using `git clone`:

```
git clone git@gitlab.com:flightgear/simgear
```

Then enter the newly cloned directory:
```
cd simgear
```

In order to compile SimGear, you will need the CMake build system. The process for building SimGear follows the same conventions as building any other CMake-based project:

```
mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make
sudo make install
```

## License

This project is distributed under the GNU Lesser General Public License (LGPL) v2.1. See [LICENSE.md](LICENSE.md) for more information.

## Acknowledgements

Please see [CONTRIBUTORS.md](CONTRIBUTORS.md) for an (incomplete) overview of the large number of volunteers who have donated their time and expertise to this project.