\mainpage btosg
A programming framework to build visual physical simulations.

### Description
**btosg** is aimed to ease building simple visual simulation applications integrating **Bullet** and **OpenSceneGraph**.
**btosg** stands on top of these two APIs but does not try to hide them.
Instead, in order to build a complete application, the programmer is able and usually needs to access directly the data structures from both **Bullet** and **OpenSceneGraph**. So, the use of **btosg** does not avoid the requirement to know and understand their APIs.

The use of **btosg** can help the programming task because it allows to create and position both the physical and graphics definitions of small objects in one step. It also keeps track of syncronizing the graphics definitions from the state of each physical object.

### Dependences
#### OpenSceneGraph:
* https://github.com/openscenegraph/OpenSceneGraph
* http://www.openscenegraph.org/

#### Bullet:
* https://github.com/bulletphysics/bullet3
* http://bulletphysics.org/

Dependences can be installed on Fedora Linux using:

    dnf install gcc-c++ mesa-libGL-devel
    dnf install bullet bullet-devel OpenSceneGraph OpenSceneGraph-devel

Dependences can be installed on Ubuntu using:

    apt install bullet openscenegraph-osg openscenegraph-osgViewer openscenegraph-osgSim openscenegraph-osgDB openscenegraph-osgGA openscenegraph-osgShadow

### Build

    git clone https://github.com/miguelleitao/btosg.git
    cd btosg
    make
    sudo make install

### Usage
Look at provided examples. Prepare your _application.cpp_ using

    #include <btosg.h>

Compile using
    
<pre>
g++ -c -I path/to/btosg/dir <i>application.cpp</i>
g++ -o <i>application</i> -L path/to/btosg/dir/lib -l btosg <i>application.o</i>
</pre>

Or, if you completed installation using `make install`:

<pre>
g++ -c `pkg-config --cflags btosg` <i>application.cpp</i>
g++ -o <i>application</i> `pkg-config --libs btosg` <i>application.o</i>
</pre>

### Examples
**btosg** is available with some working examples.
* **ball.cpp** implements a simple simulation of a ball with two planes.
* **objects.cpp** provides an example for creating a complete object (graphical and physical) from loading an external Wavefront OBJ file.
* **car.cpp** implements a basic vehicle with four wheels and suspensions. It can be compiled using a Z or Y pointing up vector.
Usage instructions are provided in source file.

To compile and try the provided examples do:

    make examples
    ./ball
    ./objects
    ./carZ
    ./carY

### Referencing
The release version can be referenced by either http://doi.org/ctz5 or doi:10.5281/zenodo.1283484.

