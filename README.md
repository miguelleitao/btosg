# btosg
A thin abstraction layer to integrate Bullet and OpenSceneGraph.

### Description
btosg is aimed to ease building simple visual simulation applications integrating Bullet and OpenSceneGraph.
btosg stands on top of these two APIs but does not try to hide them. Instead, in order to build a complete application, the programmer usually needs to access directly the data structures from both Bullet and OpenSceneGraph. So, the use os btosg does not avoid the requirement to know and understand their APIs.

The use of btosg can help the programming task because it allows to create and position both the physical and graphics definitions of small objects in one step. It also keeps track of syncronizing the graphics definitions from the state of each physical object.

### Dependences
#### OpenSceneGraph: 
* https://github.com/openscenegraph/OpenSceneGraph 
* http://www.openscenegraph.org/
#### Bullet:
* https://github.com/bulletphysics/bullet3 
* http://bulletphysics.org/
