# Basic C++ Ray Tracer

This project implements a basic ray tracing engine in C++ for rendering 3D scenes with realistic lighting, shading and spheres. The engine supports Phong shading, multithreaded rendering and anti-aliasing to improve image quality. The rendered images are saved in the PPM format.

## Features

* **Ray Tracing**: Calculates pixel colors by tracing rays through a scene and intersecting with objects.
* **Phong Shading**: Implements ambient, diffuse, and specular lighting for realistic shading.
* **Multithreading**: Uses C++ multithreading to parallelize the rendering process, optimizing performance on multi-core systems.
* **Anti-Aliasing**: Implements 4x Supersampling for anti-aliasing to reduce jagged edges.
* **PPM Output**: Saves the rendered scene as an image in the PPM format.


## Technologies Used

- C++


### Setup Instructions

1. Clone the repository.
2. Run the project using a suitable IDE.
3. A CLI will show up confirming the scene was rendered.
4. A .PPM file will be created in the project directory.
5. View the .PPM file using a compatible image viewer.
