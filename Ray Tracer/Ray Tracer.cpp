#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <thread>
#include <mutex>
#include <memory>

const float PI = 3.14159265358979323846f;

class Vector3 {
public:
    float x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(float s) const { return Vector3(x * s, y * s, z * s); }
    Vector3 operator/(float s) const { return Vector3(x / s, y / s, z / s); }
    Vector3 operator*(const Vector3& v) const { return Vector3(x * v.x, y * v.y, z * v.z); }

    float dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vector3 cross(const Vector3& v) const {
        return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vector3 normalized() const { return *this / length(); }

    Vector3 reflect(const Vector3& normal) const {
        return *this - normal * 2.0f * this->dot(normal);
    }
};

class Ray {
public:
    Vector3 origin;
    Vector3 direction;

    Ray(const Vector3& origin, const Vector3& direction)
        : origin(origin), direction(direction.normalized()) {
    }
};

class Material {
public:
    Vector3 color;
    float ambient;
    float diffuse;
    float specular;
    float shininess;
    float reflectivity;

    Material() : color(Vector3(1, 1, 1)), ambient(0.1f), diffuse(0.9f),
        specular(0.3f), shininess(50.0f), reflectivity(0.0f) {
    }

    Material(const Vector3& color, float ambient, float diffuse, float specular,
        float shininess, float reflectivity)
        : color(color), ambient(ambient), diffuse(diffuse), specular(specular),
        shininess(shininess), reflectivity(reflectivity) {
    }
};

class Sphere {
public:
    Vector3 center;
    float radius;
    Material material;

    Sphere(const Vector3& center, float radius, const Material& material)
        : center(center), radius(radius), material(material) {
    }

    bool intersect(const Ray& ray, float& t) const {
        Vector3 oc = ray.origin - center;
        float a = ray.direction.dot(ray.direction);
        float b = 2.0f * oc.dot(ray.direction);
        float c = oc.dot(oc) - radius * radius;
        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) return false;

        float sqrtd = std::sqrt(discriminant);
        float t0 = (-b - sqrtd) / (2 * a);
        float t1 = (-b + sqrtd) / (2 * a);

        t = (t0 > 0.1f) ? t0 : t1;
        return t > 0.1f;
    }
};

class Light {
public:
    Vector3 position;
    Vector3 color;
    float intensity;

    Light(const Vector3& position, const Vector3& color, float intensity)
        : position(position), color(color), intensity(intensity) {
    }
};

class Scene {
public:
    std::vector<Sphere> spheres;
    std::vector<Light> lights;
    Vector3 skyTop;
    Vector3 skyBottom;
    Vector3 groundColor;
    Material groundMaterial;

    Scene() : skyTop(Vector3(0.6f, 0.8f, 1.0f)),  // Gradient sky color (top)
        skyBottom(Vector3(0.1f, 0.2f, 0.4f)),    // Gradient sky color (bottom)
        groundColor(Vector3(0.0f, 0.1f, 0.3f)),  // Dark blue ground
        groundMaterial(groundColor, 0.2f, 0.8f, 0.5f, 50.0f, 0.3f) {
    }

    void addSphere(const Sphere& sphere) { spheres.push_back(sphere); }
    void addLight(const Light& light) { lights.push_back(light); }
};

bool isShadowed(const Scene& scene, const Vector3& point, const Vector3& lightDir, float lightDistance) {
    Ray shadowRay(point + lightDir * 0.001f, lightDir);

    for (const auto& sphere : scene.spheres) {
        float t;
        if (sphere.intersect(shadowRay, t) && t < lightDistance) {
            return true;
        }
    }
    return false;
}

Vector3 calculateLighting(const Scene& scene, const Vector3& point, const Vector3& normal,
    const Vector3& viewDir, const Material& material) {
    Vector3 result = material.color * material.ambient;

    for (const auto& light : scene.lights) {
        Vector3 lightDir = (light.position - point).normalized();
        float lightDistance = (light.position - point).length();

        // Shadow check
        if (isShadowed(scene, point, lightDir, lightDistance)) {
            continue;
        }

        // Diffuse component
        float diff = std::max(normal.dot(lightDir), 0.0f);
        Vector3 diffuse = light.color * light.intensity * diff * material.diffuse;

        // Specular component
        Vector3 reflectDir = lightDir.reflect(normal);
        float spec = std::pow(std::max(viewDir.dot(reflectDir), 0.0f), material.shininess);
        Vector3 specular = light.color * light.intensity * spec * material.specular;

        // Attenuation
        float attenuation = 1.0f / (1.0f + 0.1f * lightDistance + 0.01f * lightDistance * lightDistance);

        result = result + (diffuse + specular) * material.color * attenuation;
    }

    return result;
}

Vector3 traceRay(const Scene& scene, const Ray& ray, int depth) {
    if (depth <= 0) return scene.skyBottom;  //gradient background color

    // Find closest intersection
    float closestT = std::numeric_limits<float>::max();
    const Sphere* closestSphere = nullptr;
    bool isGround = false;

    for (const auto& sphere : scene.spheres) {
        float t;
        if (sphere.intersect(ray, t) && t < closestT) {
            closestT = t;
            closestSphere = &sphere;
        }
    }

    // Check for ground plane intersection
    if (ray.direction.y < 0) {
        float t = (-1 - ray.origin.y) / ray.direction.y;
        if (t < closestT && t > 0.1f) {
            closestT = t;
            isGround = true;
        }
    }

    if (closestT == std::numeric_limits<float>::max()) return scene.skyBottom;

    Vector3 point = ray.origin + ray.direction * closestT;
    Vector3 normal;
    Material material;

    if (isGround) {
        normal = Vector3(0, 1, 0);
        material = scene.groundMaterial;
    }
    else if (closestSphere != nullptr) {  // Check if closestSphere is valid
        normal = (point - closestSphere->center).normalized();
        material = closestSphere->material;
    }
    else {
        return scene.skyBottom;  // No sphere hit, return sky bottom color
    }

    Vector3 viewDir = (ray.origin - point).normalized();
    Vector3 color = calculateLighting(scene, point, normal, viewDir, material);

    return color;
}

void renderSection(const Scene& scene, int width, int height, int startY, int endY,
    float invWidth, float invHeight, float aspectRatio, float fov,
    std::vector<Vector3>& image, std::mutex& imageMutex) {
    for (int y = startY; y < endY; ++y) {
        for (int x = 0; x < width; ++x) {
            Vector3 color(0, 0, 0);
            // Anti-aliasing: 4x supersampling
            for (int sy = 0; sy < 2; ++sy) {
                for (int sx = 0; sx < 2; ++sx) {
                    float xx = (2 * ((x + (sx + 0.5f)) * invWidth) - 1) * aspectRatio * fov;
                    float yy = (1 - 2 * ((y + (sy + 0.5f)) * invHeight)) * fov;
                    Vector3 rayDir(xx, yy, -1);
                    Ray ray(Vector3(0, 0, 0), rayDir.normalized());
                    color = color + traceRay(scene, ray, 5);
                }
            }
            color = color / 4.0f;

            // Clamping to [0, 1]
            color.x = std::min(1.0f, std::max(0.0f, color.x));
            color.y = std::min(1.0f, std::max(0.0f, color.y));
            color.z = std::min(1.0f, std::max(0.0f, color.z));

            std::lock_guard<std::mutex> lock(imageMutex);
            image[y * width + x] = color;
        }
    }
}

void saveImage(const std::vector<Vector3>& image, int width, int height, const std::string& filename) {
    std::ofstream ofs(filename, std::ios::out | std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";

    for (const auto& pixel : image) {
        unsigned char r = static_cast<unsigned char>(pixel.x * 255);
        unsigned char g = static_cast<unsigned char>(pixel.y * 255);
        unsigned char b = static_cast<unsigned char>(pixel.z * 255);
        ofs << r << g << b;
    }

    ofs.close();
}

int main() {
    const int width = 800;
    const int height = 600;
    const float aspectRatio = float(width) / float(height);
    const float fov = tan(PI / 4);
    const float invWidth = 1.0f / width;
    const float invHeight = 1.0f / height;

    Scene scene;

    // Red, Blue, Green spheres
    scene.addSphere(Sphere(Vector3(-3.5f, 0, -7), 1, Material(Vector3(1, 0, 0), 0.1f, 0.7f, 0.5f, 32, 0)));
    scene.addSphere(Sphere(Vector3(0, 0, -7), 1, Material(Vector3(0, 0, 1), 0.1f, 0.7f, 0.5f, 32, 0)));
    scene.addSphere(Sphere(Vector3(3.5f, 0, -7), 1, Material(Vector3(0, 1, 0), 0.1f, 0.7f, 0.5f, 32, 0)));

    // Lights
    scene.addLight(Light(Vector3(5, 10, 2), Vector3(1, 1, 1), 1.5f));
    scene.addLight(Light(Vector3(-5, 10, 2), Vector3(1, 1, 1), 1.5f));

    std::vector<Vector3> image(width * height);
    std::mutex imageMutex;

    unsigned int numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0) numThreads = 4;

    std::vector<std::thread> threads;
    int rowsPerThread = height / numThreads;

    for (unsigned int i = 0; i < numThreads; ++i) {
        int startY = i * rowsPerThread;
        int endY = (i == numThreads - 1) ? height : startY + rowsPerThread;

        threads.emplace_back(renderSection, std::ref(scene), width, height,
            startY, endY, invWidth, invHeight, aspectRatio, fov,
            std::ref(image), std::ref(imageMutex));
    }

    for (auto& thread : threads) {
        thread.join();
    }

    saveImage(image, width, height, "output.ppm");

    std::cout << "Rendering complete. To view the rendered scene check the project's directory for the .PPM file." << std::endl;
    return 0;
}
