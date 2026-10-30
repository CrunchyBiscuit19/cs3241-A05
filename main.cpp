// CS3241Lab5.cpp
#include <chrono>
#include <cmath>
#include <iostream>

#include "GL\glut.h"
#include "vector3D.h"

using namespace std;

#define WINWIDTH 600
#define WINHEIGHT 400
#define NUM_OBJECTS 4
#define MAX_RT_LEVEL 50
#define NUM_SCENE 2

float* pixelBuffer = new float[WINWIDTH * WINHEIGHT * 3];

class Ray {  // a ray that start with "start" and going in the direction "dir"
public:
    Vector3 start, dir;
};

class RtObject {
public:
    virtual double intersectWithRay(
        Ray, Vector3& pos, Vector3& normal
    ) = 0;  // return a -ve if there is no intersection. Otherwise, return the smallest postive value of t

    // Materials Properties
    double ambiantReflection[3];
    double diffusetReflection[3];
    double specularReflection[3];
    double speN = 300;
};

class Sphere : public RtObject {
    Vector3 center_;
    double r_;

public:
    Sphere(Vector3 c, double r) {
        center_ = c;
        r_ = r;
    };
    Sphere() {};
    void set(Vector3 c, double r) {
        center_ = c;
        r_ = r;
    };
    double intersectWithRay(Ray, Vector3& pos, Vector3& normal);
};

RtObject** objList;  // The list of all objects in the scene

// Global Variables
// Camera Settings
Vector3 cameraPos(0, 0, -500);

// assume the the following two vectors are normalised
Vector3 lookAtDir(0, 0, 1);
Vector3 upVector(0, 1, 0);
Vector3 leftVector(1, 0, 0);
float focalLen = 500;

// Light Settings

Vector3 lightPos(900, 1000, -1500);
double ambiantLight[3] = {0.4, 0.4, 0.4};
double diffusetLight[3] = {0.7, 0.7, 0.7};
double specularLight[3] = {0.5, 0.5, 0.5};

double bgColor[3] = {0.1, 0.1, 0.4};

int sceneNo = 0;

double Sphere::intersectWithRay(Ray r, Vector3& intersection, Vector3& normal)
// return a -ve if there is no intersection. Otherwise, return the smallest postive value of t
{
    // Step 1
    // solve |start + t*dir - center|^2 = r^2, i.e. a*t^2 + b*t + c = 0
    Vector3 oc = r.start - center_;
    double a = dot_prod(r.dir, r.dir);
    double b = 2 * dot_prod(r.dir, oc);
    double c = dot_prod(oc, oc) - r_ * r_;
    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) return -1;

    double sqrtD = sqrt(discriminant);
    double t = (-b - sqrtD) / (2 * a);
    if (t <= 0) t = (-b + sqrtD) / (2 * a);
    if (t <= 0) return -1;

    intersection = r.start + r.dir * t;
    normal = (intersection - center_) / r_;
    return t;
}

void rayTrace(Ray ray, double& r, double& g, double& b, int fromObj = -1, int level = 0) {
    // Step 4

    int goBackGround = 1, i = 0;

    Vector3 intersection, normal;
    Vector3 lightV;
    Vector3 viewV;
    Vector3 lightReflectionV;
    Vector3 rayReflectionV;

    Ray newRay;
    double mint = DBL_MAX, t;
    int hitObj = -1;
    Vector3 currIntersection, currNormal;

    for (i = 0; i < NUM_OBJECTS; i++) {
        if (i == fromObj)  // a ray never hits the object it rebounds from
            continue;

        if ((t = objList[i]->intersectWithRay(ray, currIntersection, currNormal)) > 0 && t < mint) {
            // Step 2: keep only the closest object along the ray
            mint = t;
            hitObj = i;
            intersection = currIntersection;
            normal = currNormal;
            goBackGround = 0;
        }
    }

    if (!goBackGround) {
        // Step 3: Phong Illumination Equation with a single point light (no attenuation)
        RtObject* obj = objList[hitObj];

        lightV = lightPos - intersection;
        lightV.normalize();
        viewV = ray.start - intersection;
        viewV.normalize();

        double NdotL = dot_prod(normal, lightV);
        lightReflectionV = normal * (2 * NdotL) - lightV;
        lightReflectionV.normalize();
        double RdotV = dot_prod(lightReflectionV, viewV);

        double diffuseFactor = NdotL > 0 ? NdotL : 0;
        double specularFactor = (NdotL > 0 && RdotV > 0) ? pow(RdotV, obj->speN) : 0;

        double color[3];
        for (int c = 0; c < 3; c++)
            color[c] = ambiantLight[c] * obj->ambiantReflection[c] + diffusetLight[c] * obj->diffusetReflection[c] * diffuseFactor +
                       specularLight[c] * obj->specularReflection[c] * specularFactor;

        // Step 4: add the colour seen along the reflected ray, weighted by the specular coefficient
        bool isReflective = obj->specularReflection[0] > 0 || obj->specularReflection[1] > 0 || obj->specularReflection[2] > 0;
        if (level < MAX_RT_LEVEL && isReflective) {
            rayReflectionV = ray.dir - normal * (2 * dot_prod(ray.dir, normal));
            rayReflectionV.normalize();
            newRay.start = intersection;
            newRay.dir = rayReflectionV;

            double reflected[3];
            rayTrace(newRay, reflected[0], reflected[1], reflected[2], hitObj, level + 1);

            for (int c = 0; c < 3; c++) color[c] += obj->specularReflection[c] * reflected[c];
        }

        r = color[0];
        g = color[1];
        b = color[2];
    }

    if (goBackGround) {
        r = bgColor[0];
        g = bgColor[1];
        b = bgColor[2];
    }
}

void drawInPixelBuffer(int x, int y, double r, double g, double b) {
    pixelBuffer[(y * WINWIDTH + x) * 3] = (float)r;
    pixelBuffer[(y * WINWIDTH + x) * 3 + 1] = (float)g;
    pixelBuffer[(y * WINWIDTH + x) * 3 + 2] = (float)b;
}

void renderScene() {
    int x, y;
    Ray ray;
    double r, g, b;

    cout << "Rendering Scene " << sceneNo << " with resolution " << WINWIDTH << "x" << WINHEIGHT << "........... ";
    __int64 time1 = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();  // marking the starting time

    ray.start = cameraPos;

    Vector3 vpCenter = cameraPos + lookAtDir * focalLen;  // viewplane center
    Vector3 startingPt = vpCenter + leftVector * (-WINWIDTH / 2.0) + upVector * (-WINHEIGHT / 2.0);
    Vector3 currPt;

    for (x = 0; x < WINWIDTH; x++)
        for (y = 0; y < WINHEIGHT; y++) {
            currPt = startingPt + leftVector * x + upVector * y;
            ray.dir = currPt - cameraPos;
            ray.dir.normalize();
            rayTrace(ray, r, g, b);
            drawInPixelBuffer(x, y, r, g, b);
        }

    __int64 time2 = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();  // marking the ending time

    cout << "Done! \nRendering time = " << time2 - time1 << "ms" << endl << endl;
}

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_DOUBLEBUFFER);
    glDrawPixels(WINWIDTH, WINHEIGHT, GL_RGB, GL_FLOAT, pixelBuffer);
    glutSwapBuffers();
    glFlush();
}

void reshape(int w, int h) {
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(-10, 10, -10, 10, -10, 10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void setScene(int i = 0) {
    if (i > NUM_SCENE) {
        cout << "Warning: Invalid Scene Number" << endl;
        return;
    }

    if (i == 0) {
        ((Sphere*)objList[0])->set(Vector3(-130, 80, 120), 100);
        ((Sphere*)objList[1])->set(Vector3(130, -80, -80), 100);
        ((Sphere*)objList[2])->set(Vector3(-130, -80, -80), 100);
        ((Sphere*)objList[3])->set(Vector3(130, 80, 120), 100);

        objList[0]->ambiantReflection[0] = 0.1;
        objList[0]->ambiantReflection[1] = 0.4;
        objList[0]->ambiantReflection[2] = 0.4;
        objList[0]->diffusetReflection[0] = 0;
        objList[0]->diffusetReflection[1] = 1;
        objList[0]->diffusetReflection[2] = 1;
        objList[0]->specularReflection[0] = 0.2;
        objList[0]->specularReflection[1] = 0.4;
        objList[0]->specularReflection[2] = 0.4;
        objList[0]->speN = 300;

        objList[1]->ambiantReflection[0] = 0.6;
        objList[1]->ambiantReflection[1] = 0.6;
        objList[1]->ambiantReflection[2] = 0.2;
        objList[1]->diffusetReflection[0] = 1;
        objList[1]->diffusetReflection[1] = 1;
        objList[1]->diffusetReflection[2] = 0;
        objList[1]->specularReflection[0] = 0.0;
        objList[1]->specularReflection[1] = 0.0;
        objList[1]->specularReflection[2] = 0.0;
        objList[1]->speN = 50;

        objList[2]->ambiantReflection[0] = 0.1;
        objList[2]->ambiantReflection[1] = 0.6;
        objList[2]->ambiantReflection[2] = 0.1;
        objList[2]->diffusetReflection[0] = 0.1;
        objList[2]->diffusetReflection[1] = 1;
        objList[2]->diffusetReflection[2] = 0.1;
        objList[2]->specularReflection[0] = 0.3;
        objList[2]->specularReflection[1] = 0.7;
        objList[2]->specularReflection[2] = 0.3;
        objList[2]->speN = 650;

        objList[3]->ambiantReflection[0] = 0.3;
        objList[3]->ambiantReflection[1] = 0.3;
        objList[3]->ambiantReflection[2] = 0.3;
        objList[3]->diffusetReflection[0] = 0.7;
        objList[3]->diffusetReflection[1] = 0.7;
        objList[3]->diffusetReflection[2] = 0.7;
        objList[3]->specularReflection[0] = 0.6;
        objList[3]->specularReflection[1] = 0.6;
        objList[3]->specularReflection[2] = 0.6;
        objList[3]->speN = 650;
    }

    if (i == 1) {
        // Step 5
        // a small cyan ball in front of a dull yellow ball, a big green ball, all above a huge lavender "floor" ball
        ((Sphere*)objList[0])->set(Vector3(110, 20, -150), 35);
        ((Sphere*)objList[1])->set(Vector3(160, 40, 150), 100);
        ((Sphere*)objList[2])->set(Vector3(-120, 80, 200), 150);
        ((Sphere*)objList[3])->set(Vector3(0, -1200, 300), 1050);

        objList[0]->ambiantReflection[0] = 0.1;
        objList[0]->ambiantReflection[1] = 0.4;
        objList[0]->ambiantReflection[2] = 0.4;
        objList[0]->diffusetReflection[0] = 0;
        objList[0]->diffusetReflection[1] = 0.8;
        objList[0]->diffusetReflection[2] = 1;
        objList[0]->specularReflection[0] = 0.3;
        objList[0]->specularReflection[1] = 0.5;
        objList[0]->specularReflection[2] = 0.5;
        objList[0]->speN = 400;

        objList[1]->ambiantReflection[0] = 0.6;
        objList[1]->ambiantReflection[1] = 0.6;
        objList[1]->ambiantReflection[2] = 0.2;
        objList[1]->diffusetReflection[0] = 1;
        objList[1]->diffusetReflection[1] = 1;
        objList[1]->diffusetReflection[2] = 0;
        objList[1]->specularReflection[0] = 0.0;
        objList[1]->specularReflection[1] = 0.0;
        objList[1]->specularReflection[2] = 0.0;
        objList[1]->speN = 50;

        objList[2]->ambiantReflection[0] = 0.1;
        objList[2]->ambiantReflection[1] = 0.6;
        objList[2]->ambiantReflection[2] = 0.1;
        objList[2]->diffusetReflection[0] = 0.1;
        objList[2]->diffusetReflection[1] = 1;
        objList[2]->diffusetReflection[2] = 0.1;
        objList[2]->specularReflection[0] = 0.3;
        objList[2]->specularReflection[1] = 0.7;
        objList[2]->specularReflection[2] = 0.3;
        objList[2]->speN = 650;

        objList[3]->ambiantReflection[0] = 0.4;
        objList[3]->ambiantReflection[1] = 0.4;
        objList[3]->ambiantReflection[2] = 0.6;
        objList[3]->diffusetReflection[0] = 0.5;
        objList[3]->diffusetReflection[1] = 0.5;
        objList[3]->diffusetReflection[2] = 0.7;
        objList[3]->specularReflection[0] = 0.4;
        objList[3]->specularReflection[1] = 0.4;
        objList[3]->specularReflection[2] = 0.5;
        objList[3]->speN = 300;
    }
}

void keyboard(unsigned char key, int x, int y) {
    // keys to control scaling - k
    // keys to control rotation - alpha
    // keys to control translation - tx, ty
    switch (key) {
        case 's':
        case 'S':
            sceneNo = (sceneNo + 1) % NUM_SCENE;
            setScene(sceneNo);
            renderScene();
            glutPostRedisplay();
            break;
        case 'q':
        case 'Q':
            exit(0);

        default:
            break;
    }
}

int main(int argc, char** argv) {
    cout << "<<CS3241 Lab 5>>\n\n" << endl;
    cout << "S to go to next scene" << endl;
    cout << "Q to quit" << endl;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(WINWIDTH, WINHEIGHT);

    glutCreateWindow("CS3241 Lab 5: Ray Tracing");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    glutKeyboardFunc(keyboard);

    objList = new RtObject*[NUM_OBJECTS];

    // create four spheres
    objList[0] = new Sphere(Vector3(-130, 80, 120), 100);
    objList[1] = new Sphere(Vector3(130, -80, -80), 100);
    objList[2] = new Sphere(Vector3(-130, -80, -80), 100);
    objList[3] = new Sphere(Vector3(130, 80, 120), 100);

    setScene(0);

    setScene(sceneNo);
    renderScene();

    glutMainLoop();

    for (int i = 0; i < NUM_OBJECTS; i++) delete objList[i];
    delete[] objList;

    delete[] pixelBuffer;

    return 0;
}
