// Steps 1 to 5 completed.
// Shadows and transparency + refraction implemented.

#include <chrono>
#include <cmath>
#include <iostream>

#include "GL\glut.h"
#include "vector3D.h"

using namespace std;

#define WINWIDTH 600
#define WINHEIGHT 400
#define NUM_OBJECTS 5
#define MAX_RT_LEVEL 50
#define NUM_SCENE 2

float* pixelBuffer = new float[WINWIDTH * WINHEIGHT * 3];

class Ray { 
public:
    Vector3 start, dir;
};

class RtObject {
public:
    virtual double intersectWithRay(
        Ray, Vector3& pos, Vector3& normal
    ) = 0; 

    double ambientReflection[3];
    double diffusetReflection[3];
    double specularReflection[3];
    double speN = 300;
    double transparency = 0;      
    double refractiveIndex = 1;
    double shellThickness = 0;
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
    Vector3 center() { return center_; };
    double radius() { return r_; };
    double intersectWithRay(Ray, Vector3& pos, Vector3& normal);
};

RtObject** objList; 

// Global Variables
Vector3 cameraPos(0, 0, -500);

// assume the the following two vectors are normalised
Vector3 lookAtDir(0, 0, 1);
Vector3 upVector(0, 1, 0);
Vector3 leftVector(1, 0, 0);
float focalLen = 500;

// Light Settings

Vector3 lightPos(900, 1000, -1500);
double ambientLight[3] = {0.4, 0.4, 0.4};
double diffusetLight[3] = {0.7, 0.7, 0.7};
double specularLight[3] = {0.5, 0.5, 0.5};

double bgColor[3] = {0.1, 0.1, 0.4};

int sceneNo = 0;

double Sphere::intersectWithRay(Ray r, Vector3& intersection, Vector3& normal) {
    // |start + t*dir - center|^2 = r^2, i.e. a*t^2 + b*t + c = 0
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

// Snell's law
bool refract(Vector3 I, Vector3 N, double eta, Vector3& T) {
    double cosi = -dot_prod(N, I);
    double k = 1 - eta * eta * (1 - cosi * cosi);
    if (k < 0) return false;

    T = I * eta + N * (eta * cosi - sqrt(k));
    T.normalize();
    return true;
}

Vector3 reflect(Vector3 I, Vector3 N) {
    return I - N * (2 * dot_prod(I, N));
}

// Schlick's approximation
double fresnel(Vector3 I, Vector3 N, double n) {
    double r0 = (n - 1) / (n + 1);
    r0 *= r0;
    double cosi = fabs(dot_prod(I, N));
    return r0 + (1 - r0) * pow(1 - cosi, 5);
}

bool passThrough(RtObject* obj, Vector3 p, Vector3 d, Ray& outRay) {
    double n = obj->refractiveIndex;
    bool hollow = obj->shellThickness > 0;
    Sphere cavity;
    if (hollow)
        cavity.set(((Sphere*)obj)->center(), ((Sphere*)obj)->radius() - obj->shellThickness);

    bool inCavity = false;
    Vector3 hit, hitNormal, cavityHit, cavityNormal, newDir;
    Ray seg;

    for (int bounce = 0; bounce < 20; bounce++) {
        seg.start = p + d * 1e-4; 
        seg.dir = d;

        if (inCavity) {
            if (cavity.intersectWithRay(seg, hit, hitNormal) <= 0) return false;
            refract(d, -hitNormal, 1 / n, newDir);
            p = hit;
            d = newDir;
            inCavity = false;
            continue;
        }

        double tOuter = obj->intersectWithRay(seg, hit, hitNormal);
        double tCavity = hollow ? cavity.intersectWithRay(seg, cavityHit, cavityNormal) : -1;

        if (tCavity > 0 && (tOuter <= 0 || tCavity < tOuter)) {
            p = cavityHit;
            if (refract(d, cavityNormal, n, newDir)) {
                d = newDir;
                inCavity = true;
            } else
                d = reflect(d, cavityNormal);
        } else if (tOuter > 0) {
            p = hit;
            if (refract(d, -hitNormal, n, newDir)) {
                outRay.start = hit;
                outRay.dir = newDir;
                return true;
            }
            d = reflect(d, hitNormal);
        } else
            return false;
    }
    return false;
}

void rayTrace(Ray ray, double& r, double& g, double& b, int fromObj = -1, int level = 0) {
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
        if (i == fromObj)
            continue;

        if ((t = objList[i]->intersectWithRay(ray, currIntersection, currNormal)) > 0 && t < mint) {
            // keep only the closest object along the ray
            mint = t;
            hitObj = i;
            intersection = currIntersection;
            normal = currNormal;
            goBackGround = 0;
        }
    }

    if (!goBackGround) {
        RtObject* obj = objList[hitObj];

        lightV = lightPos - intersection;
        double lightDist = lightV.length();
        lightV.normalize();
        viewV = ray.start - intersection;
        viewV.normalize();

        double NdotL = dot_prod(normal, lightV);
        lightReflectionV = normal * (2 * NdotL) - lightV;
        lightReflectionV.normalize();
        double RdotV = dot_prod(lightReflectionV, viewV);

        // shadows
        double lightVisibility = 1;
        Ray shadowRay;
        shadowRay.start = intersection;
        shadowRay.dir = lightV;
        for (int j = 0; j < NUM_OBJECTS && lightVisibility > 0; j++) {
            if (j == hitObj)
                continue;
            if ((t = objList[j]->intersectWithRay(shadowRay, currIntersection, currNormal)) > 0 && t < lightDist)
                lightVisibility *= objList[j]->transparency;
        }

        double diffuseFactor = NdotL > 0 ? lightVisibility * NdotL : 0;
        double specularFactor = (NdotL > 0 && RdotV > 0) ? lightVisibility * pow(RdotV, obj->speN) : 0;

        // transparency
        double kt = obj->transparency;
        double transmitted[3] = {bgColor[0], bgColor[1], bgColor[2]};
        Vector3 insideDir;
        if (level < MAX_RT_LEVEL && kt > 0 && refract(ray.dir, normal, 1 / obj->refractiveIndex, insideDir) &&
            passThrough(obj, intersection, insideDir, newRay))
            rayTrace(newRay, transmitted[0], transmitted[1], transmitted[2], hitObj, level + 1);

        double reflected[3] = {0, 0, 0};
        bool isReflective = kt > 0 || obj->specularReflection[0] > 0 || obj->specularReflection[1] > 0 || obj->specularReflection[2] > 0;
        if (level < MAX_RT_LEVEL && isReflective) {
            rayReflectionV = ray.dir - normal * (2 * dot_prod(ray.dir, normal));
            rayReflectionV.normalize();
            newRay.start = intersection;
            newRay.dir = rayReflectionV;

            rayTrace(newRay, reflected[0], reflected[1], reflected[2], hitObj, level + 1);
        }

        // the transparent part replaces the surface colour, and glass reflects by Fresnel instead of Ks
        double fr = kt > 0 ? fresnel(ray.dir, normal, obj->refractiveIndex) : 0;
        double color[3];
        for (int c = 0; c < 3; c++) {
            double reflectWeight = kt > 0 ? fr : obj->specularReflection[c];
            color[c] = (1 - kt) * (ambientLight[c] * obj->ambientReflection[c] + diffusetLight[c] * obj->diffusetReflection[c] * diffuseFactor) +
                       kt * (1 - fr) * transmitted[c] + reflectWeight * reflected[c] +
                       specularLight[c] * obj->specularReflection[c] * specularFactor;
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
    __int64 time1 = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();

    ray.start = cameraPos;

    Vector3 vpCenter = cameraPos + lookAtDir * focalLen;
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

    __int64 time2 = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();

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

        objList[0]->ambientReflection[0] = 0.1;
        objList[0]->ambientReflection[1] = 0.4;
        objList[0]->ambientReflection[2] = 0.4;
        objList[0]->diffusetReflection[0] = 0;
        objList[0]->diffusetReflection[1] = 1;
        objList[0]->diffusetReflection[2] = 1;
        objList[0]->specularReflection[0] = 0.2;
        objList[0]->specularReflection[1] = 0.4;
        objList[0]->specularReflection[2] = 0.4;
        objList[0]->speN = 300;

        objList[1]->ambientReflection[0] = 0.6;
        objList[1]->ambientReflection[1] = 0.6;
        objList[1]->ambientReflection[2] = 0.2;
        objList[1]->diffusetReflection[0] = 1;
        objList[1]->diffusetReflection[1] = 1;
        objList[1]->diffusetReflection[2] = 0;
        objList[1]->specularReflection[0] = 0.0;
        objList[1]->specularReflection[1] = 0.0;
        objList[1]->specularReflection[2] = 0.0;
        objList[1]->speN = 50;

        objList[2]->ambientReflection[0] = 0.1;
        objList[2]->ambientReflection[1] = 0.6;
        objList[2]->ambientReflection[2] = 0.1;
        objList[2]->diffusetReflection[0] = 0.1;
        objList[2]->diffusetReflection[1] = 1;
        objList[2]->diffusetReflection[2] = 0.1;
        objList[2]->specularReflection[0] = 0.3;
        objList[2]->specularReflection[1] = 0.7;
        objList[2]->specularReflection[2] = 0.3;
        objList[2]->speN = 650;

        objList[3]->ambientReflection[0] = 0.3;
        objList[3]->ambientReflection[1] = 0.3;
        objList[3]->ambientReflection[2] = 0.3;
        objList[3]->diffusetReflection[0] = 0.7;
        objList[3]->diffusetReflection[1] = 0.7;
        objList[3]->diffusetReflection[2] = 0.7;
        objList[3]->specularReflection[0] = 0.6;
        objList[3]->specularReflection[1] = 0.6;
        objList[3]->specularReflection[2] = 0.6;
        objList[3]->speN = 650;

        // clear glass bubble in front of the bottom two
        ((Sphere*)objList[4])->set(Vector3(0, -20, -300), 40);
        objList[4]->ambientReflection[0] = 0.05;
        objList[4]->ambientReflection[1] = 0.05;
        objList[4]->ambientReflection[2] = 0.05;
        objList[4]->diffusetReflection[0] = 0.1;
        objList[4]->diffusetReflection[1] = 0.1;
        objList[4]->diffusetReflection[2] = 0.1;
        objList[4]->specularReflection[0] = 0.2;
        objList[4]->specularReflection[1] = 0.2;
        objList[4]->specularReflection[2] = 0.2;
        objList[4]->speN = 300;
        objList[4]->transparency = 0.9;
        objList[4]->refractiveIndex = 1.5;
        objList[4]->shellThickness = 3;
    }

    if (i == 1) {
        ((Sphere*)objList[0])->set(Vector3(110, 20, -150), 35);
        ((Sphere*)objList[1])->set(Vector3(160, 40, 150), 100);
        ((Sphere*)objList[2])->set(Vector3(-120, 80, 200), 150);
        ((Sphere*)objList[3])->set(Vector3(0, -1200, 300), 1050);

        objList[0]->ambientReflection[0] = 0.1;
        objList[0]->ambientReflection[1] = 0.4;
        objList[0]->ambientReflection[2] = 0.4;
        objList[0]->diffusetReflection[0] = 0;
        objList[0]->diffusetReflection[1] = 0.8;
        objList[0]->diffusetReflection[2] = 1;
        objList[0]->specularReflection[0] = 0.3;
        objList[0]->specularReflection[1] = 0.5;
        objList[0]->specularReflection[2] = 0.5;
        objList[0]->speN = 400;

        objList[1]->ambientReflection[0] = 0.6;
        objList[1]->ambientReflection[1] = 0.6;
        objList[1]->ambientReflection[2] = 0.2;
        objList[1]->diffusetReflection[0] = 1;
        objList[1]->diffusetReflection[1] = 1;
        objList[1]->diffusetReflection[2] = 0;
        objList[1]->specularReflection[0] = 0.0;
        objList[1]->specularReflection[1] = 0.0;
        objList[1]->specularReflection[2] = 0.0;
        objList[1]->speN = 50;

        objList[2]->ambientReflection[0] = 0.1;
        objList[2]->ambientReflection[1] = 0.6;
        objList[2]->ambientReflection[2] = 0.1;
        objList[2]->diffusetReflection[0] = 0.1;
        objList[2]->diffusetReflection[1] = 1;
        objList[2]->diffusetReflection[2] = 0.1;
        objList[2]->specularReflection[0] = 0.3;
        objList[2]->specularReflection[1] = 0.7;
        objList[2]->specularReflection[2] = 0.3;
        objList[2]->speN = 650;

        objList[3]->ambientReflection[0] = 0.4;
        objList[3]->ambientReflection[1] = 0.4;
        objList[3]->ambientReflection[2] = 0.6;
        objList[3]->diffusetReflection[0] = 0.5;
        objList[3]->diffusetReflection[1] = 0.5;
        objList[3]->diffusetReflection[2] = 0.7;
        objList[3]->specularReflection[0] = 0.4;
        objList[3]->specularReflection[1] = 0.4;
        objList[3]->specularReflection[2] = 0.5;
        objList[3]->speN = 300;

        // pink-tinted glass bubble above the floor, left of the cyan ball
        ((Sphere*)objList[4])->set(Vector3(-90, -60, -120), 50);
        objList[4]->ambientReflection[0] = 0.3;
        objList[4]->ambientReflection[1] = 0.1;
        objList[4]->ambientReflection[2] = 0.2;
        objList[4]->diffusetReflection[0] = 0.6;
        objList[4]->diffusetReflection[1] = 0.2;
        objList[4]->diffusetReflection[2] = 0.4;
        objList[4]->specularReflection[0] = 0.2;
        objList[4]->specularReflection[1] = 0.2;
        objList[4]->specularReflection[2] = 0.2;
        objList[4]->speN = 300;
        objList[4]->transparency = 0.75;
        objList[4]->refractiveIndex = 1.5;
        objList[4]->shellThickness = 4;
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

    objList[0] = new Sphere(Vector3(-130, 80, 120), 100);
    objList[1] = new Sphere(Vector3(130, -80, -80), 100);
    objList[2] = new Sphere(Vector3(-130, -80, -80), 100);
    objList[3] = new Sphere(Vector3(130, 80, 120), 100);
    objList[4] = new Sphere(Vector3(0, -20, -300), 40);

    setScene(0);

    setScene(sceneNo);
    renderScene();

    glutMainLoop();

    for (int i = 0; i < NUM_OBJECTS; i++) delete objList[i];
    delete[] objList;

    delete[] pixelBuffer;

    return 0;
}
