#include <SDL3/SDL.h>
#include <math.h>
#include <stdio.h>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define __PI 3.14159265f
#define FOCAL_LENGTH 350.0f      // "Kamera"-Brennweite
#define BODY_HEIGHT 150.0f       // Globale Konstante
#define CAM_RADIUS 400.0f        // Abstand der Kamera vom Ziel
#define CAM_HEIGHT 50.0f         // Höhe der Kamera über dem Ziel

typedef struct {
    float x, y;                  
    float angle1, angle2;      
} Stickman;


struct tPoint {
    float x, y, z;
    tPoint() : x(0), y(0), z(0) {}
    tPoint(float _x, float _y, float _z = 0.0f) : x(_x), y(_y), z(_z) {}

    tPoint& operator+=(const tPoint& p) {
        x += p.x; y += p.y; z += p.z;
        return *this;
    }
    tPoint operator+(const tPoint& p) const {
        return tPoint(x + p.x, y + p.y, z + p.z);
    }
    tPoint operator-(const tPoint& p) const {
        return tPoint(x - p.x, y - p.y, z - p.z);
    }
    tPoint operator*(float scalar) const {
        return tPoint(x * scalar, y * scalar, z * scalar);
    }

    float length() const {
        return sqrtf(x * x + y * y + z * z);
    }
    tPoint normalize() const {
        float l = length();
        if (l == 0) return tPoint(0, 0, 0);
        return tPoint(x / l, y / l, z / l);
    }
    float dot(const tPoint& p) const {
        return x * p.x + y * p.y + z * p.z;
    }
    tPoint cross(const tPoint& p) const {
        return tPoint(
            y * p.z - z * p.y,
            z * p.x - x * p.z,
            x * p.y - y * p.x
        );
    }
};

struct tTriangle {
    tPoint p1, p2, p3;
    tTriangle() {}
    tTriangle(const tPoint& a, const tPoint& b, const tPoint& c) : p1(a), p2(b), p3(c) {}
    tTriangle operator+(const tPoint& off) const {
        return tTriangle(p1 + off, p2 + off, p3 + off);
    }
    tTriangle& operator+=(const tPoint& off) {
        p1 += off; p2 += off; p3 += off;
        return *this;
    }
};

// --- Globale Kameravariablen ---
tPoint camPos;
tPoint camTarget;

// --- NEUE 3D-Projektionsfunktion (View + Projection) ---
SDL_FPoint project(tPoint p_world) {
    // 1. View-Transformation (LookAt)
    tPoint up = tPoint(0, 1, 0);
    tPoint z_axis = (camTarget - camPos).normalize();
    tPoint x_axis = up.cross(z_axis).normalize();
    tPoint y_axis = z_axis.cross(x_axis);

    // Wandle Weltkoordinate in Kamerakoordinate um
    tPoint p_relative = p_world - camPos;
    tPoint p_cam;
    p_cam.x = p_relative.dot(x_axis);
    p_cam.y = p_relative.dot(y_axis);
    p_cam.z = p_relative.dot(z_axis); 

    // 2. Perspektivische Projektion
    if (p_cam.z <= 0) {
        return { -1000, -1000 };
    }

    float scale = FOCAL_LENGTH / p_cam.z;
    float screen_x = p_cam.x * scale;
    float screen_y = p_cam.y * scale;

    // 3. In Bildschirmkoordinaten umwandeln (Zentrum ist die Fenstermitte)
    return {
        screen_x + WINDOW_WIDTH / 2.0f,
        screen_y + WINDOW_HEIGHT / 2.0f
    };
}

void RotatePoint(float* cx, float* cy, float angle, float ox, float oy) {
    float x = *cx - ox;
    float y = *cy - oy;
    float tempX = x * cosf(angle) - y * sinf(angle);
    float tempY = x * sinf(angle) + y * cosf(angle);
    *cx = tempX + ox;
    *cy = tempY + oy;
}

void RotateTriangle(tTriangle* t, float angle, tPoint originPos) {
    RotatePoint(&t->p1.x, &t->p1.y, angle, originPos.x, originPos.y);
    RotatePoint(&t->p2.x, &t->p2.y, angle, originPos.x, originPos.y);
    RotatePoint(&t->p3.x, &t->p3.y, angle, originPos.x, originPos.y);
}


void RotatePointXZ(float* cx, float* cz, float angle, float ox, float oz) {
    float x = *cx - ox;
    float z = *cz - oz;
    float tempX = x * cosf(angle) - z * sinf(angle);
    float tempZ = x * sinf(angle) + z * cosf(angle);
    *cx = tempX + ox;
    *cz = tempZ + oz;
}

void RotateTriangleXZ(tTriangle* t, float angle, tPoint originPos) {
    RotatePointXZ(&t->p1.x, &t->p1.z, angle, originPos.x, originPos.z);
    RotatePointXZ(&t->p2.x, &t->p2.z, angle, originPos.x, originPos.z);
    RotatePointXZ(&t->p3.x, &t->p3.z, angle, originPos.x, originPos.z);
}

void drawPyramid(SDL_Renderer* renderer, const tTriangle& base, const tPoint& apex, const tPoint& worldOffset) {
    // Füge den Welt-Offset zu den lokalen Punkten hinzu
    tPoint p1_world = base.p1 + worldOffset;
    tPoint p2_world = base.p2 + worldOffset;
    tPoint p3_world = base.p3 + worldOffset;
    tPoint apex_world = apex + worldOffset;

    // Projiziere alle 4 3D-Weltpunkte auf 2D-Bildschirmkoordinaten
    SDL_FPoint p1_2d = project(p1_world);
    SDL_FPoint p2_2d = project(p2_world);
    SDL_FPoint p3_2d = project(p3_world);
    SDL_FPoint apex_2d = project(apex_world);

    // Zeichne die Basis
    SDL_RenderLine(renderer, p1_2d.x, p1_2d.y, p2_2d.x, p2_2d.y);
    SDL_RenderLine(renderer, p2_2d.x, p2_2d.y, p3_2d.x, p3_2d.y);
    SDL_RenderLine(renderer, p3_2d.x, p3_2d.y, p1_2d.x, p1_2d.y);

    // Zeichne die Seiten zur Spitze
    SDL_RenderLine(renderer, p1_2d.x, p1_2d.y, apex_2d.x, apex_2d.y);
    SDL_RenderLine(renderer, p2_2d.x, p2_2d.y, apex_2d.x, apex_2d.y);
    SDL_RenderLine(renderer, p3_2d.x, p3_2d.y, apex_2d.x, apex_2d.y);
}

void drawStickman(SDL_Renderer* renderer, Stickman s) {
    float armLen = 50.0f, armWidth = 11.0f;
    float bodyWidth = 30.0f;
    float headWidth = 22.0f, headHeight = 30.0f;
    float legLen = 60.0f, legWidth = 11.0f;
    const float PYRAMID_DEPTH = 30.0f;

    const float Z_LEFT_ARMS = bodyWidth - 5.0f; 
    const float Z_RIGHT_ARMS = bodyWidth - 5.0f; 

    const float Z_LEG_OFFSET = 5.0f;
    const float Z_LEFT_LEGS = Z_LEG_OFFSET;
    const float Z_RIGHT_LEGS = Z_LEG_OFFSET;  

    const float Z_CENTER = 0.0f;

    //Definitionen (Basen und Spitzen)
    // Linke Gliedmaßen
    tTriangle leftArm({ armWidth, 0, Z_LEFT_ARMS }, { -armWidth, 0, Z_LEFT_ARMS }, { 0, armLen, Z_LEFT_ARMS });
    tTriangle leftForearm({ armWidth, 0, Z_LEFT_ARMS }, { -armWidth, 0, Z_LEFT_ARMS }, { 0, armLen, Z_LEFT_ARMS });

    tTriangle leftLeg({ legWidth, 0, Z_LEFT_LEGS }, { -legWidth, 0, Z_LEFT_LEGS }, { 0, legLen, Z_LEFT_LEGS });
    tTriangle leftShin({ legWidth, 0, Z_LEFT_LEGS }, { -legWidth, 0, Z_LEFT_LEGS }, { 0, legLen, Z_LEFT_LEGS });

    tPoint leftArmApex(0, 0, Z_LEFT_ARMS + PYRAMID_DEPTH);
    tPoint leftForearmApex(0, 0, Z_LEFT_ARMS + PYRAMID_DEPTH);
    tPoint leftLegApex(0, 0, Z_LEFT_LEGS + PYRAMID_DEPTH); 
    tPoint leftShinApex(0, 0, Z_LEFT_LEGS + PYRAMID_DEPTH); 

    // Rechte Gliedmaßen
    tTriangle rightArm({ armWidth, 0, Z_RIGHT_ARMS }, { -armWidth, 0, Z_RIGHT_ARMS }, { 0, armLen, Z_RIGHT_ARMS });
    tTriangle rightForearm({ armWidth, 0, Z_RIGHT_ARMS }, { -armWidth, 0, Z_RIGHT_ARMS }, { 0, armLen, Z_RIGHT_ARMS });

    tTriangle rightLeg({ legWidth, 0, Z_RIGHT_LEGS }, { -legWidth, 0, Z_RIGHT_LEGS }, { 0, legLen, Z_RIGHT_LEGS });
    tTriangle rightShin({ legWidth, 0, Z_RIGHT_LEGS }, { -legWidth, 0, Z_RIGHT_LEGS }, { 0, legLen, Z_RIGHT_LEGS });

    tPoint rightArmApex(0, 0, Z_RIGHT_ARMS + PYRAMID_DEPTH);
    tPoint rightForearmApex(0, 0, Z_RIGHT_ARMS + PYRAMID_DEPTH);
    tPoint rightLegApex(0, 0, Z_RIGHT_LEGS + PYRAMID_DEPTH); 
    tPoint rightShinApex(0, 0, Z_RIGHT_LEGS + PYRAMID_DEPTH); 

    // Körper und Kopf
    tTriangle body({ bodyWidth, 0, Z_CENTER }, { -bodyWidth, 0, Z_CENTER }, { 0, BODY_HEIGHT, Z_CENTER });
    tTriangle head({ headWidth, -headHeight, Z_CENTER }, { -headWidth, -headHeight, Z_CENTER }, { 0, 0, Z_CENTER });

    tPoint bodyApex(0, 0, Z_CENTER + PYRAMID_DEPTH);
    tPoint headApex(0, -headHeight * 2.0f / 3.0f, Z_CENTER + PYRAMID_DEPTH);

    const float bodyRotationAngle = -__PI / 2.0f; 

    // Rotiere den Körper
    RotateTriangleXZ(&body, bodyRotationAngle, { 0, 0, 0 });
    RotatePointXZ(&bodyApex.x, &bodyApex.z, bodyRotationAngle, 0, 0);

    // Rotiere den Kopf
    RotateTriangleXZ(&head, bodyRotationAngle, { 0, 0, 0 });
    RotatePointXZ(&headApex.x, &headApex.z, bodyRotationAngle, 0, 0);

    const float mirrorAngle = __PI;
    tPoint rotationOrigin = { 0, 0, 0 };

    // Rechter Arm
    RotateTriangleXZ(&rightArm, mirrorAngle, rotationOrigin);
    RotatePointXZ(&rightArmApex.x, &rightArmApex.z, mirrorAngle, rotationOrigin.x, rotationOrigin.z);

    // Rechter Unterarm
    RotateTriangleXZ(&rightForearm, mirrorAngle, rotationOrigin);
    RotatePointXZ(&rightForearmApex.x, &rightForearmApex.z, mirrorAngle, rotationOrigin.x, rotationOrigin.z);

    // Rechtes Bein
    RotateTriangleXZ(&rightLeg, mirrorAngle, rotationOrigin);
    RotatePointXZ(&rightLegApex.x, &rightLegApex.z, mirrorAngle, rotationOrigin.x, rotationOrigin.z);

    // Rechter Unterschenkel
    RotateTriangleXZ(&rightShin, mirrorAngle, rotationOrigin);
    RotatePointXZ(&rightShinApex.x, &rightShinApex.z, mirrorAngle, rotationOrigin.x, rotationOrigin.z);


    // Rotationen der Gliedmaßen
    float leftArmAngle = (s.angle1 - __PI / 4);
    RotateTriangle(&leftArm, leftArmAngle, { 0, 0 });
    RotatePoint(&leftArmApex.x, &leftArmApex.y, leftArmAngle, 0, 0);
    float leftForearmAngle = (s.angle1 - __PI * 3 / 4);
    RotateTriangle(&leftForearm, leftForearmAngle, { 0, 0 });
    RotatePoint(&leftForearmApex.x, &leftForearmApex.y, leftForearmAngle, 0, 0);
    float rightArmAngle = (s.angle2 + __PI / 4);
    RotateTriangle(&rightArm, rightArmAngle, { 0, 0 });
    RotatePoint(&rightArmApex.x, &rightArmApex.y, rightArmAngle, 0, 0);
    float rightForearmAngle = (s.angle2 - __PI / 4);
    RotateTriangle(&rightForearm, rightForearmAngle, { 0, 0 });
    RotatePoint(&rightForearmApex.x, &rightForearmApex.y, rightForearmAngle, 0, 0);
    float leftLegAngle = (s.angle1 - __PI / 4);
    RotateTriangle(&leftLeg, leftLegAngle, { 0, 0 });
    RotatePoint(&leftLegApex.x, &leftLegApex.y, leftLegAngle, 0, 0);
    float leftShinAngle = (s.angle1 - __PI / 8);
    RotateTriangle(&leftShin, leftShinAngle, { 0, 0 });
    RotatePoint(&leftShinApex.x, &leftShinApex.y, leftShinAngle, 0, 0);
    float rightLegAngle = (s.angle2 + __PI / 4);
    RotateTriangle(&rightLeg, rightLegAngle, { 0, 0 });
    RotatePoint(&rightLegApex.x, &rightLegApex.y, rightLegAngle, 0, 0);
    float rightShinAngle = (s.angle2 + __PI / 3);
    RotateTriangle(&rightShin, rightShinAngle, { 0, 0 });
    RotatePoint(&rightShinApex.x, &rightShinApex.y, rightShinAngle, 0, 0);

    tPoint leftArmOffset(leftArm.p3.x, 15 + leftArm.p3.y);
    leftForearm += leftArmOffset;
    leftForearmApex += leftArmOffset;

    tPoint rightArmOffset(rightArm.p3.x, 15 + rightArm.p3.y);
    rightForearm += rightArmOffset;
    rightForearmApex += rightArmOffset;

    tPoint bodyLegOffset(0, BODY_HEIGHT - 20); 

    tPoint leftLegOffset(leftLeg.p3.x, leftLeg.p3.y + (BODY_HEIGHT - 20));
    leftShin += leftLegOffset;
    leftShinApex += leftLegOffset;

    tPoint rightLegOffset(rightLeg.p3.x, rightLeg.p3.y + (BODY_HEIGHT - 20));
    rightShin += rightLegOffset;
    rightShinApex += rightLegOffset;

    tPoint armBaseOffset(0, 15);

    tPoint stickmanWorldPos(s.x, s.y, 0);

    drawPyramid(renderer, leftArm + armBaseOffset, leftArmApex + armBaseOffset, stickmanWorldPos);
    drawPyramid(renderer, leftForearm, leftForearmApex, stickmanWorldPos);
    drawPyramid(renderer, rightArm + armBaseOffset, rightArmApex + armBaseOffset, stickmanWorldPos);
    drawPyramid(renderer, rightForearm, rightForearmApex, stickmanWorldPos);
    drawPyramid(renderer, leftLeg + bodyLegOffset, leftLegApex + bodyLegOffset, stickmanWorldPos);
    drawPyramid(renderer, leftShin, leftShinApex, stickmanWorldPos);
    drawPyramid(renderer, rightLeg + bodyLegOffset, rightLegApex + bodyLegOffset, stickmanWorldPos);
    drawPyramid(renderer, rightShin, rightShinApex, stickmanWorldPos);
    drawPyramid(renderer, head, headApex, stickmanWorldPos);
    drawPyramid(renderer, body, bodyApex, stickmanWorldPos);
}

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("SDL_Init error: %s", SDL_GetError());
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow("Rotating Camera Stickman (3D)",
        WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_RESIZABLE);
    if (!window) {
        SDL_Log("SDL_CreateWindow error: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    if (!renderer) {
        SDL_Log("SDL_CreateRenderer error: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    Stickman man = { 0.0f, 300.0f, 0.0f, 0.0f };
    float AngleSteps = __PI / 100;
    float camAngle = 0.1f;

    int running = 1;
    SDL_Event e;
    while (running) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT)
                running = 0;
        }


        man.angle1 += AngleSteps;
        man.angle2 -= AngleSteps;
        if (man.angle1 > __PI / 2 || man.angle1 < 0) AngleSteps *= -1;

        man.x += 2;
        if (man.x > WINDOW_WIDTH) man.x = -WINDOW_WIDTH; 

        camAngle += 0.01f; 

        camTarget = tPoint(man.x, man.y + BODY_HEIGHT / 2.0f, 0);

        camPos.x = camTarget.x + CAM_RADIUS * sinf(camAngle);
        camPos.z = camTarget.z - CAM_RADIUS * cosf(camAngle); 
        camPos.y = camTarget.y + CAM_HEIGHT; 

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        drawStickman(renderer, man);

        SDL_RenderPresent(renderer);
        SDL_Delay(16); 
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}