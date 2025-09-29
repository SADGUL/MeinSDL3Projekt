#include <SDL3/SDL.h>
#include <math.h>
#include <stdio.h>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define __PI 3.14159265f

typedef struct {
    float x, y,z;      // Position of body center
    float angle1,angle2;     // Animation angle
} Stickman;
struct tPoint{
    float x, y;      // Position of body center
    tPoint operator+=(tPoint p) {
		this->x += p.x;
		this->y += p.y;
		return *this;
	}
} ;

struct tTriangle {
	tPoint p1, p2, p3;
    tTriangle operator+=(const tPoint offset) {
        p1.x = p1.x + offset.x;
        p1.y = p1.y + offset.y;
        p2.x = p2.x + offset.x;
        p2.y = p2.y + offset.y;
        p3.x = p3.x + offset.x;
        p3.y = p3.y + offset.y;
		return *this;
	}
    tTriangle operator+(const tPoint offset) {
		p1.x = p1.x + offset.x;
        p1.y = p1.y + offset.y;
		p2.x = p2.x + offset.x;
        p2.y = p2.y + offset.y;
		p3.x = p3.x + offset.x;
        p3.y = p3.y + offset.y;
        return *this;
    }
    
} ;
class TBodypart
{
private:
    tPoint p[4];
public:
	TBodypart() {
    this->p[0].x= }
    TBodypart(tPoint p1, tPoint p2, tPoint p3, tPoint p4) {
        p[0] = p1; p[1] = p2; p[2] = p3; p[3] = p4;
    }
    void Draw(SDL_Renderer* renderer, Stickman s) {
        for (int i = 0; i < 4; i++) {
            SDL_RenderLine(renderer, s.x + p[i].x, s.y + p[i].y, s.x + p[(i + 1) % 4].x, s.y + p[(i + 1) % 4].y);
        }
	}
};
void PrintTriangle(tTriangle t) {
    printf("P1: (%.2f, %.2f)\n", t.p1.x, t.p1.y);
    printf("P2: (%.2f, %.2f)\n", t.p2.x, t.p2.y);
    printf("P3: (%.2f, %.2f)\n", t.p3.x, t.p3.y);
}

void RotatePoint(float* cx, float* cy, float angle, float ox, float oy) {
    // Translate point so that pivot (ox, oy) is at the origin
    float x = *cx - ox;
    float y = *cy - oy;

    // Rotate around origin
    float tempX = x * cos(angle) - y * sin(angle);
    float tempY = x * sin(angle) + y * cos(angle);

    // Translate back
    *cx = tempX + ox;
    *cy = tempY + oy;
}

void RotateTriangle(tTriangle* t, float angle,tPoint originPos) {
    // Rotate around p1 (keeps p1 fixed)
	RotatePoint(&t->p1.x, &t->p1.y, angle, originPos.x, originPos.y);
    RotatePoint(&t->p2.x, &t->p2.y, angle, originPos.x, originPos.y);
    RotatePoint(&t->p3.x, &t->p3.y, angle, originPos.x, originPos.y);
}

void drawTriangle(SDL_Renderer* renderer,
    tTriangle t,Stickman s) {
    SDL_RenderLine(renderer, s.x + t.p1.x, s.y + t.p1.y, s.x + t.p2.x, s.y + t.p2.y);
    SDL_RenderLine(renderer, s.x + t.p2.x, s.y + t.p2.y, s.x + t.p3.x, s.y + t.p3.y);
    SDL_RenderLine(renderer, s.x + t.p3.x, s.y + t.p3.y, s.x + t.p1.x, s.y + t.p1.y);
}

void drawStickman(SDL_Renderer* renderer, Stickman s) {
	tPoint origin = { s.x,s.y };
	float armLen = 50;
	float armWidth = 10;

	float bodyWidth = 30;
	float bodyHeight = 150;

	float headwidth = 22;
	float headHeight = 30;

	float legLen = 60;
	float legWidth = 10;

	tTriangle leftArm, rightArm;
	tTriangle leftLeg, rightLeg;
	tTriangle head, body;
	tTriangle leftForearm, rightForearm;
	tTriangle leftShin, rightShin;


    //ARMS
	leftArm = { { armWidth, 0 },{ -armWidth, 0 },{ 0, armLen } };

	leftForearm = { { armWidth, 0 },{ -armWidth, 0 },{ 0, armLen } };

	rightArm = { { armWidth, 0 },{ -armWidth, 0 },{ 0, armLen } };

	rightForearm = { { armWidth, 0 },{ -armWidth, 0 },{ 0, armLen } };

	
    //LEGS
	leftLeg = { { legWidth, 0 },{ -legWidth, 0 },{ 0, legLen } };
	
	leftShin = { { legWidth, 0 },{ -legWidth, 0 },{ 0, legLen } };

    rightLeg = { { legWidth, 0 },{ -legWidth, 0 },{ 0, legLen } };
    
	rightShin = { { legWidth, 0 },{ -legWidth, 0 },{ 0, legLen } };


	body = { { bodyWidth, 0 },{ -bodyWidth, 0 },{ 0, bodyHeight } };

	head = { { headwidth, -headHeight },{ -headwidth, -headHeight },{ 0, 0 } };

    RotateTriangle(&leftArm, (s.angle1 - __PI/4), {0,0});
    RotateTriangle(&leftForearm, (s.angle1-__PI*3/4 ), { 0,0 });
    
    RotateTriangle(&rightArm, (s.angle2 + __PI / 4), { 0,0 });
    RotateTriangle(&rightForearm, (s.angle2-__PI *1/ 4), { 0,0 });


    //Forearms correction
    leftForearm += tPoint{ leftArm.p3.x,15+leftArm.p3.y };
	rightForearm += tPoint{ rightArm.p3.x,15 + rightArm.p3.y };

    drawTriangle(renderer, leftArm + tPoint{0, 15}, s);
	drawTriangle(renderer, leftForearm, s);
    

    RotateTriangle(&leftLeg, (s.angle1 - __PI / 4), { 0,0 });
    RotateTriangle(&rightLeg, (s.angle2 + __PI / 4), { 0,0 });
	RotateTriangle(&leftShin, (s.angle1 - __PI/8), { 0,0 });
	RotateTriangle(&rightShin, (s.angle2 + __PI/3), { 0,0 });
    //Shins correction
	leftShin += tPoint{ leftLeg.p3.x,leftLeg.p3.y + (bodyHeight - 20) };
	rightShin += tPoint{ rightLeg.p3.x,rightLeg.p3.y + (bodyHeight - 20) };

    drawTriangle(renderer, leftLeg + tPoint{ 0,(bodyHeight - 20) }, s);
	drawTriangle(renderer, leftShin , s);
	drawTriangle(renderer, head , s);
    SDL_Vertex verts[3] = {
        {body.p1.x + s.x,body.p1.y + s.y, {0, 0, 0, 255}, {0,0}},
        {body.p2.x + s.x,body.p2.y + s.y, {0, 0, 0, 255}, {0,0}},
        {body.p3.x + s.x,body.p3.y + s.y, {0, 0, 0, 255}, {0,0}}
    };
    SDL_RenderGeometry(renderer, NULL, verts, 3, NULL, 0);
	drawTriangle(renderer, rightArm + tPoint{ 0, 15 }, s);
    drawTriangle(renderer, rightForearm, s);
    drawTriangle(renderer, rightLeg + tPoint{ 0,(bodyHeight - 20) }, s);
	drawTriangle(renderer, rightShin, s);
    drawTriangle(renderer, body, s);    
}


int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL_Init error: %s\n", SDL_GetError());
        return 1;
    }
	float AngleSteps = __PI / 100.0;

    SDL_Window* window = SDL_CreateWindow("Running Stickman",
        WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);

    Stickman man = {WINDOW_WIDTH / 2, 300, 0.0f };

    int running = 1;
    SDL_Event e;
    while (running) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) running = 0;
        }

        man.angle1 += AngleSteps;       // advance animation
        man.angle2 -= AngleSteps;
        man.x +=2;            // move forward
        if (man.x > WINDOW_WIDTH) man.x = 0;
        if (man.angle1 > __PI / 2) {
            AngleSteps *=-1;
        }
		else if (man.angle1 < 0) {
            AngleSteps *= -1;
		}

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        drawStickman(renderer, man);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);           // ~60 FPS
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
