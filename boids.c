#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/ioctl.h>
#include <time.h>

#define SLEEP_TIME 100000

#define NUM_BOIDS 100

int WINDOW_HEIGHT;
int WINDOW_WIDTH;

typedef struct Boid
{
    float x;
    float y;
    float vx;
    float vy;
    float ax;
    float ay;
} Boid;

Boid boids[NUM_BOIDS];

void updateBoids()
{  
    // Update the position and velocity of each boid based on the rules
    // of the simulation
    for (int i = 0; i < NUM_BOIDS; i++)
    {
        Boid *b = &boids[i];

        // Initialize acceleration to zero
        b->ax = 0.0f;
        b->ay = 0.0f;

        // Calculate the center of mass of the group
        float cx = 0.0f, cy = 0.0f;
        for (int j = 0; j < NUM_BOIDS; j++)
        {
            if (j != i)
            {
                cx += boids[j].x;
                cy += boids[j].y;
            }
        }
        cx /= NUM_BOIDS - 1;
        cy /= NUM_BOIDS - 1;

        // Rule 1: Cohesion
        // Boids should try to move towards the center of mass of the group
        b->ax += (cx - b->x) / 100.0f;
        b->ay += (cy - b->y) / 100.0f;

        // Rule 2: Separation
        // Boids should try to avoid colliding with other boids
        for (int j = 0; j < NUM_BOIDS; j++)
        {
            if (j != i)
            {
                Boid *other = &boids[j];
                float dx = b->x - other->x;
                float dy = b->y - other->y;

                float dist = sqrt(dx * dx + dy * dy) + 0.01;

                if (dist < 20.0f)
                {
                    // Too close! Flee!
                    b->ax -= dx / (dist * 4.0f);
                    b->ay -= dy / (dist * 4.0f);
                }
            }
        }

        // Rule 3: Alignment
        // Boids should try to match the velocity of the other boids in the group
        float vx = 0.0f, vy = 0.0f;
        for (int j = 0; j < NUM_BOIDS; j++)
        {
            if (j != i)
            {
                vx += boids[j].vx;
                vy += boids[j].vy;
            }
        }

        vx /= NUM_BOIDS - 1;
        vy /= NUM_BOIDS - 1;
        b->ax += (vx - b->vx) / 8.0f;
        b->ay += (vy - b->vy) / 8.0f;

        b->ax -= b->vx * 0.3;
        b->ay -= b->vy * 0.3;

        float distToLeftEdge = b->x;
        float distToRightEdge = WINDOW_WIDTH - b->x;
        float distToTopEdge = b->y;
        float distToBottomEdge = WINDOW_HEIGHT - b->y;

        float minDistToEdge = fmin(distToLeftEdge, fmin(distToRightEdge, fmin(distToTopEdge, distToBottomEdge)));

        if (minDistToEdge < 50)
        {
            // Boid is too close to an edge, add a force to move it away from the edge
            if (minDistToEdge == distToLeftEdge)
            {
                b->ax += 5;
            }
            else if (minDistToEdge == distToRightEdge)
            {
                b->ax -= 5;
            }
            if (minDistToEdge == distToTopEdge)
            {
                b->ay += 5;
            }
            else if (minDistToEdge == distToBottomEdge)
            {
                b->ay -= 5;
            }
        }

        // Update velocity and position based on acceleration
        b->vx += b->ax;
        b->vy += b->ay;
        b->x += b->vx;
        b->y += b->vy;

        // Check for collisions with the edge of the window
        if (b->x < 0)
        {
            // Collision with left edge of the window, set position to the right edge
            b->x = WINDOW_WIDTH - 1;
        }
        else if (b->x > WINDOW_WIDTH)
        {
            // Collision with right edge of the window, set position to the left edge
            b->x = 1;
        }
        if (b->y < 0)
        {
            // Collision with top edge of the window, set position to the bottom edge
            b->y = WINDOW_HEIGHT - 1;
        }
        else if (b->y > WINDOW_HEIGHT)
        {
            // Collision with bottom edge of the window, set position to the top edge
            b->y = 1;
        }
        
        // printf("%f\n", b->vx);
        // printf("%f\n", b->vy);
        
    }

    
    usleep(SLEEP_TIME);
}

void renderBoids()
{

    system("clear");

    // Render each boid as a character on the terminal
    for (int i = 0; i < NUM_BOIDS; i++)
    {
        Boid *b = &boids[i];
        int x = (int)b->x;
        int y = (int)b->y;

        // Move the cursor to the correct position
        printf("\033[%d;%dH", y, x);

        // Print the boid character
        fputc('*', stdout);

        fflush(stdout);
    }

    usleep(SLEEP_TIME);
}




int main(int argc, char *argv[])
{
    srand(time(0));

    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

    WINDOW_WIDTH = w.ws_col;
    WINDOW_HEIGHT = w.ws_row;

    // Initialize the boids
    for (int i = 0; i < NUM_BOIDS; i++)
    {
        boids[i].x = rand() % WINDOW_WIDTH;
        boids[i].y = rand() % WINDOW_HEIGHT;
        boids[i].vx = 0.0f;
        boids[i].vy = 0.0f;
        boids[i].ax = 0.0f;
        boids[i].ay = 0.0f;
    }

    while (1)
    {
        // Update the boids
        updateBoids();
        // Render the boids
        renderBoids();
        
    }



    return 0;
}


