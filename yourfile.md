# gravity7

``` c
/*  For linux:  -lglut -lGL -lGLU -lm */
/*  For windows:  -lfreeglut -lopengl32 -lglu32 -lm */

#ifdef __APPLE__
#include <OpenGL/gl.h>		/* Header File For OpenGL Library */
#include <OpenGL/glu.h>		/* Header File For The GLU Library */
#include <GLUT/glut.h>        /* Header File For The GLUT Library */
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "gravity.h"

/* Define the window dimensions */
#define WIN_WIDTH	800
#define WIN_HEIGHT	800

/* Define the key to exit the system */
#define ESC	27

/* function prototypes */
void myDraw(void);
void myKey(unsigned char key,int x, int y);


// Simulation parameters (do not change)
#define VECTOR_DIM 3  // vector dimension
#define MAX_BODIES 10000  // maximum number of bodies supported
#define TRACEN 1000  // number of trajectory points to store


#define TIMER 20  // number of milliseconds for each time step (for glutTimerFunc)

const double timeStep = 0.001;  // in seconds
double zoom, gtime, year=0,tcount = 0;  // zoom factor and global time
float azi = 0, alt = 0;  // azimuth and altitude
const float dstep = 2;  // step-size for the azimuth and altitude
const double reps = 0.01;  // radius of each object
int traceon = 1, showstats = 1;  // flag variable for indicating whether trajectory trace is on or not


// Universe, containing everything
Universe uni ;


/* main program – execution begins here */
int main(int argc, char *argv[])
{
	int i, seed, res ;

	glutInit(&argc, argv); 				// initialise gl utility system
	glutInitWindowSize(WIN_WIDTH, WIN_HEIGHT); // define window size
	glutInitDisplayMode(GLUT_DOUBLE);
	//glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow ("[Insert your name here] gravity simulator"); 	// create window with title

	/* Register the callback functions to the current window */
	glutDisplayFunc(myDraw);  		// register function for drawing event
	glutKeyboardFunc(myKey);		// register function for keyboard event
	glutTimerFunc(TIMER, simulateGravity, 0);

	zoom = 1;  // initialise zoom to 1
	// set collision flags for each body to 0 (i.e. no collision)
	for (i = 0; i < MAX_BODIES; i++)
		uni.collided[i] = 0;


	// set the random seed
	seed = (int)time(NULL);
	srand(seed);

    res = loadUniverse ( &uni, "universe.txt");
    if ( res > 0 ){
        uni.fixed = 1 ;
    } else {
        uni.fixed = 0 ;
        uni.numBodies = 2 ;
    }

    initialise();  // initialise simulation

	/* Draw window and loop forever. Any keypress will call mykey() */
	glutMainLoop();

	/* program will never get here */
	return (0);
}

/* WEEK 1 FUNCTIONS - arrayZero, vectorDot and calculateGravity */

// zeros a vector
void arrayZero(double a[DIM])
{
	/* Write code here to change supplied array to contain all zeros */
	int i = 0;
	for (i=0;i<DIM;i++)
		{
		a[i] = 0;
		}
}


// performs a dot product of vector a and b
double vectorDot(double a[DIM], double b[DIM])
{
double dotProduct = 0;
int i = 0; //counter variable

	/* 	Write code here to perform a dot product between vectors a and b
		and return the result */

	for(i=0;i<DIM;i++)
		{
		dotProduct += (a[i]*b[i]);
		}
	return (dotProduct);

}

void newYear(void)
    {
    int i = 0, c=0;
    tcount += timeStep*10;
    if (tcount >= 5)
        {
        year += 1;
        if ((int)floor(gtime)%1 == 0)
            {
            for(i=0;i<uni.numBodies;i++)
                {
                c = uni.bodies[i].pop*0.014;
                c *= 1+(uni.bodies[i].popgrow/100);
                uni.bodies[i].pop += c;
                if (uni.bodies[i].pop > uni.bodies[i].maxpop)
                    {
                    uni.bodies[i].pop = uni.bodies[i].maxpop;
                    }
                }
            }
        tcount = 0;
        }
    }

void calculateGravity ( double m1, double m2, double pos1[DIM], double pos2[DIM], double G, double result[DIM]){

    /* Calculates the gravitations force between two objects with the given masses and positions.
    The gravitational constant is also supplied as an input, and the resulting force vector should
    be stored in the 'result' array.
    General algorithm:
    Calculate distance vector from object1 to object 2 by subtracting pos1 from pos2 (each dimension separately)
    Calculate the magnitude of this distance vector (this is 'r' in the gravity equation)
    Divide the distance vector by 'r' to get a unit vector in the same direction
    Calculate the magnitude of the gravitational force using the normal equation
    Multiply the unit vector you created earlier by the force magnitude, and store this in result.

    Most of these steps will require a loop running over all dimensions (3 times, but use DIM instead).
    */
	double distVect[DIM], distMag=0, gForce=0;
	int i = 0; //counter variable

	for(i=0;i<DIM;i++)
		{
		distVect[i] = pos2[i]-pos1[i];
		distMag += distVect[i]*distVect[i];
		}
	distMag = sqrt(distMag);
	for(i=0;i<DIM;i++)
		{
		distVect[i] = distVect[i]/distMag;
		}
	gForce = G*m1*m2/(distMag*distMag+0.25);
	for(i=0;i<DIM;i++)
		{
		result[i] = distVect[i]*gForce;
		}

    //used for showstats

    if (showstats > 1)
        {
        showstats -= .1;
        }

    if (showstats < 0)
        {
        showstats += .1;
        }
  // calculateGravityLib ( m1, m2, pos1, pos2, G, result);
}

/* END OF WEEK 1 FUNCTIONS */


/* WEEK 2 FUNCTIONS */

void simulateGravity(int data)
{
	int j=0,curBody=0,othBody=0;
	int i = 0; //counter variable

	double  netForce[DIM], result[DIM],dispVect[DIM], totRad = 0, dispMagn = 0;
	arrayZero(netForce);
	for(curBody=0;curBody<uni.numBodies;curBody++) //computes new accel, displacement and velocity
		{
		arrayZero(netForce);
		for(othBody=0;othBody<uni.numBodies;othBody++)
			{
			if (othBody != curBody)
				{
				calculateGravity(uni.bodies[curBody].mass,uni.bodies[othBody].mass,uni.bodies[curBody].position,uni.bodies[othBody].position,uni.G,result);
				for(j=0;j<DIM;j++)
					{
					netForce[j]+=result[j];
					}
				arrayZero(result);
				}
			else {}
			}

		for(j=0;j<DIM;j++)
			{

			uni.bodies[curBody].accel[j] = netForce[j]/uni.bodies[curBody].mass;
			}

		}
	for(curBody=0;curBody<uni.numBodies;curBody++)
		{
		for(i=0;i<DIM;i++)
			{
			uni.bodies[curBody].position[i] += (uni.bodies[curBody].velocity[i]*timeStep)+(.5*uni.bodies[curBody].accel[i]*(timeStep*timeStep));
			uni.bodies[curBody].velocity[i] += (uni.bodies[curBody].accel[i]*timeStep);

			}
		}

/*	for(curBody=0;curBody<uni.numBodies;curBody++)//checks collisions
		{
		for(i=0;i<uni.numBodies;i++)
			{
			if (i != curBody)
				{
				totRad = uni.bodies[curBody].radius+uni.bodies[i].radius;
				dispMagn = 0;
				for(j=0;j<DIM;j++)
					{
					dispVect[j] = uni.bodies[curBody].position[j] - uni.bodies[i].position[j];
					dispMagn += dispVect[j]*dispVect[j];
					}
				dispMagn = sqrt(dispMagn);
				if (dispMagn <= totRad && uni.collided[i] == 0)
					{
					//computeCollisionLib(uni.bodies,uni.bodies);
					uni.collided[i] = 1;
					uni.collided[curBody] = 1;
					uni.numCollisions += 1;
					}
				else
					{
					uni.collided[i] = 0;
					uni.collided[curBody] = 0;
					}
				}
			}
		}*/
/*
		You would also check for collisions here and call the computeCollision function if a collision
		between two objects does occur.

		==========================
		Algorithm:

		FOR each body
			FOR each body except current one
				compute the displacement vector between two bodies
				IF (diplacement < sum of radii of bodies) AND (body has not collided) THEN
					increment collisions
					set collided flag to TRUE for both bodies
				ELSE
					set collided flag to FALSE for both bodies
				END IF
			END FOR
		END FOR

		==========================
*/

    // this is the call to the library version of this function. Remove once you have written your own code.
	//simulateGravityLib(data);
	gtime += timeStep ;
	glutPostRedisplay();
	glutTimerFunc ( TIMER, simulateGravity, 0 ) ;
}

/* END OF WEEK 2 FUNCTIONS */


/* WEEK 3 FUNCTIONS */

int loadUniverse ( Universe *u, char filename[]){

	int c=0, j=0;
	int i = 0; //counter variable

	char buff[50];
	double  bodData[11];
	FILE* univ;
	arrayZero(bodData);
	printf("TEST");
	univ = fopen("universe.txt","r");
	uni.G = DEFAULT_G;
	if (univ == NULL)
		{
		printf("No saved file found");
		return(0);
		}
	while(fgets(buff,sizeof buff,univ)!= NULL)
		{
		if (buff[0] == 'g')
			{
			uni.G = .5;// buff[1]-48;
			}
		else
		if (buff[0] == '#'  || buff[0] == '\n' || buff[0] == '0')
			{
			continue;
			}
		else
			{
			sscanf(buff,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",&bodData[0],&bodData[1],&bodData[2],&bodData[3],&bodData[4],&bodData[5],&bodData[6],&bodData[7],&bodData[8],&bodData[9],&bodData[10]);
			if (bodData[8] > 0 || bodData[9] >0 || bodData[10]>0)
				{
				if (bodData[0] > 0 && bodData[1] > 0)
					{
					uni.bodies[c].mass = bodData[0];
					uni.bodies[c].radius = bodData[1];
					for(j=0;j<DIM;j++)
						{
						uni.bodies[c].position[j] = bodData[2+j];
						uni.bodies[c].velocity[j] = bodData[5+j];
						uni.bodies[c].colour[j] = bodData[8+j];
						}
					c += 1;
					arrayZero(bodData);
					}
				}
			}
		}
	fclose(univ);
	uni.numBodies += c;
	return(uni.numBodies);
//return loadUniverseLib ( u, filename ) ;

}


// initialise bodies at random locations within the "world of view"
void initialise(void)
{
   	int i = 0,j = 0;
	uni.numCollisions = 0;
	gtime = 0;
	year = 0;
	if (uni.fixed == 0)
		{
		uni.G = DEFAULT_G;
		for(i=0;i<uni.numBodies;i++)
			{
            uni.bodies[i].mass = ((float)rand()/(float)RAND_MAX)*10+.1;
            uni.bodies[i].radius = (uni.bodies[i].mass/1000);
            for(j=0;j<DIM;j++)
                {
                uni.bodies[i].position[j] = (((float)rand()/(float)RAND_MAX) -.5)*2;
                uni.bodies[i].colour[j] = ((float)rand()/(float)RAND_MAX)+.1;
                }
            genworld(i);
			//printf("%d: %lf %lf %lf %lf %lf\n",i,uni.bodies[i].mass,uni.bodies[i].radius,uni.bodies[i].colour[0],uni.bodies[i].colour[1],uni.bodies[i].colour[2]);
			}

            uni.bodies[uni.numBodies+1].mass = 100;
            uni.bodies[uni.numBodies+1].radius = .1;
            for(j=0;j<DIM;j++)
                {
                uni.bodies[uni.numBodies+1].position[j] = 0;
                uni.bodies[uni.numBodies+1].colour[j] = 0;
                }
            uni.bodies[uni.numBodies+1].colour[0] = 1;
            uni.bodies[uni.numBodies+1].colour[1] = 1;
            uni.bodies[uni.numBodies+1].type = 8;
            sscanf("Sun","%s",uni.bodies[uni.numBodies+1].typnm);
            uni.bodies[uni.numBodies+1].atm = -1;
            uni.bodies[uni.numBodies+1].savage = -1;
            uni.bodies[uni.numBodies+1].enviro = 1000;
            uni.bodies[uni.numBodies+1].pop = 0;
            uni.bodies[uni.numBodies+1].maxpop = 0;
            uni.numBodies += 1;
            }

	 //initialiseLib() ;


}

void genworld (int num)
    {
    uni.bodies[num].type = rand() % 7;
    switch(uni.bodies[num].type)
        {
        case 0: //arboreal
            {
            sscanf("Arboreal","%s",uni.bodies[num].typnm);
            uni.bodies[num].atm = rand() % 3;
            uni.bodies[num].enviro = rand() % 3;
            uni.bodies[num].savage = (rand() % 6)+4;
            uni.bodies[num].popgrow = (rand() % 10)+5;
            break;
            }
        case 1: //Oceanic
            {
            sscanf("Oceanic","%s",uni.bodies[num].typnm);
            uni.bodies[num].atm = (rand() % 5);
            uni.bodies[num].savage = (rand() % 10);
            uni.bodies[num].enviro = (rand() % 4)+1;
            uni.bodies[num].popgrow = (rand() % 10)-5;
            break;
            }
        case 2: //Desert
            {
            sscanf("Desert","%s",uni.bodies[num].typnm);
            uni.bodies[num].atm = (rand() % 4);
            uni.bodies[num].savage = (rand() % 6);
            uni.bodies[num].enviro = (rand() % 3)+2;
            uni.bodies[num].popgrow = (rand() % 5)-5;
            break;
            }
        case 3: //Primordial
            {
            sscanf("Primordial","%s",uni.bodies[num].typnm);
            uni.bodies[num].atm = (rand() % 3)+2;
            uni.bodies[num].savage = (rand() % 3);
            uni.bodies[num].enviro = (rand() % 1)+4;
            uni.bodies[num].popgrow = -20;
            break;
            }
        case 4: //Ruins
            {
            sscanf("Ruins","%s",uni.bodies[num].typnm);
            uni.bodies[num].atm = (rand() % 4)+1;
            uni.bodies[num].savage = (rand() % 8);
            uni.bodies[num].enviro = (rand() % 5);
            uni.bodies[num].popgrow = (rand() % 25)-5;
            break;
            }
        case 5: //Ice
            {
            sscanf("Ice","%s",uni.bodies[num].typnm);
            uni.bodies[num].atm = (rand() % 5);
            uni.bodies[num].savage = (rand() % 7);
            uni.bodies[num].enviro = (rand() % 5);
            uni.bodies[num].popgrow = (rand() % 5)-15;
            break;
            }
        case 6: //Terran
            {
            sscanf("Terran","%s",uni.bodies[num].typnm);
            uni.bodies[num].atm = (rand() % 2);
            uni.bodies[num].savage = (rand() % 6);
            uni.bodies[num].enviro = (rand() % 2);
            uni.bodies[num].popgrow = +10;
            break;
            }
        case 7: //Gas
            {
            sscanf("Gas","%s",uni.bodies[num].typnm);
            uni.bodies[num].atm = (rand() % 2)+3;
            uni.bodies[num].savage = (rand() % 2);
            uni.bodies[num].enviro = (rand() % 3)+2;
            uni.bodies[num].popgrow = -5;
            break;
            }
        }
    uni.bodies[num].maxpop = (100000*uni.bodies[num].radius)*(1-(uni.bodies[num].savage/10))*(1-((uni.bodies[num].atm*5)/100))*(1-((uni.bodies[num].enviro*5)/100));
    uni.bodies[num].pop = uni.bodies[num].maxpop/2;
    }

void myKey(unsigned char key, int x, int y)
{
	//myKeyLib(key, x, y);

	switch(key)
		{
		case 'a':
			{
			if (azi < 360)
				azi += dstep;
			else
				azi -= 360;
			break;
			}
		case 'd':
			{
			if (azi <0)
				azi += 360;
			else
				azi -= dstep;
			break;
			}
		case 'w':
			{
			if (alt > 360)
				alt -= 360;
			else
				alt += dstep;
			break;
			}
		case 's':
			{
			if (alt < 0)
				alt += 360;
			else
				alt -= dstep;
			break;
			}
		case 'r':
			{
			initialise();
			break;
			}
		case 'z':
			{
			zoom /= 0.9;
			break;
			}
		case 'x':
			{
			zoom *= 0.9;
			break;
			}
        case 'q':
			{
			if (showstats==0)
                {
                showstats = 2;
                }
            if (showstats == 1)
                {
                showstats = -1;
                }
            printf("%d\n",showstats);
			break;
			}
		case 'b':
			{
			if (uni.fixed == 0 && uni.numBodies < 1024)
				{
				uni.numBodies *= 2;
				initialise();
				}
			break;
			}
		case 'v':
			{
			if (uni.fixed == 0 && uni.numBodies > 2)
				{
				uni.numBodies /= 2;
				initialise();
				}
			break;
			}
		}

	// Uncomment the following to enable ESC key
		int win=0;
		if (key == ESC) {
		win=glutGetWindow() ; 		/* get identifier for current window */
		glutDestroyWindow(win) ; 	/* close the current window */
		exit(-1) ; 					/* leave the program */
	}
	}


/* END OF WEEK 3 FUNCTIONS */


/* WEEK 4 FUNCTIONS */

// compute collision and update velocities after collision


void computeCollision(Object *a, Object *b)
{
	/*	This function contains code for simulating perfectly elastic collisions.
		Compute and update the new velocities of these two objects right after the collision
	*//*
	int i = 0, vecMag = 0;
	double vec[DIM], adot, bdot;
	arrayZero(vec);

	for(i=0;i<DIM;i++)
		{
		vec[i] = uni.bodies[a].pos[i] - uni.bodies[b].pos[i];
		vecMag += vec[i]*vec[i];
		}
	vecMag = sqrt(vecMag);
	for(i=0;i<DIM;i++)
		{
		vec[i] /= vecMag;
		}
	adot = vectorDot(a.velocity,vec);
	bdot = vectorDot(b.velocity,vec);
	*/


	computeCollisionLib(a, b);
}


void myDraw(void)
{
	/* clear the screen */
	glClear(GL_COLOR_BUFFER_BIT);

	/* Put your drawing code in here */

	//myDrawLib();

    char buffer[256];
    int i = 0, bodyNum=0;

    glLoadIdentity();
    glPushMatrix();
    glColor3f(1,1,1);
    glRasterPos2f(-0.99,-0.99);
    sprintf(buffer,"Number of collisions: %d",uni.numCollisions);
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);

    glRasterPos2f(-0.99,-0.75);
    sprintf(buffer,"Number of bodies (-v/+b): %d",uni.numBodies);
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);

    glRasterPos2f(-0.99,-0.70);
    sprintf(buffer,"Azimuth (-a/+d): %f Altitude (-s/+w): %f Zoom (-z/+x): %lf",azi, alt, zoom);
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);

    glRasterPos2f(-0.99,-0.65);
    sprintf(buffer,"Year: %.0lf",year);
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);

    glPopMatrix();

    glRotatef(azi,0,1,0);
    glRotatef(alt,1,0,0);

    for(i=0;i<uni.numBodies;i++)
        {
        if (traceon ==1)
            {
            //NOTHIIIIIIIING
            }
        if (showstats == 1)
            {
            glPushMatrix();
                glRasterPos3f(uni.bodies[i].position[0] * zoom +uni.bodies[i].radius + 0.03, uni.bodies[i].position[1] * zoom+uni.bodies[i].radius  + 0.03, uni.bodies[i].position[2] * zoom+uni.bodies[i].radius  + 0.03);
                glColor3f(uni.bodies[i].colour[0] + 0.2, uni.bodies[i].colour[1] + 0.2, uni.bodies[i].colour[2] + 0.2);
                sprintf(buffer,"Pop: %.0lfk/%.0lfk\nType: %s\nSavage: %d\nAtm: %d\nEnviro: %d\nDiameter: %.0lfkm",uni.bodies[i].pop,uni.bodies[i].maxpop,uni.bodies[i].typnm,uni.bodies[i].savage,uni.bodies[i].atm,uni.bodies[i].enviro,uni.bodies[i].radius*2*100000);
                glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);
            glPopMatrix();
            }
        glPushMatrix();
        glTranslatef(uni.bodies[i].position[0] *zoom,uni.bodies[i].position[1] *zoom,uni.bodies[i].position[2] *zoom);
        glColor3f(uni.bodies[i].colour[0],uni.bodies[i].colour[1],uni.bodies[i].colour[2]);
        glutSolidSphere(uni.bodies[i].radius*zoom,10,10);
        glPopMatrix();
        }



    glPushMatrix();
    glColor3f(1,1,1);
    glutWireCube(zoom);
    glBegin(GL_LINES);
        glColor3f(1,0,0);
        glVertex3f(-0.5*zoom,-0.5*zoom,-0.5*zoom);
        glVertex3f(-0.2*zoom,-0.5*zoom,-0.5*zoom);
    glEnd();

    glBegin(GL_LINES);
        glColor3f(0,1,0);
        glVertex3f(-0.5*zoom,-0.5*zoom,-0.5*zoom);
        glVertex3f(-0.5*zoom,-0.2*zoom,-0.5*zoom);
    glEnd();

    glBegin(GL_LINES);
        glColor3f(0,0,1);
        glVertex3f(-0.5*zoom,-0.5*zoom,-0.5*zoom);
        glVertex3f(-0.5*zoom,-0.5*zoom,-0.2*zoom);
    glEnd();

    glPopMatrix();
    glutSwapBuffers();

	/* You will need to do a number of things in this function

	- print the simulation parameters as text in the screen (Hint: sprintf is your friend)
	- print instructions on what keys to press to rotate, increase bodies, zoom, etc.
	- draw each body as a sphere using the stored colour and x,y,z coordinate positions.
	  These are stored in the global bodies[] array.
	- plot the trajectories from the global trace[] array

	*/
	newYear();
}

/* ADDITIONAL FUNCTIONS PROVIDED FOR YOU */

/* Use the following function to debug your code and print out an array to the screen
*/
void arrayPrint(double a[DIM])
{
	int i;

	printf("[ ");

	for (i = 0; i < DIM; i++)
		printf("%lg, ", a[i]);

	printf(" ] ");
}


```
