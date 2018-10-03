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
int traceon = 1, showstats = 1, click = 0, gravtype = 0;  // flag variable for indicating whether trajectory trace is on or not


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

	if (gravtype == 0)
		{
		glutTimerFunc(TIMER, simulateGravity, 0);
		}
	else
	if (gravtype == 1)
		{
		//glutTimerFunc(TIMER,simulateGravityNewt,0);
		}

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
        uni.numBodies = 7 ;
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
    printf("NewYear\n");
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
                nameSav(i,uni.bodies[i].savNm);
				nameAtmo(i,uni.bodies[i].atmNm);
				nameEnviro(i,uni.bodies[i].envNm);
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

printf("CalGrav\n");
	double distVect[DIM], distMag=0, gForce=0;
	int i = 0; //counter variable
    arrayZero(result);
    arrayZero(distVect);
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
  // calculateGravityLib ( m1, m2, pos1, pos2, G, result);
}

void simulateGravity(int data)
	{
	printf("SimGrav\n");
	int i=0;
	for(i=1;i<uni.numBodies;i++)
		{
		uni.bodies[i].position[0] = cos(uni.bodies[i].ang) * uni.bodies[i].dist;
		uni.bodies[i].position[1] = sin(uni.bodies[i].ang) * uni.bodies[i].dist;
		uni.bodies[i].ang += ((1-uni.bodies[i].dist/5)+(uni.bodies[i].mass/1000))/300;
		if (uni.bodies[i].ang >=6.28)
			{
			uni.bodies[i].ang -= 6.28;
			}
		}
	for(i=0;i<uni.numMoons;i++)
		{
		uni.moons[i].position[0] = uni.bodies[uni.moons[i].home].position[0]+cos(uni.moons[i].ang) * uni.moons[i].dist;
		uni.moons[i].position[1] = uni.bodies[uni.moons[i].home].position[1]+sin(uni.moons[i].ang) * uni.moons[i].dist;
		uni.moons[i].ang += ((1-uni.moons[i].dist/5)+(uni.moons[i].mass/1000))/50;
		if (uni.moons[i].ang >=6.28)
			{
			uni.moons[i].ang -= 6.28;
			}
		}

	gtime += timeStep ;
	glutPostRedisplay();
	glutTimerFunc ( TIMER, simulateGravity, 0 ) ;
	}


void simulateGravityNewt(int data)
{
printf("SimGravNew\n");
	int j=0,i=0,curBody=0,othBody=0;
	double  netForce[DIM], result[DIM];
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
	for(i=0;i<uni.numMoons;i++)
		{
		uni.moons[i].position[0] = uni.bodies[uni.moons[i].home].position[0]+cos(uni.moons[i].ang) * uni.moons[i].dist;
		uni.moons[i].position[1] = uni.bodies[uni.moons[i].home].position[1]+sin(uni.moons[i].ang) * uni.moons[i].dist;
		uni.moons[i].ang += ((1-uni.moons[i].dist/5)+(uni.moons[i].mass/1000))/10;
		if (uni.moons[i].ang >=6.28)
			{
			uni.moons[i].ang -= 6.28;
			}
		}
	gtime += timeStep ;
	glutPostRedisplay();
	glutTimerFunc ( TIMER, simulateGravityNewt, 0 ) ;
		}


int loadUniverse ( Universe *u, char filename[]){

	int c=0, j=0;

	char buff[50];
	double  bodData[11];
	FILE* univ;
	arrayZero(bodData);
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
	printf("Initialise\n");
   	int i = 0,j = 0, dist=0;
   	double res[DIM], iniForce[DIM];
	uni.numCollisions = 0;
	gtime = 0;
	year = 0;
	if (uni.fixed == 0)
		{
		for(i=0;i<(uni.numBodies-1);i++)
			{
			uni.buttons[i].selected = 0;
			}
		uni.buttons[0].selected = 1;
		uni.G = DEFAULT_G;
		for(i=0;i<uni.numBodies;i++)
			{
			uni.bodies[i].mass = ((float)rand()/(float)RAND_MAX)*20+.1;
            uni.bodies[i].radius = (uni.bodies[i].mass/100);
            for(j=0;j<DIM;j++)
                {
                uni.bodies[i].colour[j] = ((float)rand()/(float)RAND_MAX)+.1;
                }
			uni.bodies[i].position[0] = .25+(.25*i);
			uni.bodies[i].position[1] = .25+(.25*i);
			uni.bodies[i].position[2] = 0;
			uni.bodies[i].dist = 0;
			uni.bodies[i].ang = rand() % 6;
            for(j=0;j<DIM;j++)
				{
				uni.bodies[i].dist += pow(uni.bodies[i].position[j],2);
				}
			uni.bodies[i].dist = sqrt(uni.bodies[i].dist);
            genworld(i);
			}

            uni.bodies[0].mass = 2000;
            uni.bodies[0].radius = (uni.bodies[0].mass/5000);
            for(j=0;j<DIM;j++)
                {
                uni.bodies[0].position[j] = 0;
                uni.bodies[0].colour[j] = 1;
                }
            sscanf("Sun","%s",uni.bodies[0].typnm);

			for(i=1;i<uni.numBodies;i++)
				{
				uni.bodies[0].mass += (uni.bodies[i].mass*1000); //ensures Star mass is much greater than every planets combined
				//printf("Plan%d\nPosX: %lf PosY: %lf PosZ: %lf\n",i,uni.bodies[i].position[0],uni.bodies[i].position[1],uni.bodies[i].position[2]);
				genMoons(i); //generate moons
				}
			for(i=0;i<uni.numMoons;i++)
				{
				uni.bodies[uni.moons[i].home].moonnum ++;
				}

            for(i=1;i<uni.numBodies;i++)
                {
                arrayZero(res);
                arrayZero(iniForce);
				for(j=0;j<DIM;j++)
					{
					dist += pow(uni.bodies[i].position[j]-uni.bodies[0].position[j],2);
					}
				dist = sqrt(dist)/2; //gives us the radius of the orbit
				uni.bodies[i].velocity[2] = (sqrt((uni.G*uni.bodies[0].mass)));
                }
        }
    }

	 //initialiseLib() ;

void genworld (int num)
    {
    printf("Worlds\n");
    uni.bodies[num].moonnum = 0;
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
            uni.bodies[num].atm = (rand() 	% 2)+3;
            uni.bodies[num].savage = (rand() % 2);
            uni.bodies[num].enviro = (rand() % 3)+2;
            uni.bodies[num].popgrow = -5;
            break;
            }
        }
    uni.bodies[num].maxpop = (100000*uni.bodies[num].radius)*(1-(uni.bodies[num].savage/10))*(1-((uni.bodies[num].atm*5)/100))*(1-((uni.bodies[num].enviro*5)/100));
    uni.bodies[num].pop = rand() % (int)uni.bodies[num].maxpop;
    nameSav(num,uni.bodies[num].savNm);
    nameAtmo(num,uni.bodies[num].atmNm);
	nameEnviro(num,uni.bodies[num].envNm);
	}

void genMoons (num)
	{
	printf("Moons\n");
	int cc=0, j=0, i =0;
	double dist=0;
	cc = rand()%3;
	if (cc > 0)
		{
		for(i=0;i<cc;i++)
			{
			uni.moons[uni.numMoons].position[0] = uni.bodies[num].position[0]+(uni.bodies[num].position[0]*.15);
			uni.moons[uni.numMoons].position[1] = uni.bodies[num].position[1]+(uni.bodies[num].position[1]*.15);
			uni.moons[uni.numMoons].position[2] = uni.bodies[num].position[2];
			//printf("Moon%d\nPosX: %lf PosY: %lf PosZ: %lf\n",uni.numMoons,uni.moons[uni.numMoons].position[0],uni.moons[uni.numMoons].position[1],uni.moons[uni.numMoons].position[2]);
			uni.moons[uni.numMoons].ang = rand()%6;
			uni.moons[uni.numMoons].mass = ((float)rand()/(float)RAND_MAX)*2+.1;
			uni.moons[uni.numMoons].radius = (uni.moons[uni.numMoons].mass/100);
			uni.moons[uni.numMoons].home = num;
			for(j=0;j<DIM;j++)
				{
				uni.moons[uni.numMoons].colour[j] = rand()/RAND_MAX;
				dist += pow(uni.moons[uni.numMoons].position[j]-uni.bodies[num].position[j],2);
				//printf("Moon%d Pos%d %lf Dist %lf\n",uni.numMoons,j,uni.moons[uni.numMoons].position[j],dist);
				}
			if (uni.moons[uni.numMoons].colour[0]+uni.moons[uni.numMoons].colour[1]+uni.moons[uni.numMoons].colour[2] == 0)
				{
				uni.moons[uni.numMoons].colour[(rand()%2)] = 1;
				}
			uni.moons[uni.numMoons].dist = sqrt(dist)/2+uni.bodies[uni.moons[uni.numMoons].home].radius;

			uni.numMoons ++;
			}
		}
    }

void myKey(unsigned char key, int x, int y)
{
printf("Keys\n");
	//myKeyLib(key, x, y);
	int i=0,j=0;
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
			break;
			}
		/*case 'g':
			{
			if (gravtype==0)
				{
				gravtype = 2;
				}
			if (gravtype == 1)
				{
				for(i=0;i<uni.numBodies;i++)
					{
					for(j=0;j<DIM;j++)
						{
						uni.bodies[i].dist += pow(uni.bodies[i].position[j],2);
						}
					uni.bodies[i].dist = sqrt(uni.bodies[i].dist);
					}
				gravtype = -1;
				}
			break;
			}*/
		case 'b':
			{
			if (uni.fixed == 0 && uni.numBodies < 1024)
				{
				//uni.numBodies *= 2;
				//initialise();
				}
			break;
			}
		case 'v':
			{
			if (uni.fixed == 0 && uni.numBodies > 2)
				{
				//uni.numBodies /= 2;
				//initialise();
				}
			break;
			}
		case 'o':
			{
			for(i=1;i<(uni.numBodies-1);i++)
				{
				if (uni.buttons[i].selected == 1 && click == 0)
					{
					uni.buttons[i].selected = 0;
					uni.buttons[i-1].selected = 1;
					click = 2;
					}
				}
			break;
			}
		case 'l':
			{
			for(i=0;i<(uni.numBodies-2);i++)
				{
				if (uni.buttons[i].selected == 1 && click == 0)
					{
					uni.buttons[i].selected = 0;
					uni.buttons[i+1].selected = 1;
					click = 2;
					}
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

    if (showstats > 1)
        {
        showstats -= .1;
        }

    if (showstats < 0)
        {
        showstats += .1;
        }

	if (gravtype > 1)
        {
        gravtype -= .1;
        }

    if (gravtype < 0)
        {
        gravtype += .1;
        }

	if (click > 0)
		{
		click -= .1;
		}


    char buffer[256];
    int i = 0;

    glLoadIdentity();
    glPushMatrix();
    glColor3f(1,1,1);
    glRasterPos2f(-0.99,-0.75);
    //sprintf(buffer,"Number of collisions: %d",uni.numCollisions);
    sprintf(buffer,"o and l to navigate planets");
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);
    glRasterPos2f(-0.99,-0.8);
    sprintf(buffer,"g to change gravity types");
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);

    glRasterPos2f(-0.99,-0.70);
    sprintf(buffer,"Number of bodies (-v/+b): %d",uni.numBodies);
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);

    glRasterPos2f(-0.99,-0.65);
    sprintf(buffer,"Azimuth (-a/+d): %.0f Altitude (-s/+w): %.0f Zoom (-z/+x): %lf",azi, alt, zoom);
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);

    glRasterPos2f(-0.99,-0.60);
    sprintf(buffer,"Year: %.0lf",year);
    glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);

    for(i=0;i<(uni.numBodies-1);i++)
		{
		glColor3f(1,1,1);
		if (uni.buttons[i].selected == 1)
			{
			glRasterPos2f(0.45,.95);
            glColor3f(1,1,1);
			//sprintf(buffer,"Planet %d\nPop: %.0lfk/%.0lfk\nType: %s\nSavage: %s\nAtm: %s\nEnviro: %s\nDiameter: %.0lfkm",i+1,uni.bodies[i+1].pop,uni.bodies[i+1].maxpop,uni.bodies[i+1].typnm,uni.bodies[i+1].savage,uni.bodies[i+1].atm,uni.bodies[i+1].enviro,uni.bodies[i+1].radius*2*100000);
			sprintf(buffer,"Planet %d\nPop: %.0lfk/%.0lfk\nType: %s\nSavage: %s\nAtm: %s\nEnviro: %s\nDiameter: %.0lfkm\nMoons: %d\n",i+1,uni.bodies[i+1].pop,uni.bodies[i+1].maxpop,uni.bodies[i+1].typnm,uni.bodies[i+1].savNm,uni.bodies[i+1].atmNm,uni.bodies[i+1].envNm,uni.bodies[i+1].radius*2*100000,uni.bodies[i].moonnum);
			glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);
			glColor3f(0,0,1);
			}
		glRasterPos2f(-0.99,.9-(.1*i));
		sprintf(buffer,"Planet %d",i+1);
		glutBitmapString(GLUT_BITMAP_8_BY_13,buffer);
		}
	glColor3f(1,1,1);
    glPopMatrix();

    glRotatef(azi,0,1,0);
    glRotatef(alt,1,0,0);

	for(i=0;i<uni.numBodies;i++)
		{
        glPushMatrix();
        glTranslatef(uni.bodies[i].position[0] *zoom,uni.bodies[i].position[1] *zoom,uni.bodies[i].position[2] *zoom);
        glColor3f(uni.bodies[i].colour[0],uni.bodies[i].colour[1],uni.bodies[i].colour[2]);
        glutSolidSphere(uni.bodies[i].radius*zoom,10,10);
        if (uni.buttons[i-1].selected == 1)
			{
			glColor3f(0,0,1);
			glutWireSphere((uni.bodies[i].radius)*1.25,4,4);
			}
        glPopMatrix();
        }
	for(i=0;i<uni.numMoons;i++)
		{
        glPushMatrix();
        glTranslatef(uni.moons[i].position[0] *zoom,uni.moons[i].position[1] *zoom,uni.moons[i].position[2] *zoom);
        glColor3f(uni.moons[i].colour[0],uni.moons[i].colour[1],uni.moons[i].colour[2]);
        glutSolidSphere(uni.moons[i].radius*zoom,10,10);
        glPopMatrix();
        }
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

	newYear();
}

void nameSav (int plan, char savNm[20])
	{
	if (uni.bodies[plan].savage <= 1)
		{
		sscanf("Peaceful","%s",savNm);
		}
	if (uni.bodies[plan].savage > 1)
		{
		sscanf("Earthlike","%s",savNm);
		}
	if (uni.bodies[plan].savage >= 4)
		{
		sscanf("Wild","%s",savNm);
		}
	if (uni.bodies[plan].savage >= 6)
		{
		sscanf("Dangerous","%s",savNm);
		}
	if (uni.bodies[plan].savage >= 8)
		{
		sscanf("Predatory","%s",savNm);
		}
	if (uni.bodies[plan].savage == 10)
		{
		sscanf("Extreme","%s",savNm);
		}
	}

void nameAtmo (int plan, char atmNm[20])
	{
	if (uni.bodies[plan].atm <= 0)
		{
		sscanf("Earth Air","%s",atmNm);
		}
	if (uni.bodies[plan].atm >= 1)
		{
		sscanf("Uncomfortable","%s",atmNm);
		}
	if (uni.bodies[plan].atm >= 3)
		{
		sscanf("Unhealthy","%s",atmNm);
		}
	if (uni.bodies[plan].atm == 5)
		{
		sscanf("Poison","%s",atmNm);
		}
	}

void nameEnviro (int plan, char envNm[20])
	{
	if (uni.bodies[plan].enviro <= 1)
		{
		sscanf("Eden-like","%s",envNm);
		}
	if (uni.bodies[plan].enviro >= 2)
		{
		sscanf("Earth-like","%s",envNm);
		}
	if (uni.bodies[plan].enviro == 4)
		{
		sscanf("Dangerous","%s",envNm);
		}
	if (uni.bodies[plan].enviro >= 5)
		{
		sscanf("Deadly","%s",envNm);
		}
	}
