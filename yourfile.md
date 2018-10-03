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

const double timeStep = 0.001;  // in seconds, the rate at which things happen
double zoom, gtime, year=0,tcount = 0;  // zoom factor. global time, current year and a time counter for the year
float azi = 0, alt = 0;  // azimuth and altitude
const float dstep = 2;  // step-size for the azimuth and altitude
const double reps = 0.01;  // radius of each object
int traceon = 1, ;  // flag variable for indicating whether trajectory trace is on or not
int click = 0; //clicker varuiable to ensure things don't happen all at once

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
//Increases tcount until it's hit the year point (>=5), at which point populations are updated
//also updates the planets terrain descriptions in case of changes (changes not implemented)
void newYear(void)
	{
	int i = 0, c=0;
	tcount += timeStep*10;
	if (tcount >= 5)
		{
		year += 1;
		for(i=0;i<uni.numBodies;i++)
			{
			c = uni.bodies[i].pop*0.014; //sets a base pop increase of 1.4% of population
			c *= 1+(uni.bodies[i].popgrow/100); //adjusts 'c' to include the population modification of the planet
			uni.bodies[i].pop += c; //increases population
			nameSav(i,uni.bodies[i].savNm);
			nameAtmo(i,uni.bodies[i].atmNm);
			nameEnviro(i,uni.bodies[i].envNm); //re-updates planetary terrain descriptors
                		if (uni.bodies[i].pop > uni.bodies[i].maxpop)
                    		{
                    		uni.bodies[i].pop = uni.bodies[i].maxpop; //keeps the pop from going over the maximum allowed
                    		}
                	}
            	}
        tcount = 0;
        }
    }

void calculateGravity ( double m1, double m2, double pos1[DIM], double pos2[DIM], double G, double result[DIM])
	{
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
//a new version of simulateGravity, forcing planets and moons into circular orbits
void simulateGravity(int data)
	{
	int i=0;
	for(i=1;i<uni.numBodies;i++)
		{
		uni.bodies[i].position[0] = cos(uni.bodies[i].ang) * uni.bodies[i].dist;
		uni.bodies[i].position[1] = sin(uni.bodies[i].ang) * uni.bodies[i].dist; //works out the new position based on their angle from the centre
		uni.bodies[i].ang += ((1-uni.bodies[i].dist/5)+(uni.bodies[i].mass/1000))/300; //increases angle based on distance from the sun and the mass of the planet
		if (uni.bodies[i].ang >=6.28)
			{
			uni.bodies[i].ang -= 6.28; //resets down below 2pi
			}
		}
	for(i=0;i<uni.numMoons;i++) //same as above, but for moons. Main difference is the addition of the homeworlds position - as they don't orbit the centre(sun), we need to adjust their position by their homeworlds
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

//original, newtonian style gravity
void simulateGravityNewt(int data)
{
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


// initialise planets, moons and the sun
void initialise(void)
	{
	int i = 0,j = 0, dist=0;
   	double res[DIM], iniForce[DIM];
	uni.numCollisions = 0;
	gtime = 0;
	year = 0; //resets core variables
	if (uni.fixed == 0)
		{
		for(i=0;i<(uni.numBodies-1);i++)
			{
			uni.buttons[i].selected = 0; //creates the side panel menu buttons. uni.numBodies-1 is due to the sun, and not needing a button for it
			}
		uni.buttons[0].selected = 1; //auto-selects the first menu button
		uni.G = DEFAULT_G;
		for(i=0;i<uni.numBodies;i++)
			{
			uni.bodies[i].mass = ((float)rand()/(float)RAND_MAX)*20+.1;
			uni.bodies[i].radius = (uni.bodies[i].mass/100); //gives the planets mass and radius
			for(j=0;j<DIM;j++)
		                {
		                uni.bodies[i].colour[j] = ((float)rand()/(float)RAND_MAX)+.1; //sets the planets colour
                		}
			uni.bodies[i].position[0] = .25+(.25*i);
			uni.bodies[i].position[1] = .25+(.25*i); //position is set from the sun according to which number this planet is
			uni.bodies[i].position[2] = 0;
			uni.bodies[i].dist = 0;
			uni.bodies[i].ang = rand() % 6; //ensures that all planets don't start in line with each other
            		for(j=0;j<DIM;j++)
				{
				uni.bodies[i].dist += pow(uni.bodies[i].position[j],2); //works out the distance between the planet and the sun
				}
			uni.bodies[i].dist = sqrt(uni.bodies[i].dist);
            		genworld(i); //generates the worlds other stats
			}
	uni.bodies[0].mass = 2000; //sets the suns base mass
	uni.bodies[0].radius = (uni.bodies[0].mass/5000); //sets the suns radius
	for(j=0;j<DIM;j++)
		{
		uni.bodies[0].position[j] = 0;
                uni.bodies[0].colour[j] = 1; //suns colour
                }
	sscanf("Sun","%s",uni.bodies[0].typnm); //name
	for(i=1;i<uni.numBodies;i++)
		{
		uni.bodies[0].mass += (uni.bodies[i].mass*1000); //ensures Star mass is much greater than every planets combined
		genMoons(i); //generate moons
		}
		for(i=0;i<uni.numMoons;i++)
			{
			uni.bodies[uni.moons[i].home].moonnum ++; //ensures each planet has an accurate count of moons
			}
		//this function works out all the distances for the planets from the sun
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

//This generates the worlds terrain data
void genworld (int num)
	{
	uni.bodies[num].moonnum = 0;
	uni.bodies[num].type = rand() % 7;
	//after getting a random type, these case statements set the name of the planet type, the atmosphere, environment, savagery and population growth
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
	uni.bodies[num].maxpop = (100000*uni.bodies[num].radius)*(1-(uni.bodies[num].savage/10))*(1-((uni.bodies[num].atm*5)/100))*(1-((uni.bodies[num].enviro*5)/100)); //works out max population based on the size of the planet, the savagery, the atmosphere and environment
	uni.bodies[num].pop = rand() % (int)uni.bodies[num].maxpop; //sets a starting population
	nameSav(num,uni.bodies[num].savNm);
	nameAtmo(num,uni.bodies[num].atmNm);
	nameEnviro(num,uni.bodies[num].envNm); //this works out the descripors for the planets terrain types
	}

//generates moons for each planet
void genMoons (num)
	{
	int cc=0, j=0, i =0;
	double dist=0;
	cc = rand()%3; //chooses a number of moons up to 3
	if (cc > 0)
		{
		for(i=0;i<cc;i++)
			{
			uni.moons[uni.numMoons].position[0] = uni.bodies[num].position[0]+(uni.bodies[num].position[0]*.15);
			uni.moons[uni.numMoons].position[1] = uni.bodies[num].position[1]+(uni.bodies[num].position[1]*.15); //sets the position in relation to the homeworld
			uni.moons[uni.numMoons].position[2] = uni.bodies[num].position[2];
			uni.moons[uni.numMoons].ang = rand()%6;
			uni.moons[uni.numMoons].mass = ((float)rand()/(float)RAND_MAX)*2+.1;
			uni.moons[uni.numMoons].radius = (uni.moons[uni.numMoons].mass/100); //sets the starting angle, random mass, and radius accordingly
			uni.moons[uni.numMoons].home = num; //sets the homeworld for the moon
			for(j=0;j<DIM;j++)
				{
				uni.moons[uni.numMoons].colour[j] = rand()/RAND_MAX;
				dist += pow(uni.moons[uni.numMoons].position[j]-uni.bodies[num].position[j],2);
				//sets the moons colour and distance from home planet
				}
			if (uni.moons[uni.numMoons].colour[0]+uni.moons[uni.numMoons].colour[1]+uni.moons[uni.numMoons].colour[2] == 0)
				{
				uni.moons[uni.numMoons].colour[(rand()%2)] = 1; //ensures the moon is never black
				}
			uni.moons[uni.numMoons].dist = sqrt(dist)/2+uni.bodies[uni.moons[uni.numMoons].home].radius; //works out the actual distance from the homeworld and assigns it

			uni.numMoons ++; //increases number of moons
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
		case 'o': //goes up the menu tree
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
		case 'l': //goes down the menu tree
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

	if (click > 0)
		{
		click -= .1; //keeps click from allowing buttons to repeat dozens of times per timestep
		}


    char buffer[256];
    int i = 0;

    glLoadIdentity();
    glPushMatrix();
    glColor3f(1,1,1);
    glRasterPos2f(-0.99,-0.75);
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
//These three functions take the terrain rating of the planet and work out its corresponding description
void nameSav (int plan, char savNm[20])
	{
	sscanf("Peaceful","%s",savNm);
	
	if (uni.bodies[plan].savage >= 2)
		{
		sscanf("Earthlike","%s",savNm);
		}
	if (uni.bodies[plan].savage == 4)
		{
		sscanf("Wild","%s",savNm);
		}
	if (uni.bodies[plan].savage >= 5)
		{
		sscanf("Dangerous","%s",savNm);
		}		
	if (uni.bodies[plan].savage >= 7)
		{
		sscanf("Predatory","%s",savNm);
		}
	if (uni.bodies[plan].savage >= 9)
		{
		sscanf("Extreme","%s",savNm);
		}
	}

void nameAtmo (int plan, char atmNm[20])
	{
	switch(uni.bodies[plan].atm)
		{
		case 0:
			{
			sscanf("Healthy","%s",atmNm);
			break;
			}
		case 1:
			{
			sscanf("Earth Standard","%s",atmNm);
			break;
			}
		case 2:
			{
			sscanf("Survivable","%s",atmNm);
			break;
			}
		case 3:
			{
			sscanf("Unhealthy","%s",atmNm);
			break;
			}
		case 4:
			{
			sscanf("Poison","%s",atmNm);
			break;
			}
		case 5:
			{
			sscanf("None","%s",atmNm);
			break;
			}
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
	if (uni.bodies[plan].enviro == 3)
		{
		sscanf("Dangerous","%s",envNm);
		}
	if (uni.bodies[plan].enviro >= 4)
		{
		sscanf("Deadly","%s",envNm);
		}
	}
