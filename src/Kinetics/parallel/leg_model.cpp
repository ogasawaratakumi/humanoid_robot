#include "Link.h"
#include "Kinematics.h"

#include <cmath>
#include <fstream>
#include <cstdio>
#include <GL/glut.h>

Link ulink[LINK_NUM], LEG_Link;
Kinematics *ik_node;

static double pos_step = 0.01;
static double rot_step = 0.5;

static double angle[6];
static double initial_angle[7] = { 0.0, 90.0, -135.0, -45.0, -60.0, 60.0, 90.0 };

double rad2deg( double rad ) {
  return rad*180.0f/M_PI;
}

double deg2rad( double deg ) {
  return deg*M_PI/180.0f;
}

void init() {
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glShadeModel(GL_FLAT);
}

void display() {
  glClear(GL_COLOR_BUFFER_BIT);
  glColor3f(1.0,1.0,0.0);
  glPushMatrix();
  
  glTranslatef(-0.3, 0.0, 0.0);
  glRotatef((GLfloat) angle[1], 0.0, 0.0, 1.0);
  glTranslatef(0.3, 0.0, 0.0);
  glPushMatrix();
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.3);
  glPopMatrix();

  glTranslatef(-0.3, 0.0, 0.0);
  glRotatef((GLfloat) angle[2], 0.0, 0.0, 1.0);
  glTranslatef(0.3, 0.0, 0.0);
  glPushMatrix();
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.3);
  glPopMatrix();

  glTranslatef(0.3, 0.0, 0.0);
  glRotatef((GLfloat) angle[3], 0.0, 0.0, 1.0);
  glTranslatef(0.3, 0.0, 0.0);
  glPushMatrix();
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.2);
  glPopMatrix();

  glTranslatef(0.3, 0.0, 0.0);
  glRotatef((GLfloat) angle[4], 0.0, 0.0, 1.0);
  glTranslatef(0.3, 0.0, 0.0);
  glPushMatrix();
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.3);
  glPopMatrix();

  glTranslatef(0.3, 0.0, 0.0);
  glRotatef((GLfloat) angle[5], 0.0, 0.0, 1.0);
  glTranslatef(0.30, 0.0, 0.0);
  glPushMatrix();
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.2);
  glPopMatrix();

  glTranslatef(0.3, 0.0, 0.0);
  glRotatef((GLfloat) angle[6], 0.0, 0.0, 1.0);
  glTranslatef(0.30, 0.0, 0.0);
  glPushMatrix();
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.3);
  glPopMatrix();

  glPopMatrix();
  glutSwapBuffers();
}

void reshape( int w, int h ) {
  glViewport(0, 0, (GLsizei) w, (GLsizei) h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 20.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -5.0);
}

void keyboard( unsigned char key, int x, int y ) {
  switch(key) {
	case 'a':
	angle[2] += 1;
	glutPostRedisplay();
	break;
	case 'A':
	angle[2] -= 1;
	glutPostRedisplay();
	break;
	case 's':
	angle[3] += 1;
	//printf("angle[3] = %lf\n", angle[3]);
	glutPostRedisplay();
	break;
	case 'S':
	angle[3] -= 1;
	if(angle[3] <= -90) {
	  angle[3] = -90;
	  break;
	}
	glutPostRedisplay();
	break;
	case 'd':
	angle[4] += 1;
	//printf("angle[4] = %lf\n", angle[4]);
	if(angle[4] >= 0) {
	  angle[4] = 0;
	  break;
	}
	glutPostRedisplay();
	break;
	case 'f':
	angle[5] += 1;
	glutPostRedisplay();
	break;
	case 'F':
	angle[5] -= 1;
	glutPostRedisplay();
	break;
	case 'D':
	angle[4] -= 1;
	//printf("angle[4] = %lf\n", angle[4]);
	if(angle[4] <= -90) {
	  angle[4] = -90;
	  break;
	}
	glutPostRedisplay();
	break;
	case 'q':
	exit(0);
	break;
	default:
	break;
  }
  /*
  ik_node->calcInverseKinematics(RP2, LEG_Link);
  for(int i=0; i<6; i++) {
	if(i==2 || i==3 || i==4 || i==5) {
	  angle[i] = rad2deg(ulink[i+1].q);
	}
  }
  */
}

void mouse( int button, int state, int x, int y ) {
  switch(button) {
	case GLUT_LEFT_BUTTON:
	  printf("x = %d, y = %d\n", x, y);
	  break;
	case GLUT_RIGHT_BUTTON:
	  if(state == GLUT_DOWN) {
		printf("saved!\n");
		FILE *fp = fopen("./LEG_data.txt","a");
		if(fp != NULL) {
		  //for(int i=0; i=7; i++) {
			fprintf(fp,"angle[2] = %lf angle[3] = %lf angle[4] = %lf\n", angle[2]+180, angle[3], angle[4]);
		  //}
		}
		fclose(fp);
	  }
  }
}

int main( int argc, char *argv[] ) {
  
  ik_node = new Kinematics(ulink);
  SetJointInfo(ulink);

  for(int i=0; i<7; i++) {
	if(i==1 || i==2 || i==3 || i==4 || i==5 || i==6) {
	  ulink[i+1].q = deg2rad(initial_angle[i]);
	  angle[i] = initial_angle[i];
	}
  }
  ik_node->calcForwardKinematics(BASE);
  LEG_Link = ulink[RP2];
  

  glutInit( &argc, argv );
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowSize(800, 800);
  glutInitWindowPosition(400, 100);
  glutCreateWindow(argv[0]);
  init();
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMainLoop();

  return 0;
}
