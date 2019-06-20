#include "Kinematics.h"
#include "Link.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GL/glut.h>

#define WIDTH 320
#define HEIGHT 240

Link ulink[LINK_NUM], LEG_Link;
Kinematics *ik_node;

static double pos_step = 0.01;
static double rot_step = 0.5;

static double roll = 0.0f;
static double pitch = 0.0f;
static double yaw = 0.0f;

static double angle[6];
//static double initial_angle[6] = {0.0,-50.0,50.0,0.0,50.0,3.0};
static double initial_angle[6] = { -50.0, 50.0, 50.0, -50.0, 0.0, 0.0 };
//static double initial_angle[6] = { -0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static GLdouble centerX = 0.0f;
static GLdouble centerY = 0.0f;
static GLdouble centerZ = 0.0f;

int Mouse_X, Mouse_Y;

struct quaternion {
  double w;
  double x;
  double y;
  double z;
};

double Rotate[16];

quaternion Target;
quaternion current = { 1.0, 0.0, 0.0, 0.0 };

quaternion & operator *( quaternion &q1, quaternion &q2 ) {
  quaternion q0 = {
	q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
	q1.w * q2.x - q1.x * q2.w - q1.y * q2.z - q1.z * q2.y,
	q1.w * q2.y - q1.x * q2.z - q1.y * q2.w - q1.z * q2.x,
	q1.w * q2.z - q1.x * q2.y - q1.y * q2.x - q1.z * q2.w,
  };
  q1 = q0;
  return q1;
}

void qtor( double *r, quaternion q ) {
  double xx = q.x * q.x * 2.0;
  double yy = q.y * q.y * 2.0;
  double zz = q.z * q.z * 2.0;
  double xy = q.x * q.y * 2.0;
  double yz = q.y * q.z * 2.0;
  double zx = q.z * q.x * 2.0;
  double xw = q.x * q.w * 2.0;
  double yw = q.y * q.w * 2.0;
  double zw = q.z * q.w * 2.0;
  double r1[16] = { 1.0 - yy - zz, xy + zx, zx - yw, 0.0,
	xy - zw, 1.0 - zz - xx, yz + xw, 0.0,
	zx + yw, yz - xw, 1.0 - xx - yy, 0.0,
	0.0, 0.0, 0.0, 1.0 };
  for( int i=0; i<16; i++ ) {
	r[i] = r1[i];
  }
}

double rad2deg( double rad ) {
  return rad*180.0f/M_PI;
}

double deg2rad( double deg ) {
  return deg*M_PI/180.0f;
}
/*
 * 直方体を描く
 */
static void myBox(double x, double y, double z)
{
  GLdouble vertex[][3] = {
	{ -x, -y, -z },
	{  x, -y, -z },
	{  x,  y, -z },
	{ -x,  y, -z },
	{ -x, -y,  z },
	{  x, -y,  z },
	{  x,  y,  z },
	{ -x,  y,  z }
  };

  const static int face[][4] = {
	{ 0, 1, 2, 3 },
	{ 1, 5, 6, 2 },
	{ 5, 4, 7, 6 },
	{ 4, 0, 3, 7 },
	{ 4, 5, 1, 0 },
	{ 3, 2, 6, 7 }
  };

  const static GLdouble normal[][3] = {
	{ 0.0, 0.0,-1.0 },
	{ 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0 },
	{-1.0, 0.0, 0.0 },
	{ 0.0,-1.0, 0.0 },
	{ 0.0, 1.0, 0.0 }
  };

  const static GLfloat red[] = { 0.3, 0.6, 0.8, 1.0 };

  int i, j;

  /* 材質を設定する */
  glMaterialfv(GL_FRONT, GL_DIFFUSE, red);

  glBegin(GL_QUADS);
  for (j = 0; j < 6; ++j) {
	glNormal3dv(normal[j]);
	for (i = 4; --i >= 0;) {
	  glVertex3dv(vertex[face[j][i]]);
	}
  }
  glEnd();
}

/*
 * 円柱を描く
 */
static void myCylinder(double radius, double height, int sides)
{
  const static GLfloat yellow[] = { 0.5, 0.5, 0.5, 1.0 };
  double step = 6.28318530717958647692 / (double)sides;
  int i = 0;

  /* 材質を設定する */
  glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);

  /* 上面 */
  glNormal3d(0.0, 1.0, 0.0);
  glBegin(GL_TRIANGLE_FAN);
  while (i < sides) {
	double t = step * (double)i++;
	glVertex3d(radius * sin(t), height, radius * cos(t));
  }
  glEnd();

  /* 底面 */
  glNormal3d(0.0, -1.0, 0.0);
  glBegin(GL_TRIANGLE_FAN);
  while (--i >= 0) {
	double t = step * (double)i;
	glVertex3d(radius * sin(t), -height, radius * cos(t));
  }
  glEnd();

  /* 側面 */
  glBegin(GL_QUAD_STRIP);
  while (i <= sides) {
	double t = step * (double)i++;
	double x = sin(t);
	double z = cos(t);

	glNormal3d(x, 0.0, z);
	glVertex3f(radius * x, height, radius * z);
	glVertex3f(radius * x, -height, radius * z);
  }
  glEnd();
}

/*
 * 地面を描く
 */
static void myGround(double height)
{
  const static GLfloat ground[][4] = {
	{ 0.6, 0.6, 0.6, 1.0 },
	{ 0.3, 0.3, 0.3, 1.0 }
  };

  int i, j;

  glBegin(GL_QUADS);
  glNormal3d(0.0, 1.0, 0.0);
  for (j = -5; j < 5; ++j) {
	for (i = -5; i < 5; ++i) {
	  glMaterialfv(GL_FRONT, GL_DIFFUSE, ground[(i + j) & 1]);
	  glVertex3d((GLdouble)i, height, (GLdouble)j);
	  glVertex3d((GLdouble)i, height, (GLdouble)(j + 1));
	  glVertex3d((GLdouble)(i + 1), height, (GLdouble)(j + 1));
	  glVertex3d((GLdouble)(i + 1), height, (GLdouble)j);
	}
  }
  glEnd();
}

/*
 * 画面表示
 */
static void display(void)
{
  //const static GLfloat blue[] = { 0.2, 0.2, 0.8, 1.0 };     /* 球の色 */
  const static GLfloat lightpos[] = { 3.0, 4.0, 5.0, 1.0 }; /* 光源の位置 */

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity();

  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

  /* 視点の移動（シーンの方を奥に移す）*/
  glTranslated(0.0, 0.0, -5.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt( -20.0f, 10.0f, -20.0f,
	  centerX, centerY, centerZ,
	  0.5f, 0.5f, 0.5f);

  glMultMatrixd(Rotate);

  /* シーンの描画 */
  myGround(-4.5);                           

  /*各リンクは相対座標で考えられている。*/

  glPushMatrix();

  //股
  glTranslated(0.0, 0.25, -1.4);
  glRotated(90.0, 1.0, 0.0, 0.0);
  glRotated(angle[0], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  //膝上
  glTranslated(0.0,-0.0, 0.8);
  //glRotated(90.0, 1.0, 0.0, 0.0);
  myBox(0.2,0.2,0.6);
 
  //関節
  glTranslated(0.0, -0.0, 0.8);
  //glRotated(90.0, 1.0, 0.0, 0.0);
  glRotated(angle[1], 0.0, 1.0, 0.0);
  myCylinder(0.3,0.4,12);

  //膝
  glTranslated(0.0, 0.0, 0.45);
  myBox(0.2,0.2,0.2);

  //関節
  glTranslated(0.0, 0.0, 0.45);
  glRotated(angle[2], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  //膝下
  glTranslated(0.0, 0.0, 0.8);
  //glRotated(90.0, 1.0, 0.0, 0.0);
  myBox(0.2,0.2,0.6);

  glTranslated(0.0, 0.0, 0.45);
  glRotated(angle[3], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  glTranslated(0.0, 0.0, 0.45);
  myBox(0.2,0.2,0.2);

  glTranslated(0.0,0.0,0.20);
  myBox(0.8,0.6,0.1);

  glPopMatrix();

  glFlush();
}

static void resize(int w, int h)
{
  glViewport(0, 0, w, h);

  glMatrixMode(GL_PROJECTION);

  glLoadIdentity();
  gluPerspective(30.0, (double)w / (double)h, 1.0, 100.0);

  glMatrixMode(GL_MODELVIEW);
}

static void keyboard(unsigned char key, int x, int y)
{
  // ESC か q をタイプしたら終了 
  if (key == '\033' || key == 'q') {
	exit(0);
  }else if(key == 'f'){	// x
	LEG_Link.p(0) += pos_step;	
	printf("pos[x] = %lf\n", LEG_Link.p(0));
  }else if(key == 'j'){
	LEG_Link.p(0) -= pos_step; 
	printf("pos[x] = %lf\n", LEG_Link.p(0));
  }else if(key == 'd'){	// y
	LEG_Link.p(1) += pos_step;
	printf("pos[y] = %lf\n", LEG_Link.p(1));
  }else if(key == 'k'){
	LEG_Link.p(1) -= pos_step;
	printf("pos[y] = %lf\n", LEG_Link.p(1));
  }else if(key == 's'){	// z
	LEG_Link.p(2) += pos_step;
	printf("pos[z] = %lf\n", LEG_Link.p(2));
  }else if(key == 'l'){
	LEG_Link.p(2) -= pos_step;
	printf("pos[z] = %lf\n", LEG_Link.p(2));
  }else if(key == 'l'){
  }else if(key == 'r'){	// roll
	roll += rot_step;
  }else if(key == 'u'){
	roll -= rot_step;
  }else if(key == 'e'){	// pitch
	pitch += rot_step;
  }else if(key == 'i'){
	pitch -= rot_step;
  }else if(key == 'w'){	// yaw
	yaw += rot_step;
  }else if(key == 'o'){
	yaw -= rot_step;
  }else if(key == 't') {
	angle[1] += 1;
  }else if(key == 'T') {
	angle[1] -= 1;
  }

	 LEG_Link.R = ik_node->computeMatrixFromAngles(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));

	 ik_node->calcInverseKinematics(RR2, LEG_Link);
	 //for(int i=0;i<6;i++)
	 //angle[i] = rad2deg(ulink[i+1].q);
	 angle[0] = rad2deg(ulink[RP1].q);
	 angle[1] = rad2deg(ulink[RP2].q);
	 angle[2] = rad2deg(ulink[RP3].q);
	 angle[3] = rad2deg(ulink[RP4].q);
}

void mousemove( int x, int y ) {
  double dx = (x - Mouse_X) * 1.33/WIDTH;
  double dy = (y - Mouse_Y) * 1.0/HEIGHT;

  double length = sqrt(dx * dx + dy * dy);

  if(length != 0.0) {
	double radian = length * M_PI;
	double theta = sin(radian) / length;
	quaternion after = { cos(radian), dy * theta, dx * theta, 0.0 };

	Target = after * current;
	qtor(Rotate, Target);
  }
}

void mouse( int button, int state, int x, int y ) {
  if(button) {
	switch ( button ) {
	  case GLUT_RIGHT_BUTTON:
		if( state == GLUT_DOWN ) {
		  printf("saved!\n");
		  FILE *fp = fopen("./LEG_data.txt","a");
		  if( fp != NULL ) {
			fprintf( fp, "angle[0] = %lf, angle[1] = %lf, angle[2] = %lf, angle[3] = %lf\n", angle[0], angle[1], angle[2], angle[3] );
		  }
		  fclose( fp );
		}
	  default:
		break;
	}
	switch(state) {
	  case GLUT_DOWN:
		Mouse_X = x;
		Mouse_Y = y;
		break;
	  case GLUT_UP:
		current = Target;
		break;
	  default:
		break;
	}
  }
}

void idle(void)
{
  glutPostRedisplay();
}

static void init(void)
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  qtor(Rotate, current);
}

int main(int argc, char *argv[])
{
  ik_node = new Kinematics(ulink);
  SetJointInfo(ulink);

  for(int i=0;i<6;i++){
	ulink[i+1].q = deg2rad(initial_angle[i]);
	angle[i] = initial_angle[i];
  }
  ik_node->calcForwardKinematics(BASE);
  LEG_Link = ulink[RR2];

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH);
  glutCreateWindow(argv[0]);
  glutDisplayFunc(display);
  glutReshapeFunc(resize);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(mousemove);
  glutIdleFunc(idle);
  init();
  glutMainLoop();
  return 0;
}
