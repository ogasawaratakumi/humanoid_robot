//テキストファイルにする際に（足首ピッチ）バグあり！
#include "Kinematics.h"
#include "Link.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GL/glut.h>

#define WIDTH 320
#define HEIGHT 240

Link ulink1[LINK_NUM], ulink2[LINK_NUM], LEG_Link1, LEG_Link2;
Kinematics *ik_node1, *ik_node2;

static double pos_step = 0.01;
static double rot_step = 0.5;

static double roll1 = 0.0f;
static double pitch1 = 0.0f;
static double yaw1 = 0.0f;
static double roll2 = 0.0f;
static double pitch2 = 0.0f;
static double yaw2 = 0.0f;

static double angle_r[7];
static double angle_l[7];
static double initial_angle_r[7] = { 0.0, 0.0, 20.0, -20.0, -20.0, 20.0, 0.0 };
static double initial_angle_l[7] = { 0.0, 0.0, 20.0, -20.0, -20.0, 20.0, 0.0 };
static GLdouble centerX = 0.0f;
static GLdouble centerY = 0.0f;
static GLdouble centerZ = 0.0f;

int Mouse_X, Mouse_Y;
int speed;
int n=2;

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

  const static GLfloat black[] = { 0.6, 0.6, 0.8, 0.0 };

  int i, j;

  /* 材質を設定する */
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);

  glBegin(GL_QUADS);
  for (j = 0; j < 6; ++j) {
	glNormal3dv(normal[j]);
	for (i = 4; --i >= 0;) {
	  glVertex3dv(vertex[face[j][i]]);
	}
  }
  glEnd();
}

static void myCylinder(double radius, double height, int sides)
{
  const static GLfloat yellow[] = { 0.2, 0.1, 0.3, 1.0 };
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


static void display(void)
{
  const static GLfloat lightpos[] = { 3.0, 4.0, 5.0, 1.0 }; /* 光源の位置 */

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity();

  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
  glTranslated(0.0, 0.0, -5.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt( -20.0f, 0.0f, -20.0f,
	  centerX, centerY, centerZ,
	  0.5f, 0.5f, 0.5f);

  glMultMatrixd(Rotate);

  myGround(-4.5);                           

  glPushMatrix();
  //股
  glTranslated(0.0, 0.25, -1.0);
  glRotated(90.0, 1.0, 0.0, 0.0);
  glRotated(angle_r[0], 0.0, 0.0, 1.0);
  glRotated(angle_r[1], 1.0, 0.0, 0.0);
  glRotated(angle_r[2], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  //膝上
  glTranslated(0.0,-0.0, 0.8);
  myBox(0.2, 0.2, 0.6);

  //関節
  glTranslated(0.0, -0.0, 0.8);
  glRotated(angle_r[3], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  //膝
  glTranslated(0.0, 0.0, 0.45);
  myBox(0.2, 0.2, 0.2);

  //関節
  glTranslated(0.0, 0.0, 0.45);
  glRotated(angle_r[4], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  //膝下
  glTranslated(0.0, 0.0, 0.8);
  myBox(0.2, 0.2, 0.6);

  glTranslated(0.0, 0.0, 0.45);
  glRotated(angle_r[5], 0.0, 1.0, 0.0);
  glRotated(angle_r[6], 1.0, 0.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  glTranslated(0.0, 0.0, 0.45);
  myBox(0.2, 0.2, 0.2);

  glTranslated(0.0,0.0,0.20);
  myBox(0.8, 0.6, 0.1);
  glPopMatrix();

  glPushMatrix();
  //股
  glTranslated(0.0, 0.25, 1.0);
  glRotated(90.0, 1.0, 0.0, 0.0);
  glRotated(angle_l[0], 0.0, 0.0, 1.0);
  glRotated(angle_l[1], 1.0, 0.0, 0.0);
  glRotated(angle_l[2], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  //膝上
  glTranslated(0.0, 0.0, 0.8);
  myBox(0.2, 0.2, 0.6);

  //関節
  glTranslated(0.0, -0.0, 0.8);
  glRotated(angle_l[3], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  //膝
  glTranslated(0.0, 0.0, 0.45);
  myBox(0.2, 0.2, 0.2);

  //関節
  glTranslated(0.0, 0.0, 0.45);
  glRotated(angle_l[4], 0.0, 1.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  //膝下
  glTranslated(0.0, 0.0, 0.8);
  myBox(0.2, 0.2, 0.6);

  glTranslated(0.0, 0.0, 0.45);
  glRotated(angle_l[5], 0.0, 1.0, 0.0);
  glRotated(angle_l[6], 1.0, 0.0, 0.0);
  myCylinder(0.3, 0.4, 12);

  glTranslated(0.0, 0.0, 0.45);
  myBox(0.2, 0.2, 0.2);

  glTranslated(0.0, 0.0, 0.20);
  myBox(0.8, 0.6, 0.1);
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
	LEG_Link2.p(0) += pos_step;	
  }else if(key == 'j'){
	LEG_Link2.p(0) -= pos_step; 
  }else if(key == 'd'){	// y
	LEG_Link2.p(1) += pos_step;
  }else if(key == 'k'){
	LEG_Link2.p(1) -= pos_step;
  }else if(key == 's'){	// z
	LEG_Link2.p(2) += pos_step;
	printf("z = %lf\n", LEG_Link2.p(2));
  }else if(key == 'l'){
	LEG_Link2.p(2) -= pos_step;
  }else if(key == 'r'){	// roll
	roll2 += rot_step;
  }else if(key == 'u'){
	roll2 -= rot_step;
  }else if(key == 'e'){	// pitch
	pitch2 += rot_step;
  }else if(key == 'i'){
	pitch2 -= rot_step;
  }else if(key == 'w'){	// yaw
	yaw2 += rot_step;
  }else if(key == 'o'){
	yaw2 -= rot_step;
  }else if(key == 'F'){
	LEG_Link1.p(0) += pos_step;
  }else if(key == 'J'){
	LEG_Link1.p(0) -= pos_step;
  }else if(key == 'D'){
	LEG_Link1.p(1) += pos_step;
  }else if(key == 'K'){
	LEG_Link1.p(1) -= pos_step;
  }else if(key == 'S'){
	LEG_Link1.p(2) += pos_step;
  }else if(key == 'L'){
	LEG_Link1.p(2) -= pos_step;
  }else if(key == 'R'){
	roll1 += rot_step;
  }else if(key == 'U'){
	roll1 -= rot_step;
  }else if(key == 'E'){
	pitch1 += rot_step;
  }else if(key == 'I'){
	pitch1 -= rot_step;
  }else if(key == 'W'){
	yaw1 += rot_step;
  }else if(key == 'O'){
	yaw1 -= rot_step;
  }

  LEG_Link1.R = ik_node1->computeMatrixFromAngles(deg2rad(roll1), deg2rad(pitch1), deg2rad(yaw1));
  LEG_Link2.R = ik_node2->computeMatrixFromAngles(deg2rad(roll2), deg2rad(pitch2), deg2rad(yaw2));

  ik_node1->calcInverseKinematics(RR2, LEG_Link1);
  ik_node2->calcInverseKinematics(LR2, LEG_Link2);
  for(int i=0;i<7;i++)
	angle_r[i] = rad2deg(ulink1[i+1].q);

  for(int i=0; i<7; i++ ) 
	angle_l[i] = rad2deg(ulink2[i+9+1].q);
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
		  printf("speed :");
		  scanf("%d", &speed);
		  printf("saved!\n");
		  FILE *fp = fopen("./041_hr42_kick_r.txt","a");
		  if( fp != NULL ) {
			fprintf( fp, "%d,", speed );
			for(int i=6; i>=0; i--) {
			  if(i==5) {
				//fprintf(fp, "%d,", (int)angle_r[i]-20);
				fprintf( fp, "0," );
			  }
			  else if(i==3) {
				continue;
			  }else {
				fprintf( fp, "%d,", (int)angle_l[i] );
			  }
			}
			fprintf(fp, "0,0,0," );
			for(int i=6; i>=0; i--) {
			  if(i==5) {
				//fprintf(fp, "%d,", (int)angle_r[i]-20);
				fprintf( fp, "0," );
			  }else if(i==3) {
				continue;
			  //}else if(i==4 || i==2) {
				//fprintf( fp, "%d,", ((int)angle_r[i])*(-1));
			  }else{
				fprintf( fp, "%d,", (int)angle_r[i] );
			  }
			}
			fprintf( fp, "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,%d,%d,0x0\n", n,n );
			n++;
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
  ik_node1 = new Kinematics(ulink1);
  SetJointInfo(ulink1);

  for( int i=0; i<7; i++ ){
	ulink1[i+1].q = deg2rad(initial_angle_r[i]);
	angle_r[i] = initial_angle_r[i];
  }
  ik_node1->calcForwardKinematics(BASE);
  LEG_Link1 = ulink1[RR2];

  ik_node2 = new Kinematics(ulink2);
  SetJointInfo(ulink2);

  for( int i=0; i<7; i++ ) {
	ulink2[i+9+1].q = deg2rad(initial_angle_l[i]);
	angle_l[i] = initial_angle_l[i];
  }
  ik_node2->calcForwardKinematics(BASE);
  LEG_Link2 = ulink2[LR2];

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
