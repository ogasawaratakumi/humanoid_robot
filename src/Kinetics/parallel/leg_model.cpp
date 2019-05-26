#include <cstdio>
#include <GL/glut.h>

static int J1 = 0, J2 = 0, J3 = 0, J4 = 0;
static int shoulder = 0, elbow = 0;

void init() {
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glShadeModel(GL_FLAT);
}

void display() {
  glClear(GL_COLOR_BUFFER_BIT);
  glColor3f(1.0,1.0,0.0);
  glPushMatrix();

  glTranslatef(-0.3, 0.0, 0.0);
  glRotatef((GLfloat) shoulder, 0.0, 0.0, 1.0);
  glTranslatef(0.3, 0.0, 0.0);
  glPushMatrix();
  //x,y,zに拡大縮小
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.3);
  glPopMatrix();

  //glTranslatef(0.60, -0.9, 0.0);
  glTranslatef(0.3, 0.0, 0.0);
  glRotatef((GLfloat) elbow, 0.0, 0.0, 1.0);
  glTranslatef(0.3, 0.0, 0.0);
  glPushMatrix();
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.3);
  glPopMatrix();

  glTranslatef(0.3, 0.0, 0.0);
  glRotatef((GLfloat) J3, 0.0, 0.0, 1.0);
  glTranslatef(0.3, 0.0, 0.0);
  glPushMatrix();
  glScalef(2.0, 0.4, 0.0);
  glutWireCube(0.3);
  glPopMatrix();

  glTranslatef(0.3, 0.0, 0.0);
  glRotatef((GLfloat) J4, 0.0, 0.0, 1.0);
  glTranslatef(0.3, 0.0, 0.0);
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
	case 's':
	shoulder = (shoulder+5) % 360;
	glutPostRedisplay();
	break;
	case 'S':
	shoulder = (shoulder-5) % 360;
	glutPostRedisplay();
	break;
	case 'e':
	elbow = (elbow+5) % 360;
	glutPostRedisplay();
	break;
	case 'E':
	elbow = (elbow-5) % 360;
	glutPostRedisplay();
	case 'w':
	J3 = (J3+5) % 360;
	glutPostRedisplay();
	break;
	case 'W':
	J3 = (J3-5) % 360;
	glutPostRedisplay();
	break;
	case 'r':
	J4 = (J4+5) % 360;
	glutPostRedisplay();
	break;
	case 'R':
	J4 = (J4-5) % 360;
	glutPostRedisplay();
	break;
	case 'q':
	exit(0);
	break;
	default:
	break;
  }
}

void mouse( int button, int state, int x, int y ) {
  switch(button) {
	case GLUT_LEFT_BUTTON:
	printf("x = %d, y = %d\n", x, y);
	break;
  }
}

int main( int argc, char *argv[] ) {
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
