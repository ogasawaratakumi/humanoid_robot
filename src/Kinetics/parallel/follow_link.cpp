#include <GL/glut.h>


void display() {
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3f(0.0, 1.0, 0.0);
  glBegin(GL_LINE_LOOP);
  //RR1
  glVertex2d(0.0,  0.00);
  glVertex2d(0.0, -4.28);
  glEnd();
  glFlush();
  //RP1
  glColor3f(1.0, 1.0, 0.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(0.0, -4.28);
  glVertex2d(1.5, -4.28);
  glEnd();
  //RP2
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(1.5,  -4.28);
  glVertex2d(1.5, -15.08);
  glEnd();
  //RR3
  glColor3f(1.0, 0.0, 0.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(1.5, -15.08);
  glVertex2d(1.5, -19.18);
  glEnd();
  //RP4
  glColor3f(1.0, 0.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(1.5, -19.18);
  glVertex2d(1.5, -29.98);
  glEnd();
  //RR2
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(1.5, -29.98);
  glEnd();
  //RF
  glColor3f(0.0, 0.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(1.5, -29.98);
  glVertex2d(1.5, -34.33);
  glEnd();

  glutSwapBuffers();
}

void simu() {
  glutPostRedisplay();
}

void init() {
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glShadeModel(GL_FLAT);
}

void reshape(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-50.0, 50.0, -50.0, 50.0, -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

int main(int argc, char *argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA| GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize (600, 600);
  glutInitWindowPosition(550,250);
  glutCreateWindow(argv[0]);
  init();
  glutReshapeFunc(reshape);
  //glutKeyboardFunc(keyboard);
  //glutMouseFunc(mouse);
  glutDisplayFunc(display);
  glutMainLoop();
  return 0;
}
