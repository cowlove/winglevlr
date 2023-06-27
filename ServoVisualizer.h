#ifndef UBUNTU
class ServoVisualizer { 
   public:
      void init() {}
      void update(float, float) {}
};
#else 
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/freeglut_ext.h>
#include <math.h>
#include <cstdio>
#include <utility>

class ServoVisualizer { 
   float len0 = .5, len1 = .5;
   float baseLen = .2;
   static const int HISTLEN = 300;
   std::pair<float,float> hist[HISTLEN];
   int histIdx = 0;
   float yoffset = +.4;
public:
   float startTime = 0;
   void init() { 
      char *argv[]={NULL};
      int argc = 0;
      glutInit(&argc, argv);
      glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
      glutInitWindowSize(800,800);
      glutCreateWindow("Hello World");
   }
   void line(float x1, float y1, float x2, float y2) {
      glBegin(GL_LINES);
      glVertex2d(x1, y1 + yoffset);
      glVertex2d(x2, y2 + yoffset);
      glEnd();
   }
   void point(float x, float y) { 
      glBegin(GL_POINTS);
      glVertex2d(x, y + yoffset);
      glEnd();
   }

   void update(float ang0, float ang1) { 
      ang0 = DEG2RAD(ang0);
      ang1 = DEG2RAD(ang1);
      float x = 0 + len0 * sin(ang0) + len1 * sin(ang1);
      float y = 0 + len0 * cos(ang0) + len1 * cos(ang1);
      hist[histIdx] = std::pair<float,float>(x,y);
      histIdx = (histIdx + 1) % HISTLEN;

      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);      

      glPointSize(10);
      glColor3f(1,0,0);
      for(int i = 0; i < HISTLEN; i++) { 
         point(hist[i].first, hist[i].second);
      }

      glColor3f(0,0,0);
      glLineWidth(10);
      line(0, baseLen, 0, 0);
      line(0, 0, len0 * sin(ang0), len0 * cos(ang0));
      line(len0 * sin(ang0), len0*cos(ang0), len0 * sin(ang0) + len1 * sin(ang1), 
         len0 * cos(ang0) + len1 * cos(ang1));

      glColor3f(0,1,0);
      point(0, baseLen);
      point(0,0);
      point(len0 * sin(ang0), len0 * cos(ang0));
      point(len0 * sin(ang0) + len1 * sin(ang1), 
         len0 * cos(ang0) + len1 * cos(ang1));

      glFlush();
      glutSwapBuffers();
      glutMainLoopEvent();
      glutPostRedisplay();
   }
};
#endif

#if 0 
int main(int argc, char *argv[])
  {
     ServoVisualizer sv;
     float ang0, ang1;
     sv.init();

     while(true){
         ang0 += .05;
         if (ang0 > 2 * M_PI) {
            ang0 = 0;
         }
         ang1 += .02;
         if (ang1 > 2 * M_PI) {
            ang1 = 0;
         }
         sv.update(ang0, ang1);
     }
     return 0;
  }

#endif