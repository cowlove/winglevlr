#ifndef UBUNTU
class ServoVisualizer { 
   public:
      float startTime = 0;
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
public:
   float len0 = .5, len1 = .5;
   float baseLen = .2;
   static const int HISTLEN = 300;
   std::pair<float,float> hist[HISTLEN];
   int histIdx = 0;
   float yoffset = +.4;
   float startTime = 0;
   float scale = 1.0;
   ServoVisualizer() { 
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

   void update(float ang0, float ang1, vector<pair<float,float>> pts) { 
      ang0 = DEG2RAD(-ang0);
      ang1 = DEG2RAD(-ang1);
      float x = 0 - len0 * sin(ang0) - len1 * sin(ang1);
      float y = 0 + len0 * cos(ang0) + len1 * cos(ang1);
      hist[histIdx] = std::pair<float,float>(x,y);
      histIdx = (histIdx + 1) % HISTLEN;

      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);      

      glColor3f(.5,.5,.5);
      glLineWidth(2);
      line(-len0 * sin(ang0), len0*cos(ang0), 
         -len0 * sin(ang0) - len1 * sin(ang0 + M_PI / 2), 
         len0 * cos(ang0) + len1 * cos(ang0 + M_PI / 2));


      line(-.2, 0 - yoffset, .2, -yoffset);
      line(0, -.2 - yoffset, 0, .2 -yoffset);
      
      glPointSize(12);
      glColor3f(1,0,0);
      for(int i = 0; i < HISTLEN; i++) { 
         point(hist[i].first, hist[i].second);
      }

      glColor3f(0,0,0);
      glLineWidth(10);
   
      line(0, baseLen, 0, 0);
      line(0, 0, -len0 * sin(ang0), len0 * cos(ang0));
      line(-len0 * sin(ang0), len0*cos(ang0), 
         -len0 * sin(ang0) - len1 * sin(ang1), len0 * cos(ang0) + len1 * cos(ang1));

      glColor3f(0,0,1);
      point(0, baseLen);
      point(0,0);
      point(-len0 * sin(ang0), len0 * cos(ang0));
      point(-len0 * sin(ang0) - len1 * sin(ang1), 
         len0 * cos(ang0) + len1 * cos(ang1));

      glColor3f(0,1,0);
      for(auto it = pts.begin(); it != pts.end(); it++) { 
         point(it->first * scale, it->second * scale - yoffset);
      }

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