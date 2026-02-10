/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

double PlanePointDistance(const struct point &p, const struct point &n, const double d)
{
    return (n.x * p.x + n.y * p.y + n.z * p.z + d)/sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
}

point computerHooksLaw(struct point a, struct point b, double hook, double restLength)
{
    point L = a - b;
    double length = sqrt(L.x*L.x + L.y*L.y + L.z*L.z);
    double forceMagnitude = -1 * hook * (length - restLength);
    point force = L * (forceMagnitude / length);
    return force;
}
/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
    double kHook = jello->kElastic;
    for (int i=0; i<=7; i++)
      for (int j=0; j<=7; j++)
        for (int k=0; k<=7; k++)
        {
            // For simplicity, we only consider gravity in this placeholder implementation
            a[i][j][k].x = 0;
            a[i][j][k].y = 0; // gravity in negative y direction
            a[i][j][k].z = -1000;

            point force = {0.0f,0.0f,0.0f};
            // Calculate structural spring forces
            double restLength = 1.0f/7.0f;
            force += i < 7 ? computerHooksLaw(jello->p[i][j][k], jello->p[i+1][j][k], kHook, restLength) : point{0,0,0};
            force += i > 0 ? computerHooksLaw(jello->p[i][j][k], jello->p[i-1][j][k], kHook, restLength) : point{0,0,0};
            force += j < 7 ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j+1][k], kHook, restLength) : point{0,0,0};
            force += j > 0 ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j-1][k], kHook, restLength) : point{0,0,0};
            force += k < 7 ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j][k+1], kHook, restLength) : point{0,0,0};
            force += k > 0 ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j][k-1], kHook, restLength) : point{0,0,0};

            // Calculate shear spring forces
            restLength = (1.0f/7.0f) * sqrt(2);
            force += (i < 7) && (j < 7) ? computerHooksLaw(jello->p[i][j][k], jello->p[i+1][j+1][k], kHook, restLength) : point{0,0,0};
            force += (i > 0) && (j > 0) ? computerHooksLaw(jello->p[i][j][k], jello->p[i-1][j-1][k], kHook, restLength) : point{0,0,0};
            force += (i > 0) && (j < 7) ? computerHooksLaw(jello->p[i][j][k], jello->p[i-1][j+1][k], kHook, restLength) : point{0,0,0};
            force += (i < 7) && (j > 0) ? computerHooksLaw(jello->p[i][j][k], jello->p[i+1][j-1][k], kHook, restLength) : point{0,0,0};

            force += (j < 7) && (k < 7) ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j+1][k+1], kHook, restLength) : point{0,0,0};
            force += (j > 0) && (k > 0) ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j-1][k-1], kHook, restLength) : point{0,0,0};
            force += (j > 0) && (k < 7) ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j-1][k+1], kHook, restLength) : point{0,0,0};
            force += (j < 7) && (k > 0) ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j+1][k-1], kHook, restLength) : point{0,0,0};

            force += (k < 7) && (i < 7) ? computerHooksLaw(jello->p[i][j][k], jello->p[i+1][j][k+1], kHook, restLength) : point{0,0,0};
            force += (k > 0) && (i > 0) ? computerHooksLaw(jello->p[i][j][k], jello->p[i-1][j][k-1], kHook, restLength) : point{0,0,0};
            force += (k > 0) && (i < 7) ? computerHooksLaw(jello->p[i][j][k], jello->p[i+1][j][k-1], kHook, restLength) : point{0,0,0};
            force += (k < 7) && (i > 0) ? computerHooksLaw(jello->p[i][j][k], jello->p[i-1][j][k+1], kHook, restLength) : point{0,0,0};

            // Calculate bend spring forces
            restLength = 2.0f/7.0f;
            force += i < 6 ? computerHooksLaw(jello->p[i][j][k], jello->p[i+2][j][k], kHook, restLength) : point{0,0,0};
            force += i > 1 ? computerHooksLaw(jello->p[i][j][k], jello->p[i-2][j][k], kHook, restLength) : point{0,0,0};
            force += j < 6 ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j+2][k], kHook, restLength) : point{0,0,0};
            force += j > 1 ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j-2][k], kHook, restLength) : point{0,0,0};
            force += k < 6 ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j][k+2], kHook, restLength) : point{0,0,0};
            force += k > 1 ? computerHooksLaw(jello->p[i][j][k], jello->p[i][j][k-2], kHook, restLength) : point{0,0,0};

            // Collision Springs
            point planeNormal = {0.0f,0.0f,1.0f};
            point intersectionPoint = jello->p[i][j][k];
            double dot = planeNormal * intersectionPoint;

            if (dot < 0) {
                double penetrationDepth = PlanePointDistance(intersectionPoint, planeNormal, 0.0f);
                point springPoint = intersectionPoint + penetrationDepth * planeNormal;
                force += -1*computerHooksLaw(intersectionPoint, springPoint, jello->kElastic, 0.0f);
            }

            // Calculate acceleration
            a[i][j][k] += force * (1.0 / jello->mass);
        }
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
