/*
 * (C) Copyright 2013 WPI-ARC (http://arc.wpi.edu) and others.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser General Public License
 * (LGPL) version 2.1 which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/lgpl-2.1.html
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * Contributors:
 *      Jim Mainprice
 */

#include "op_utils.h"

#include "robotics/RobotDynamics3D.h"

#include <cmath>
#include <GL/gl.h>

using std::cout;
using std::endl;

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

std::vector<int> OpSpaceControl::GetStdVector(int value)
{
    std::vector<int> vect;
    vect.push_back(value);
    return vect;
}

std::vector<double> OpSpaceControl::GetStdVector(double value)
{
    std::vector<double> vect;
    vect.push_back(value);
    return vect;
}

std::vector<double> OpSpaceControl::GetStdVector(const Vector3& pos)
{
    std::vector<double> vect(3);
    vect[0] = pos[0];
    vect[1] = pos[1];
    vect[2] = pos[2];
    return vect;
}

Vector3 OpSpaceControl::GetVector3(const Vector& vect)
{
    Vector3 pos;
    pos[0] = vect[0];
    pos[1] = vect[1];
    pos[2] = vect[2];
    return pos;
}

float* OpSpaceControl::GetGlVector3(const Math3D::Vector3& pos)
{
    float* vect = new float[3];
    vect[0] = pos[0];
    vect[1] = pos[1];
    vect[2] = pos[2];
    return vect;
}

Vector OpSpaceControl::GetVector(const Math3D::Vector3& pos)
{
    Vector vect(3);
    vect[0] = pos[0];
    vect[1] = pos[1];
    vect[2] = pos[2];
    return vect;
}

std::vector< std::vector<double> > OpSpaceControl::GetStdMatrix( const Matrix& mat )
{
    std::vector< std::vector<double> > out( mat.numRows() );

    for(int i=0;i<mat.numRows();i++)
    {
        out[i].resize( mat.numCols() );

        for(int j=0;j<mat.numCols();j++)
        {
            out[i][j] = mat(i,j);
        }
    }

    return out;
}

Matrix OpSpaceControl::GetKrisMatrix( const std::vector< std::vector<double> >& mat )
{
    if( mat.empty() ){
        Matrix out(0,0);
        return out;
    }

    Matrix out( mat.size(), mat[0].size() );

    for(int i=0;i<mat.size();i++)
    {
        for(int j=0;j<mat[0].size();j++)
        {
            out(i,j) = mat[i][j];
        }
    }

    return out;
}

void OpSpaceControl::PushRotationToVector( const Matrix3& R, Vector& x )
{
    int size = x.size();
    Vector x_temp = x;
    x.resize(size+9);

    for(int i=0;i<size;i++) // Copy the old values to resized vector
        x[i] = x_temp[i];

    x[size+0] = R(0,0); x[size+1] = R(0,1); x[size+2] = R(0,2);
    x[size+3] = R(1,0); x[size+4] = R(1,1); x[size+5] = R(1,2);
    x[size+6] = R(2,0); x[size+7] = R(2,1); x[size+8] = R(2,2);
}

void OpSpaceControl::PopRotationFromVector( Vector& x, Matrix3& R )
{
    int size = x.size();
    if( size<9 )
        return;

    R(0,0) = x[size-9]; R(0,1) = x[size-8]; R(0,2) = x[size-7];
    R(1,0) = x[size-6]; R(1,1) = x[size-5]; R(1,2) = x[size-4];
    R(2,0) = x[size-3]; R(2,1) = x[size-2]; R(2,2) = x[size-1];

    Vector x_temp = x;
    x.resize(size-9);

    for(int i=0;i<x.size();i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];
}

void OpSpaceControl::PushPosToVector( const Vector3& p, Vector& x )
{
    int size = x.size();
    Vector x_temp = x;
    x.resize(size+3);

    for(int i=0;i<size;i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];

    x[size+0] = p[0];
    x[size+1] = p[1];
    x[size+2] = p[2];
}

void OpSpaceControl::PopPosFromVector( Vector& x, Vector3& p )
{
    int size = x.size();
    if(size < 3)
        return;

    p[0]   = x[size-3];
    p[1]   = x[size-2];
    p[2]   = x[size-1];

    Vector x_temp = x;
    x.resize(size-3);

    for(int i=0;i<x.size();i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];
}

void OpSpaceControl::PushFrameToVector( const Frame3D& T, Vector& x )
{
    OpSpaceControl::PushRotationToVector( T.R, x );
    OpSpaceControl::PushPosToVector( T.t, x );
}

void OpSpaceControl::PopFrameFromVector( Vector& x, Frame3D& T )
{
    OpSpaceControl::PopPosFromVector( x, T.t );
    OpSpaceControl::PopRotationFromVector( x, T.R );
}

Vector OpSpaceControl::GetPushedFrame( const Frame3D& T )
{
    Vector vect;
    OpSpaceControl::PushFrameToVector( T, vect );
    return vect;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

// used in python interface
Matrix GetJacobian( RobotDynamics3D& robot, int index, Vector3 p )
{
    Matrix Jmat;
    robot.GetFullJacobian( p, index, Jmat );
    return Jmat;
}

// used in python interface
Matrix GetPositionJacobian( RobotDynamics3D& robot, int index,  Vector3 p )
{
    Matrix Jmat;
    robot.GetPositionJacobian( p, index, Jmat );
    return Jmat;
}

// used in python interface
Matrix GetOrientationJacobian( RobotDynamics3D& robot, int index )
{
    Matrix Jmat;
    // TODO exeption
    //throw PyException("Orientation Jacobian not supported yet");
    //robot.GetOrientationJacobian(index,Jmat);
    return Jmat;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

double angle(const Matrix3& R)
{
    double ctheta = (R.trace() - 1.0)*0.5;
    return std::acos(std::max(std::min(ctheta,1.0),-1.0));
}

//! Returns the moment w (exponential map) representation of R such
//! that e^[w] = R. Equivalent to axis-angle representation with
//! w/||w||=axis; ||w||=angle;
Vector3 Moment(const Matrix3& R)
{
    double theta = angle(R);

    if( std::abs(theta-M_PI) < 1e-5 )
    {
        //can't do normal version because the scale factor reaches a singularity
        double x2=(R[0]+1.)*0.5;
        double y2=(R[4]+1.)*0.5;
        double z2=(R[8]+1.)*0.5;
        if( x2 < 0) {
            assert(x2>-1e-5);
            x2=0;
        }
        if( y2 < 0 ) {
            assert(y2>-1e-5);
            y2=0;
        }
        if( z2 < 0 ) {
            assert(z2>-1e-5);
            z2=0;
        }

        double x = M_PI*std::sqrt(x2);
        double y = M_PI*std::sqrt(y2);
        double z = M_PI*std::sqrt(z2);
        //determined up to sign changes, we know r12=2xy,r13=2xz,r23=2yz
        double xy=R[3];
        double xz=R[6];
        double yz=R[7];

        if(x > y) {
            if(x > z) {
                //x is largest
                if(xy < 0) y=-y;
                if(xz < 0) z=-z;
            }
            else {
                //z is largest
                if(yz < 0) y=-y;
                if(xz < 0) x=-x;
            }
        }
        else {
            if(y > z) {
                //y is largest
                if(xy < 0) x=-x;
                if(yz < 0) z=-z;
            }
            else {
                //z is largest
                if(yz < 0) y=-y;
                if(xz < 0) x=-x;
            }
        }
        return Vector3(x,y,z);
    }

    //normal
    double scale = 0.5;
    if( std::abs(theta) > 1e-5 )
        scale = 0.5*theta/std::sin(theta);
    double x = (R[3+2]-R[6+1]) * scale;
    double y = (R[6+0]-R[0+2]) * scale;
    double z = (R[0+1]-R[3+0]) * scale;
    return Vector3(x,y,z);
}

Vector3 OpSpaceControl::Error( const Matrix3& R1, const Matrix3& R2 )
{
    Matrix3 R2inv;
    R2inv.setInverse( R2 );
    return Moment( R1 * R2inv );
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

Matrix OpSpaceControl::VStack( const Matrix& mat1, const Matrix& mat2 )
{
    if( mat1.numCols() != mat2.numCols()) {
        cout << "Error in Vstack" << endl;
        return Matrix();
    }

    std::vector<Vector> mattmp;

    for(int i=0;i<mat1.numRows();i++)
        mattmp.push_back( mat1.row(i) );

    for(int i=0;i<mat2.numRows();i++)
        mattmp.push_back( mat1.row(i) );

    Matrix out( mattmp.size(), mat1.numCols() );
    for(int i=0;i<int(mattmp.size());i++)
    {
        Vector row;
        out.getRowRef( i, row );
        row = mattmp[i];
    }

    return out;
}

Vector OpSpaceControl::HStack(const Vector& vec1, const Vector& vec2 )
{
    Vector out( vec1.size() + vec2.size() );

    for(int i=0;i<vec1.size();i++)
        out[i] = vec1[i];

    for(int i=vec1.size();i<(vec1.size()+vec2.size());i++)
        out[i] = vec2[i-vec1.size()];

    return out;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

//! @ingroup graphic
//! Computes the coordinates of the points of a unit radius circle discretized in n points.
//! sint et cost each have |n+1| elements.
//! Memory is allocated inside the function and must consequently be freed outside the function.
//! The signe of n defines in which direction the circle is travelled around.
int g3d_circle_table(double **sint, double **cost, const int n)
{
    if(sint==NULL || cost==NULL)
    {
        printf("%s: %d: g3d_circle_table(): NULL input(s) (%p %p).\n", __FILE__, __LINE__,sint,cost);
        return 0;
    }

    int i;
    /* Table size, the sign of n flips the circle direction */
    const int size = abs(n);
    /* Determine the angle between samples */
    const double angle = 2*M_PI/( (double) n);
    /* Allocate memory for n samples, plus duplicate of first entry at the end */
    *sint = (double *) malloc((size+1)*sizeof(double));
    *cost = (double *) malloc((size+1)*sizeof(double));
    /* Bail out if memory allocation fails*/
    if (!(*sint) || !(*cost))
    {
        free(*sint);
        free(*cost);
        printf("%s: %d: g3d_circle_table(): memory allocation error.\n",__FILE__,__LINE__);
    }
    /* Compute cos and sin around the circle */
    for (i=0; i<size; i++)
    {
        (*sint)[i] = sin(angle*i);
        (*cost)[i] = cos(angle*i);
    }
    /* Last sample is duplicate of the first */
    (*sint)[size] = (*sint)[0];
    (*cost)[size] = (*cost)[0];

    return 1;
}

//! @ingroup graphic
//! Draws a sphere with OpenGL functions.
//! \param x x coordinate of the sphere center
//! \param y y coordinate of the sphere center
//! \param z z coordinate of the sphere center
//! \param radius radius of the sphere
//! \param nbSegments number of segments of the discretization of the sphere silhouette
void g3d_draw_solid_sphere(double x_, double y_, double z_, double radius, int nbSegments)
{
    int i, j;
    double r, r0;
    double x, y, z, z0;
    double *sint1, *cost1;
    double *sint2, *cost2;
    int n;
    if(nbSegments%2==0)
    {   n= nbSegments;  }
    else
    {   n= nbSegments+1;  }

    g3d_circle_table(&sint1, &cost1, -n);
    g3d_circle_table(&sint2, &cost2, n);

    for (i=1; i<=n/2; i++)
    {
        z0= cost2[i-1];
        r0= sint2[i-1];
        z = cost2[i];
        r = sint2[i];
        glPushAttrib(GL_LIGHTING_BIT);
        glShadeModel(GL_SMOOTH);
        glBegin(GL_TRIANGLE_STRIP);
        for(j=0; j<=n; j++)
        {
            x= cost1[j];
            y= sint1[j];
            glNormal3d(x*r, y*r, z);
            glTexCoord2d( 1-j/((double) n), 2*i/((double) n) );
            glVertex3d(x_ + x*r*radius, y_ + y*r*radius, z_ + z*radius);
            glNormal3d(x*r0, y*r0, z0);
            glTexCoord2d( 1-j/((double) n), 2*(i-1)/((double) n) );
            glVertex3d(x_ + x*r0*radius, y_ + y*r0*radius, z_ + z0*radius);
        }
        glEnd();
        glPopAttrib();
    }
    free(cost1);
    free(sint1);
    free(cost2);
    free(sint2);
}

//! @ingroup graphic
//! Cette fonction dessine un cone solide -dont les facettes sont
//! remplies- d'axe z et dont la pointe est en (0,0,0).
//! A utiliser dans une fonction d'affichage OpenGL.
void g3d_draw_solid_cone(double radius, double height, int nbSegments)
{
    int i, j;
    double *sint, *cost, z, dz, dr;
    double alpha= atan(height/radius);
    double ca= cos(alpha);
    double sa= sin(alpha);
    g3d_circle_table(&sint, &cost, -nbSegments);
    z= height/2;
    int nbSegments2= nbSegments;

    dz= height/nbSegments2;
    dr= radius*dz/height;
    //Les triangles des cÃ´tes:
    glBegin(GL_TRIANGLE_STRIP);
    for(i=0; i<nbSegments2; i++)
    {
        for(j=nbSegments; j>=0; j--)
        {
            glNormal3d(cost[j]*sa, sint[j]*sa, ca);
            glTexCoord2d(1-j/(double)nbSegments, 1.0f);
            glVertex3d(cost[j]*i*dr, sint[j]*i*dr, z-i*dz);
            glNormal3d(cost[j]*sa, sint[j]*sa, ca);
            glTexCoord2d(1-j/(double)nbSegments, 0.0f);
            glVertex3d(cost[j]*(i+1)*dr, sint[j]*(i+1)*dr, z-(i+1)*dz);
        }
    }
    glEnd();

    //Les triangles du dessous:
    glBegin(GL_TRIANGLE_FAN);
    glNormal3d(0, 0, -1);
    glTexCoord2d(0, 0.0f);
    glVertex3d(0, 0, -z);
    for(i=0; i<=nbSegments; i++)
    { glTexCoord2d(1-i/(double)nbSegments, 1.0f);
        glVertex3d(cost[i]*radius, sint[i]*radius, -z);
    }
    glEnd();

    free(sint);
    free(cost);
}

//! @ingroup graphic
//! Draws a 3D arrow.
//! \param p1 arrow's starting point
//! \param p2 arrow's ending point
//! \param red red component of the arrow's color
//! \param green green component of the arrow's color
//! \param blue blue component of the arrow's color
void g3d_draw_arrow(std::vector<double> p1, std::vector<double> p2, double red, double green, double blue)
{
    double length, cone_height;
    std::vector<double> p(3);
    p[0]= p2[0] - p1[0];
    p[1]= p2[1] - p1[1];
    p[2]= p2[2] - p1[2];
    length= sqrt( p[0]*p[0] + p[1]*p[1] + p[2]*p[2] );
    p[0]/= length;
    p[1]/= length;
    p[2]/= length;

    cone_height= 0.3*length;

    glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
    glLineWidth(5);
    glDisable(GL_LIGHTING);
    glColor3d(red, green, blue);
    glBegin(GL_LINES);
    glVertex3d(p1[0], p1[1], p1[2]);
    glVertex3d(p2[0], p2[1], p2[2]);
    glVertex3d(p2[0]-0.05*cone_height*p[0], p2[1]-0.05*cone_height*p[1], p2[2]-0.05*cone_height*p[2]);
    glEnd();
    glEnable(GL_LIGHTING);

    glPushMatrix();
    glTranslatef(p2[0]-0.05*length*p[0], p2[1]-0.05*length*p[1], p2[2]-0.05*length*p[2]);
    if( sqrt(p[0]*p[0]+p[1]*p[1]) > 1e-9 )
    {  glRotatef((180.0/M_PI)*asin(p[2]) - 90, p[1], -p[0], 0);  }
    else
    {
        if( p[2] < 0 )
        { glRotatef(180, 1, 0, 0); }
    }
    glColor3d( red, green, blue );
    g3d_draw_solid_cone( 0.3*cone_height, cone_height, 6);
    glPopMatrix();
    glPopAttrib();
}

//! @ingroup graphic
//! Fonction d'affichage d'un repere (matrice 4x4).
//! Les axes sont dessines sur une longueur "length".
//! A utiliser dans une fonction d'affichage OpenGL.
void g3d_draw_frame( const Frame3D& T, double length)
{
    std::vector<double> origin(3);
    std::vector<double> xAxis(3);
    std::vector<double> yAxis(3);
    std::vector<double> zAxis(3);

    origin[0]= T.R(0,3);
    origin[1]= T.R(1,3);
    origin[2]= T.R(2,3);

    xAxis[0]=  T.t[0] + length*T.R(0,0);
    xAxis[1]=  T.t[1] + length*T.R(1,0);
    xAxis[2]=  T.t[2] + length*T.R(2,0);

    yAxis[0]=  T.t[0] + length*T.R(0,1);
    yAxis[1]=  T.t[1] + length*T.R(1,1);
    yAxis[2]=  T.t[2] + length*T.R(2,1);

    zAxis[0]=  T.t[0] + length*T.R(0,2);
    zAxis[1]=  T.t[1] + length*T.R(1,2);
    zAxis[2]=  T.t[2] + length*T.R(2,2);

    g3d_draw_arrow( OpSpaceControl::GetStdVector(T.t), xAxis, 1.0, 0.0, 0.0);
    g3d_draw_arrow( OpSpaceControl::GetStdVector(T.t), yAxis, 0.0, 1.0, 0.0);
    g3d_draw_arrow( OpSpaceControl::GetStdVector(T.t), zAxis, 0.0, 0.0, 1.0);
}
